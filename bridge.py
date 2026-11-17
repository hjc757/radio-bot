#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mumble <-> 车载电台 互联桥（N1 + CM108B 单卡方案，不依赖 COR）

链路：
  电台 LINE OUT ──> CM108 录音端 ──> 本程序 ──> Mumble
  Mumble ──> 本程序 ──> CM108 播放端 ──> 电台 MIC IN
  CM108 GPIO3 ──> MOSFET ──> 电台 PTT

"不吃字"的三条实现：
  1) 接收方向：300ms 环形缓冲常驻，软件 VAD 一触发就把最近 150ms 一起补发（pre-roll）
  2) 发射方向：先攒 60ms 音频、按 PTT、再插入 45ms 静音等电台起键，音频一帧不丢
  3) 释放方向：最后一段音频播完后再保持 200ms 才落 PTT（尾音不被咬掉）

半双工互锁（软件 COR 的替代）：
  - 发射期间丢弃采集数据（防止电台自身发射音频回流成回声）
  - 采集侧判定"忙"时不允许开 PTT
"""

import argparse
import collections
import configparser
import logging
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time

import pymumble_py3 as pymumble
import pymumble_py3.constants
from pymumble_py3.cm108_ptt import CM108PTT, list_cm108_devices

RATE = 48000
BYTES_PER_SAMPLE = 2


def db2gain(db):
    return 10.0 ** (db / 20.0)


def rms_int16(data):
    """int16 小端 PCM 的 RMS"""
    n = len(data) // 2
    if n == 0:
        return 0
    fmt = "<%dh" % n
    samples = struct.unpack(fmt, data[: n * 2])
    return int(math.sqrt(sum(s * s for s in samples) / n))


def scale_frame(data, factor):
    # 按系数缩放一帧 PCM，用于淡入淡出，避免爆音
    if factor >= 1.0:
        return data
    n = len(data) // 2
    samples = struct.unpack("<%dh" % n, data[: n * 2])
    out = [int(x * factor) for x in samples]
    return struct.pack("<%dh" % n, *out)


def apply_gain(data, gain):
    if abs(gain - 1.0) < 1e-6:
        return data
    n = len(data) // 2
    samples = struct.unpack("<%dh" % n, data[: n * 2])
    out = []
    for s in samples:
        v = int(s * gain)
        if v > 32767:
            v = 32767
        elif v < -32768:
            v = -32768
        out.append(v)
    return struct.pack("<%dh" % n, *out)


class Config(object):
    DEFAULTS = {
        "mumble": {
            "host": "127.0.0.1",
            "port": "64738",
            "username": "RADIO",
            "password": "",
            "certfile": "/root/radio/radio.pem",
            "tokens": "",
            "channel": "",
            "ignore_users": "",
            "mic_gain_db": "0",
            "volume_db": "0",
        },
        "audio": {
            "capture_device": "plughw:1,0",
            "playback_device": "plughw:1,0",
            "capture_gain_db": "0",
            "playback_gain_db": "0",
            "rate": "48000",
            "period_ms": "10",
            "buffer_ms": "40",
        },
        "ptt": {
            "hidraw": "auto",
            "gpio_pin": "3",
            "invert": "true",
            "lead_ms": "45",
            "fade_ms": "30",
            "prime_ms": "60",
            "tail_ms": "200",
            "max_tx_sec": "60",
        },
        "gate": {
            "uplink_enabled": "true",
            "always_send": "false",
            "vad_threshold": "800",
            "vad_hold_ms": "250",
            "pre_roll_ms": "150",
            "tx_queue_max_ms": "1500",
        },
        "service": {
            "heartbeat_file": "/root/radio/run/heartbeat",
            "diag": "true",
            "diag_period_sec": "5",
            "heartbeat_interval_sec": "0.2",
            "keepalive_sec": "2.0",
            "lag_warn_frames": "5",
            "lag_warn_interval_sec": "5",
            "mumble_buf_warn_ms": "400",
            "selftest_interval_sec": "4.0",
        },
    }

    def __init__(self, path):
        self.cfg = configparser.ConfigParser()
        # 写入默认值
        for sec, kv in self.DEFAULTS.items():
            self.cfg[sec] = dict(kv)
        if path and os.path.exists(path):
            self.cfg.read(path, encoding="utf-8")

    def get(self, sec, key):
        return self.cfg.get(sec, key, fallback=self.DEFAULTS[sec][key]).strip()

    def getint(self, sec, key):
        return int(self.get(sec, key))

    def getfloat(self, sec, key):
        return float(self.get(sec, key))

    def getbool(self, sec, key):
        return self.get(sec, key).lower() in ("1", "true", "yes", "on")


class Bridge(object):
    def __init__(self, cfg, dry_run=False):
        self.cfg = cfg
        self.dry_run = dry_run
        self.log = logging.getLogger("bridge")

        self.rate = cfg.getint("audio", "rate")
        self.period_ms = cfg.getint("audio", "period_ms")
        self.frame_samples = int(self.rate * self.period_ms / 1000)
        self.frame_bytes = self.frame_samples * BYTES_PER_SAMPLE

        self.cap_gain = db2gain(cfg.getfloat("audio", "capture_gain_db"))
        self.play_gain = db2gain(cfg.getfloat("audio", "playback_gain_db"))

        # 电台→Mumble 方向总开关：关掉后只做 Mumble→电台，电台声音不再送回 Mumble
        # 话筒灵敏度（电台→Mumble 方向的增益，dB，正数更灵敏）
        self.mic_gain = db2gain(cfg.getfloat("mumble", "mic_gain_db"))
        # 音量（Mumble→电台 方向的增益，dB，正数更响）
        self.mumble_volume = db2gain(cfg.getfloat("mumble", "volume_db"))
        self.uplink_enabled = cfg.getbool("gate", "uplink_enabled")
        # 连续发声：不做 VAD 门控，采集到的声音一帧不留地持续送进 Mumble（像客户端一直按键）
        self.always_send = cfg.getbool("gate", "always_send")
        self.vad_threshold = cfg.getint("gate", "vad_threshold")
        self.vad_hold = cfg.getfloat("gate", "vad_hold_ms") / 1000.0
        self.pre_roll_frames = max(1, cfg.getint("gate", "pre_roll_ms") // self.period_ms)
        self.tx_queue_max = max(10, cfg.getint("gate", "tx_queue_max_ms") // self.period_ms)

        self.lead_frames = max(1, cfg.getint("ptt", "lead_ms") // self.period_ms)
        # 起音/落音淡入淡出，避免 PTT 打开瞬间的啵/弹簧爆音
        try:
            fade_ms = cfg.getfloat("ptt", "fade_ms")
        except Exception:
            fade_ms = 30.0
        self.fade_frames = max(1, int(fade_ms // self.period_ms))
        self.fade_in_left = 0
        self.prime_frames = max(1, cfg.getint("ptt", "prime_ms") // self.period_ms)
        self.tail = cfg.getfloat("ptt", "tail_ms") / 1000.0
        self.max_tx = cfg.getfloat("ptt", "max_tx_sec")

        self.heartbeat_file = cfg.get("service", "heartbeat_file")
        # 心跳/保活/告警等节奏参数，都可在 radio.ini 里调
        self.heartbeat_interval = cfg.getfloat("service", "heartbeat_interval_sec")
        self.keepalive_sec = cfg.getfloat("service", "keepalive_sec")
        self.lag_warn_frames = cfg.getint("service", "lag_warn_frames")
        self.lag_warn_interval = cfg.getfloat("service", "lag_warn_interval_sec")
        self.buf_warn_ms = cfg.getint("service", "mumble_buf_warn_ms")
        self.selftest_interval = cfg.getfloat("service", "selftest_interval_sec")

        # 环形缓冲：常驻最近 N 帧，用于补发第一声
        self.ring = collections.deque(maxlen=self.pre_roll_frames + 2)
        self.rx_busy = False
        self.rx_last_active = 0.0

        self.txq = collections.deque()
        self.tx_lock = threading.Lock()

        self.ptt_on = False
        self.ptt_key_time = 0.0
        self.tx_last_audio = 0.0
        self.tx_last_audio_mono = 0.0
        self.pad_left = 0
        self.ptt = None

        self.silence = b"\x00" * self.frame_bytes
        self.running = True
        self.arecord = None
        self.aplay = None

        self.selftest = False
        self._st_frame = 0

        self.stats_rx_frames = 0
        self.stats_tx_frames = 0

        # ---- 诊断统计 ----
        self.diag = cfg.getbool("service", "diag")
        self.diag_period = cfg.getfloat("service", "diag_period_sec")
        self.diag_last = 0.0
        self.diag_rx_triggers = 0
        self.diag_prefill_last = 0
        self.diag_rx_lag_max = 0
        self.diag_rx_lag_warn_t = 0.0
        self.diag_cap_start = None
        self.diag_cap_frames = 0
        self.diag_ptt_on_t = 0.0
        self.diag_lead_ms = None
        self.diag_lead_pending = False
        self.diag_tail_ms = None
        self.diag_tx_dropped = 0
        self.diag_mumble_buf_max = 0.0

    # ---------------- CM108 PTT ----------------

    def open_ptt(self):
        if self.dry_run:
            self.log.warning("dry-run：PTT 只记录状态，不真正操作 CM108")
            return
        dev = self.cfg.get("ptt", "hidraw")
        path = None if dev in ("", "auto") else dev
        self.ptt = CM108PTT(device_path=path,
                            gpio_pin=self.cfg.getint("ptt", "gpio_pin"),
                            invert=self.cfg.getbool("ptt", "invert"))
        self.log.info("CM108 就绪：%s GPIO%d invert=%s",
                      self.ptt.device_path, self.ptt.gpio_pin, self.ptt.invert)

    def key_ptt(self, on):
        if on and self.ptt_on:
            return
        if (not on) and (not self.ptt_on):
            return

        if not self.dry_run:
            try:
                if on:
                    self.ptt.ptt_on()
                else:
                    self.ptt.ptt_off()
            except Exception as e:
                self.log.error("PTT 写 CM108 失败：%s", e)
                return

        self.ptt_on = on
        suffix = " [dry-run]" if self.dry_run else ""

        if on:
            self.ptt_key_time = time.time()
            self.diag_ptt_on_t = time.monotonic()
            self.diag_lead_pending = True
            self.pad_left = self.lead_frames
            self.fade_in_left = self.fade_frames
            self.log.info("PTT ON（电台开始发射）%s", suffix)
        else:
            tail_ms = None
            if self.tx_last_audio_mono:
                tail_ms = (time.monotonic() - self.tx_last_audio_mono) * 1000.0
                self.diag_tail_ms = tail_ms
            self.log.info("PTT OFF（尾音保持 %.0fms / Mumble 积压 %.0fms）%s",
                          tail_ms if tail_ms is not None else 0.0,
                          self.current_mumble_buffer_ms(), suffix)

    def ptt_keepalive(self):
        """CM108 GPIO 是保持型：周期性重写，确保状态机与硬件一致"""
        if self.dry_run or self.ptt is None:
            return
        try:
            if self.ptt_on:
                self.ptt.ptt_on()
            else:
                self.ptt.ptt_off()
        except Exception as e:
            self.log.error("PTT 保活失败：%s", e)

    # ---------------- 音频设备 ----------------

    def start_audio(self):
        if self.selftest:
            self.log.warning("自检模式：不打开声卡，使用合成音频")
            threading.Thread(target=self._selftest_tx_feeder, daemon=True).start()
            return

        period_us = self.period_ms * 1000
        buffer_us = self.cfg.getint("audio", "buffer_ms") * 1000
        cap_dev = self.cfg.get("audio", "capture_device")
        play_dev = self.cfg.get("audio", "playback_device")

        rec_cmd = ["arecord", "-q", "-D", cap_dev, "-f", "S16_LE", "-r", str(self.rate),
                   "-c", "1", "-t", "raw",
                   "--period-time=%d" % period_us, "--buffer-time=%d" % buffer_us]
        play_cmd = ["aplay", "-q", "-D", play_dev, "-f", "S16_LE", "-r", str(self.rate),
                    "-c", "1", "-t", "raw",
                    "--period-time=%d" % period_us, "--buffer-time=%d" % buffer_us]

        self.log.info("采集：%s", " ".join(rec_cmd))
        self.arecord = subprocess.Popen(rec_cmd, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, bufsize=0)
        self.log.info("播放：%s", " ".join(play_cmd))
        self.aplay = subprocess.Popen(play_cmd, stdin=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, bufsize=0)

    def _synth_frame(self):
        """合成：2.5s 静音 + 1.0s 1kHz 正弦（模拟"有人讲话"）"""
        cycle = 350  # 帧数：25 静音 + 10 有声
        idx = self._st_frame % 350
        self._st_frame += 1
        if idx < 250:      # 2.5s 静音（25 帧）
            return b"\x00" * self.frame_bytes
        # 1kHz 正弦，幅度 8000
        n = self.frame_samples
        out = []
        for i in range(n):
            v = int(8000 * math.sin(2 * math.pi * 1000 * (idx * n + i) / self.rate))
            out.append(v)
        return struct.pack("<%dh" % n, *out)

    def read_frame(self):
        if self.selftest:
            self._st_next = getattr(self, "_st_next", time.monotonic())
            self._st_next += self.period_ms / 1000.0
            wait = self._st_next - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            return self._synth_frame()
        buf = b""
        while len(buf) < self.frame_bytes and self.running:
            chunk = self.arecord.stdout.read(self.frame_bytes - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    # ---------------- Mumble ----------------

    def start_mumble(self):
        c = self.cfg
        self.ignore_users = [x.strip() for x in c.get("mumble", "ignore_users").split(",") if x.strip()]
        if self.ignore_users:
            self.log.info("忽略账号（不往电台分流）：%s", ", ".join(self.ignore_users))
        self.mumble = pymumble.Mumble(
            c.get("mumble", "host"),
            user=c.get("mumble", "username"),
            port=c.getint("mumble", "port"),
            password=c.get("mumble", "password") or "",
            certfile=c.get("mumble", "certfile") or None,
            tokens=[t for t in c.get("mumble", "tokens").split(",") if t] or None,
            stereo=False,
            debug=False,
        )
        self.mumble.callbacks.set_callback(
            pymumble.constants.PYMUMBLE_CLBK_SOUNDRECEIVED, self.on_sound)
        self.mumble.set_codec_profile("audio")
        self.mumble.start()
        self.mumble.is_ready()
        if self.mumble.connected >= pymumble.constants.PYMUMBLE_CONN_STATE_FAILED:
            raise RuntimeError("Mumble 连接失败")
        self.log.info("Mumble 已连接：%s:%d 用户 %s", c.get("mumble", "host"),
                      c.getint("mumble", "port"), c.get("mumble", "username"))
        try:
            self.mumble.users.myself.unmute()
        except Exception:
            pass
        self.mumble.set_receive_sound(True)
        chan = c.get("mumble", "channel")
        if chan:
            try:
                ch = self.mumble.channels.find_by_name(chan)
                if ch:
                    ch.move_in()
                    self.log.info("已进入频道 %s", chan)
            except Exception as e:
                self.log.warning("进入频道 %s 失败：%s", chan, e)

    def on_sound(self, user, sound):
        """Mumble 收到别人的音频（48k mono 20ms 一块）"""
        if isinstance(user, dict):
            name = user.get("name", "") or ""
        else:
            name = getattr(user, "name", "") or ""
        if name == self.cfg.get("mumble", "username"):
            return  # 自己的回环
        seen = getattr(self, "seen_src", None)
        if seen is None:
            seen = self.seen_src = set()
        if name not in seen:
            seen.add(name)
            self.log.info("音频来源: %s", name)
        if name in getattr(self, "ignore_users", ()):
            return  # 音乐机器人等：不往电台分流，避免长期占住 PTT
        with self.tx_lock:
            if len(self.txq) >= self.tx_queue_max:
                self.txq.popleft()   # 队列过长说明积压，丢最老的保证低延迟
                self.diag_tx_dropped += 1
            self.txq.append(sound.pcm)

    # ---------------- 诊断 ----------------

    def current_mumble_buffer_ms(self):
        """当前已交给 pymumble 但还没发出去的量（毫秒）"""
        try:
            return self.mumble.sound_output.get_buffer_size() * 1000
        except Exception:
            return 0.0

    def check_capture_lag(self):
        """按墙钟推算应采到的帧数，和实际帧数比，差值就是采集掉帧"""
        if self.diag_cap_start is None:
            return
        expected = int((time.monotonic() - self.diag_cap_start) * 1000.0 / self.period_ms)
        lag = expected - self.diag_cap_frames
        if lag > self.diag_rx_lag_max:
            self.diag_rx_lag_max = lag
        if lag >= self.lag_warn_frames and time.monotonic() - self.diag_rx_lag_warn_t > self.lag_warn_interval:
            self.log.warning("采集落后 %d 帧（约 %dms），疑似 arecord underrun 或 CPU 卡顿"
                             "（检查 buffer_ms / 系统负载）", lag, lag * self.period_ms)
            self.diag_rx_lag_warn_t = time.monotonic()

    def maybe_diag_summary(self, now):
        if not self.diag or now - self.diag_last < self.diag_period:
            return
        self.diag_last = now
        buf = self.current_mumble_buffer_ms()
        if buf > self.diag_mumble_buf_max:
            self.diag_mumble_buf_max = buf
        lead = "%.0f" % self.diag_lead_ms if self.diag_lead_ms is not None else "-"
        tail = "%.0f" % self.diag_tail_ms if self.diag_tail_ms is not None else "-"
        self.log.info("诊断：RX触发%d次 预录%d帧 采集落后≤%dms | TX队列丢弃%d | "
                      "起键→首帧%s ms 尾音%s ms | Mumble积压当前%.0fms 峰值%.0fms | RX帧%d TX帧%d",
                      self.diag_rx_triggers, self.diag_prefill_last,
                      self.diag_rx_lag_max * self.period_ms, self.diag_tx_dropped,
                      lead, tail, buf, self.diag_mumble_buf_max,
                      self.stats_rx_frames, self.stats_tx_frames)
        if buf > self.buf_warn_ms:
            self.log.warning("Mumble 侧积压 %.0fms，编码/网络跟不上，先查 CPU 与网络", buf)

    # ---------------- 采集（电台 → Mumble） ----------------

    def capture_loop(self):
        self.diag_cap_start = time.monotonic()
        self.diag_cap_frames = 0
        while self.running:
            data = self.read_frame()
            if data is None:
                self.log.warning("采集流中断")
                break
            data = apply_gain(data, self.cap_gain)
            self.ring.append(data)

            # 半双工：我方发射时，电台自身音频会回灌，直接丢弃
            if self.ptt_on:
                self.rx_busy = False
                continue

            # 关闭上行时：只保留采集（防止 arecord 堵塞），不送 Mumble
            if not self.uplink_enabled:
                continue

            # 连续发声模式：不等 VAD，每帧都送 Mumble（电台侧相当于客户端一直按住说话）
            if self.always_send:
                self.mumble.sound_output.add_sound(apply_gain(data, self.mic_gain))
                self.stats_rx_frames += 1
                continue

            self.diag_cap_frames += 1
            if not self.selftest:
                self.check_capture_lag()

            level = rms_int16(data)
            now = time.time()

            if level >= self.vad_threshold:
                if not self.rx_busy:
                    self.rx_busy = True
                    # 补发第一声：把触发前的历史帧一起送
                    hist = list(self.ring)[:-1][-self.pre_roll_frames:]
                    for f in hist:
                        self.mumble.sound_output.add_sound(f)
                    self.diag_rx_triggers += 1
                    self.diag_prefill_last = len(hist)
                    self.log.info("RX 开始（预录 %d 帧=%.0fms / 触发 RMS %d / 阈值 %d）",
                                  len(hist), len(hist) * self.period_ms,
                                  level, self.vad_threshold)
                self.rx_last_active = now
                self.mumble.sound_output.add_sound(apply_gain(data, self.mic_gain))
                self.stats_rx_frames += 1
            elif self.rx_busy:
                if now - self.rx_last_active > self.vad_hold:
                    self.rx_busy = False
                    self.log.info("RX 结束")
                else:
                    self.mumble.sound_output.add_sound(data)

    # ---------------- 播放（Mumble → 电台） ----------------

    def tx_loop(self):
        hb_last = 0.0
        pa_last = 0.0
        while self.running:
            now = time.time()
            self.maybe_diag_summary(now)

            with self.tx_lock:
                pending = len(self.txq)

            if not self.ptt_on:
                if pending >= self.prime_frames and not self.rx_busy:
                    self.key_ptt(True)
                    # 插入起键静音（电台 PTT 建立需要时间）
                    with self.tx_lock:
                        self.txq = collections.deque(
                            [self.silence] * self.lead_frames + list(self.txq))
                    self.tx_last_audio = now
                else:
                    self.write(silence=True)
                    self.write_frame_heartbeat(now)
                    continue

            # 已处于发射态
            with self.tx_lock:
                frame = self.txq.popleft() if self.txq else None

            if frame is None:
                self.write(silence=True)
                if now - self.tx_last_audio > self.tail:
                    self.key_ptt(False)
            else:
                if self.pad_left > 0:
                    self.pad_left -= 1          # 这段是起键填充静音，不计入延时
                else:
                    if self.diag_lead_pending:
                        lead = (time.monotonic() - self.diag_ptt_on_t) * 1000.0
                        self.diag_lead_ms = lead
                        self.diag_lead_pending = False
                        self.log.info("起键到首帧音频 %.0fms（含 lead 静音 %dms + ALSA 缓冲约 %dms）",
                                      lead, self.lead_frames * self.period_ms,
                                      self.cfg.getint("audio", "buffer_ms") // 2)
                frame = apply_gain(frame, self.play_gain * self.mumble_volume)
                if self.fade_in_left > 0:
                    k = self.fade_frames - self.fade_in_left + 1
                    frame = scale_frame(frame, float(k) / self.fade_frames)
                    self.fade_in_left -= 1
                self.write(frame)
                self.tx_last_audio = now
                self.tx_last_audio_mono = time.monotonic()
                self.stats_tx_frames += 1

            # 卡键保护
            if self.ptt_on and now - self.ptt_key_time > self.max_tx:
                self.log.error("PTT 超过 %.0fs 上限，强制落键", self.max_tx)
                self.key_ptt(False)
                with self.tx_lock:
                    self.txq.clear()

            self.write_frame_heartbeat(now, hb_last)
            hb_last = now

            if now - pa_last > self.keepalive_sec:
                self.ptt_keepalive()
                pa_last = now

    def write(self, data=None, silence=False):
        payload = self.silence if silence else data
        if self.selftest:
            time.sleep(self.period_ms / 1000.0)
            return
        try:
            self.aplay.stdin.write(payload)   # 阻塞写，由 ALSA 消费速率自然节流
        except Exception as e:
            self.log.error("播放写入失败：%s", e)

    def write_frame_heartbeat(self, now, last=0.0):
        if now - last < self.heartbeat_interval:
            return
        try:
            os.makedirs(os.path.dirname(self.heartbeat_file), exist_ok=True)
            with open(self.heartbeat_file, "w") as f:
                f.write("%.3f\n" % now)
        except Exception:
            pass

    def _selftest_tx_feeder(self):
        while self.running:
            time.sleep(self.selftest_interval)
            chunk = []
            for _ in range(60):     # 600ms
                chunk.append(self._synth_frame())
            with self.tx_lock:
                self.txq.extend(chunk)
            self.log.info("自检：注入 600ms 下行音频")

    # ---------------- 主循环 ----------------

    def run(self):
        self.open_ptt()
        self.start_audio()
        if self.selftest:
            self.log.warning("自检模式：不连接 Mumble")
            class _Null:
                class sound_output:
                    @staticmethod
                    def add_sound(pcm):
                        pass
            self.mumble = _Null()
        else:
            self.start_mumble()

        t_cap = threading.Thread(target=self.capture_loop, name="capture", daemon=True)
        t_cap.start()
        try:
            self.tx_loop()
        finally:
            self.shutdown()

    def shutdown(self):
        self.running = False
        try:
            if self.ptt_on or self.ptt:
                self.key_ptt(False)
        except Exception:
            pass
        for p in (self.arecord, self.aplay):
            try:
                if p:
                    p.terminate()
                    p.wait(timeout=2)
            except Exception:
                pass
        self.log.info("已退出，统计：RX %d 帧 / TX %d 帧",
                      self.stats_rx_frames, self.stats_tx_frames)


def main():
    ap = argparse.ArgumentParser(description="Mumble <-> 车载电台互联桥")
    ap.add_argument("-c", "--config", default="/root/radio/radio.ini")
    ap.add_argument("--dry-run", action="store_true",
                    help="不操作 CM108 PTT，只记录状态（给电台接线前自测用）")
    ap.add_argument("--selftest", action="store_true",
                    help="自检模式：用合成音频替代声卡，验证 VAD/PTT 时序与诊断日志")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    cfg = Config(args.config)
    bridge = Bridge(cfg, dry_run=args.dry_run)
    bridge.selftest = args.selftest

    def _stop(signum, frame):
        bridge.running = False
        bridge.log.info("收到信号 %s，准备退出", signum)

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        bridge.run()
    except Exception as e:
        logging.getLogger("bridge").error("启动失败：%s", e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
