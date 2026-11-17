#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PTT 保险服务（CM108 卡键保护）

原理：bridge.py 每 200ms 写一次心跳文件。CM108 的 GPIO 是"写一次就保持"的类型，
      主程序崩溃/被杀/卡死时 PTT 会一直保持发射——这属于非法干扰。
      本服务独立进程运行，心跳超过 2 秒未更新就强行给 CM108 写 PTT-off。

独立进程的意义：bridge 崩了它还在，能兜底。
"""

import argparse
import configparser
import logging
import os
import signal
import sys
import time

from pymumble_py3.cm108_ptt import CM108PTT

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s guard: %(message)s")
log = logging.getLogger("guard")

running = True


def _stop(signum, frame):
    global running
    running = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default="/root/radio/radio.ini")
    ap.add_argument("--stale", type=float, default=2.0, help="心跳超时秒数")
    args = ap.parse_args()

    cfg = configparser.ConfigParser()
    cfg.read(args.config, encoding="utf-8")
    hb_file = cfg.get("service", "heartbeat_file",
                      fallback="/root/radio/run/heartbeat")
    dev = cfg.get("ptt", "hidraw", fallback="auto")
    gpio = cfg.getint("ptt", "gpio_pin", fallback=3)
    invert = cfg.get("ptt", "invert", fallback="true").lower() in ("1", "true", "yes")

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    ptt = CM108PTT(device_path=None if dev in ("", "auto") else dev,
                   gpio_pin=gpio, invert=invert)
    log.info("监控心跳 %s（超时 %.1fs），CM108=%s GPIO%d", hb_file, args.stale,
             ptt.device_path, gpio)
    ptt.ptt_off()

    last_warn = 0.0
    while running:
        time.sleep(0.3)
        try:
            mtime = os.path.getmtime(hb_file)
        except OSError:
            mtime = 0.0
        age = time.time() - mtime
        if age > args.stale:
            try:
                ptt.ptt_off()
            except Exception as e:
                log.error("强制落键失败：%s", e)
            if time.time() - last_warn > 5:
                log.warning("心跳已 %.1fs 未更新，强制 PTT OFF（bridge 可能已退出）", age)
                last_warn = time.time()

    ptt.ptt_off()
    ptt.close()
    log.info("退出")


if __name__ == "__main__":
    sys.exit(main())
