# radio-bot — Mumble ⇄ 电台互联桥

基于 pymumble 1.5（UDP 语音）+ CM108B USB 声卡的半双工桥：Mumble 里说的话从电台发出去，电台收到的声音回传 Mumble。

## 组成

| 文件 | 说明 |
| --- | --- |
| `bridge.py` | 主程序（pymumble 收发、ALSA 采集/播放、PTT 控制、VAD 门控） |
| `ptt-guard.py` | 卡键保护，PTT 异常按住时强制落键 |
| `radio.ini.example` | 配置模板，复制成 `radio.ini` 后按机器实际情况改 |
| `radio-bot.service` / `radio-guard.service` | systemd 单元 |

## 部署

```bash
mkdir -p /root/radio && cp bridge.py ptt-guard.py /root/radio/
cp radio.ini.example /root/radio/radio.ini
cp radio-bot.service radio-guard.service /etc/systemd/system/
systemctl daemon-reload && systemctl enable --now radio-bot
```

音频设备建议走 ALSA 共享插件，避免和报时、广播等程序抢声卡：

- 放音 `plug:dmix_out`
- 录音 `plug:dsnoop_in`

## 主要可调参数（都在 radio.ini）

### [mumble]
| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `ignore_users` | 空（注释） | 这些账号的音频不送电台；取消注释即可过滤音乐机器人。多个用逗号分隔 |
| `mic_gain_db` | 0 | 话筒灵敏度：电台→Mumble 方向增益，dB，只影响 Mumble 里的声音 |
| `volume_db` | 0 | 音量：Mumble→电台 方向增益，dB，只影响从 Mumble 发出的声音 |

### [audio]
| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `capture_device` / `playback_device` | plughw:1,0 | 采集/播放设备 |
| `capture_gain_db` / `playback_gain_db` | 0 | 声卡侧电平（电台 LINE OUT / MIC 校准用） |
| `rate` | 48000 | 采样率，别随便改 |
| `period_ms` | 10 | 帧长，>20 会明显增大延时 |
| `buffer_ms` | 40 | ALSA 缓冲 |

### [ptt]
| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `lead_ms` | 100 | 按下 PTT 后先发多久静音，等电台完成收发切换 |
| `fade_ms` | 30 | 起音淡入时长，抑制“弹簧/啵”爆音 |
| `prime_ms` | 60 | 缓冲多少毫秒音频才按下 PTT |
| `tail_ms` | 200 | 音频结束后继续按住多久再落键 |
| `max_tx_sec` | 60 | 最长按住 PTT，和电台自身超时同步，超时强制落键 |

### [gate]
| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `uplink_enabled` | true | 电台→Mumble 方向总开关，false 则只做单向 |
| `always_send` | false | true = 连续发声（不做 VAD，相当于客户端一直按键） |
| `vad_threshold` | 800 | VAD 触发阈值（RMS） |
| `vad_hold_ms` | 250 | 静音多久算说完 |
| `pre_roll_ms` | 150 | 触发时补发之前的音频，防止吞掉第一个字 |
| `tx_queue_max_ms` | 1500 | 下行队列上限，超了丢最老的保证低延时 |

### [service]
| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `diag` / `diag_period_sec` | true / 5 | 诊断日志开关与周期 |
| `heartbeat_interval_sec` | 0.2 | 心跳文件写入间隔（guard 靠它判断存活） |
| `keepalive_sec` | 2.0 | PTT 保活间隔 |
| `lag_warn_frames` / `lag_warn_interval_sec` | 5 / 5 | 采集落后告警阈值与节流 |
| `mumble_buf_warn_ms` | 400 | Mumble 发送积压告警阈值 |
| `selftest_interval_sec` | 4.0 | 自检模式注入间隔 |

改完 `systemctl restart radio-bot` 生效。
