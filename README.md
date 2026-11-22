# Orbis F118 · JC3636F118 旋钮圆屏 Home Assistant 控制面板

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.1-red.svg)](https://docs.espressif.com/projects/esp-idf/)
[![Target](https://img.shields.io/badge/target-ESP32--S31-8A2BE2.svg)](https://www.espressif.com/en/products/socs/esp32-s31)
[![Switchyard](https://img.shields.io/badge/HA%20%E6%8F%92%E4%BB%B6-switchyard-41BDF5.svg)](https://github.com/xiao-si-qi/switchyard)

> 硬件：GUITION **JC3636F118C**（铝合金 CNC 外壳，带电池、旋钮、麦克风、3.5 mm 音频口）。
> 本项目不是厂家官方项目。

基于 **ESP32-S31（RISC-V 双核 320 MHz）+ 1.65" 圆形 360×360 ST77916 + 旋转编码器** 的
Home Assistant 触屏控制面板：弧形刻度盘调灯 / 调空调 / 调风扇，卡片页一屏管全家。

## 同系列的另一块屏

本仓库是 **F118 硬件适配**，与另一块屏共用同一套软件架构（JSON 布局、switchyard 协议、HA 通道），
只有板级驱动和部分 UI 尺寸不同：

| | **本仓库** `orbis-f118` | [`orbis-w518`](https://github.com/xiao-si-qi/orbis-w518) |
|---|---|---|
| 硬件 | GUITION **JC3636F118C** | GUITION 太极派 **JC3636W518** |
| MCU | **ESP32-S31**：RISC-V 双核 320 MHz + LP 核 | ESP32-S3R8：Xtensa 双核 240 MHz |
| 屏幕 | 1.65" 360×360 ST77916 **QSPI**（80 MHz）| 1.85" 360×360 ST77916 **4-line QSPI**（40 MHz）|
| 触摸 | CST816S（I2C 0x15）| CST816S（I2C 0x15）|
| 旋钮 | ✅ **有**（GPIO 6/7）| ❌ 无 |
| 触觉 | DRV2605L（与触摸共用 I2C）| 无 |
| 音频 | PDM 麦克风 + PCM5100A 线路输出（**无喇叭**）| I2S 麦克风 + PCM5100A |
| 电池 | ✅ 有（ADC GPIO55，未标定）| ❌ 无 |
| 加速度计 | ❌ 未贴片 | — |
| ESP-IDF | **v6.1**（`--preview`，target `esp32s31`）| v5.5.4 |
| 固件差异 | 蓝牙/语音默认编译排除，内部 RAM 紧张 | 功能全开 |

两块屏的**布局协议、MQTT 约定、HA 通道完全一致**，布局可以在
[switchyard](https://github.com/xiao-si-qi/switchyard) 里互通编辑。

## 界面

```
┌──────────── 卡片页 ────────────┐   ┌──────────── 灯光页 ────────────┐
│      ◉ ᛒ  ▭  ⌂  )))           │   │            ╭─────────╮         │
│           客厅                 │   │         ╱   ▓▓▓▓▓░░   ╲       │
│   ┌──────────┐ ┌──────────┐    │   │        │      💡       │      │
│   │ 💡     ● │ │ ❄      ● │    │   │        │      68%      │      │
│   │     68%  │ │     25°  │    │   │        │    3500K      │      │
│   │  客厅主灯 │ │  主卧空调 │    │   │         ╲  [亮度|色温] ╱       │
│   └──────────┘ └──────────┘    │   │          ╰──── ◉ ────╯         │
│   ┌──────────┐ ┌──────────┐    │   │        旋转调节 · 左滑返回      │
│   │ 🌀     ● │ │ 🌤     ● │    │   └────────────────────────────────┘
│   └──────────┘ └──────────┘    │
│           ● ● ○                │   空调页 / 风扇页用同一套弧线语言
└────────────────────────────────┘
```

| 页面 | 交互 |
|---|---|
| **卡片页** | 2×2 卡片（140×90，圆角 18），图标左上、状态点右上；点卡片进详情；顶部状态栏是弧线排布的四个图标（蓝牙/电池/HA/WiFi），页面点是胶囊 |
| **灯光页** | 单环 270°（缺口朝下），环外=亮度、环心=色温（也可用分段控件切换）；中央大字数值 + 大字色温；圆形电源键；灯色实时驱动页面氛围光 |
| **空调页** | 温度环 + 目标温度 + 室内温度 + **每台空调自适应步进**（0.1 °C 整数运算）+ 5 个模式键 + 风速/摆风胶囊 + 电源键 |
| **风扇页** | 风速环 + 预设风胶囊（直吹/自然/循环/睡眠，按实体真实 preset_modes 循环）+ 电源键 |
| **天气页** | 当前天气 + 六格数据 + 逐小时 6 条 + 逐日 3 条，**旋钮上下滚动**（不翻页）|
| **传感器页** | 任意 `sensor.*` 卡片点进来，大号数值 + 单位 + 图标 |
| **设置页** | 系统状态（固件/MQTT/WiFi/运行/内存/电池）、检查更新（OTA）、蓝牙、音频输出，整页可滚动 |

旋钮在所有页面都是**翻页**，只有详情页接管它（调值 / 滚动天气页）。

## 技术要点

| 项目 | 做法 |
|---|---|
| 图形 | LVGL 9.6，**堆放在 PSRAM**（自定义分配器），内部 RAM 让给 DMA/TLS |
| 刷新 | 绘制缓冲在**内部 RAM + 64 字节对齐**，SPI 直接零拷贝 DMA（无中转缓冲）|
| 背景图 | **硬件 JPEG 解码**（3 ms/张，软件解码 94 ms）；解码缓冲在 PSRAM |
| 布局 | switchyard 通过 MQTT 下发 JSON，`layout.c` 解析后动态建页面，不重烧固件就能改界面 |
| HA 通道 | REST 轮询 + **WebSocket 长连接**（`state_changed` 实时推送、命令快通道），全程只保留一条 TLS |
| 内部 RAM | 关闭蓝牙/语音后稳态 **126–132 KB** 空闲 |
| 远程调试 | `screenshot` 截图（硬件 JPEG 编码走 MQTT）、`page`/`open`/`close` 远程导航 |

## 硬件

| 项目 | 参数 |
|---|---|
| MCU | ESP32-S31：RISC-V 双核 320 MHz + LP 核，512 KB SRAM |
| 存储 | 16 MB Flash + **16 MB 八线 PSRAM @200 MHz** |
| 屏幕 | 1.65" 360×360 IPS，ST77916，QSPI 80 MHz，RGB565 大端，INVON |
| 触摸 | CST816S @0x15（SDA 11 / SCL 10 / INT 8 / RST 9）|
| 触觉 | DRV2605L @0x5A（与触摸共用 I2C）|
| 旋钮 | 增量编码器 A=7 / B=6，1 ms 轮询解码，一格半个周期 |
| 背光 | GPIO5，LEDC 5 kHz 调光 |
| 音频 | PDM 麦克风 CLK 39 / DATA 37；I2S 输出 BCLK 53 / LRCK 51 / DOUT 52 → PCM5100A → 3.5 mm |
| 电池 | ADC GPIO55（15k/10k 分压，**未标定**，只上报原始值）|
| 其他 | TF 卡（SDMMC slot0）、蜂鸣器 GPIO44、USB 原生 USB-Serial/JTAG |

完整实测记录（含踩坑、QSPI 命令编码、分区表）见 **[docs/hardware-f118.md](docs/hardware-f118.md)**。

## 编译 & 烧写

```bash
# ESP-IDF v6.1（esp32s31 仍是 preview，所有 idf.py 命令都要 --preview）
. $HOME/esp/esp-idf/export.sh          # Windows: C:\esp\v6.1\esp-idf\export.ps1
idf.py --preview set-target esp32s31   # 只需一次
idf.py --preview build
idf.py --preview -p /dev/ttyACM0 flash monitor
```

Windows 下串口是原生 USB（`VID 303A PID 1001`），设备名形如 `COM9`。

### 首次配置

```bash
cp main/secrets.h.example main/secrets.h   # 然后填入自己的值
```

`secrets.h` 已被 `.gitignore` 忽略，**不要提交**。首次开机这些值会写进 NVS，之后以 NVS 为准。

也可以不开网页配置服务（默认关闭，省 6–10 KB 内部 RAM），直接改 `secrets.h` 重烧。

## 配置开关

`sdkconfig.defaults` 里两个默认关闭的功能（这块屏内部 RAM 紧张，砍掉才够 LVGL + TLS）：

| 配置 | 默认 | 说明 |
|---|---|---|
| `CONFIG_APP_VOICE_ENABLED` | **n** | 小智语音助手（WebSocket + Opus + PDM 录音）。开启后点说话页会 CPU 锁死复位，音频路径仍需调试 |
| `CONFIG_BT_ENABLED` | **n** | 经典蓝牙 A2DP（连蓝牙音箱）。静态占约 52 KB IRAM，且 S31 上 `esp_bt_controller_mem_release` 是空实现，关掉才能回收 |

开启方法：改 `sdkconfig.defaults` → **删掉 `sdkconfig`** → 重新 `idf.py --preview reconfigure`（已有 `sdkconfig` 时 defaults 不覆盖旧值）。

## 布局（JSON 驱动）

布局由 [switchyard](https://github.com/xiao-si-qi/switchyard) 在 HA 里编辑，通过 MQTT
`switchyard/<device_id>/cfg` 下发；设备解析后动态建页面，改布局**不用重烧固件**。

```json
{
  "pages": [
    { "title": "客厅", "tiles": [
      { "type": "toggle", "entity": "light.xiaomi_cn_..._light" },
      { "type": "sensor", "entity": "weather.he_feng_tian_qi" }
    ]},
    { "title": "卧室", "tiles": [
      { "type": "toggle", "entity": "climate.1" },
      { "type": "toggle", "entity": "fan.xiaomi_cn_..._fan" }
    ]}
  ]
}
```

`light.*` / `climate.*` / `fan.*` 点开各自的弧形详情页，`weather.*` 进天气页，
其它 `sensor.*` 进通用传感器页。

## 远程调试命令（MQTT）

发布到 `switchyard/<device_id>/cmd`：

| 命令 | 作用 |
|---|---|
| `{"cmd":"screenshot"}` | 抓当前画面 → 硬件 JPEG 编码 → 分块 base64 发到 `switchyard/<id>/shot` |
| `{"cmd":"shot_serial"}` | 同样的图，走串口 base64（约 4–5 秒）|
| `{"cmd":"page","value":2}` / `"next"` / `"prev"` | 切页 |
| `{"cmd":"open","entity":"light.x..."}` | 打开指定实体详情页（灯/空调/风扇/天气）|
| `{"cmd":"close"}` | 关闭详情页回卡片页 |
| `{"cmd":"state"}` / `{"cmd":"log"}` | 上报状态 / 打开日志转发 |

PC 端接收脚本在 [`tools/shot_recv_mqtt.py`](tools/shot_recv_mqtt.py) 和
[`tools/shot_recv_serial.py`](tools/shot_recv_serial.py)。

## OTA

固件从 HA 的 `/local/switchyard/<device_id>/firmware.json` 拉清单和 bin（由 switchyard 上传固件时生成）。
设备按字符串比较版本号，不同即升级；双分区 app0/app1 可回滚。

## 已知限制

| 项 | 状态 |
|---|---|
| 电池电量 | **未标定**（S31 只有一档 ADC 衰减、无校准方案），界面显示 `未知` + 原始码值；标定后改一个宏即可显示百分比 |
| 蓝牙音箱 | 编译排除（省 52 KB），代码完整保留 |
| 小智语音 | 编译排除；音频流启动时 CPU 锁死，待查（已排除内存/栈/锁竞争） |
| 网页配置 | 默认关闭（Kconfig 开关），`http://<设备IP>/` 可改 WiFi/HA/密码/网页 OTA |

## 相关仓库

| 仓库 | 说明 |
|---|---|
| [switchyard](https://github.com/xiao-si-qi/switchyard) | HA 插件：设备台账、布局编辑下发、固件 OTA |
| [orbis-w518](https://github.com/xiao-si-qi/orbis-w518) | 同系列另一块屏（JC3636W518 / ESP32-S3）|

## License

[MIT](LICENSE) © 2026 xiao-si-qi
