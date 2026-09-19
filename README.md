# Zephyr Ultrasonic Radar Scanner

An ultrasonic radar system on the **NXP FRDM-MCXA153** board, built with **Zephyr RTOS v4.4.0**. An MG90S/SG90 servo sweeps an HC-SR04 ultrasonic sensor across 180°, a 1.8" ST7735 TFT renders a live polar radar view, and an on-board algorithm clusters echoes into object width/distance estimates — all on a Cortex-M33 MCU with only **32 KB SRAM**.

![Demo photo](docs/images/demo_overview.jpg)
<!-- TODO: add hardware photos to docs/images/ (assembled unit, wiring, serial log) -->

## Features

- 180° servo scan with 1° steps, 50 Hz PWM (500–2500 µs)
- HC-SR04 echo capture via dual-edge GPIO interrupt, distance in mm
- ST7735 160×128 radar GUI: static frame, live scan line, persistent target blips, red occlusion bars, on-screen range switch
- Object detection: per-frame clustering of angular samples, object width via the law of cosines
- Range button (SW2): cycles 30 / 60 / 100 / 200 cm at runtime
- Pause/resume button (SW3) with 50 ms debounce
- On-board P3T1755DP temperature sensor (I3C) shown on UART every 5 s
- UART telemetry: `angle= xx deg  dist= xxx mm` every second, plus per-frame object reports
- No framebuffer: the static radar frame is restored analytically (Q15 lookup) to fit 32 KB SRAM

## Hardware

| Module | Connection |
| :----- | :--------- |
| HC-SR04 Trig | P3_28 (GPIO out, 10 µs pulse) |
| HC-SR04 Echo | P3_27 (GPIO in, pull-down, dual-edge IRQ) |
| Servo PWM | P3_9 (FlexPWM0 SM1 B1, 50 Hz) |
| ST7735 SPI (SCK/MOSI/MISO) | P1_1 / P1_0 / P1_2 (LPSPI0) |
| **Shared chip-select** | **P1_3** — low = ST7735, high = GT20L font ROM (via on-module inverter) |
| LCD DC / backlight | P3_30 / P3_1 |
| Range button (SW2) | P3_29 |
| Pause button (SW3) | P1_7 |
| Temperature | on-board P3T1755DP, I3C address 0x48 |

> See [docs/wiring.md](docs/wiring.md) for the full table and [docs/architecture.md](docs/architecture.md) for the software design.

### ⚠️ Hardware cautions

- The HC-SR04 Echo pin outputs **5 V** — use a divider / level shifter, or a 3.3 V-compatible HC-SR04P, before connecting to P3_27.
- Power the servo from an **external 5 V supply with common ground**; servo inrush current can reset the MCU otherwise.

## Quick start

Requirements: Python 3.10+, West 1.2+, CMake 3.20.5+, Ninja, Zephyr SDK 0.16+ (or Arm GNU toolchain).

```bash
# 1. Create a workspace and fetch this manifest (pulls Zephyr + hal_nxp into external/)
west init -m https://github.com/Tomimi-HCH/zephyr-ultrasonic-radar-scanner.git radar-workspace
cd radar-workspace/zephyr-ultrasonic-radar-scanner

# 2. Set up Python environment
python -m venv .venv
source .venv/Scripts/activate        # PowerShell: .venv\Scripts\Activate.ps1
pip install -U west
west update
west zephyr-export
pip install -r external/zephyr/scripts/requirements.txt

# 3. Build and flash (NXP LinkServer; pyOCD cannot drive the on-board MCU-Link on Windows)
west build -b frdm_mcxa153 -p auto
west flash -r linkserver
```

Serial log: `115200 8-N-1`.

If the screen shows garbage: the overlay already carries `inversion-on;` and limits SPI to 8 MHz — if it persists, shorten the SPI fly wires or add ~33 Ω series resistors.

## Repo layout

```
├── west.yml                  # West manifest: Zephyr v4.4.0 + hal_nxp, cmsis_6, picolibc, segger
├── CMakeLists.txt / prj.conf
├── boards/frdm_mcxa153.overlay
├── src/
│   ├── main.c                # device init, servo/sonar threads, message queue, coordination
│   ├── gui_radar.c           # polar radar rendering (no framebuffer)
│   ├── algo_object_detect.c  # angular clustering + object size estimation
│   └── font_chip.c           # GT20L font ROM driver (kept in tree, NOT compiled)
└── docs/                     # architecture.md, wiring.md, images/
```

> The font chip driver is intentionally excluded from the build since 2026-09. To re-enable: add `src/font_chip.c` to `target_sources` in `CMakeLists.txt` and restore the font calls in `main.c`.

## License

Apache-2.0, see [LICENSE](LICENSE).

---

# Zephyr 超声波雷达扫描系统（中文）

基于 **NXP FRDM-MCXA153** 开发板与 **Zephyr RTOS v4.4.0** 的超声波雷达系统：MG90S/SG90 舵机携带 HC-SR04 在 180° 范围内扫描，1.8 寸 ST7735 屏实时绘制极坐标雷达画面，片上算法对回波聚类并估算目标宽度——全程运行在仅 **32 KB SRAM** 的 Cortex-M33 上。

## 功能特性

- 舵机 180° 往返扫描，1° 步进，50 Hz PWM（500–2500 µs）
- HC-SR04 双边沿中断捕获回波，输出毫米级距离
- ST7735 雷达 GUI：静态底图、实时扫描线、目标点保持、红色遮挡线、量程切换
- 目标检测：按帧聚类角度样本，余弦定理估算目标宽度
- SW2 循环切换量程 30/60/100/200 cm；SW3 暂停/继续（50 ms 消抖）
- 板载 P3T1755DP 温度传感器，每 5 秒串口输出
- 每秒串口输出 `angle= xx deg  dist= xxx mm`，帧末输出目标报告
- 无帧缓冲：静态底图用解析几何 + Q15 查表恢复，适配 32 KB SRAM

## 快速上手

```bash
west init -m https://github.com/Tomimi-HCH/zephyr-ultrasonic-radar-scanner.git radar-workspace
cd radar-workspace/zephyr-ultrasonic-radar-scanner
python -m venv .venv && source .venv/Scripts/activate   # PowerShell: .venv\Scripts\Activate.ps1
pip install -U west
west update && west zephyr-export
pip install -r external/zephyr/scripts/requirements.txt
west build -b frdm_mcxa153 -p auto
west flash -r linkserver      # Windows 推荐 LinkServer；pyOCD 无法驱动板载 MCU-Link
```

串口参数：`115200 8-N-1`。

## 硬件注意事项

- HC-SR04 的 Echo 是 **5 V 电平**，必须分压/电平转换后接 P3_27，或改用 3.3 V 兼容的 HC-SR04P。
- 舵机需**外部 5 V 独立供电并与板卡共地**，否则启动电流可能使 MCU 复位。
- 若花屏：overlay 已带 `inversion-on;` 且 SPI 限 8 MHz；仍花屏则缩短 SPI 飞线或串 ~33 Ω 电阻。

详细接线表见 [docs/wiring.md](docs/wiring.md)，软件架构见 [docs/architecture.md](docs/architecture.md)。

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
