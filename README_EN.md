# Zephyr Ultrasonic Radar Scanner

**English | [中文](README.md)**

An ultrasonic radar system on the **NXP FRDM-MCXA153** board, built with **Zephyr RTOS v4.4.0**. An MG90S/SG90 servo sweeps an HC-SR04 ultrasonic sensor across 180°, a 1.8" ST7735 TFT renders a live polar radar view, and an on-board algorithm clusters echoes into object width/distance estimates — all on a Cortex-M33 MCU with only **32 KB SRAM**, without a framebuffer.

![Demo photo](docs/images/demo_overview.jpg)

![Radar screen close-up](docs/images/demo_closeup.jpg)

## Features

- 180° servo scan with 1° steps, 18 ms/step, 50 Hz PWM (500–2500 µs)
- HC-SR04 echo capture via dual-edge GPIO interrupt, distance in mm (20 mm – 4 m)
- ST7735 160×128 radar GUI: static frame, live scan line, persistent target blips, red occlusion bars
- Object detection: per-frame clustering of angular samples, object width via the law of cosines (with 8° beam-angle compensation)
- Range button (SW2): cycles 30 / 60 / 100 / 200 cm at runtime
- Pause/resume button (SW3) with 50 ms debounce
- On-board P3T1755DP temperature sensor (I3C) reported over UART every 5 s
- UART telemetry: `angle= xx deg  dist= xxx mm` every second, plus per-frame object reports
- No framebuffer: the static radar frame is restored analytically (Q15 lookup) to fit 32 KB SRAM; total RAM usage ≈ 16 KB

## Hardware

| Module | Connection |
| :----- | :--------- |
| HC-SR04 Trig | P3_28 (GPIO out, 12 µs pulse) |
| HC-SR04 Echo | P3_27 (GPIO in, pull-down, dual-edge IRQ) |
| Servo PWM | P3_9 (FlexPWM0 SM1 B1, 50 Hz) |
| ST7735 SPI (SCK/MOSI/MISO) | P1_1 / P1_0 / P1_2 (LPSPI0) |
| **Shared chip-select** | **P1_3** — low = ST7735, high = GT20L font ROM (via on-module inverter) |
| LCD DC / backlight | P3_30 / P3_1 |
| Range button (SW2) | P3_29 (on-board) |
| Pause button (SW3) | P1_7 (on-board) |
| Temperature | on-board P3T1755DP, I3C address 0x48 |

> See [docs/wiring.md](docs/wiring.md) for the full wiring table and photos, and [docs/architecture.md](docs/architecture.md) for the software design.

### ⚠️ Hardware cautions

- The HC-SR04 Echo pin outputs **5 V** — use a divider / level shifter, or a 3.3 V-compatible HC-SR04P, before connecting to P3_27.
- Power the servo from an **external 5 V supply with common ground**; servo inrush current can reset the MCU otherwise.

## Quick start

Requirements: Python 3.10+, CMake 3.20.5+, Ninja, Zephyr SDK 0.16+ (or Arm GNU toolchain).

```bash
# 1. Prepare a Python environment and install west (Zephyr's build/meta tool)
python -m venv .venv
source .venv/Scripts/activate        # PowerShell: .venv\Scripts\Activate.ps1
pip install -U west

# 2. Create a workspace from this manifest (pulls Zephyr v4.4.0 + hal_nxp into external/)
west init -m https://github.com/Tomimi-HCH/zephyr-ultrasonic-radar-scanner.git radar-workspace
cd radar-workspace/zephyr-ultrasonic-radar-scanner
west update
west zephyr-export
pip install -r external/zephyr/scripts/requirements.txt

# 3. Build and flash (NXP LinkServer; pyOCD cannot drive the on-board MCU-Link on Windows)
west build -b frdm_mcxa153 -p auto
west flash -r linkserver
```

Serial log: `115200 8-N-1`.

> If the workspace is already initialized, do not repeat `west init`; run `west update` when dependencies change.

## What you should see

**On screen:** a black half-disc radar view — dark-green grid (3 range rings + 7 spokes), a bright-green scan line following the servo; when an object is detected at an angle, a red occlusion bar is drawn from the object distance to the range edge and refreshed on the next sweep.

**On UART:**

```text
[00:00:12.345] <inf> app: angle= 45 deg  dist= 512 mm
[00:00:12.381] <inf> app: angle= 46 deg  dist=  -- (no echo)
[00:00:15.000] <inf> app: Ambient temperature: 26.3 C
[00:00:20.107] <inf> app: object #0: 40-52 deg  L=18.4cm  D=51cm
```

If the screen shows garbage: the overlay already carries `inversion-on;` and limits SPI to 8 MHz — if it persists, shorten the SPI fly wires or add ~33 Ω series resistors.

## Repo layout

```
├── west.yml                  # West manifest: Zephyr v4.4.0 + hal_nxp, cmsis_6, picolibc, segger
├── CMakeLists.txt / prj.conf
├── boards/frdm_mcxa153.overlay  # all pin/peripheral configuration lives here
├── src/
│   ├── main.c                # device init, servo/sonar threads, message queue, coordination
│   ├── gui_radar.c           # polar radar rendering (no framebuffer)
│   ├── algo_object_detect.c  # angular clustering + object size estimation
│   └── font_chip.c           # GT20L font ROM driver (kept in tree, NOT compiled)
└── docs/                     # architecture.md, wiring.md, images/
```

> The font chip driver is intentionally excluded from the build since 2026-09 (suspended during shared-chip-select interference debugging). To re-enable: add `src/font_chip.c` to `target_sources` in `CMakeLists.txt` and restore the font calls in `main.c`.

## License

Apache-2.0, see [LICENSE](LICENSE).

## Acknowledgments

- Thanks to [DigiKey](https://www.digikey.com/) and NXP for the FRDM-MCXA153 board;
- The radar target board used in testing comes from an open-source design by [aknice](https://oshwhub.com/aknice) ([project page](https://oshwhub.com/aknice/project_wjjeuibp));
- Thanks to the Zephyr community — devicetree and the driver model meant zero display/sensor driver code in this project.
