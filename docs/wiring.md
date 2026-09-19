# Wiring & Pinout

All signals come from `boards/frdm_mcxa153.overlay`; this table is the authoritative map.

| Function | Pin | Notes |
| :------- | :-- | :---- |
| HC-SR04 Trig | P3_28 | GPIO output, 10 µs trigger pulse |
| HC-SR04 Echo | P3_27 | GPIO input, pull-down, dual-edge interrupt — **5 V signal, level-shift before connecting** |
| Servo PWM | P3_9 | FlexPWM0 SM1 B1, 50 Hz, 500–2500 µs (MG90S/SG90) |
| LCD / font ROM SCK / MOSI / MISO | P1_1 / P1_0 / P1_2 | LPSPI0 |
| **Shared chip-select** | **P1_3** | **physical low = ST7735, physical high = GT20L font ROM** |
| LCD DC | P3_30 | data / command select |
| LCD backlight | P3_1 | active high |
| Range button (SW2) | P3_29 | alias `range-btn`; cycles 30/60/100/200 cm at runtime |
| Pause button (SW3) | P1_7 | alias `sw0` remapped to `user_button_3` |
| Temperature sensor | on-board P3T1755DP | I3C, Zephyr native `nxp,p3t1755` driver, 7-bit address 0x48 |

## Shared chip-select convention (important)

The LCD module shares one physical chip-select line (P1_3) between the ST7735 and the GT20L16S1Y font ROM, split by an on-module inverter:

- **P1_3 = low** → ST7735 selected. In the overlay `cs-gpios = <&gpio1 3 GPIO_ACTIVE_LOW>`, driven automatically by the display driver.
- **P1_3 = high** → font ROM selected. `font_chip.c` declares its alias `GPIO_ACTIVE_HIGH` and drives the line explicitly around each read.

`font_chip.c` therefore does **not** use automatic CS (its `spi_config.cs` stays zero-initialized); `get_n_bytes_from_rom()` wraps the whole `0x03 + 24-bit address + N bytes` transaction manually so the ROM stays selected for the full transfer and every error path returns the line to the LCD-safe (low) state. Any documentation showing two separate LCD_CS / ZK_CS lines is obsolete.

## Electrical cautions

- **HC-SR04 Echo is 5 V.** Use a resistor divider / level shifter before P3_27, or use a 3.3 V-compatible HC-SR04P. Do not treat this as a software problem.
- **Servo power:** feed the MG90S/SG90 from a proper external 5 V supply with its ground tied to the board. Servo inrush/stall current can brown-out and reset the MCU.
- Keep SPI fly wires short; if the display glitches under servo load, add ~33 Ω series resistors on SCK/MOSI and separate the servo supply wiring from the SPI bundle.

## Display bring-up notes (2026-09 fixes)

- `inversion-on;` is **required** on this 1.8" ST7735S panel — without it the whole screen shows garbage.
- `mipi-max-frequency` is capped at **8 MHz** (15 MHz rings on fly wires and garbles the image); drop to 4/2 MHz when debugging.
- Panel geometry: 160×128, `madctl 0x60`, `colmod 0x55`, full power-control and gamma tables in the overlay.

## Servo direction

If the physical sweep runs opposite to the on-screen scan direction, flip `SERVO_ANGLE_REVERSED` in `src/main.c` (0/1).

---

## 中文要点

- 权威引脚定义在 `boards/frdm_mcxa153.overlay`，上表与其一致。
- **共享片选**：P1_3 物理低选 LCD、物理高选字库（板上反相器）；`font_chip.c` 手动包围 SPI 事务，异常路径也会恢复到 LCD 安全态。旧文档的两根独立片选说法已过时。
- **电平与电源**：HC-SR04 Echo 为 5 V，必须分压或用 HC-SR04P；舵机独立 5 V 供电并共地。
- **花屏修复**：本屏必须 `inversion-on;`，SPI ≤ 8 MHz（15 MHz 飞线振铃）；舵机负载下花屏可串 33 Ω 电阻、分开走线。
