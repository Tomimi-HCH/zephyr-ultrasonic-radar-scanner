# Architecture

Software design of the radar firmware: task topology, synchronization, and the memory tricks that make it fit in 32 KB SRAM.

## Task topology

```
 GPIO ISR (SW3)                GPIO ISR (Echo dual-edge)
      | k_work_reschedule             | k_sem_give(sem_echo_done)
      v                               v
 debounce (delayed work)      +------------------------+
      | scan_paused           |  sonar_thread (prio 5) |
      v                       |  trigger / capture / mm|
+------------------------+    +-----------+------------+
|  servo_thread (prio 5) |                | k_msgq_put
|  0°~180° ping-pong,    | -- sem_servo_ready
|  1° steps              |
+-----------+------------+
            | g_current_angle / g_current_frame_id
            v
+--------------------------------------------------------+
| main thread (consumes radar_msgq)                      |
|  scan-line / target rendering -> end of frame ->       |
|  object clustering -> UART report                      |
|  angle/distance every 1 s, temperature every 5 s       |
+--------------------------------------------------------+
```

- `servo_thread` moves the PWM servo one degree at a time and gives `sem_servo_ready`; `sonar_thread` waits for it, fires the Trig pulse, then blocks on `sem_echo_done` which the echo GPIO ISR gives on the captured edge.
- `radar_msgq` between sonar and main: when full, the **oldest** sample is dropped so the newest always gets in (`put_latest_sample()`).
- Before each measurement the echo semaphore is drained and an `echo_armed` flag masks edges outside the measurement window, so a late echo from a timed-out shot cannot pollute the next one.
- SW3's ISR only schedules a 50 ms delayed work item; the state toggle and logging run in thread context.
- Every sample carries a `frame_id`; on frame change the scan buffer is reset to `-1` (invalid) so dropped samples are not mistaken for real echoes by the clustering algorithm.

## Fitting 32 KB SRAM

- **No framebuffer / bitmap** for the static radar frame: `static_pixel_is_set()` decides analytically (squared ring distance + ray cross-product/projection with a Q15 lookup table) whether a pixel belongs to the static frame, so erasing the scan line can redraw those pixels one by one. Saves ~2.5 KB RAM.
- `CONFIG_HEAP_MEM_POOL_SIZE=2048` — the app never calls `k_malloc`; the Zephyr default is pure waste here.
- Object detection and OSD run **once per frame** (at 180°), not per sample.
- Scan-line erase redraws only the pixels it covered, never the whole static frame.

## Kconfig essentials (`prj.conf`)

```text
CONFIG_GPIO=y / CONFIG_PWM=y / CONFIG_SPI=y / CONFIG_DISPLAY=y / CONFIG_ST7735R=y
CONFIG_SENSOR=y / CONFIG_P3T1755=y / CONFIG_I3C=y   # on-board temperature sensor
CONFIG_FPU=y / CONFIG_CBPRINTF_FP_SUPPORT=y         # cosine-law sizing & float printf
CONFIG_HEAP_MEM_POOL_SIZE=2048
CONFIG_MAIN_STACK_SIZE=1536
```

## Expected serial log

1. Boot banner: `Display device ready` / `Servo PWM ready` / `Sonar/echo/button GPIO ready` / `Display blanking off, GUI ready` / `Screen cleared, static radar frame drawn, entering main loop` — any failure prints `<err>` naming the step.
2. Every second: `angle= xx deg  dist= xxx mm` (`dist= -- (no echo)` when out of range).
3. End of each 180° frame: `object #1: 30-60 deg  L=12.3cm  D=45cm`, or `frame N: no object`.
4. SW2 press: `SW2: range switched to ...`; temperature every 5 s: `Ambient temperature: xx.x C`.

---

## 中文要点

- 三线程模型：`servo_thread`（转角度）→ `sonar_thread`（触发+测距）→ `main`（渲染+聚类+串口），经信号量与 `radar_msgq` 同步；队列满时丢最旧样本。
- 32 KB SRAM 优化：无帧缓冲（解析几何+Q15 查表恢复底图）、heap 只留 2048、检测每帧只做一次、擦除只重绘被覆盖像素。
- 上电自检日志、每秒角度距离、帧末目标报告均见上方英文说明。
