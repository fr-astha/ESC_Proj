# STM32 Custom-Scheduler ESC Simulation

A bare-metal **custom preemptive scheduler** for the STM32F103 (Blue Pill), running three sensor/telemetry tasks, with a **virtual skid-correction controller** and a **Python hardware-in-the-loop simulator** that draws a virtual car driven by the real sensors.

No motors are connected. The "ESC" part is simulated: the firmware computes left/right wheel commands and a speed ramp exactly as it would for real motors, and the PC dashboard shows the result.

## Highlights

- Hand-written **context switch** (`PendSV_Handler`, naked assembly), fake initial stack frames, 1 ms SysTick, priority-based scheduling, `Task_Delay` and drift-free `Task_DelayUntil`. No RTOS.
- **2 ms IMU loop with ~1993–2000 µs period** (measured with the DWT cycle counter), zero deadline misses under load.
- **Robust I2C**: bus recovery (9 SCL pulses + STOP), retry with back-off, a config watchdog that re-programs the MPU6050 if it resets, and an `imu_valid` freshness flag.
- **Skid detection** with hysteresis (on above 30 °/s for 5 samples, off below 20 °/s for 25 samples).
- **Virtual correction controller**: P-control on yaw error, throttle cut while skidding, stop on obstacle or sensor fault.
- **Bidirectional UART**: telemetry out at 10 Hz; speed/acceleration commands in (`S<speed>,<accel>`), with the speed ramp running on the MCU.
- **Python simulator** (side view, top-down view, live graphs, sliders) driven by the board's real data.

## Hardware

| Part | Notes |
|---|---|
| STM32F103C8T6 "Blue Pill" | 72 MHz, Cortex-M3 |
| MPU6050 (clone, `WHO_AM_I = 0x70`) | I2C, AD0 to GND |
| HC-SR04 ultrasonic sensor | trigger/echo, echo measured with EXTI |
| USB-serial adapter | 115200 baud, to PC |
| ST-Link | programming/debug |

### Pin map

| Pin | Function |
|---|---|
| PA9 / PA10 | USART1 TX / RX |
| PB0 | HC-SR04 TRIG |
| PB1 | HC-SR04 ECHO (EXTI1, both edges) |
| PB6 / PB7 | I2C1 SCL / SDA (400 kHz, Fast Mode) |
| PC13 | Status LED |
| PA13 / PA14 | SWD |

PA6/PA7 (TIM3 PWM), PB12–PB15 and PA4 were configured for a TB6612FNG motor driver but are unused in this version.

## Software architecture

```
SysTick (1 ms) ──► tick counter, requests PendSV
PendSV_Handler ──► saves/restores context, switches task (naked asm)

Task_IMU         priority 0   every 2 ms    read MPU6050, skid detection, control law, speed ramp
Task_Ultrasonic  priority 1   every 20 ms   trigger HC-SR04, compute distance
Task_Telemetry   priority 2   every 100 ms  print one status line over UART
Task_Idle        priority 3                 runs when nothing else is ready
```

Notes:
- The HAL timebase is TIM4 (SysTick belongs to the scheduler).
- Timing is measured with `DWT->CYCCNT` (72 MHz).
- `Task_DelayUntil` always yields at least one tick when it is late. Otherwise a late priority-0 task starves everything below it.

### Control logic (runs every 2 ms)

| State (`st`) | Meaning | Wheel commands / speed |
|---|---|---|
| 0 | running | base speed 60 %, P-correction on yaw error (deadband 5 °/s, limit ±40 %) |
| 1 | skid | throttle cut to 50 % |
| 2 | obstacle (< 20 cm) | stop |
| 3 | sensor fault (IMU data stale > 20 ms) | stop |

The speed ramp uses the commanded target (`tg`, 0–100 %, where 100 % = 100 cm/s) and acceleration (`ac`, 20–300 cm/s²). Braking uses a fixed 400 cm/s².

## UART protocol (115200 8N1)

**Telemetry (board → PC), every 100 ms:**

```
t=405100 dist=999 imu=202551 jit[1993..2000] rd=403us gz=-399 ay=19 az=955 err=0 miss=0 skid=1 ev=1 rst=0 v=1 L=0 R=69 c=39 st=1 tg=80 ac=200 rx=3 sp=40
```

| Field | Meaning |
|---|---|
| `t` | uptime, ms |
| `dist` | ultrasonic distance, cm (999 = no echo) |
| `imu` | number of IMU cycles (about t/2) |
| `jit[min..max]` | IMU period min/max, µs |
| `rd` | worst I2C read time, µs |
| `gz` | yaw rate, tenths of °/s |
| `ay`, `az` | acceleration, mg |
| `err` | I2C error count |
| `miss` | deadline misses |
| `skid`, `ev` | skid flag, number of skid events |
| `rst` | MPU6050 resets detected and re-initialised |
| `v` | IMU data valid |
| `L`, `R`, `c` | left/right wheel command %, steering correction % |
| `st` | controller state (see above) |
| `tg`, `ac`, `rx` | target %, accel, number of valid commands received |
| `sp` | board's speed, cm/s |

**Command (PC → board):** `S<speed>,<accel>\n`, for example `S80,200`. Out-of-range or malformed lines are ignored.

## Build and flash

1. Open the project in STM32CubeIDE (the `.ioc` is included).
2. Build, then flash with ST-Link.
3. Open a serial terminal (PuTTY, 115200) to see the telemetry. Close it before running the dashboard, since only one program can use the port.

CubeMX settings that matter: I2C1 in **Fast Mode** (400 kHz), HAL timebase **TIM4**, USART1 **global interrupt enabled**.

## Simulator (`dashboard.py`)

```
pip install pyserial matplotlib numpy
python dashboard.py                  # real data, default port COM5
python dashboard.py --port COM7
python dashboard.py --demo           # fake data, no board needed
python dashboard.py --log run1.csv   # also save every sample to CSV
```

- **Side view**: virtual car with pitch, spinning wheels, scrolling road, skid marks, and an obstacle placed at the real ultrasonic distance.
- **Top-down view**: heading integrated from the real gyro, left/right wheel-command bars, and the obstacle beam.
- **Graphs**: distance, yaw rate, wheel commands, speed vs. allowed speed, IMU jitter. Red bands mark skids and orange bands mark invalid IMU data.
- **Sliders**: target speed and acceleration are sent to the board, which runs the ramp and the safety logic. The car only follows the speed the board reports.
- **Keys**: `r` resets the heading, `c` clears a collision.

The sensors are real; the car physics are deliberately simple (a speed ramp plus a braking distance).

## Test results

| Test | Result |
|---|---|
| IMU period under full load | 1993–2000 µs, `miss=0`, `err=0` |
| Rotate board both directions | wheel correction flips side correctly (gz < 0 → R > L; gz > 0 → L > R) |
| Fast rotation | skid detected, `ev` increments once per skid, throttle cut (`st=1`) |
| Loose/pulled SDA wire | bus recovery, `v=0` while data is stale, recovers without reset |
| Obstacle < 20 cm | `L=0 R=0 st=2`, speed drops to 0 |
| Slider commands | `tg`/`ac`/`rx` update; ramp follows the set acceleration |

## Known limitations

- The HAL I2C transfer is blocking. During a stuck bus, one read was observed to block for about 42 ms (the priority-0 task stalls). The proper fix is DMA/interrupt-driven I2C or shorter timeouts.
- There is no command timeout yet: if the PC program closes, the board keeps its last target speed.
- Telemetry fields are not snapshotted atomically, so a printed line can occasionally mix values from two IMU cycles.
- No real motors: wheel commands are computed but not output to a driver.

## Roadmap

- Command timeout (stop if no command for 2 s)
- Sudden-hurdle escalation in the ultrasonic task
- Baseline (super-loop) vs. scheduler jitter and latency comparison logged to CSV
- Optional: TB6612FNG motor driver output on TIM3 PWM


