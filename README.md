# M3508_Lib

Firmware for a 4-wheel **mecanum chassis**: a NUCLEO-G474RE receives velocity commands
(`vx`, `vy`, `wz`) from a Jetson over UART and drives four RoboMaster **M3508 motors with C620
speed controllers** over CAN.

> Status: builds cleanly and behaves correctly in a PC simulation (command frames → chassis → four
> simulated motors). It has **not run on the real robot yet**. PID gains, wheel directions and
> chassis dimensions are starting values that must be checked on hardware (section 5).

---

## 1. Architecture

```
 Jetson                      STM32 (FreeRTOS)                                              C620 x4
 ------      UART 921600     -----------------------------------------------------         -------
 joystick ─► vx, vy, wz ───► inputTask ──► processTask (1 kHz) ──► outputTask ── CAN 0x200 ──► motors
                             parse frame   ramp robot velocity     pack 4 currents              │
                                           mecanum kinematics      into ONE frame               │
                                           4 speed PIDs                                         │
                                               ▲                                                │
                                               └────── rpm, angle, current ◄── CAN 0x201..0x204 ┘
```

| Task | File | Wakes on | Priority | Job |
|---|---|---|---|---|
| `inputTask` | `input_task.c` | UART idle-line interrupt (a frame just ended) | High | Drain the DMA buffer, parse frames, store the latest command |
| `processTask` | `process_task.c` | Fixed 1 ms tick | Realtime | Command → robot ramp → wheel speeds → 4 speed PIDs → 4 currents |
| `outputTask` | `output_task.c` | Trigger from `processTask` | Realtime1 (highest) | Send the 4 currents in one CAN frame |
| `defaultTask` | `app_freertos.c` | 100 / 500 ms delay | Normal | Status LED only |

Data flows one way: input → process → output. Tasks hand over through small functions
(`cmd_link_get()`, `outputTask_trigger()`), never through shared variables.

### One control cycle (every 1 ms, in `processTask`)

1. `cmd_link_get()` – latest command; zero if the link has timed out.
2. `chassis_update()` – limit and ramp the robot velocity, convert to four wheel rpm setpoints.
3. `m3508_update_all()` – decode the newest CAN feedback, run the four speed PIDs → four currents.
4. `outputTask_trigger()` – `outputTask` runs immediately (it has higher priority) and queues the
   CAN frame.

### What keeps the four wheels synchronised

- **The ramp is on the robot, not on each wheel.** `vx`/`vy` are ramped as one vector and `wz`
  separately; wheel speeds are computed from the result. All four wheels therefore speed up and
  slow down in proportion and the robot keeps its direction while accelerating.
- **Shared saturation.** If any wheel would exceed `CHASSIS_MAX_WHEEL_RPM`, all four are scaled by
  the same factor.
- **Same cycle, same frame.** The four PIDs run in the same millisecond and their currents leave
  in one CAN frame, so the C620s get them at the same instant.
- **All or nothing.** With `CHASSIS_REQUIRE_ALL_MOTORS = 1`, the chassis coasts if any motor stops
  answering.

There is no cross-coupling between wheels: if one wheel is loaded harder than the others it can
lag slightly during hard acceleration. In simulation with wheels of 15 % different load the spread
was a few percent during a straight-line ramp and under 1 rpm at steady speed.

### What makes it fast and deterministic

- UART: circular DMA, no per-byte interrupt; `inputTask` wakes only when a frame has ended.
- CAN receive: the interrupt only copies 8 bytes; all maths happens in `processTask`.
- CAN send: non-blocking; a full TX FIFO drops the frame and counts it.
- Fixed 1 ms period (`vTaskDelayUntil`), constant `dt`, no dynamic memory, no `printf`, no waits.
- Worst-case command latency: frame end → next 1 ms tick → CAN frame queued tens of microseconds
  later.

---

## 2. Behaviour

| Situation | Chassis mode | What the robot does | Status LED |
|---|---|---|---|
| No command since power-up | `CHASSIS_IDLE` | Motors coast | Fast blink |
| Commands arriving | `CHASSIS_DRIVE` | Follows them within the limits | Slow blink |
| Commands stop for `CMD_TIMEOUT_MS` (200 ms) | `CHASSIS_FAILSAFE` | Brakes to a stop at the brake rate, then holds still | Fast blink |
| A motor stops answering for 100 ms | `CHASSIS_MOTOR_FAULT` | All motors coast until it returns | Fast blink |

When driving resumes after coasting, the ramp starts from the robot's measured velocity, so there
is no jerk.

**Robot frame** (same as ROS `cmd_vel`): `x` forward, `y` left, `wz` counter-clockwise seen from
above. Units: m/s and rad/s.

---

## 3. Command frame (Jetson → STM32)

18 bytes, little-endian, 921600 baud 8N1:

```
0xA5 0x5A | len = 13 | seq (u8) | vx (float32) | vy (float32) | wz (float32) | crc16 (u16)
```

- `crc16` = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over the `len` byte and the payload.
- `seq` increments by one per frame; the STM32 uses it to count lost frames.
- Send continuously at 50–200 Hz, **including while the command is zero**. Silence for 200 ms is
  treated as a lost link.
- Frames with a bad CRC, a wrong length or a NaN / infinite value are dropped.

**On the Jetson**, `tools/jetson_cmd_sender.py` (needs `pip3 install pyserial`):

```
python3 jetson_cmd_sender.py --port /dev/ttyTHS1                 # zeros: link test, robot holds still
python3 jetson_cmd_sender.py --port /dev/ttyTHS1 --vx 0.2        # constant 0.2 m/s forward
python3 jetson_cmd_sender.py --port /dev/ttyTHS1 --source ros2   # forwards the ROS 2 cmd_vel topic
```

From your own joystick code: `from jetson_cmd_sender import pack_frame`, then
`ser.write(pack_frame(seq, vx, vy, wz))`.

The ROS 2 mode has not been run; the frame packing has been tested against the STM32 parser.

---

## 4. Wiring

| Signal | STM32 pin | Connects to |
|---|---|---|
| CAN RX | PA11 | CAN transceiver RXD |
| CAN TX | PA12 | CAN transceiver TXD |
| UART RX | PA10 | Jetson UART TX |
| UART TX | PA9 | Jetson UART RX (optional – nothing is sent back yet) |
| GND | GND | Jetson GND and transceiver GND |

- CAN bus: 1 Mbit/s, 120 Ω terminator at each end. C620 IDs: 1 = front-left, 2 = front-right,
  3 = rear-left, 4 = rear-right.
- UART: 3.3 V logic on both sides. PA10 has its pull-up enabled in code.
- The clock runs from the board's 24 MHz crystal; CAN timing depends on it.

---

## 5. Build, flash and first run

### Build and flash

Plug the Nucleo into USB (the on-board ST-LINK), then either:

- **VS Code**: `Ctrl+Shift+B` runs the default task **Build + Flash (Release)**. Other tasks
  (Debug build, build only) are under *Terminal → Run Task*.
- **Terminal**, from the project folder:

  ```
  powershell -ExecutionPolicy Bypass -File tools\build_flash.ps1                 # Release, flash
  powershell -ExecutionPolicy Bypass -File tools\build_flash.ps1 -Config Debug   # Debug, flash
  powershell -ExecutionPolicy Bypass -File tools\build_flash.ps1 -NoFlash        # build only
  ```

`tools/build_flash.ps1` uses the compiler, CMake, Ninja and STM32CubeProgrammer that the
*STM32CubeIDE for VS Code* extension installed under `%LOCALAPPDATA%\stm32cube\bundles`, so nothing
needs to be on PATH. Output goes to `build\Release\M3508_Lib.elf` (or `build\Debug\`).

| Problem | Fix |
|---|---|
| `No ST-Link detected` / flashing failed | USB cable must carry data; close any open debug session; try another USB port |
| `Bundle ... not found` | Install or update the *STM32CubeIDE for VS Code* extension |
| Script blocked by execution policy | Use the `powershell -ExecutionPolicy Bypass -File ...` form shown above |

Use **Release** for driving: Debug is compiled without optimisation, so the control cycle takes
longer. Use **Debug** when you need to step through code with breakpoints. Check
`s_proc_stats.exec_us_max` (process_task.c) to see the actual cycle time of either build.

### First run on hardware

Do these in order, wheels off the ground.

1. **Flash and power the C620s.** The LED blinks fast (idle). In the debugger: `s_proc_stats.loops`
   (process_task.c) climbs by 1000 per second and `s_motors[0..3].online` are all `1`.
2. **Link test.** Run the sender with zeros. `s_link_stats.frames_ok` (cmd_link.c) climbs; the LED
   changes to a slow blink; wheels hold still and resist being turned by hand.
3. **Wheel directions.** Send `--vx 0.2`. **Every wheel must roll the robot forward.** For any
   wheel that turns backward, flip its `CHASSIS_DIR_xx` sign in `robot_config.h`.
4. **Wheel positions.** Send `--vy 0.2` (strafe left). Front-left and rear-right must roll
   backward, front-right and rear-left forward. If not, the IDs are on the wrong wheels
   (`CHASSIS_ID_xx`) or the mecanum rollers are mounted in the wrong pattern (seen from above the
   rollers should form an X).
5. **Rotation.** Send `--wz 0.5`. Left wheels roll backward, right wheels forward.
6. **Failsafe.** While driving, stop the sender. The wheels must brake to a stop within about
   half a second.
7. **On the ground**, at low speed, then tune (section 7).

---

## 6. Settings (`Core/Inc/robot_config.h`)

### Chassis

| Setting | Default | Meaning |
|---|---|---|
| `CHASSIS_WHEEL_RADIUS_M` | 0.0762 | Wheel radius. 1 m/s = 125 wheel rpm with this value |
| `CHASSIS_HALF_TRACK_M` / `CHASSIS_HALF_BASE_M` | 0.20 / 0.20 | Half the left-right / front-rear wheel distance. **Measure your robot** |
| `CHASSIS_ID_FL/FR/RL/RR` | 1 / 2 / 3 / 4 | Which C620 ID is on which wheel |
| `CHASSIS_DIR_FL/FR/RL/RR` | +1 / −1 / +1 / −1 | Motor direction that rolls the robot forward |
| `CHASSIS_MAX_LIN_MPS` | 2.0 | Speed limit for `(vx, vy)`; larger commands are shrunk, direction kept |
| `CHASSIS_MAX_ANG_RPS` | 4.0 | Limit for `wz` |
| `CHASSIS_ACCEL_LIN` / `DECEL_LIN` / `BRAKE_LIN` | 3 / 4 / 6 m/s² | Speeding up / slowing on command / stopping after link loss |
| `CHASSIS_ACCEL_ANG` / `DECEL_ANG` / `BRAKE_ANG` | 8 / 10 / 16 rad/s² | Same for rotation |
| `CHASSIS_MAX_WHEEL_RPM` | 450 | Wheel speed ceiling (the M3508 tops out near 480) |
| `CHASSIS_REQUIRE_ALL_MOTORS` | 1 | 0 lets you bench-test with fewer than four motors |

### Link and timing

| Setting | Default | Meaning |
|---|---|---|
| `CMD_TIMEOUT_MS` | 200 | Silence before the failsafe brakes |
| `CTRL_PERIOD_MS` | 1 | Control loop period |
| `M3508_FEEDBACK_TIMEOUT_MS` | 100 | Silence before a motor counts as offline |

### Motors

| Setting | Default | Meaning |
|---|---|---|
| `M3508_SPEED_KP / KI / KD` | 8 / 60 / 0 | Speed loop: rotor rpm error → raw current (`ki`, `kd` per second) |
| `M3508_DEF_CURRENT_A` | 10 | Per-motor torque limit while driving |
| `M3508_CURRENT_RAW_LIMIT` | 10000 (~12.2 A) | Absolute ceiling on every command. Raise both this and the limit above for more torque (max 16384 = 20 A) |

The other `M3508_DEF_*` and `M3508_ANGLE_*` values belong to motor-library features the chassis
does not use (section 9).

---

## 7. Tuning

1. **Speed loop** (wheels off the ground, then on). Set `M3508_SPEED_KI` to 0. Raise `KP` until
   the wheels follow speed changes crisply; back off if they buzz or oscillate. Then raise `KI`
   until the measured speed reaches the setpoint under load without much overshoot.
2. **Acceleration limits.** Watch the motor currents during a hard start or stop. If they sit at
   the current limit, the robot cannot follow the ramp: the wheels fall behind, slip, or overshoot
   at the end. Lower `CHASSIS_ACCEL_LIN` / `DECEL_LIN` / `BRAKE_LIN`, or raise the current limit.
3. **Geometry.** If the robot turns more or less than commanded, correct `CHASSIS_HALF_TRACK_M`
   and `CHASSIS_HALF_BASE_M`. If it covers the wrong distance, correct `CHASSIS_WHEEL_RADIUS_M`.

Compare `s_chassis.sp_vx/vy/wz` with `s_chassis.meas_vx/vy/wz` (chassis.c): they should track closely.

---

## 8. Debugging

### What to watch in the debugger

| Expression (file) | Healthy | Tells you |
|---|---|---|
| `s_proc_stats.loops` (process_task.c) | +1000 per second | Control loop is running |
| `s_proc_stats.overruns` | 0 | Each cycle finishes inside 1 ms |
| `s_proc_stats.exec_us_max` | well under 1000 | Worst cycle time, microseconds |
| `s_proc_stats.can_ok` | 1 | CAN started |
| `s_frames` (output_task.c) | +1000 per second | Command frames are being queued |
| `s_rx_count` (bsp_can.c) | climbing fast | Motor feedback is arriving |
| `s_tx_fail` (bsp_can.c) | 0 | Commands are leaving the board |
| `s_bus_off`, `s_rx_lost` (bsp_can.c) | 0 | No bus failures, no dropped feedback |
| `s_motors[i].online` (m3508.c) | 1 | Motor i+1 feedback is fresh |
| `s_motors[i].fb` | changing | Decoded encoder, rotor rpm, current, temperature |
| `s_motors[i].sp_rpm` vs `fb.rpm` | close | Speed loop tracking (rotor rpm = wheel rpm × 19.2) |
| `s_motors[i].current_cmd` | below the limit | Raw current sent (819 ≈ 1 A) |
| `s_link_stats.bytes` (cmd_link.c) | climbing | Bytes arriving from the Jetson |
| `s_link_stats.frames_ok` | climbing at the send rate | Valid commands |
| `s_link_stats.crc_errors`, `len_errors`, `lost_frames` | 0 or rare | Link quality |
| `s_errors`, `s_restarts` (bsp_uart.c) | 0 | UART line errors |
| `s_chassis.mode` (chassis.c) | `CHASSIS_DRIVE` | Why the robot is or is not driving |
| `s_chassis.wheel_sp_rpm[]` vs `wheel_rpm[]` | close | Per-wheel tracking, order FL FR RL RR |

From code, use `processTask_get_stats()`, `cmd_link_get_stats()`, `chassis_get_state()` and
`m3508_get_state()`.

### Symptoms and causes

| Symptom | Likely cause | Check |
|---|---|---|
| LED not blinking at all | Firmware not running | Stuck in `Error_Handler`, `HardFault_Handler` or `vApplicationStackOverflowHook` (the hook's `pcTaskName` names the task) |
| LED fast, `s_chassis.mode` = `CHASSIS_IDLE` | No valid command has ever arrived | `s_link_stats.bytes` in cmd_link.c. 0 → wiring (Jetson TX → PA10, common ground), port name, sender running. Climbing but `frames_ok` 0 → baud rate or frame format mismatch |
| Mode = `CHASSIS_FAILSAFE` | Commands stopped or arrive slower than every 200 ms | Sender rate; `crc_errors` climbing means a noisy line |
| Mode = `CHASSIS_MOTOR_FAULT` | At least one motor is offline | Which `s_motors[i].online` is 0; then that C620's power, CAN wiring and ID |
| All motors offline, `s_rx_count` 0, `s_tx_fail` climbing | Nobody acknowledges CAN frames | C620 power (24 V), CAN H / L swapped, transceiver power and wiring to PA11 / PA12, 120 Ω termination |
| `s_bus_off` climbing | Electrical problem on the bus | Termination, loose connector, clock source changed away from the 24 MHz crystal |
| `s_rx_lost` climbing | Feedback frames dropped | Something is blocking interrupts too long; look for long critical sections or a higher-priority interrupt you added |
| A wheel spins the wrong way going forward | Direction sign | Flip that wheel's `CHASSIS_DIR_xx` |
| Forward works, strafing rotates or goes diagonal | IDs on the wrong wheels, or roller pattern wrong | `CHASSIS_ID_xx`; rollers should form an X seen from above |
| Robot curves when accelerating | Wheels slipping or currents saturating | Lower the accel limits; check `current_cmd` against the limit |
| Wheels buzz or oscillate | Speed `KP` too high | Lower `M3508_SPEED_KP` |
| Sluggish, sags under load | `KI` too low or current limit too low | Raise `KI`; check whether `current_cmd` is pinned at the limit |
| Overshoots when stopping | Decel / brake rate faster than the motors can brake | Lower `CHASSIS_DECEL_LIN` / `BRAKE_LIN`, or raise the current limit |
| C620 cuts out while braking | Braking energy raises the supply voltage | Use a battery instead of a bench supply, or lower the brake rates |
| `s_proc_stats.overruns` climbing | Control cycle too slow | Use the Release build; remove anything slow added to `processTask` |
| Jerk when the link returns | Should not happen: the ramp restarts from measured velocity | `s_chassis.sp_*` vs `meas_*` at that moment |

### Rules that keep it deterministic

- Only `processTask` calls `chassis_update()` and `m3508_update_all()`; only `outputTask` calls
  `m3508_send_group()`.
- Keep any task you add at a lower priority than `inputTask`.
- No `HAL_Delay`, `printf` or other slow calls in the three tasks or in interrupt callbacks.
- Call the `m3508_*`, `chassis_get_state()` and `cmd_link_get()` functions from tasks, never from
  interrupts.
- CAN, UART and DMA interrupt priorities must stay at 5 or numerically higher (CubeMX → NVIC).

---

## 9. Motor library extras

`m3508.c` can do more than the chassis uses. These work on any motor not driven by the chassis
(the chassis overwrites the command of its four wheels every millisecond):

| Call | What it does |
|---|---|
| `m3508_set_speed(m, rpm)` | Speed loop, with an optional per-motor ramp (`m3508_set_accel`) |
| `m3508_set_angle(m, deg)` / `m3508_move_angle(m, delta)` | Position cascade with a trapezoidal speed profile |
| `m3508_set_current(m, amps)` / `m3508_set_torque(m, nm)` | Direct torque, no ramp |
| `m3508_brake(m, SOFT / HOLD / HARD)` | Controlled stop; hold zero speed or lock position |
| `m3508_coast(m)` | Zero current |
| `m3508_get_state(m, &s)` | Speed, angle, current, torque, temperature, online |

All in output-shaft units (rpm, degrees, amps). To add motors, raise `M3508_NUM_MOTORS` (IDs 5–8
are sent in a second frame, 0x1FF).

---

## 10. Files

| File | Role |
|---|---|
| `Core/Inc/robot_config.h` | Every tunable constant |
| `Core/Src/app_tasks.c`, `Core/Inc/app_tasks.h` | Creates the three tasks; task overview |
| `Core/Src/input_task.c` | `inputTask`: UART → command |
| `Core/Src/process_task.c` | `processTask`: the 1 kHz control loop and its timing statistics |
| `Core/Src/output_task.c` | `outputTask`: currents → CAN |
| `Core/Src/cmd_link.c`, `.h` | Command frame parser, timeout failsafe, link statistics |
| `Core/Src/chassis.c`, `.h` | Robot velocity ramp, mecanum kinematics, wheel synchronisation, modes |
| `Core/Src/m3508.c`, `.h` | Motor driver: feedback decoding, speed / angle / brake control |
| `Core/Src/pid.c`, `.h` | PID with integrator clamp and anti-windup |
| `Core/Inc/util.h` | Shared helpers: clamping, critical sections |
| `Core/Src/bsp_can.c`, `.h` | CAN filter, receive interrupt, non-blocking send, counters |
| `Core/Src/bsp_uart.c`, `.h` | UART circular-DMA receive, idle-line wake-up, error restart |
| `Core/Src/app_freertos.c` | Calls `app_tasks_create()`; `StartDefaultTask` blinks the status LED |
| `tools/jetson_cmd_sender.py` | Runs on the Jetson: packs and sends command frames |
| `tools/build_flash.ps1`, `.vscode/tasks.json` | One-command build and flash |
| `CMakeLists.txt` | Lists the source files above under "Add user sources here" |

Everything else is generated by STM32CubeMX from `M3508_Lib.ioc`. Regenerating keeps the files
above and anything inside `USER CODE` blocks.
