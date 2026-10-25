# ESP32 Xbox Bluetooth Controller → Differential Drive Motor Control

Drives a two-wheel differential-drive robot from an Xbox-protocol Bluetooth gamepad (HyperX / Xbox / Xbox-style pads) using two Cytron MD10C (10 A) motor drivers in **sign-magnitude mode at 20 kHz PWM**.

## Hardware

| Part | Role |
|---|---|
| ESP32 DevKit | Main controller, classic Bluetooth (BR/EDR) |
| 2x Cytron MD10C (10 A) | H-bridge motor drivers, one per wheel |
| 2x DC brushed motor | Left / right drive wheels |
| Xbox-style BT gamepad | Remote control (HyperX pads using the Xbox wireless protocol work as "Xbox" controllers) |

## Wiring (each MD10C)

| MD10C | ESP32 |
|---|---|
| PWM | Left: GPIO 25 · Right: GPIO 32 |
| DIR | Left: GPIO 26 · Right: GPIO 33 |
| GND (input side) | ESP32 GND (common ground required) |
| VCC (input side) | ESP32 3V3 (MD10C Rev2+ inputs are 3.3 V tolerant) |

- Keep the MD10C mode jumper in the **factory position (PWM+DIR)**, **not** "ANTIPHASE".
- Motor power (batteries) goes to the MD10C Vmotor / M1 / M2 terminals as usual.
- 0 % PWM = coast — the MD10C has no active brake input in this mode.

## Software setup (Arduino IDE)

1. **File > Preferences > Additional Boards Manager URLs** — add:
   ```
   https://raw.githubusercontent.com/ricardoquesada/espressif-arduino-bluepad32-static/refs/heads/main/package_esp32_bluepad32_index.json
   ```
2. **Boards Manager** — install the `esp32` package by *Bluepad32 Contributors*.
3. Select your board under that package (e.g. *ESP32 Dev Module*).
4. Open `Esp32XboxBluetoothControllerMotorControl.ino` and upload.
   - Works on arduino-esp32 core 2.x (what the Bluepad32 board package is built
     on) and 3.x — the correct LEDC API is selected automatically at compile time.
5. Open **Serial Monitor at 115200** to see status.

### PlatformIO alternative

`Esp32Controller/` is a ready-to-build PlatformIO project using a PIO-compatible
build of the Bluepad32 board package (arduino-esp32 core 2.0.x + Bluepad32 v4.0.2):

```ini
[env:upesy_wroom]
platform = espressif32
board = upesy_wroom
framework = arduino
monitor_speed = 115200
platform_packages =
    framework-arduinoespressif32@https://github.com/maxgerhardt/pio-framework-bluepad32/archive/refs/heads/main.zip
```

Sketch: `Esp32Controller/src/Esp32XboxBluetoothControllerMotorControl.ino` —
build with `pio run`, upload with `pio run -t upload`, serial monitor with `pio device monitor`.

**Pairing:** put the pad in Bluetooth pairing mode (hold Xbox button + pair button until the lights flash). Bluepad32 connects automatically; the pad gets player LED 1 and a short rumble on connect.

## How it works

### Control flow

```
gamepad (Bluetooth) -> Bluepad32 -> stick axes -> deadzone/expo -> arcade mix
    -> slew limiter -> sign-magnitude output (PWM + DIR) -> MD10C -> motors
```

1. **`loop()` calls `BP32.update()`** as fast as possible, which pulls fresh gamepad packets from the Bluetooth stack.
2. **Stick input** is read as raw values (-512..511) and normalized to -1.0..+1.0:
   - Left stick Y → **throttle** (forward / reverse)
   - Right stick X → **steering** (left / right) — "split sticks" layout
3. **Deadzone + expo**: an 8 % deadband removes stick idle noise (and is rescaled so full stick still equals full output), then a 1.6-power expo curve makes the robot less twitchy around center and finer to steer.
4. **Arcade mixing** converts throttle + steering into per-wheel speeds:
   ```
   left  = throttle + steering
   right = throttle - steering
   ```
   Both sides are then divided by the largest magnitude if it exceeds 1.0 (normalization), so hard cornering at full throttle can't overflow a motor — it just limits both wheels proportionally.
5. **Slew limiting**: the output of each motor can only change by `SLEW_PER_SEC` per second (default 6.0 → 0 to full in ~170 ms). This smooths abrupt stick snaps and protects gearboxes / batteries from current spikes.
6. **Sign-magnitude output** (`driveMotor()`): the **PWM pin carries the magnitude** (duty = |speed| × 2047) and the **DIR pin carries the sign** (HIGH = forward, LOW = reverse, per-motor software-invertible). At 20 % duty the motor gets clean 20 kHz pulses in the commanded direction.
7. **Failsafe**: if the controller disconnects, or no packet arrives for 300 ms (`PACKET_TIMEOUT_MS`), the target speed goes to 0 and the slew limiter ramps the motors down instead of cutting them dead.

### Why 20 kHz and 11-bit?

- 20 kHz is **above human hearing**, so the motors don't whine like they do at the typical 1–5 kHz.
- The ESP32 LEDC timer runs from an 80 MHz clock. 80 MHz ÷ 20 kHz = **4000 counts** per period, and 4000 < 2^12, so the maximum resolution at exactly 20 kHz is **2^11 = 2048 duty steps** (0–2047). That's ~0.05 % granularity — far more than a brushed motor needs.

### Why sign-magnitude?

Each half of the PWM period the H-bridge applies either +V (forward) or −V (reverse) per the DIR pin, and 0 V (coast) for the off-time (slow-decay current recirculation on MD10C). Compared to locked-antiphase, sign-magnitude gives simpler code, better low-speed behavior on brushed motors, and maps 1:1 to how the MD10C's PWM+DIR inputs work.

## Tuning (top of the sketch)

| Constant | Default | Effect |
|---|---|---|
| `DEADZONE` | 0.08 | Stick idle band (0–1). Raise if the robot creeps. |
| `EXPO` | 1.6 | >1 = softer around center, 1.0 = fully linear. |
| `SLEW_PER_SEC` | 6.0 | Max output change per second. Raise to 50+ for near-instant response. |
| `PACKET_TIMEOUT_MS` | 300 | Failsafe trigger time. |
| `AXIS_Y_INVERT` / `AXIS_X_INVERT` | −1 / +1 | Flip if throttle or steering feels backwards. |

## First-run checklist

1. **Wheels off the ground** for the first test.
2. If a wheel spins backward: set `dirInvert = true` for that motor.
3. If throttle/steering feels reversed: flip the axis invert constants.
4. Watch Serial Monitor for `L +0.50  R +0.50` style output to confirm mixing.
5. Motors should be silent at rest and at speed (20 kHz); if you hear a faint tone, the driver's switching edges are slow — that's normal and harmless.

## Troubleshooting

- **Pad won't pair** → make sure it's in pairing mode (lights flashing, not just Xbox button on). HyperX pads must follow the Xbox wireless protocol (BR/EDR); check [Bluepad32's supported controllers](https://bluepad32.readthedocs.io/en/latest/supported_gamepads/).
- **Pairing stuck after testing another pad** → `BP32.forgetBluetoothKeys()` is called on boot to clear stale pairings; comment it out once your pad pairs reliably.
- **`ledcAttach failed`** → you're on arduino-esp32 core 2.x; see the core 2.x note above, or upgrade the core.
- **Robot twitches with no stick input** → raise `DEADZONE`.
- **One motor slower than the other** → that's normal motor variance; add a trim offset per motor in `driveMotor()` if needed.
