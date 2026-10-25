# ESP32 FlySky IBUS Receiver → Differential Drive Motor Control

Drives a two-wheel differential-drive robot from a **FlySky RC receiver** (read over its **serial IBUS** output) using two Cytron MD10C (10 A) motor drivers in **sign-magnitude mode at 20 kHz PWM**.

## Hardware

| Part | Role |
|---|---|
| ESP32 DevKit | Main controller |
| 2x Cytron MD10C (10 A) | H-bridge motor drivers, one per wheel |
| 2x DC brushed motor | Left / right drive wheels |
| FlySky AFHDS receiver + transmitter | Remote control (e.g. FS-i6 / FS-i6X transmitter with FS-iA6B / FS-iA10B receiver) |

## Wiring

### FlySky receiver → ESP32

| Receiver | ESP32 / power |
|---|---|
| 5V | 5 V BEC or battery pack (FS-iA6B accepts 4–8.4 V — **not** the ESP32 3V3 pin) |
| GND | ESP32 GND (common ground required) |
| **I-BUS (SERVO)** | GPIO 16 (UART2 RX) |

- Use the **I-BUS SERVO** pin — on the FS-iA6B it's the I-BUS port next to the servo pins (I-BUS SENS is the telemetry port, not servo data).
- The receiver's I-BUS signal is ~3.3 V logic, safe to wire straight to the ESP32.
- Bind the receiver to your transmitter as usual (bind plug / bind button per receiver manual).

### Each MD10C → ESP32

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

1. Install the **esp32** board package by Espressif via Boards Manager.
2. Select your board (e.g. *ESP32 Dev Module*).
3. Open `Esp32XboxBluetoothControllerMotorControl.ino` and upload.
   - Works on arduino-esp32 core 2.x and 3.x — the correct LEDC API is selected automatically at compile time.
4. Power the receiver, turn the transmitter on, and open **Serial Monitor at 115200** to see live channel values and motor outputs.

### PlatformIO alternative

`Esp32Controller/` is a ready-to-build PlatformIO project (stock `espressif32` platform, no extra board package):

```ini
[env:upesy_wroom]
platform = espressif32
board = upesy_wroom
framework = arduino
monitor_speed = 115200
```

Build with `pio run`, upload with `pio run -t upload`, serial monitor with `pio device monitor`.

## How it works

### Control flow

```
FlySky TX -> receiver -> IBUS (UART, 115200) -> channel values -> deadzone/expo -> arcade mix
    -> slew limiter -> sign-magnitude output (PWM + DIR) -> MD10C -> motors
```

1. **IBUS input**: the receiver streams 32-byte frames at 115200 baud 8N1 every ~7 ms. Each frame holds 14 channels as little-endian values (1000–2000, center 1500) plus a 16-bit checksum (`0xFFFF − sum of first 30 bytes`). The parser validates the header and checksum, so a glitched or misaligned stream resyncs automatically.
2. **Channel mapping** (FlySky Mode 2 transmitter defaults, configurable at the top of the sketch):
   - **CH1** (right stick X) → **steering** (left / right)
   - **CH3** (left stick Y) → **throttle** (forward / reverse)
3. **Deadzone + expo**: an 8 % deadband around center removes idle noise (rescaled so full stick still equals full output), then a 1.6-power expo curve makes the robot less twitchy around center.
4. **Arcade mixing** converts throttle + steering into per-wheel speeds:
   ```
   left  = throttle + steering
   right = throttle - steering
   ```
   Both sides are then divided by the largest magnitude if it exceeds 1.0 (normalization), so hard cornering at full throttle can't overflow a motor — it just limits both wheels proportionally.
5. **Slew limiting**: each motor's output can only change by `SLEW_PER_SEC` per second (default 6.0 → 0 to full in ~170 ms). This smooths abrupt stick snaps and protects gearboxes / batteries from current spikes.
6. **Sign-magnitude output** (`driveMotor()`): the **PWM pin carries the magnitude** (duty = |speed| × 2047) and the **DIR pin carries the sign** (HIGH = forward, LOW = reverse, per-motor software-invertible).
7. **Failsafe**: if no valid IBUS frame arrives for 300 ms (`PACKET_TIMEOUT_MS`), the target speed goes to 0 and the slew limiter ramps the motors down instead of cutting them dead.

### Why 20 kHz and 11-bit?

- 20 kHz is **above human hearing**, so the motors don't whine like they do at the typical 1–5 kHz.
- The ESP32 LEDC timer runs from an 80 MHz clock. 80 MHz ÷ 20 kHz = **4000 counts** per period, and 4000 < 2^12, so the maximum resolution at exactly 20 kHz is **2^11 = 2048 duty steps** (0–2047). That's ~0.05 % granularity — far more than a brushed motor needs.

### Why sign-magnitude?

Each half of the PWM period the H-bridge applies either +V (forward) or −V (reverse) per the DIR pin, and 0 V (coast) for the off-time (slow-decay current recirculation on MD10C). Compared to locked-antiphase, sign-magnitude gives simpler code, better low-speed behavior on brushed motors, and maps 1:1 to how the MD10C's PWM+DIR inputs work.

## Tuning (top of the sketch)

| Constant | Default | Effect |
|---|---|---|
| `STEERING_CH` / `THROTTLE_CH` | CH1 / CH3 | Remap which receiver channel drives steering / throttle. |
| `DEADZONE` | 0.08 | Stick idle band (0–1). Raise if the robot creeps. |
| `EXPO` | 1.6 | >1 = softer around center, 1.0 = fully linear. |
| `SLEW_PER_SEC` | 6.0 | Max output change per second. Raise to 50+ for near-instant response. |
| `PACKET_TIMEOUT_MS` | 300 | Failsafe trigger time. |
| `AXIS_Y_INVERT` / `AXIS_X_INVERT` | +1 / +1 | Flip if throttle or steering feels backwards. |
| `dirInvert` (per motor) | false | Flip if one wheel spins the wrong way. |

## First-run checklist

1. **Wheels off the ground** for the first test.
2. Verify the serial monitor prints live `CH1 ... CH3 ...` values when you move the sticks — if not, see troubleshooting.
3. If a wheel spins backward: set `dirInvert = true` for that motor.
4. If throttle/steering feels reversed: flip the axis invert constants (or reassign the mix in your transmitter, e.g. reverse CH3).
5. Watch for `L +0.50  R +0.50` style output to confirm mixing.

## Troubleshooting

- **"no IBUS frames" on serial** → check the receiver has 5 V power, GND is common with the ESP32, and the **I-BUS SERVO** pin (not SENS/PPM/S.BUS) goes to GPIO 16. Confirm the transmitter is on and bound.
- **Sticks move but one axis dead** → you may have bound a PPM/S.BUS output or the channel map differs; watch the CH1–CH4 values in the serial monitor while moving each stick and adjust `STEERING_CH` / `THROTTLE_CH` accordingly.
- **Values jump / robot glitches** → check the receiver's ground connection and keep the IBUS wire short; the checksum validator drops corrupted frames, but a missing ground causes constant drops.
- **TX lost with receiver still powered** → the sketch times out and stops the motors after 300 ms. For receivers that *hold* the last frame instead of stopping output, also set per-channel failsafe values on the transmitter (center = 1500).
- **`ledcAttach failed`** → you're on arduino-esp32 core 2.x; the sketch auto-falls back to the core 2.x `ledcSetup/ledcAttachPin` API — see the note at the top of the sketch.
- **Robot twitches with sticks centered** → raise `DEADZONE`, or increase the transmitter's stick-center trim.
- **One motor slower than the other** → that's normal motor variance; add a trim offset per motor in `driveMotor()` if needed.
