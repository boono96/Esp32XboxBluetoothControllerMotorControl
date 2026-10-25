/*
 * ESP32 differential-drive robot
 *   Controller : Xbox-protocol Bluetooth gamepad (HyperX / Xbox / Xbox-style pads)
 *   Drivers    : 2x Cytron MD10C (10 A) in factory sign-magnitude mode (PWM + DIR)
 *   PWM        : 20 kHz, 11-bit resolution (the maximum the ESP32 LEDC can do at 20 kHz,
 *                since 80 MHz / 20 kHz = 4000 counts < 2^12)
 *
 * Drive layout (split sticks):
 *   Left  stick Y -> throttle (forward / reverse)
 *   Right stick X -> steering (left / right)
 *
 * ---- Bluepad32 setup (Arduino IDE) ----
 *   1. File > Preferences > Additional Boards Manager URLs, add:
 *      https://raw.githubusercontent.com/ricardoquesada/espressif-arduino-bluepad32-static/refs/heads/main/package_esp32_bluepad32_index.json
 *   2. Boards Manager: install the "esp32" package by Bluepad32 Contributors
 *   3. Select your board under that package (e.g. "ESP32 Dev Module")
 *   4. LEDC API is auto-selected at compile time: arduino-esp32 core 2.x
 *      (the Bluepad32 board package, v4.0.2) uses ledcSetup/ledcAttachPin;
 *      core 3.x uses ledcAttach.
 *
 * ---- Wiring (each MD10C) ----
 *   MD10C PWM <- ESP32 PWM pin (see pins below)
 *   MD10C DIR <- ESP32 DIR pin
 *   MD10C GND -> ESP32 GND (common ground, opto input side)
 *   MD10C VCC <- ESP32 3V3 (MD10C Rev2+ inputs are 3.3 V tolerant)
 *   Leave the MD10C mode jumper in the factory position (PWM+DIR), NOT "ANTIPHASE".
 *   Sign-magnitude on this driver: duty = magnitude, DIR = sign, 0 % = coast.
 *
 * Pairing: put the pad in Bluetooth pairing mode (Xbox button + pair button,
 * lights flashing). Bluepad32 connects automatically. HyperX pads that follow
 * the Xbox wireless protocol show up as Xbox controllers.
 */

#include <Bluepad32.h>
#include <esp_arduino_version.h>  // ESP_ARDUINO_VERSION_MAJOR (LEDC API selection)

// ---------------- Pins ----------------
constexpr int LEFT_PWM_PIN  = 25;
constexpr int LEFT_DIR_PIN  = 26;
constexpr int RIGHT_PWM_PIN = 32;
constexpr int RIGHT_DIR_PIN = 33;

// ---------------- PWM ----------------
constexpr uint32_t PWM_FREQ_HZ  = 20000;
constexpr uint8_t  PWM_RES_BITS = 11;
constexpr uint32_t PWM_MAX      = (1u << PWM_RES_BITS) - 1;  // 2047

// ---------------- Tuning ----------------
constexpr float DEADZONE          = 0.08f;  // stick idle band, 0..1
constexpr float EXPO              = 1.6f;   // >1 = softer around center, 1.0 = linear
constexpr float SLEW_PER_SEC      = 6.0f;   // max output change per second (6 = 0->full in ~170 ms)
constexpr uint32_t PACKET_TIMEOUT_MS = 300; // no packets for this long -> failsafe stop

// Bluepad32 follows the SDL convention (stick up / left = negative).
constexpr float AXIS_Y_INVERT = -1.0f;  // flip if throttle is backwards
constexpr float AXIS_X_INVERT = +1.0f;  // flip if steering is backwards

struct Motor {
  int     pwmPin;
  uint8_t pwmCh;     // LEDC channel (core 2.x only; ignored on core 3.x)
  int     dirPin;
  bool    dirInvert; // flip if one wheel spins the wrong way
  float   out;       // slewed output, -1..1
};

Motor leftMotor  = { LEFT_PWM_PIN,  0, LEFT_DIR_PIN,  false, 0.0f };
Motor rightMotor = { RIGHT_PWM_PIN, 1, RIGHT_DIR_PIN, false, 0.0f };

ControllerPtr myController;
uint32_t lastPacketMs = 0;

// ---------------- PWM helpers ----------------
// arduino-esp32 core 2.x (Bluepad32 board package) configures LEDC per channel;
// core 3.x attaches per pin. Both produce the same 20 kHz / 11-bit output.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmBegin(const Motor& m) {
  if (!ledcAttach(m.pwmPin, PWM_FREQ_HZ, PWM_RES_BITS)) {
    Serial.printf("ledcAttach failed on pin %d\n", m.pwmPin);
  }
  ledcWrite(m.pwmPin, 0);
}
void pwmWrite(const Motor& m, uint32_t duty) { ledcWrite(m.pwmPin, duty); }
#else
void pwmBegin(const Motor& m) {
  ledcSetup(m.pwmCh, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(m.pwmPin, m.pwmCh);
  ledcWrite(m.pwmCh, 0);
}
void pwmWrite(const Motor& m, uint32_t duty) { ledcWrite(m.pwmCh, duty); }
#endif

// ---------------- Motor helpers ----------------
void motorBegin(const Motor& m) {
  pinMode(m.dirPin, OUTPUT);
  digitalWrite(m.dirPin, LOW);
  pwmBegin(m);
}

// Deadzone with rescale (full stick still = full output) + expo curve.
float shapeAxis(float v) {
  v = constrain(v, -1.0f, 1.0f);
  float mag = fabsf(v);
  if (mag < DEADZONE) return 0.0f;
  mag = powf((mag - DEADZONE) / (1.0f - DEADZONE), EXPO);
  return copysignf(mag, v);
}

// Sign-magnitude output: PWM = magnitude, DIR = sign.
void driveMotor(Motor& m, float target, float dt) {
  float slew = SLEW_PER_SEC * dt;
  if (target > m.out)      m.out = min(target, m.out + slew);
  else if (target < m.out) m.out = max(target, m.out - slew);

  bool forward = (m.out >= 0.0f) != m.dirInvert;
  digitalWrite(m.dirPin, forward ? HIGH : LOW);
  uint32_t duty = (uint32_t)(fabsf(m.out) * PWM_MAX + 0.5f);
  pwmWrite(m, duty);
}

// ---------------- Bluepad32 callbacks ----------------
void onConnected(ControllerPtr ctl) {
  if (myController) return;  // only one driver at a time
  myController = ctl;
  ctl->setPlayerLEDs(0x01);
  ctl->playDualRumble(0, 150, 0x40, 0x80);
  Serial.printf("Controller connected: %s\n", String(ctl->getModelName()).c_str());
}

void onDisconnected(ControllerPtr ctl) {
  if (ctl == myController) myController = nullptr;
  Serial.println("Controller disconnected");
}

// ---------------- Debug ----------------
void debugPrint(uint32_t now, bool alive, float l, float r) {
  static uint32_t lastPrint = 0;
  if (now - lastPrint < 250) return;
  lastPrint = now;
  if (!alive) {
    Serial.println("waiting for controller / packet timeout -> motors stopped");
    return;
  }
  Serial.printf("L %+0.2f  R %+0.2f\n", l, r);
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
//   Serial.println("ESP32 diff-drive, 20 kHz sign-magnitude (Bluepad32)");

  motorBegin(leftMotor);
  motorBegin(rightMotor);

  BP32.setup(&onConnected, &onDisconnected);
  BP32.forgetBluetoothKeys();  // clears stale pairings; comment out after first successful pairing
  BP32.enableVirtualDevice(false);
}

void loop() {
  static uint32_t lastMs = millis();
  uint32_t now = millis();
  float dt = (now - lastMs) * 0.001f;
  lastMs = now;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;

  bool fresh = BP32.update();
  if (fresh) lastPacketMs = now;

  bool alive = myController && myController->isConnected() &&
               (now - lastPacketMs <= PACKET_TIMEOUT_MS);

  float throttle = 0.0f, steering = 0.0f;
  if (alive) {
    throttle = shapeAxis(AXIS_Y_INVERT * myController->axisY() / 512.0f);
    steering = shapeAxis(AXIS_X_INVERT * myController->axisRX() / 512.0f);
  }

  // Arcade mix with normalization so the sum never exceeds full scale.
  float left  = throttle + steering;
  float right = throttle - steering;
  float scale = max(1.0f, max(fabsf(left), fabsf(right)));
  left  /= scale;
  right /= scale;

  driveMotor(leftMotor, left, dt);
  driveMotor(rightMotor, right, dt);

  debugPrint(now, alive, left, right);
}
