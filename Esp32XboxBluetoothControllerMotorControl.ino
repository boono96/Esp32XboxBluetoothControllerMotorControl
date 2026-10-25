/*
 * ESP32 differential-drive robot
 *   Receiver : FlySky AFHDS receiver with serial IBUS output (FS-iA6B, FS-iA10B, ...)
 *   Drivers  : 2x Cytron MD10C (10 A) in factory sign-magnitude mode (PWM + DIR)
 *   PWM      : 20 kHz, 11-bit resolution (the maximum the ESP32 LEDC can do at 20 kHz,
 *              since 80 MHz / 20 kHz = 4000 counts < 2^12)
 *
 * Drive layout (FlySky Mode 2 transmitter defaults):
 *   CH1 (right stick X) -> steering (left / right)
 *   CH3 (left stick Y)  -> throttle (forward / reverse)
 *   Channel numbers can be remapped with STEERING_CH / THROTTLE_CH below.
 *
 * ---- Wiring (FlySky receiver) ----
 *   Receiver 5V  <- 5 V BEC / battery pack (FS-iA6B accepts 4-8.4 V; NOT the ESP32 3V3 pin)
 *   Receiver GND -> ESP32 GND (common ground)
 *   Receiver I-BUS (SERVO) pin -> ESP32 GPIO 16 (RX2); IBUS is ~3.3 V logic, connect directly
 *   Use the I-BUS SERVO pin (I-BUS SENS is the telemetry pin, not servo data).
 *
 * ---- IBUS protocol ----
 *   115200 baud 8N1, one 32-byte frame every ~7 ms:
 *   [0]=0x20 [1]=0x40, 14 channels as little-endian uint16 (1000-2000, center 1500),
 *   last 2 bytes = 16-bit checksum (0xFFFF - sum of the first 30 bytes).
 *
 * Failsafe: the sketch stops the motors if no valid frame arrives for PACKET_TIMEOUT_MS.
 * Also set channel failsafes on the transmitter (center = 1500) so a TX loss holds the
 * sticks at neutral even on receivers that keep outputting held frames.
 */

#include <esp_arduino_version.h>  // ESP_ARDUINO_VERSION_MAJOR (LEDC API selection)

// ---------------- Pins ----------------
constexpr int LEFT_PWM_PIN  = 25;
constexpr int LEFT_DIR_PIN  = 26;
constexpr int RIGHT_PWM_PIN = 32;
constexpr int RIGHT_DIR_PIN = 33;
constexpr int IBUS_RX_PIN   = 16;  // ESP32 UART2 RX <- receiver I-BUS pin

// ---------------- PWM ----------------
constexpr uint32_t PWM_FREQ_HZ  = 20000;
constexpr uint8_t  PWM_RES_BITS = 11;
constexpr uint32_t PWM_MAX      = (1u << PWM_RES_BITS) - 1;  // 2047

// ---------------- IBUS ----------------
constexpr uint32_t IBUS_BAUD = 115200;

// Channel map, 0-indexed: CH1 = right stick X (steering), CH3 = left stick Y (throttle).
constexpr int STEERING_CH = 0;  // CH1
constexpr int THROTTLE_CH = 2;  // CH3

// ---------------- Tuning ----------------
constexpr float DEADZONE          = 0.08f;  // stick idle band, 0..1
constexpr float EXPO              = 1.6f;   // >1 = softer around center, 1.0 = linear
constexpr float SLEW_PER_SEC      = 6.0f;   // max output change per second (6 = 0->full in ~170 ms)
constexpr uint32_t PACKET_TIMEOUT_MS = 300; // no valid IBUS frame for this long -> failsafe stop

constexpr float AXIS_Y_INVERT = +1.0f;  // RC: stick up = high pulse = forward. Flip if backwards
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

uint32_t lastPacketMs = 0;

// ---------------- PWM helpers ----------------
// arduino-esp32 core 2.x configures LEDC per channel; core 3.x attaches per pin.
// Both produce the same 20 kHz / 11-bit output.
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

// ---------------- IBUS (FlySky serial receiver) ----------------
HardwareSerial ibusSerial(2);  // UART2

constexpr uint8_t  IBUS_FRAME_LEN = 32;
constexpr uint8_t  IBUS_NUM_CH    = 14;

uint8_t  ibusBuf[IBUS_FRAME_LEN];
uint8_t  ibusPos = 0;
uint16_t ibusCh[IBUS_NUM_CH] = {0};

void ibusBegin() {
  ibusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, -1);
}

bool ibusFrameOk() {
  if (ibusBuf[0] != 0x20 || ibusBuf[1] != 0x40) return false;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < IBUS_FRAME_LEN - 2; i++) sum += ibusBuf[i];
  uint16_t chk = ibusBuf[30] | (ibusBuf[31] << 8);
  return (uint16_t)(0xFFFF - sum) == chk;
}

bool ibusChannelsOk() {
  for (uint8_t i = 0; i < IBUS_NUM_CH; i++) {
    if (ibusCh[i] < 900 || ibusCh[i] > 2100) return false;
  }
  return true;
}

// Returns true once per complete, checksum-valid frame. A rejected frame
// (misaligned stream or corrupted bytes) resyncs on a later header+checksum.
bool ibusUpdate() {
  bool fresh = false;
  while (ibusSerial.available()) {
    ibusBuf[ibusPos++] = ibusSerial.read();
    if (ibusPos < IBUS_FRAME_LEN) continue;
    ibusPos = 0;
    if (ibusFrameOk()) {
      for (uint8_t i = 0; i < IBUS_NUM_CH; i++) {
        ibusCh[i] = ibusBuf[2 + 2 * i] | (ibusBuf[3 + 2 * i] << 8);
      }
      if (ibusChannelsOk()) fresh = true;
    }
  }
  return fresh;
}

// IBUS channel value 1000..2000 (center 1500) -> -1..+1
float channelToAxis(int ch) {
  return constrain((float)(ibusCh[ch] - 1500) / 500.0f, -1.0f, 1.0f);
}

// ---------------- Debug ----------------
void debugPrint(uint32_t now, bool alive, float l, float r) {
  static uint32_t lastPrint = 0;
  if (now - lastPrint < 250) return;
  lastPrint = now;
  if (!alive) {
    Serial.println("no IBUS frames -> motors stopped (TX on? bound? wired to GPIO16?)");
    return;
  }
  Serial.printf("CH1 %4u  CH2 %4u  CH3 %4u  CH4 %4u  |  L %+0.2f  R %+0.2f\n",
                ibusCh[0], ibusCh[1], ibusCh[2], ibusCh[3], l, r);
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);

  motorBegin(leftMotor);
  motorBegin(rightMotor);

  ibusBegin();
}

void loop() {
  static uint32_t lastMs = millis();
  uint32_t now = millis();
  float dt = (now - lastMs) * 0.001f;
  lastMs = now;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;

  if (ibusUpdate()) lastPacketMs = now;

  bool alive = (now - lastPacketMs <= PACKET_TIMEOUT_MS);

  float throttle = 0.0f, steering = 0.0f;
  if (alive) {
    throttle = shapeAxis(AXIS_Y_INVERT * channelToAxis(THROTTLE_CH));
    steering = shapeAxis(AXIS_X_INVERT * channelToAxis(STEERING_CH));
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
