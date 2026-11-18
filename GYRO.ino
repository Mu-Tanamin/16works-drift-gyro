/*
 * =====================================================================================
 *  PRO RC DRIFT GYROSCOPE CONTROLLER (V3.4 - PURE NORMAL RATE MODE)
 *  Target Board : ESP32-C3 Super Mini (ESP32 Arduino Core 3.x)
 *  Sensor       : MPU6050 (I2C: SDA=GPIO6, SCL=GPIO7)
 * =====================================================================================
 *  ฟีเจอร์เด่น (Pure Normal Rate Mode P+D):
 *   1. Pure Normal Rate Mode: ควบคุมแบบธรรมชาติ พวงมาลัยคืนตัวตามสไตล์รถดริฟท์จริง
 *   2. CH3 Gain Control: ปรับระดับความไว (Gain) 0% - 100% จากรีโมทโดยตรง (1000 - 2000 µs)
 *   3. Asymmetric Slew Rate (Gyro Correction Only):
 *      - Fast Snap Speed: ล้อเด้งสวนแก้อาการทันที ป้องกันรถสปินหลุดไลน์ (ไม่หน่วงมือคนขับ)
 *      - Smooth Return Speed: คืนพวงมาลัยนุ่มนวล ป้องกันอาการท้ายดีดกลับ (Tank-slapper)
 *   4. Dynamic S-Curve Expo: ตรงกลางนุ่มนวล แต่ดึงแน่นทวีคูณตอนสะบัดแรง
 *   5. Predictive D-Term (dt-scaled): คาดเดาความเร่งการหมุนจริง (deg/s^2) ล้อหน้าแก้ทันที
 *   6. Steering Priority: ลดทอนไจโรตอนคนขับเลี้ยวสุด ช่วยให้สลับไลน์ (Flick/Transition) ได้คล่องตัว
 *   7. BLE 5.0 Web Bluetooth Tuner: ชื่อแยกตาม MAC + UUID เฉพาะ + Tuning PIN + Driving Guard
 *   8. Fail-Safe I2C & NaN Guard: ตรวจจับสายเซ็นเซอร์หลุด (ตัดเป็น Pass-through) และกันค่าเสีย
 *   9. รองรับเซอร์โว AFRC D2114TG-S ที่ 333Hz เต็มประสิทธิภาพ (หรือเลือก 50Hz)
 * =====================================================================================
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

// Uncomment to run BLE command parser self-tests at boot via Serial
// #define SELFTEST

// Forward Declarations
static inline float clampf(float v, float lo, float hi, float fallback);
static inline int getMedian3(volatile int* buf);
void readCH1();
void readCH3();
float getCH3Gain();
bool readGyroZRaw(int16_t& out_z);
void initServoPWM();
void writeServo(int pwm_us);
void loadTuningDefaults();
void loadMechanicalDefaults();
void loadSettings();
void saveSettings();
void blinkLEDWarning(int count, int on_ms);
void calibrateGyroOffset(bool from_boot);
void setupBLE();
void updateBLEConfigValue();
void notifyBLETelemetry();
void parseBLECommand(const String& body);
void runDriftMode(float dt);
void blinkLED();
void runSetupMode();
void handleButton();
#ifdef SELFTEST
void runParserSelfTest();
#endif

// ==========================================
// 1. PIN CONFIGURATION (ESP32-C3 Super Mini)
// ==========================================
const int PIN_CH1_IN    = 2;  // Steering Input (RC Receiver CH1)
const int PIN_CH3_IN    = 3;  // Gyro Gain Input (RC Receiver CH3)
const int PIN_SERVO_OUT = 4;  // Steering Servo Signal
const int PIN_LED       = 8;  // On-board Blue LED (Active LOW)
const int PIN_BUTTON    = 9;  // On-board BOOT Button (Active LOW)

const int I2C_SDA_PIN   = 6;  // MPU6050 SDA
const int I2C_SCL_PIN   = 7;  // MPU6050 SCL

// ==========================================
// 2. CONFIGURATION STRUCTURE & DEFAULTS
// ==========================================
struct GyroConfig {
  float p_gain;             // Proportional Gain (ความไวหลัก)
  float d_gain;             // Derivative Gain (Predictive Lead-Lag)
  float s_curve;            // S-Curve Expo Factor (0.0 = Linear, 1.0 = Max Expo)
  float steering_priority;  // สัดส่วนลดทอนไจโรตอนคนขับเลี้ยวสุด (0.0 - 1.0)
  float snap_speed;         // ความเร็วล้อเด้งสวน (µs/s)
  float return_speed;       // ความเร็วล้อคืนตัว (µs/s)
  float damper;             // Low-pass Filter Factor (0.1 - 0.9)
  float deadband;           // ตัด Noise ที่จุดศูนย์กลาง (deg/s)
  int   limit_left;         // Endpoint ซ้าย (µs)
  int   limit_right;        // Endpoint ขวา (µs)
  bool  is_reversed;        // กลับทิศทาง Gyro
  int   servo_freq;         // 50Hz หรือ 333Hz
  int   servo_trim;         // Servo Sub-Trim สำหรับ Direct Steering (-150 ถึง +150 µs)
} cfg;

char cfg_pin[9] = "1616";
const char* DEFAULT_TUNING_PIN = "1616";
const char* FW_VERSION = "3.4";

MPU6050 mpu;
Preferences preferences;

#define BLE_SERVICE_UUID     "16a00001-7e10-4b5a-9c3d-160000000001"
#define BLE_CHAR_STATUS_UUID "16a00002-7e10-4b5a-9c3d-160000000002"
#define BLE_CHAR_CONFIG_UUID "16a00003-7e10-4b5a-9c3d-160000000003"

BLEServer* pServer = nullptr;
BLECharacteristic* pStatusChar = nullptr;
BLECharacteristic* pConfigChar = nullptr;
volatile bool ble_connected = false;
volatile bool pending_calibrate = false;
volatile bool pending_save = false;
volatile bool pending_pwm_reinit = false;
unsigned long last_ble_notify_ms = 0;
unsigned long last_nvs_save_ms = 0;

// ==========================================
// 3. RC SIGNAL VARIABLES & LOW-LATENCY ANTI-JITTER FILTER
// ==========================================
volatile unsigned long ch1_start_time = 0;
volatile unsigned long ch3_start_time = 0;
volatile int ch1_buf[3] = {1500, 1500, 1500};
volatile int ch3_buf[3] = {1500, 1500, 1500};
volatile uint8_t ch1_idx = 0;
volatile uint8_t ch3_idx = 0;
volatile float ch1_filtered = 1500.0f;
volatile float ch3_filtered = 1500.0f;

volatile int ch1_pwm = 1500;
volatile int ch3_pwm = 1500;
volatile unsigned long last_ch1_time = 0;
volatile unsigned long last_ch3_time = 0;

static inline float clampf(float v, float lo, float hi, float fallback) {
  if (!isfinite(v)) return fallback;
  return fminf(fmaxf(v, lo), hi);
}

// ฟังก์ชันหาค่ามัธยฐาน 3 จุด (3-Point Fast Median) ตัด Glitch โดยไม่หน่วงมือคนขับ
static inline int getMedian3(volatile int* buf) {
  int a = buf[0], b = buf[1], c = buf[2];
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

void IRAM_ATTR readCH1() {
  unsigned long now = micros();
  if (digitalRead(PIN_CH1_IN) == HIGH) {
    ch1_start_time = now;
  } else {
    unsigned long pulse = now - ch1_start_time;
    if (pulse >= 850 && pulse <= 2150) {
      ch1_buf[ch1_idx] = (int)pulse;
      ch1_idx = (ch1_idx + 1) % 3;
      int med = getMedian3(ch1_buf);

      // Ultra-Responsive EMA Smoothing บน Steering (α = 0.60)
      ch1_filtered = (ch1_filtered * 0.40f) + ((float)med * 0.60f);
      int target_pwm = (int)(ch1_filtered + 0.5f);

      // Center Snap: ถ้าอยู่ใกล้จุดศูนย์กลาง ±3 µs ล็อกเป็น 1500 µs ทันที
      if (abs(target_pwm - 1500) <= 3) {
        target_pwm = 1500;
        ch1_filtered = 1500.0f;
      }

      // Hysteresis Deadband (2 µs): ลด jitter โดยไม่เสียความละเอียด
      if (abs(target_pwm - ch1_pwm) >= 2 || target_pwm == 1500) {
        ch1_pwm = target_pwm;
      }
      last_ch1_time = millis();
    }
  }
}

void IRAM_ATTR readCH3() {
  unsigned long now = micros();
  if (digitalRead(PIN_CH3_IN) == HIGH) {
    ch3_start_time = now;
  } else {
    unsigned long pulse = now - ch3_start_time;
    if (pulse >= 850 && pulse <= 2150) {
      ch3_buf[ch3_idx] = (int)pulse;
      ch3_idx = (ch3_idx + 1) % 3;
      int med = getMedian3(ch3_buf);

      // Heavy EMA Smoothing บน Gain Knob (α = 0.25)
      ch3_filtered = (ch3_filtered * 0.75f) + ((float)med * 0.25f);
      int target_pwm = (int)(ch3_filtered + 0.5f);

      // Center Snap: ถ้าอยู่ใกล้จุดศูนย์กลาง ±5 µs ล็อกเป็น 1500 µs ทันที
      if (abs(target_pwm - 1500) <= 5) {
        target_pwm = 1500;
        ch3_filtered = 1500.0f;
      }

      // Hysteresis Deadband (4 µs): ป้องกันตัวเลขเปอร์เซ็นต์กระพริบสลับไปมา
      if (abs(target_pwm - ch3_pwm) >= 4 || target_pwm == 1500) {
        ch3_pwm = target_pwm;
      }
      last_ch3_time = millis();
    }
  }
}

// ==========================================
// 4. CONTROL SYSTEM VARIABLES
// ==========================================
float gyro_z_offset = 0.0f;
float filtered_gyro_z = 0.0f;
float prev_gyro_z = 0.0f;
float filtered_gyro_d = 0.0f;
float current_gyro_corr = 0.0f;

bool mpu_present = false;
unsigned long i2c_error_count = 0;

float current_servo_pos = 1520.0f;
unsigned long last_control_time = 0;

const unsigned long CONTROL_PERIOD_US = 4000; // Deterministic 250 Hz Control Loop
const int SERVO_NATIVE_CENTER = 1520;         // สเปกโรงงาน AFRC D2114TG-S คือ 1520 µs
const int SERVO_TIMER_RES_BITS = 14;
const int SERVO_MAX_DUTY = (1 << SERVO_TIMER_RES_BITS) - 1; // 16383
const int CONFIG_SCHEMA_VERSION = 2;

// State Machine สำหรับปุ่มฮาร์ดแวร์
enum SystemState { CALIBRATING, NORMAL_MODE, SET_LEFT_LIMIT, SET_RIGHT_LIMIT, SET_REVERSE };
SystemState currentState = CALIBRATING;
unsigned long buttonPressTime = 0;
bool isButtonPressed = false;
bool temp_reverse = false;

// ==========================================
// 5. HARDWARE SERVO PWM & SENSOR HELPERS
// ==========================================
void initServoPWM() {
  cfg.servo_freq = (cfg.servo_freq == 50) ? 50 : 333;
  ledcAttach(PIN_SERVO_OUT, cfg.servo_freq, SERVO_TIMER_RES_BITS);
}

void writeServo(int pwm_us) {
  pwm_us = constrain(pwm_us, 800, 2200);
  float period_us = 1000000.0f / (float)cfg.servo_freq;
  uint32_t duty = (uint32_t)(((float)pwm_us / period_us) * SERVO_MAX_DUTY);
  ledcWrite(PIN_SERVO_OUT, duty);
}

float getCH3Gain() {
  // CH3: กึ่งกลาง (1500 µs หรือเลข 0 บนรีโมท) = Gain 0% (ปิดไจโร)
  // หมุนไปฝั่งบวก (>1500 ถึง 2000 µs) หรือฝั่งลบ (<1500 ถึง 1000 µs) เพิ่ม Gain 0% - 100% เท่ากัน
  int ch3_val = constrain(ch3_pwm, 1000, 2000);
  int ch3_offset = abs(ch3_val - 1500);
  if (ch3_offset <= 10) ch3_offset = 0;
  float gain = (float)ch3_offset / 490.0f;
  return (gain > 1.0f) ? 1.0f : gain;
}

bool readGyroZRaw(int16_t& out_z) {
  uint8_t buf[2];
  int8_t n = I2Cdev::readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 2, buf, 5);
  if (n != 2) {
    return false;
  }
  out_z = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  return true;
}

void blinkLEDWarning(int count, int on_ms) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(on_ms);
    digitalWrite(PIN_LED, HIGH);
    delay(on_ms);
  }
}

// ==========================================
// 6. PREFERENCES & DEFAULT CONFIG
// ==========================================
void loadTuningDefaults() {
  cfg.p_gain            = 2.6f;
  cfg.d_gain            = 0.15f;
  cfg.s_curve           = 0.20f;
  cfg.steering_priority = 0.35f;
  cfg.snap_speed        = 4500.0f;
  cfg.return_speed      = 2200.0f;
  cfg.damper            = 0.50f;
  cfg.deadband          = 0.5f;
}

void loadMechanicalDefaults() {
  cfg.limit_left        = 1100;
  cfg.limit_right       = 1900;
  cfg.is_reversed       = false;
  cfg.servo_freq        = 333;
  cfg.servo_trim        = 0;
}

void loadSettings() {
  preferences.begin("gyro_pro", false);

  String saved_pin = preferences.getString("pin", DEFAULT_TUNING_PIN);
  if (saved_pin.length() >= 4 && saved_pin.length() <= 8) {
    saved_pin.toCharArray(cfg_pin, sizeof(cfg_pin));
  } else {
    strlcpy(cfg_pin, DEFAULT_TUNING_PIN, sizeof(cfg_pin));
  }

  int ver = preferences.getInt("cfg_ver", 0);
  if (ver < CONFIG_SCHEMA_VERSION) {
    // อัปเกรดพารามิเตอร์ควบคุมจากเวอร์ชันเก่า โดยคงค่าตั้งเชิงกลของแชสซีไว้
    loadTuningDefaults();
    cfg.limit_left  = constrain(preferences.getInt("lim_l", 1100), 800, 2200);
    cfg.limit_right = constrain(preferences.getInt("lim_r", 1900), 800, 2200);
    cfg.is_reversed = preferences.getBool("reversed", false);
    cfg.servo_freq  = (preferences.getInt("freq", 333) == 50) ? 50 : 333;
    cfg.servo_trim  = constrain(preferences.getInt("servo_trim", 0), -150, 150);
    saveSettings();
    return;
  }

  cfg.p_gain            = clampf(preferences.getFloat("p_gain", 2.6f), 0.5f, 8.0f, 2.6f);
  cfg.d_gain            = clampf(preferences.getFloat("d_gain", 0.15f), 0.0f, 1.0f, 0.15f);
  cfg.s_curve           = clampf(preferences.getFloat("s_curve", 0.20f), 0.0f, 1.0f, 0.20f);
  cfg.steering_priority = clampf(preferences.getFloat("st_prio", 0.35f), 0.0f, 1.0f, 0.35f);
  cfg.snap_speed        = clampf(preferences.getFloat("snap_spd", 4500.0f), 800.0f, 8000.0f, 4500.0f);
  cfg.return_speed      = clampf(preferences.getFloat("ret_spd", 2200.0f), 600.0f, 5000.0f, 2200.0f);
  cfg.damper            = clampf(preferences.getFloat("damper", 0.50f), 0.10f, 0.90f, 0.50f);
  cfg.deadband          = clampf(preferences.getFloat("deadband", 0.5f), 0.0f, 3.0f, 0.5f);
  cfg.limit_left        = constrain(preferences.getInt("lim_l", 1100), 800, 2200);
  cfg.limit_right       = constrain(preferences.getInt("lim_r", 1900), 800, 2200);
  cfg.is_reversed       = preferences.getBool("reversed", false);
  cfg.servo_freq        = (preferences.getInt("freq", 333) == 50) ? 50 : 333;
  cfg.servo_trim        = constrain(preferences.getInt("servo_trim", 0), -150, 150);
}

void saveSettings() {
  static GyroConfig last_saved_cfg;
  static char last_saved_pin[9] = "";
  static bool has_saved_cache = false;

  if (has_saved_cache &&
      memcmp(&cfg, &last_saved_cfg, sizeof(GyroConfig)) == 0 &&
      strcmp(cfg_pin, last_saved_pin) == 0) {
    return;
  }

  preferences.putInt("cfg_ver", CONFIG_SCHEMA_VERSION);
  preferences.putFloat("p_gain", cfg.p_gain);
  preferences.putFloat("d_gain", cfg.d_gain);
  preferences.putFloat("s_curve", cfg.s_curve);
  preferences.putFloat("st_prio", cfg.steering_priority);
  preferences.putFloat("snap_spd", cfg.snap_speed);
  preferences.putFloat("ret_spd", cfg.return_speed);
  preferences.putFloat("damper", cfg.damper);
  preferences.putFloat("deadband", cfg.deadband);
  preferences.putInt("lim_l", cfg.limit_left);
  preferences.putInt("lim_r", cfg.limit_right);
  preferences.putBool("reversed", cfg.is_reversed);
  preferences.putInt("freq", cfg.servo_freq);
  preferences.putInt("servo_trim", cfg.servo_trim);
  preferences.putString("pin", cfg_pin);

  last_saved_cfg = cfg;
  strlcpy(last_saved_pin, cfg_pin, sizeof(last_saved_pin));
  has_saved_cache = true;
}

// ==========================================
// 7. AUTO GYRO OFFSET CALIBRATION (STILLNESS VERIFIED)
// ==========================================
void calibrateGyroOffset(bool from_boot) {
  if (!mpu_present) {
    Serial.println(F("[GYRO] Skipping calibration: MPU6050 offline."));
    return;
  }

  const int SAMPLES = 300;
  const int16_t STILLNESS_MAX_SPAN = 200; // ~3.05 deg/s peak-to-peak at +/-500 deg/s

  while (true) {
    Serial.println(F("[GYRO] Calibrating Zero-Offset... Keep car still!"));
    writeServo(SERVO_NATIVE_CENTER + cfg.servo_trim);

    long sum_z = 0;
    int16_t min_z = 32767;
    int16_t max_z = -32768;
    bool i2c_ok = true;

    for (int i = 0; i < SAMPLES; i++) {
      int16_t z = 0;
      if (!readGyroZRaw(z)) {
        i2c_ok = false;
        i2c_error_count++;
        break;
      }
      sum_z += z;
      if (z < min_z) min_z = z;
      if (z > max_z) max_z = z;
      digitalWrite(PIN_LED, (i % 20 < 10) ? LOW : HIGH);
      delay(5);
    }

    if (!i2c_ok) {
      Serial.println(F("[GYRO] Calibration aborted: I2C read failed."));
      blinkLEDWarning(4, 60);
      return;
    }

    int32_t span = (int32_t)max_z - (int32_t)min_z;
    if (span <= STILLNESS_MAX_SPAN) {
      gyro_z_offset = (float)sum_z / (float)SAMPLES;
      filtered_gyro_z = 0.0f;
      prev_gyro_z = 0.0f;
      filtered_gyro_d = 0.0f;
      current_gyro_corr = 0.0f;
      digitalWrite(PIN_LED, HIGH);
      Serial.printf("[GYRO] Offset: %.2f (span=%ld LSB)\n", gyro_z_offset, (long)span);
      return;
    }

    Serial.printf("[GYRO] Car moved during calibration (span=%ld LSB).\n", (long)span);
    blinkLEDWarning(3, 50);

    if (!from_boot) {
      // ระหว่างใช้งานจริง: คง offset เดิมไว้แทนการเขียนค่าที่เพี้ยน
      return;
    }
    delay(250);
  }
}

// ==========================================
// 8. BLE 5.0 GATT SERVER & WEB BLUETOOTH API
// ==========================================
static bool extractJsonString(const String& body, const char* key, String& out) {
  int k_idx = body.indexOf(key);
  if (k_idx == -1) return false;
  int q1 = body.indexOf('"', k_idx + strlen(key));
  if (q1 == -1) return false;
  int q2 = body.indexOf('"', q1 + 1);
  if (q2 == -1) return false;
  out = body.substring(q1 + 1, q2);
  return true;
}

class GyroServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    ble_connected = true;
    Serial.println(F("[BLE] Client Connected."));
  }
  void onDisconnect(BLEServer* s) override {
    ble_connected = false;
    Serial.println(F("[BLE] Client Disconnected -> Restarting Advertising."));
    BLEDevice::startAdvertising();
  }
};

class GyroConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String body = pChar->getValue();
    if (body.length() > 0) {
      parseBLECommand(body);
    }
  }
};

void updateBLEConfigValue() {
  if (!pConfigChar) return;
  char json[320];
  snprintf(json, sizeof(json),
    "{\"fw\":\"%s\",\"p_gain\":%.2f,\"d_gain\":%.2f,\"s_curve\":%.2f,\"st_prio\":%.2f,"
    "\"snap_spd\":%.1f,\"ret_spd\":%.1f,\"damper\":%.2f,\"deadband\":%.2f,"
    "\"lim_l\":%d,\"lim_r\":%d,\"reversed\":%s,\"freq\":%d,\"servo_trim\":%d}",
    FW_VERSION,
    cfg.p_gain, cfg.d_gain, cfg.s_curve, cfg.steering_priority,
    cfg.snap_speed, cfg.return_speed, cfg.damper, cfg.deadband,
    cfg.limit_left, cfg.limit_right, cfg.is_reversed ? "true" : "false", cfg.servo_freq,
    cfg.servo_trim);
  pConfigChar->setValue((uint8_t*)json, strlen(json));
}

void parseBLECommand(const String& body) {
  // 1. Driving Guard: ห้ามแก้ไขค่าจูนหรือสั่ง Calibrate ขณะรถกำลังหมุนหรือเลี้ยว
  if (fabs(filtered_gyro_z) > 5.0f || abs(ch1_pwm - 1500) > 25) {
    Serial.println(F("[BLE] Rejected write: car is moving/steering."));
    return;
  }

  // 2. Tuning PIN Guard: ป้องกันเครื่องอื่นในสนามเข้ามาแก้ค่ารถโดยพลการ
  String rx_pin;
  if (!extractJsonString(body, "\"pin\":", rx_pin) || rx_pin != cfg_pin) {
    Serial.println(F("[BLE] Rejected write: invalid or missing PIN."));
    return;
  }

  String new_pin;
  if (extractJsonString(body, "\"new_pin\":", new_pin)) {
    new_pin.trim();
    if (new_pin.length() >= 4 && new_pin.length() <= 8) {
      new_pin.toCharArray(cfg_pin, sizeof(cfg_pin));
      pending_save = true;
    }
    return;
  }

  if (body.indexOf("\"cmd\":\"cal\"") != -1) {
    pending_calibrate = true;
    return;
  }

  if (body.indexOf("\"cmd\":\"reset\"") != -1) {
    loadTuningDefaults();
    pending_save = true;
    updateBLEConfigValue();
    return;
  }

  if (body.indexOf("\"cmd\":\"trim\"") != -1) {
    int v_idx = body.indexOf("\"val\":");
    if (v_idx != -1) {
      cfg.servo_trim = constrain(body.substring(v_idx + 6).toInt(), -150, 150);
      updateBLEConfigValue();
    }
    return;
  }

  int p_idx = body.indexOf("\"p_gain\":");
  if (p_idx != -1) cfg.p_gain = clampf(body.substring(p_idx + 9).toFloat(), 0.5f, 8.0f, cfg.p_gain);

  int d_idx = body.indexOf("\"d_gain\":");
  if (d_idx != -1) cfg.d_gain = clampf(body.substring(d_idx + 9).toFloat(), 0.0f, 1.0f, cfg.d_gain);

  int s_idx = body.indexOf("\"s_curve\":");
  if (s_idx != -1) cfg.s_curve = clampf(body.substring(s_idx + 10).toFloat(), 0.0f, 1.0f, cfg.s_curve);

  int st_idx = body.indexOf("\"st_prio\":");
  if (st_idx != -1) cfg.steering_priority = clampf(body.substring(st_idx + 10).toFloat(), 0.0f, 1.0f, cfg.steering_priority);

  int snap_idx = body.indexOf("\"snap_spd\":");
  if (snap_idx != -1) cfg.snap_speed = clampf(body.substring(snap_idx + 11).toFloat(), 800.0f, 8000.0f, cfg.snap_speed);

  int ret_idx = body.indexOf("\"ret_spd\":");
  if (ret_idx != -1) cfg.return_speed = clampf(body.substring(ret_idx + 10).toFloat(), 600.0f, 5000.0f, cfg.return_speed);

  int dmp_idx = body.indexOf("\"damper\":");
  if (dmp_idx != -1) cfg.damper = clampf(body.substring(dmp_idx + 9).toFloat(), 0.10f, 0.90f, cfg.damper);

  int db_idx = body.indexOf("\"deadband\":");
  if (db_idx != -1) cfg.deadband = clampf(body.substring(db_idx + 11).toFloat(), 0.0f, 3.0f, cfg.deadband);

  int ll_idx = body.indexOf("\"lim_l\":");
  if (ll_idx != -1) cfg.limit_left = constrain(body.substring(ll_idx + 8).toInt(), 800, 2200);

  int lr_idx = body.indexOf("\"lim_r\":");
  if (lr_idx != -1) cfg.limit_right = constrain(body.substring(lr_idx + 8).toInt(), 800, 2200);

  int rev_idx = body.indexOf("\"reversed\":");
  if (rev_idx != -1) {
    String rev_sub = body.substring(rev_idx + 11);
    rev_sub.trim();
    cfg.is_reversed = rev_sub.startsWith("true");
  }

  int trim_idx = body.indexOf("\"servo_trim\":");
  if (trim_idx != -1) cfg.servo_trim = constrain(body.substring(trim_idx + 13).toInt(), -150, 150);

  int frq_idx = body.indexOf("\"freq\":");
  if (frq_idx != -1) {
    int old_freq = cfg.servo_freq;
    int req_freq = body.substring(frq_idx + 7).toInt();
    cfg.servo_freq = (req_freq == 50) ? 50 : 333;
    if (old_freq != cfg.servo_freq) pending_pwm_reinit = true;
  }

  pending_save = true;
  updateBLEConfigValue();
}

void notifyBLETelemetry() {
  if (!ble_connected || !pStatusChar) return;

  float gain = getCH3Gain();
  float disp_gyro = (fabs(filtered_gyro_z) < 0.05f) ? 0.0f : filtered_gyro_z;

  char json[180];
  snprintf(json, sizeof(json),
    "{\"ch1\":%d,\"ch3\":%d,\"servo\":%d,\"gyro\":%.2f,\"gain\":%.2f,\"trim\":%d,\"i2c_err\":%lu}",
    ch1_pwm, ch3_pwm, (int)(current_servo_pos + 0.5f), disp_gyro, gain, cfg.servo_trim, i2c_error_count);
  pStatusChar->setValue((uint8_t*)json, strlen(json));
  pStatusChar->notify();
}

void setupBLE() {
  char ble_name[24];
  snprintf(ble_name, sizeof(ble_name), "16WORKs-GYRO-%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  BLEDevice::init(ble_name);
  BLEDevice::setMTU(517);
  BLEDevice::setPower(ESP_PWR_LVL_P3, ESP_BLE_PWR_TYPE_DEFAULT); // +3 dBm เพิ่มเสถียรภาพสัญญาณในสนาม
  BLEDevice::setPower(ESP_PWR_LVL_P3, ESP_BLE_PWR_TYPE_ADV);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new GyroServerCallbacks());

  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  pStatusChar = pService->createCharacteristic(
    BLE_CHAR_STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  pConfigChar = pService->createCharacteristic(
    BLE_CHAR_CONFIG_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pConfigChar->setCallbacks(new GyroConfigCallbacks());

  updateBLEConfigValue();
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0C); // 15 ms (เหมาะกับ iOS/Android ลดอาการหลุดในสนาม)
  pAdvertising->setMaxPreferred(0x18); // 30 ms
  BLEDevice::startAdvertising();

  Serial.printf("[BLE] Advertising Started: %s (+3 dBm)\n", ble_name);
}

#ifdef SELFTEST
void runParserSelfTest() {
  Serial.println(F("[SELFTEST] Running BLE parser checks..."));
  GyroConfig backup = cfg;
  bool pass = true;

  // 1. Valid write should pass
  parseBLECommand("{\"pin\":\"1616\",\"p_gain\":3.1,\"d_gain\":0.2}");
  if (fabs(cfg.p_gain - 3.1f) > 0.01f) pass = false;

  // 2. NaN / Inf / garbage should never corrupt cfg
  parseBLECommand("{\"pin\":\"1616\",\"p_gain\":nan,\"d_gain\":inf,\"s_curve\":-inf,\"snap_spd\":abc,\"ret_spd\":NaN,\"damper\":nan,\"deadband\":nan,\"st_prio\":nan}");
  if (!isfinite(cfg.p_gain) || cfg.p_gain < 0.5f || cfg.p_gain > 8.0f) pass = false;
  if (!isfinite(cfg.d_gain) || cfg.d_gain < 0.0f || cfg.d_gain > 1.0f) pass = false;
  if (!isfinite(cfg.s_curve) || cfg.s_curve < 0.0f || cfg.s_curve > 1.0f) pass = false;
  if (!isfinite(cfg.steering_priority) || cfg.steering_priority < 0.0f || cfg.steering_priority > 1.0f) pass = false;
  if (!isfinite(cfg.snap_speed) || cfg.snap_speed < 800.0f || cfg.snap_speed > 8000.0f) pass = false;
  if (!isfinite(cfg.return_speed) || cfg.return_speed < 600.0f || cfg.return_speed > 5000.0f) pass = false;
  if (!isfinite(cfg.damper) || cfg.damper < 0.10f || cfg.damper > 0.90f) pass = false;
  if (!isfinite(cfg.deadband) || cfg.deadband < 0.0f || cfg.deadband > 3.0f) pass = false;

  // 3. Wrong PIN must be rejected
  parseBLECommand("{\"pin\":\"0000\",\"p_gain\":7.5,\"reversed\":true}");
  if (fabs(cfg.p_gain - 7.5f) < 0.01f) pass = false;

  // 4. Out-of-range values must clamp
  parseBLECommand("{\"pin\":\"1616\",\"p_gain\":99.0,\"lim_l\":100,\"lim_r\":5000}");
  if (cfg.p_gain != 8.0f || cfg.limit_left != 800 || cfg.limit_right != 2200) pass = false;

  cfg = backup;
  pending_save = false;
  pending_pwm_reinit = false;
  updateBLEConfigValue();
  Serial.printf("[SELFTEST] Result: %s\n", pass ? "PASS" : "FAIL");
}
#endif

// ==========================================
// 9. SETUP FUNCTION
// ==========================================
void setup() {
  // 1. ลดความถี่ CPU จาก 160MHz เหลือ 80MHz (ประหยัดไฟและลดความร้อนลงทันที 40%)
  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=============================================="));
  Serial.println(F(" ESP32-C3 Super Mini - Drift Gyro (BLE V3.4)"));
  Serial.println(F("=============================================="));

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // 2. I2C Bus (จำกัด timeout 3ms ป้องกันบัสค้าง)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(3);

  mpu.initialize();
  mpu_present = mpu.testConnection();
  if (mpu_present) {
    Serial.println(F("[OK] MPU6050 Connected."));
    // 1) เปิด Hardware Digital Low Pass Filter (DLPF) 42Hz ลด Phase Lag แต่ยังตัดสั่นมอเตอร์ได้ดี
    mpu.setDLPFMode(3); // 3 = MPU6050_DLPF_BW_42 (Gyro Cutoff 42Hz, Delay ~4.8ms)
    // 2) ตั้งค่า Full Scale Gyro Range ±500 deg/s (65.5 LSB/deg/s) สมดุลความละเอียดสูง + รองรับ Reverse Entry
    mpu.setFullScaleGyroRange(1);
  } else {
    Serial.println(F("[WARN] MPU6050 connection error. Check wiring SDA=6, SCL=7"));
  }

  // 3. โหลด Settings
  loadMechanicalDefaults();
  loadTuningDefaults();
  loadSettings();

  // 4. เริ่มต้น PWM
  initServoPWM();
  current_servo_pos = (float)(SERVO_NATIVE_CENTER + cfg.servo_trim);
  writeServo(SERVO_NATIVE_CENTER + cfg.servo_trim);

  // 5. ตั้งค่า Interrupt รับสัญญาณ RC
  pinMode(PIN_CH1_IN, INPUT_PULLUP);
  pinMode(PIN_CH3_IN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CH1_IN), readCH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH3_IN), readCH3, CHANGE);

  // 6. เริ่มต้นระบบ BLE 5.0 ประหยัดพลังงาน
  setupBLE();

#ifdef SELFTEST
  runParserSelfTest();
#endif

  // 7. Calibrate Zero-Offset (รอจนรถนิ่งตอนเปิดเครื่อง)
  calibrateGyroOffset(true);

  last_control_time = micros();
  currentState = NORMAL_MODE;
}

// ==========================================
// 12. DRIFT CONTROL ALGORITHM (PURE NORMAL RATE P+D)
// ==========================================
void runDriftMode(float dt) {
  int servo_center = SERVO_NATIVE_CENTER + cfg.servo_trim;
  int min_lim = min(cfg.limit_left, cfg.limit_right);
  int max_lim = max(cfg.limit_left, cfg.limit_right);

  // Failsafe: รีโมทหลุดเกิน 200ms
  bool signal_lost = (millis() - last_ch1_time > 200);
  if (signal_lost) {
    current_gyro_corr = 0.0f;
    filtered_gyro_z = 0.0f;
    prev_gyro_z = 0.0f;
    filtered_gyro_d = 0.0f;
    current_servo_pos = (float)servo_center;
    writeServo(servo_center);
    digitalWrite(PIN_LED, (millis() / 200) % 2 == 0 ? LOW : HIGH);
    return;
  }

  // RC Steering Input คำนวณเทียบกับจุดศูนย์กลางของรีโมทมาตรฐาน (1500 µs)
  int raw_st = constrain(ch1_pwm, 1000, 2000) - 1500;
  if (abs(raw_st) <= 3) {
    raw_st = 0;
  }
  int steering_input = raw_st;

  // 1. อ่านค่า Gyro Z พร้อมเช็คความสมบูรณ์ของบัส I2C
  int16_t raw_z_lsb = 0;
  if (!mpu_present || !readGyroZRaw(raw_z_lsb)) {
    if (mpu_present) {
      i2c_error_count++;
    }
    // เซ็นเซอร์ล้มเหลว: ตัด gain = 0 (Pass-through พวงมาลัยคนขับตรงๆ) และกะพริบไฟเตือน
    current_gyro_corr = 0.0f;
    filtered_gyro_z = 0.0f;
    prev_gyro_z = 0.0f;
    filtered_gyro_d = 0.0f;
    float pass_target = constrain((float)servo_center + (float)steering_input, (float)min_lim, (float)max_lim);
    current_servo_pos = pass_target;
    writeServo((int)(pass_target + 0.5f));
    digitalWrite(PIN_LED, (millis() / 80) % 2 == 0 ? LOW : HIGH);
    return;
  }
  digitalWrite(PIN_LED, HIGH);

  // Full Scale ±500 deg/s -> 65.5 LSB/deg/s
  float raw_gyro_z = ((float)raw_z_lsb - gyro_z_offset) / 65.5f; // deg/s

  // 2. Smooth Deadband Filter (ตัด Noise ตอนอยู่นิ่งโดยไม่เกิดรอยต่อกระตุก)
  if (fabs(raw_gyro_z) <= cfg.deadband) {
    raw_gyro_z = 0.0f;
  } else {
    raw_gyro_z = (raw_gyro_z > 0.0f) ? (raw_gyro_z - cfg.deadband) : (raw_gyro_z + cfg.deadband);
  }

  // 3. Low-Pass Filter (Damper)
  float damper_alpha = constrain(cfg.damper, 0.05f, 0.95f);
  filtered_gyro_z = (raw_gyro_z * damper_alpha) + (filtered_gyro_z * (1.0f - damper_alpha));
  if (raw_gyro_z == 0.0f && fabs(filtered_gyro_z) < 0.10f) {
    filtered_gyro_z = 0.0f; // ตัดเศษทศนิยมตอนจอดนิ่งให้เป็น 0.0 สนิท
  }

  // 4. Predictive D-Term (Angular Acceleration deg/s^2 with Non-Destructive Vibration Gate)
  float gyro_accel = (filtered_gyro_z - prev_gyro_z) / dt;
  prev_gyro_z = filtered_gyro_z;
  filtered_gyro_d = (gyro_accel * 0.35f) + (filtered_gyro_d * 0.65f);

  const float D_VIB_DEADBAND = 120.0f; // deg/s^2
  float d_effective = 0.0f;
  if (fabs(filtered_gyro_d) > D_VIB_DEADBAND) {
    d_effective = (filtered_gyro_d > 0.0f) ? (filtered_gyro_d - D_VIB_DEADBAND) : (filtered_gyro_d + D_VIB_DEADBAND);
  }

  // 5. CH3 Gain & Steering Priority
  float gain = getCH3Gain();
  float driver_ratio = (float)abs(steering_input) / 500.0f;
  float active_gain = gain * (1.0f - (driver_ratio * cfg.steering_priority));

  // 6. Dynamic Progressive Expo บนค่า Gyro (รองรับเพดาน ±500 deg/s สำหรับ Reverse Entry)
  float gyro_sign = (filtered_gyro_z >= 0.0f) ? 1.0f : -1.0f;
  float gyro_abs = fabs(filtered_gyro_z);
  float gyro_norm = gyro_abs / 350.0f;
  if (gyro_norm > 1.5f) gyro_norm = 1.5f;
  float expo_factor = cfg.s_curve * 0.45f;
  float curved_gyro = (1.0f - expo_factor) * gyro_norm + expo_factor * (gyro_norm * min(gyro_norm, 1.3f));
  float shaped_gyro_z = gyro_sign * (curved_gyro * 350.0f);

  // 7. Control Math: Pure Normal Rate Mode (P + D)
  float p_term = shaped_gyro_z * cfg.p_gain;
  float d_term = d_effective * (cfg.d_gain * 0.25f);
  float target_gyro_corr = (p_term + d_term) * active_gain;

  if (cfg.is_reversed) {
    target_gyro_corr = -target_gyro_corr;
  }

  // จำกัดระยะ Gyro Correction ให้อยู่ในขอบเขต EPA ป้องกัน Windup
  float max_travel = (float)max(abs(max_lim - servo_center), abs(servo_center - min_lim));
  target_gyro_corr = constrain(target_gyro_corr, -max_travel, max_travel);

  // 8. Asymmetric Slew Rate Limiter บน Gyro Correction เท่านั้น (ไม่หน่วงพวงมาลัยคนขับ)
  float diff_corr = target_gyro_corr - current_gyro_corr;
  if (fabs(diff_corr) < 0.6f) {
    current_gyro_corr = target_gyro_corr;
  } else {
    bool is_snapping = (target_gyro_corr * current_gyro_corr < 0.0f) ||
                       (fabs(target_gyro_corr) > fabs(current_gyro_corr));
    float speed_limit = is_snapping ? cfg.snap_speed : cfg.return_speed;
    float max_step = speed_limit * dt;
    current_gyro_corr += constrain(diff_corr, -max_step, max_step);
  }

  // 9. รวม Servo Center (1520 + Sub-Trim) + พวงมาลัยคนขับโดยตรง - แรงแก้ไจโร และคุม Endpoints
  float final_target = (float)servo_center + (float)steering_input - current_gyro_corr;
  final_target = constrain(final_target, (float)min_lim, (float)max_lim);
  current_servo_pos = final_target;

  writeServo((int)(final_target + 0.5f));
}

// ==========================================
// 12. HARDWARE BUTTON CALIBRATION MODE
// ==========================================
void blinkLED() {
  unsigned long now = millis();
  if (currentState == SET_LEFT_LIMIT) {
    digitalWrite(PIN_LED, (now / 400) % 2 == 0 ? LOW : HIGH);
  } else if (currentState == SET_RIGHT_LIMIT) {
    digitalWrite(PIN_LED, (now / 120) % 2 == 0 ? LOW : HIGH);
  }
}

void runSetupMode() {
  int servo_center = SERVO_NATIVE_CENTER + cfg.servo_trim;
  if (currentState == SET_LEFT_LIMIT || currentState == SET_RIGHT_LIMIT) {
    blinkLED();
    writeServo(constrain(ch1_pwm, 1000, 2000));
  } else if (currentState == SET_REVERSE) {
    if (ch1_pwm > 1600) {
      temp_reverse = true;
      digitalWrite(PIN_LED, LOW);
    } else if (ch1_pwm < 1400) {
      temp_reverse = false;
      digitalWrite(PIN_LED, HIGH);
    }
    writeServo(servo_center);
  }
}

void handleButton() {
  bool buttonState = (digitalRead(PIN_BUTTON) == LOW);
  if (buttonState && !isButtonPressed) {
    isButtonPressed = true;
    buttonPressTime = millis();
  } else if (!buttonState && isButtonPressed) {
    isButtonPressed = false;
    unsigned long pressDuration = millis() - buttonPressTime;

    if (currentState == NORMAL_MODE) {
      if (pressDuration > 6000) {
        // กดค้าง > 6s: รีเซ็ต Tuning PIN กลับเป็นค่าเริ่มต้น (1616) กรณีลืมรหัส
        strlcpy(cfg_pin, DEFAULT_TUNING_PIN, sizeof(cfg_pin));
        pending_save = true;
        blinkLEDWarning(4, 90);
        last_control_time = micros();
      } else if (pressDuration > 2000) {
        // กดค้าง > 2s: เข้าสู่โหมดตั้งค่า Limit/Reverse
        currentState = SET_LEFT_LIMIT;
      } else if (pressDuration > 50 && pressDuration < 800) {
        // กดสั้น (50-800ms): รีเซ็ต Zero Gyro ใหม่ได้ทันทีข้างสนาม
        pending_calibrate = true;
      }
    } else if (currentState == SET_LEFT_LIMIT && pressDuration > 50 && pressDuration < 1000) {
      cfg.limit_left = constrain(ch1_pwm, 1000, 2000);
      saveSettings();
      currentState = SET_RIGHT_LIMIT;
    } else if (currentState == SET_RIGHT_LIMIT && pressDuration > 50 && pressDuration < 1000) {
      cfg.limit_right = constrain(ch1_pwm, 1000, 2000);
      saveSettings();
      temp_reverse = cfg.is_reversed;
      currentState = SET_REVERSE;
    } else if (currentState == SET_REVERSE && pressDuration > 50 && pressDuration < 1000) {
      cfg.is_reversed = temp_reverse;
      saveSettings();
      digitalWrite(PIN_LED, LOW);
      delay(800);
      digitalWrite(PIN_LED, HIGH);
      last_control_time = micros();
      currentState = NORMAL_MODE;
    }
  }
}

// ==========================================
// 13. MAIN LOOP (Deterministic 250Hz)
// ==========================================
void loop() {
  if (pending_pwm_reinit) {
    pending_pwm_reinit = false;
    initServoPWM();
  }

  if (pending_save && (millis() - last_nvs_save_ms >= 400)) {
    pending_save = false;
    last_nvs_save_ms = millis();
    saveSettings();
    prev_gyro_z = filtered_gyro_z;
    last_control_time = micros();
  }

  if (pending_calibrate) {
    pending_calibrate = false;
    if (fabs(filtered_gyro_z) > 5.0f || abs(ch1_pwm - 1500) > 25) {
      Serial.println(F("[GYRO] Calibration ignored: car is moving/steering."));
      blinkLEDWarning(2, 50);
    } else {
      calibrateGyroOffset(false);
    }
    last_control_time = micros();
  }

  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - last_control_time;
  if (elapsed_us >= CONTROL_PERIOD_US) {
    float dt = (float)elapsed_us / 1000000.0f;
    last_control_time = now_us;
    if (dt <= 0.0f) dt = 0.004f;
    if (dt > 0.05f) dt = 0.05f;

    handleButton(); // ตรวจปุ่มกด BOOT บนบอร์ดทุก 4ms (250Hz)

    if (currentState == NORMAL_MODE) {
      runDriftMode(dt);
    } else {
      runSetupMode();
    }

    // ส่ง BLE Telemetry หลังจบรอบควบคุม (~6.7Hz / 150ms) เพื่อลดภาระ RF และไม่ขวางลูปไจโร
    if (ble_connected && (millis() - last_ble_notify_ms >= 150)) {
      last_ble_notify_ms = millis();
      notifyBLETelemetry();
    }
  } else {
    unsigned long remain_us = CONTROL_PERIOD_US - elapsed_us;
    if (remain_us > 1200) {
      delay(1); // คืนเวลาให้ FreeRTOS / BLE Task และลดความร้อน CPU
    } else {
      yield();
      delayMicroseconds(100);
    }
  }
}
