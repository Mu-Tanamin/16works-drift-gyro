/*
 * =====================================================================================
 *  PRO RC DRIFT GYROSCOPE CONTROLLER (V3.2 - PURE NORMAL RATE MODE)
 *  Target Board : ESP32-C3 Super Mini
 *  Sensor       : MPU6050 (I2C: SDA=GPIO6, SCL=GPIO7)
 * =====================================================================================
 *  ฟีเจอร์เด่น (Pure Normal Rate Mode P+D):
 *   1. Pure Normal Rate Mode: ควบคุมแบบธรรมชาติ พวงมาลัยคืนตัวตามสไตล์รถดริฟท์จริง
 *   2. CH3 Gain Control: ปรับระดับความไว (Gain) 0% - 100% จากรีโมทโดยตรง (1000 - 2000 µs)
 *   3. Asymmetric Slew Rate:
 *      - Fast Snap Speed: ล้อเด้งสวนแก้อาการทันที ป้องกันรถสปินหลุดไลน์
 *      - Smooth Return Speed: คืนพวงมาลัยนุ่มนวล ป้องกันอาการท้ายดีดกลับ (Tank-slapper)
 *   4. Dynamic S-Curve Expo: ตรงกลางนุ่มนวล แต่ดึงแน่นทวีคูณตอนสะบัดแรง
 *   5. Predictive D-Term: คาดเดาอาการลื่นไถลจากความเร่งการหมุน ล้อหน้าแก้ทันที
 *   6. Steering Priority: ลดทอนไจโรตอนคนขับเลี้ยวสุด ช่วยให้สลับไลน์ (Flick/Transition) ได้คล่องตัว
 *   7. BLE 5.0 Web Bluetooth Tuner: เชื่อมต่อผ่านมือถือประหยัดพลังงานสูง ไม่ต้องสลับ Wi-Fi
 *   8. รองรับเซอร์โว AFRC D2114TG-S ที่ 333Hz เต็มประสิทธิภาพ (หรือเลือก 50Hz)
 * =====================================================================================
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Preferences.h>
#include <esp_arduino_version.h>

// Forward Declarations
void readCH1();
void readCH3();
void initServoPWM();
void writeServo(int pwm_us);
void loadDefaults();
void loadSettings();
void saveSettings();
void calibrateGyroOffset();
void setupBLE();
void updateBLEConfigValue();
void notifyBLETelemetry();
void parseBLECommand(const String& body);
void runDriftMode();
void blinkLED();
void runSetupMode();
void handleButton();

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

MPU6050 mpu;
Preferences preferences;

#define BLE_SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_STATUS_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_CONFIG_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

BLEServer* pServer = nullptr;
BLECharacteristic* pStatusChar = nullptr;
BLECharacteristic* pConfigChar = nullptr;
volatile bool ble_connected = false;
volatile bool pending_calibrate = false;
unsigned long last_ble_notify_ms = 0;

// ==========================================
// 3. RC SIGNAL VARIABLES & ROCK-SOLID ANTI-JITTER FILTER
// ==========================================
volatile unsigned long ch1_start_time = 0;
volatile unsigned long ch3_start_time = 0;
volatile int ch1_buf[5] = {1500, 1500, 1500, 1500, 1500};
volatile int ch3_buf[5] = {1500, 1500, 1500, 1500, 1500};
volatile uint8_t ch1_idx = 0;
volatile uint8_t ch3_idx = 0;
volatile float ch1_filtered = 1500.0f;
volatile float ch3_filtered = 1500.0f;

volatile int ch1_pwm = 1500;
volatile int ch3_pwm = 1500;
volatile unsigned long last_ch1_time = 0;
volatile unsigned long last_ch3_time = 0;

// ฟังก์ชันหาค่ามัธยฐาน 5 จุด (5-Point Fast Median) ตัด Glitch & Timer Jitter
static inline int getMedian5(volatile int* buf) {
  int arr[5];
  for (int i = 0; i < 5; i++) arr[i] = buf[i];
  for (int i = 1; i < 5; i++) {
    int key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  return arr[2];
}

void IRAM_ATTR readCH1() {
  unsigned long now = micros();
  if (digitalRead(PIN_CH1_IN) == HIGH) {
    ch1_start_time = now;
  } else {
    unsigned long pulse = now - ch1_start_time;
    if (pulse >= 850 && pulse <= 2150) {
      ch1_buf[ch1_idx] = (int)pulse;
      ch1_idx = (ch1_idx + 1) % 5;
      int med = getMedian5(ch1_buf);

      // Fast EMA Smoothing (α = 0.35)
      ch1_filtered = (ch1_filtered * 0.65f) + ((float)med * 0.35f);
      int target_pwm = (int)(ch1_filtered + 0.5f);

      // Center Snap: ถ้าอยู่ใกล้จุดศูนย์กลาง ±4 µs ล็อกเป็น 1500 µs ทันที
      if (abs(target_pwm - 1500) <= 4) {
        target_pwm = 1500;
        ch1_filtered = 1500.0f;
      }

      // Hysteresis Deadband (3 µs): ถ้าขยับไม่เกิน 3 µs ไม่เปลี่ยนค่า
      if (abs(target_pwm - ch1_pwm) >= 3 || target_pwm == 1500) {
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
      ch3_idx = (ch3_idx + 1) % 5;
      int med = getMedian5(ch3_buf);

      // Heavy EMA Smoothing บน Gain Knob (α = 0.20)
      ch3_filtered = (ch3_filtered * 0.80f) + ((float)med * 0.20f);
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

float current_servo_pos = 1520.0f;
unsigned long last_control_time = 0;

const int SERVO_NATIVE_CENTER = 1520; // สเปกโรงงาน AFRC D2114TG-S คือ 1520 µs
const int CENTER_PWM = 1500;          // ค่ากลางพัลส์มาตรฐานวิทยุ (1500 µs)
const int SERVO_TIMER_RES_BITS = 14;
const int SERVO_MAX_DUTY = (1 << SERVO_TIMER_RES_BITS) - 1; // 16383

// State Machine สำหรับปุ่มฮาร์ดแวร์
enum SystemState { CALIBRATING, NORMAL_MODE, SET_LEFT_LIMIT, SET_RIGHT_LIMIT, SET_REVERSE };
SystemState currentState = CALIBRATING;
unsigned long buttonPressTime = 0;
bool isButtonPressed = false;
bool temp_reverse = false;

// ==========================================
// 5. HARDWARE SERVO PWM ABSTRACTION
// ==========================================
void initServoPWM() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PIN_SERVO_OUT, cfg.servo_freq, SERVO_TIMER_RES_BITS);
#else
  ledcSetup(0, cfg.servo_freq, SERVO_TIMER_RES_BITS);
  ledcAttachPin(PIN_SERVO_OUT, 0);
#endif
}

void writeServo(int pwm_us) {
  pwm_us = constrain(pwm_us, 800, 2200);
  float period_us = 1000000.0f / (float)cfg.servo_freq;
  uint32_t duty = (uint32_t)(((float)pwm_us / period_us) * SERVO_MAX_DUTY);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_SERVO_OUT, duty);
#else
  ledcWrite(0, duty);
#endif
}

// ==========================================
// 6. PREFERENCES & DEFAULT CONFIG
// ==========================================
void loadDefaults() {
  cfg.p_gain            = 3.2f;
  cfg.d_gain            = 0.15f;
  cfg.s_curve           = 0.4f;
  cfg.steering_priority = 0.5f;
  cfg.snap_speed        = 2800.0f;
  cfg.return_speed      = 1600.0f;
  cfg.damper            = 0.35f;
  cfg.deadband          = 0.5f;
  cfg.limit_left        = 1100;
  cfg.limit_right       = 1900;
  cfg.is_reversed       = false;
  cfg.servo_freq        = 333;
  cfg.servo_trim        = 0;
}

void loadSettings() {
  preferences.begin("gyro_pro", false);
  cfg.p_gain            = preferences.getFloat("p_gain", 3.2f);
  cfg.d_gain            = preferences.getFloat("d_gain", 0.15f);
  cfg.s_curve           = preferences.getFloat("s_curve", 0.4f);
  cfg.steering_priority = preferences.getFloat("st_prio", 0.5f);
  cfg.snap_speed        = preferences.getFloat("snap_spd", 2800.0f);
  cfg.return_speed      = preferences.getFloat("ret_spd", 1600.0f);
  cfg.damper            = preferences.getFloat("damper", 0.35f);
  cfg.deadband          = preferences.getFloat("deadband", 0.5f);
  cfg.limit_left        = preferences.getInt("lim_l", 1100);
  cfg.limit_right       = preferences.getInt("lim_r", 1900);
  cfg.is_reversed       = preferences.getBool("reversed", false);
  cfg.servo_freq        = preferences.getInt("freq", 333);
  cfg.servo_trim        = preferences.getInt("servo_trim", 0);
}

void saveSettings() {
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
}

// ==========================================
// 7. AUTO GYRO OFFSET CALIBRATION
// ==========================================
void calibrateGyroOffset() {
  Serial.println(F("[GYRO] Calibrating Zero-Offset... Keep car still!"));
  long sum_z = 0;
  const int SAMPLES = 300;

  for (int i = 0; i < SAMPLES; i++) {
    sum_z += mpu.getRotationZ();
    digitalWrite(PIN_LED, (i % 20 < 10) ? LOW : HIGH);
    delay(5);
  }
  gyro_z_offset = (float)sum_z / (float)SAMPLES;

  digitalWrite(PIN_LED, HIGH);
  Serial.printf("[GYRO] Offset: %.2f\n", gyro_z_offset);
}

// ==========================================
// 8. BLE 5.0 GATT SERVER & WEB BLUETOOTH API
// ==========================================
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
  char json[300];
  snprintf(json, sizeof(json),
    "{\"p_gain\":%.2f,\"d_gain\":%.2f,\"s_curve\":%.2f,\"st_prio\":%.2f,"
    "\"snap_spd\":%.1f,\"ret_spd\":%.1f,\"damper\":%.2f,\"deadband\":%.2f,"
    "\"lim_l\":%d,\"lim_r\":%d,\"reversed\":%s,\"freq\":%d,\"servo_trim\":%d}",
    cfg.p_gain, cfg.d_gain, cfg.s_curve, cfg.steering_priority,
    cfg.snap_speed, cfg.return_speed, cfg.damper, cfg.deadband,
    cfg.limit_left, cfg.limit_right, cfg.is_reversed ? "true" : "false", cfg.servo_freq,
    cfg.servo_trim);
  pConfigChar->setValue((uint8_t*)json, strlen(json));
}

void parseBLECommand(const String& body) {
  if (body.indexOf("\"cmd\":\"cal\"") != -1) {
    pending_calibrate = true;
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
  if (p_idx != -1) cfg.p_gain = body.substring(p_idx + 9).toFloat();

  int d_idx = body.indexOf("\"d_gain\":");
  if (d_idx != -1) cfg.d_gain = body.substring(d_idx + 9).toFloat();

  int s_idx = body.indexOf("\"s_curve\":");
  if (s_idx != -1) cfg.s_curve = body.substring(s_idx + 10).toFloat();

  int st_idx = body.indexOf("\"st_prio\":");
  if (st_idx != -1) cfg.steering_priority = body.substring(st_idx + 10).toFloat();

  int snap_idx = body.indexOf("\"snap_spd\":");
  if (snap_idx != -1) cfg.snap_speed = body.substring(snap_idx + 11).toFloat();

  int ret_idx = body.indexOf("\"ret_spd\":");
  if (ret_idx != -1) cfg.return_speed = body.substring(ret_idx + 10).toFloat();

  int dmp_idx = body.indexOf("\"damper\":");
  if (dmp_idx != -1) cfg.damper = body.substring(dmp_idx + 9).toFloat();

  int db_idx = body.indexOf("\"deadband\":");
  if (db_idx != -1) cfg.deadband = body.substring(db_idx + 11).toFloat();

  int ll_idx = body.indexOf("\"lim_l\":");
  if (ll_idx != -1) cfg.limit_left = body.substring(ll_idx + 8).toInt();

  int lr_idx = body.indexOf("\"lim_r\":");
  if (lr_idx != -1) cfg.limit_right = body.substring(lr_idx + 8).toInt();

  int rev_idx = body.indexOf("\"reversed\":");
  if (rev_idx != -1) cfg.is_reversed = body.substring(rev_idx + 11).startsWith("true");

  int trim_idx = body.indexOf("\"servo_trim\":");
  if (trim_idx != -1) cfg.servo_trim = constrain(body.substring(trim_idx + 13).toInt(), -150, 150);

  int frq_idx = body.indexOf("\"freq\":");
  if (frq_idx != -1) {
    int old_freq = cfg.servo_freq;
    cfg.servo_freq = body.substring(frq_idx + 7).toInt();
    if (old_freq != cfg.servo_freq) initServoPWM();
  }

  saveSettings();
  updateBLEConfigValue();
}

void notifyBLETelemetry() {
  if (!ble_connected || !pStatusChar) return;

  int ch3_val = constrain(ch3_pwm, 1000, 2000);
  int ch3_offset = abs(ch3_val - 1500);
  if (ch3_offset <= 10) ch3_offset = 0;

  float gain = (float)ch3_offset / 490.0f;
  if (gain > 1.0f) gain = 1.0f;

  float disp_gyro = (fabs(filtered_gyro_z) < 0.05f) ? 0.0f : filtered_gyro_z;

  char json[180];
  snprintf(json, sizeof(json),
    "{\"ch1\":%d,\"ch3\":%d,\"servo\":%d,\"gyro\":%.2f,\"gain\":%.2f,\"trim\":%d}",
    ch1_pwm, ch3_pwm, (int)(current_servo_pos + 0.5f), disp_gyro, gain, cfg.servo_trim);
  pStatusChar->setValue((uint8_t*)json, strlen(json));
  pStatusChar->notify();
}

void setupBLE() {
  BLEDevice::init("16WORKs-GYRO");
  BLEDevice::setPower(ESP_PWR_LVL_N0); // 0 dBm ประหยัดพลังงานสูงสุด เย็นเฉียบตลอดการขับขี่

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new GyroServerCallbacks());

  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  pStatusChar = pService->createCharacteristic(
    BLE_CHAR_STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusChar->addDescriptor(new BLE2902());

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
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println(F("[BLE] Advertising Started: 16WORKs-GYRO (Ultra-Low-Power Mode)"));
}

// ==========================================
// 9. SETUP FUNCTION
// ==========================================
void setup() {
  // 1. ลดความถี่ CPU จาก 160MHz เหลือ 80MHz (ประหยัดไฟและลดความร้อนลงทันที 40%)
  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=============================================="));
  Serial.println(F(" ESP32-C3 Super Mini - Drift Gyro (BLE Edition)"));
  Serial.println(F("=============================================="));

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // 2. I2C Bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println(F("[OK] MPU6050 Connected."));
    // 1) เปิด Hardware Digital Low Pass Filter (DLPF) 20Hz เพื่อตัดการสั่นสะเทือนจากมอเตอร์รอบจัดตอนกำคันเร่งเต็ม
    mpu.setDLPFMode(4); // 4 = MPU6050_DLPF_BW_20 (Gyro Cutoff 20Hz - นิ่งสนิทตอนกดคันเร่งเต็ม)
    // 2) ตั้งค่า Full Scale Gyro Range ±500 deg/s (65.5 LSB/deg/s) สมดุลความละเอียดสูง + รองรับ Reverse Entry
    mpu.setFullScaleGyroRange(1);
  } else {
    Serial.println(F("[WARN] MPU6050 connection error. Check wiring SDA=6, SCL=7"));
  }

  // 3. โหลด Settings
  loadSettings();

  // 4. เริ่มต้น PWM
  initServoPWM();
  writeServo(SERVO_NATIVE_CENTER + cfg.servo_trim);

  // 5. ตั้งค่า Interrupt รับสัญญาณ RC
  pinMode(PIN_CH1_IN, INPUT_PULLUP);
  pinMode(PIN_CH3_IN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CH1_IN), readCH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH3_IN), readCH3, CHANGE);

  // 6. เริ่มต้นระบบ BLE 5.0 ประหยัดพลังงาน
  setupBLE();

  // 7. Calibrate Zero-Offset
  calibrateGyroOffset();

  last_control_time = micros();
  currentState = NORMAL_MODE;
}

// ==========================================
// 12. DRIFT CONTROL ALGORITHM (PURE NORMAL RATE P+D)
// ==========================================
void runDriftMode() {
  int servo_center = SERVO_NATIVE_CENTER + cfg.servo_trim;

  // Failsafe: รีโมทหลุดเกิน 200ms
  bool signal_lost = (millis() - last_ch1_time > 200);
  if (signal_lost) {
    writeServo(servo_center);
    digitalWrite(PIN_LED, (millis() / 200) % 2 == 0 ? LOW : HIGH);
    return;
  } else {
    digitalWrite(PIN_LED, HIGH);
  }

  // 1. อ่านค่า Gyro Z + Offset (Full Scale ±500 deg/s -> 65.5 LSB/deg/s)
  float raw_gyro_z = ((float)mpu.getRotationZ() - gyro_z_offset) / 65.5f; // deg/s

  // 2. Deadband Filter (ตัด Noise ตอนอยู่นิ่ง)
  if (fabs(raw_gyro_z) < cfg.deadband) {
    raw_gyro_z = 0.0f;
  }

  // 3. Low-Pass Filter (Damper)
  filtered_gyro_z = (raw_gyro_z * cfg.damper) + (filtered_gyro_z * (1.0f - cfg.damper));
  if (fabs(filtered_gyro_z) < 0.15f) {
    filtered_gyro_z = 0.0f; // ตัดเศษทศนิยมที่ค้างอยู่นิ่งๆ ให้เป็น 0.0 สนิท
  }

  // 4. D-Term (Predictive Angular Acceleration with D-Term Low-Pass Filter & Vibration Deadband)
  // กรอง D-Term แยกต่างหาก และตัดแรงสั่นสะเทือนระดับ Micro-ripple ตอนกดคันเร่งเต็ม
  float raw_gyro_d = (filtered_gyro_z - prev_gyro_z);
  prev_gyro_z = filtered_gyro_z;
  filtered_gyro_d = (raw_gyro_d * 0.12f) + (filtered_gyro_d * 0.88f);
  if (fabs(filtered_gyro_d) < 0.45f) {
    filtered_gyro_d = 0.0f; // ตัดการสั่นสะเทือนความถี่สูงจากมอเตอร์ตอนกดคันเร่งเต็ม
  }

  // 5. RC Inputs
  // CH3: กึ่งกลาง (1500 µs หรือเลข 0 บนรีโมท) = Gain 0% (ปิดไจโร)
  // หมุนไปฝั่งบวก (>1500 ถึง 2000 µs) หรือฝั่งลบ (<1500 ถึง 1000 µs) จะเพิ่ม Gain 0% - 100% เท่ากันทั้งสองฝั่ง
  int ch3_val = constrain(ch3_pwm, 1000, 2000);
  int ch3_offset = abs(ch3_val - 1500);
  if (ch3_offset <= 10) ch3_offset = 0; // Deadband ที่จุดกึ่งกลาง 1500 µs
  float gain = (float)ch3_offset / 490.0f;
  if (gain > 1.0f) gain = 1.0f;

  // Steering Input คำนวณเทียบกับจุดศูนย์กลางของรีโมทมาตรฐาน (1500 µs)
  int raw_st = constrain(ch1_pwm, 1000, 2000) - 1500;
  // Steering Center Deadband (3 µs) ตัด Noise สัญญาณรีโมทแกว่ง เพื่อให้ตอนจอดนิ่งๆ เซอร์โวหยุดทำงานสนิท ไม่กระตุก
  if (abs(raw_st) <= 3) {
    raw_st = 0;
  }
  int steering_input = raw_st;

  // 6. Steering Priority (ลดทอนไจโรตอนคนขับหักเลี้ยวสุด)
  float driver_ratio = (float)abs(steering_input) / 500.0f;
  float active_gain = gain * (1.0f - (driver_ratio * cfg.steering_priority));

  // 7. Dynamic Progressive Expo บนค่า Gyro (รองรับเพดาน ±500 deg/s สำหรับ Reverse Entry)
  float gyro_sign = (filtered_gyro_z >= 0) ? 1.0f : -1.0f;
  float gyro_abs = fabs(filtered_gyro_z);
  float gyro_norm = gyro_abs / 350.0f;
  if (gyro_norm > 1.5f) gyro_norm = 1.5f;
  // ใช้ Quadratic Progressive Blend รักษาความนุ่มกลางโค้ง และอัดแรงดึงเพิ่มทวีคูณเมื่อสะบัดมุมลึก
  float expo_factor = cfg.s_curve * 0.45f;
  float curved_gyro = (1.0f - expo_factor) * gyro_norm + expo_factor * (gyro_norm * min(gyro_norm, 1.3f));
  float shaped_gyro_z = gyro_sign * (curved_gyro * 350.0f);

  // 8. Control Math: Pure Normal Rate Mode (P + D)
  float correction = (shaped_gyro_z * cfg.p_gain + filtered_gyro_d * cfg.d_gain * 10.0f) * active_gain;

  // กลับทิศทาง Gyro
  if (cfg.is_reversed) {
    correction = -correction;
  }

  // 9. Target Servo Output & Safe Endpoints
  // รวม Servo Center (1520 + Sub-Trim) + พวงมาลัยคนขับ - แรงแก้ไจโร
  float raw_target = servo_center + steering_input - correction;
  int min_lim = min(cfg.limit_left, cfg.limit_right);
  int max_lim = max(cfg.limit_left, cfg.limit_right);
  raw_target = constrain(raw_target, min_lim, max_lim);

  // 10. Asymmetric Slew Rate Limiter (Snap Out / Smooth Return)
  unsigned long now = micros();
  float dt = (now - last_control_time) / 1000000.0f;
  last_control_time = now;
  if (dt > 0.05f) dt = 0.004f;

  float diff = raw_target - current_servo_pos;
  if (fabs(diff) < 0.6f) {
    current_servo_pos = raw_target; // Snap to target เมื่อเข้าใกล้ในระดับ sub-microsecond
  } else {
    bool is_counter_steering = (raw_target - servo_center) * diff > 0;
    float speed_limit = is_counter_steering ? cfg.snap_speed : cfg.return_speed;
    float max_step = speed_limit * dt;
    float constrained_step = constrain(diff, -max_step, max_step);
    current_servo_pos += constrained_step;
  }

  // Output Hysteresis: ป้องกันเซอร์โวสั่นกระตุกจากการปัดเศษ microsecond
  static int last_written_pwm = 1520;
  int target_write_pwm = (int)(current_servo_pos + 0.5f);
  if (abs(target_write_pwm - last_written_pwm) >= 1 || target_write_pwm == (SERVO_NATIVE_CENTER + cfg.servo_trim)) {
    last_written_pwm = target_write_pwm;
  }

  writeServo(last_written_pwm);
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
      if (pressDuration > 2000) {
        // กดค้าง > 2s: เข้าสู่โหมดตั้งค่า Limit/Reverse
        currentState = SET_LEFT_LIMIT;
      } else if (pressDuration > 50 && pressDuration < 800) {
        // กดสั้น (50-800ms): รีเซ็ต Zero Gyro ใหม่ได้ทันทีข้างสนาม
        pending_calibrate = true;
      }
    } else if (currentState == SET_LEFT_LIMIT && pressDuration > 50 && pressDuration < 1000) {
      cfg.limit_left = constrain(ch1_pwm, 800, 2200);
      saveSettings();
      currentState = SET_RIGHT_LIMIT;
    } else if (currentState == SET_RIGHT_LIMIT && pressDuration > 50 && pressDuration < 1000) {
      cfg.limit_right = constrain(ch1_pwm, 800, 2200);
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
  if (pending_calibrate) {
    pending_calibrate = false;
    calibrateGyroOffset();
    last_control_time = micros();
  }

  // ส่งข้อมูล Telemetry ผ่าน BLE Notification ทุกๆ 100ms (10Hz) เมื่อมีมือถือเชื่อมต่ออยู่
  if (ble_connected && (millis() - last_ble_notify_ms >= 100)) {
    last_ble_notify_ms = millis();
    notifyBLETelemetry();
  }

  handleButton(); // จัดการปุ่มกด BOOT บนบอร์ด

  if (currentState == NORMAL_MODE) {
    runDriftMode();
  } else {
    runSetupMode();
  }

  delay(4); // 250Hz Control Loop Rate
}
