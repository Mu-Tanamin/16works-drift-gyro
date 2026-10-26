# ESP32-C3 RC Drift Gyroscope - Workspace Guidelines & Rules

โปรเจกต์นี้คือระบบควบคุมไจโรสโคปช่วยดริฟท์สำหรับรถบังคับขนาดเล็ก (Micro RC Drift 1/24 - 1/28)

## 1. Hardware Pinout & Configuration Invariants
- **MCU Board**: ESP32-C3 Super Mini (RISC-V 160MHz, Flash 4MB)
- **Sensor**: MPU6050 เชื่อมต่อผ่าน I2C:
  - `SDA`: GPIO 6
  - `SCL`: GPIO 7
  - I2C Clock: 400kHz
- **RC Receiver Inputs**:
  - `CH1` (Steering Input): GPIO 2 (Interrupt CHANGE)
  - `CH3` (Gyro Gain): GPIO 3 (Interrupt CHANGE)
- **Actuator (Servo)**:
  - `Servo Signal`: GPIO 4
  - Servo Model: **AFRC D2114TG-S** (High-Speed Coreless Micro Digital Servo)
  - Servo Working Frequency: **333 Hz** (Period ~3003 µs, 14-bit resolution)
- **Onboard UI**:
  - `Status LED`: GPIO 8 (Active LOW, Built-in Blue LED)
  - `BOOT Button`: GPIO 9 (Active LOW, Built-in Switch with Internal Pullup)

## 2. Control System Constraints
- **Mode Policy**: ใช้ **Pure Normal Rate Mode (P + D) เท่านั้น**
  - **ห้ามใส่โหมด AVCS / Heading Hold / Integral I-Gain กลับเข้ามา**
- **CH3 Gain Mapping**:
  - สัญญาณพัลส์ CH3 (1000 - 2000 µs) กึ่งกลาง 1500 µs (เลข 0 บนรีโมท) = 0% (ปิดไจโร)
  - หมุนไปฝั่งบวก (>1500 µs) หรือฝั่งลบ (<1500 µs) เพิ่ม Gain 0% - 100% เท่ากันทั้งสองฝั่ง (`gain = abs(ch3 - 1500) / 500.0f`)
- **Micro Drift Dynamics**:
  - ต้องคงระบบ **Asymmetric Slew Rate** (Snap Speed เด้งสวนเร็วเพื่อกันสปิน vs Return Speed คืนล้อนุ่มนวลเพื่อกันท้ายดีด)
  - ต้องคง **Predictive D-Term** (คำนวณความเร่งการหมุน), **Steering Priority** (ลดทอนไจโรตอนคนขับเลี้ยวสุด) และ **S-Curve Expo**
  - ต้องมี **Damper Filter** และ **Deadband** เพื่อตัด Noise ป้องกันเซอร์โวสั่นครางบนแชสซีรถเล็ก
  - **Direct Steering & Sub-Trim Rule**: รถ Direct Steering ให้ปรับชดเชยศูนย์ล้อหน้าด้วย **Servo Sub-Trim ในกล่องไจโร (`cfg.servo_trim`)** เท่านั้น เพื่อไม่ให้ Steering Priority บนรีโมทเพี้ยน
  - **Anti-Jitter & Vibration Filters**: ต้องมี RC Input EMA Filter + Steering Center Deadband (±3 µs) เพื่อป้องกันเซอร์โวกระตุกตอนจอดนิ่ง และเปิด MPU6050 Hardware DLPF (42Hz) + D-Term Low-Pass Filter เพื่อป้องกันเซอร์โวส่ายตอนกดคันเร่งเต็ม
  - **CH3 Invariant**: ค่ากลางของ CH3 ถูกล็อกตายตัวไว้ที่ **1500 µs** เสมอ (ห้าม Auto-Capture หรือเปลี่ยนแปลง) เพื่อให้เลข 0 บนรีโมทตรงกับ Gain 0% ตลอดเวลา
  - เซอร์โว AFRC D2114TG-S วาง Native Center ไว้ที่ **1520 µs** + `servo_trim`

## 3. Coding & Arduino IDE Standards for ESP32 Core 3.x
- **Forward Declarations**: ต้องประกาศ Function Prototypes ไว้ที่ส่วนบนของไฟล์ `.ino` ทุกครั้งเพื่อป้องกันข้อผิดพลาด `'func' was not declared in this scope`
- **LEDC Dual-Core API**: ต้องเขียน `#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)` เพื่อรองรับทั้ง Core 2.x (`ledcSetup`/`ledcAttachPin`) และ Core 3.x (`ledcAttach`)
- **Arduino IDE Settings Reminder**:
  - Board: `ESP32C3 Dev Module`
  - Partition Scheme: `Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)` (ห้ามปล่อยว่าง เพื่อป้องกัน build.partitions.csv error)
  - USB CDC On Boot: `Enabled`
- **Thermal & Power Management**:
  - กำหนดความถี่ CPU เป็น **80 MHz** (`setCpuFrequencyMhz(80)`) เพื่อลดความร้อนบนบอร์ด Super Mini
  - Wi-Fi ต้องจำกัดกำลังส่งที่ **8.5 dBm** (`WiFi.setTxPower(WIFI_POWER_8_5dBm)`) และมีระบบ Auto-Sleep / Button Toggle เพื่อดับ Wi-Fi ขณะขับขี่
