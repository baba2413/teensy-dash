#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t ACT_ID_1 = 1;      // 슬레이브 모터 (따라 움직이는 모터)
const uint8_t ACT_ID_2 = 127;    // 마스터 모터 (사람이 손으로 움직이는 모터)
const uint8_t HOST_ID = 253;     // Teensy(호스트) ID

// Teensy 4.0/4.1의 CAN1 핀 사용
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// -------------------------------------------------------------
// 2. Robstride 프로토콜 물리적 제한 한계값
// -------------------------------------------------------------
const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -45.0f;
const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;
const float T_MAX = 18.0f;

// -------------------------------------------------------------
// 3. 제어 주기 설정 (텔레오퍼레이션용 500 Hz / dt = 0.002초)
// -------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 2000; // dt = 0.002s (2,000us = 500Hz)
elapsedMicros controlTimer;

// -------------------------------------------------------------
// 4. 실시간 상태 및 오프셋 변수
// -------------------------------------------------------------
volatile float master_pos = 0.0f;
volatile float slave_pos = 0.0f;
volatile float slave_trq = 0.0f; // 슬레이브 실시간 토크 피드백

float pos_offset = 0.0f; // 초기 마스터-슬레이브 각도 차이 (오프셋)

// -------------------------------------------------------------
// 5. 데이터 스케일링 헬퍼 함수
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

// -------------------------------------------------------------
// 6. Robstride CAN 송신 제어 함수군
// -------------------------------------------------------------

// 모터 활성화
void enableMotor(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12 << 24) | (HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  
  mode_msg.buf[0] = 0x05; 
  mode_msg.buf[1] = 0x70; 
  mode_msg.buf[2] = 0x00;
  mode_msg.buf[3] = 0x00;
  mode_msg.buf[4] = 0x00; 
  mode_msg.buf[5] = 0x00;
  mode_msg.buf[6] = 0x00;
  mode_msg.buf[7] = 0x00;
  
  Can0.write(mode_msg);
  delay(50); 

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3 << 24) | (HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  for (int i = 0; i < 8; i++) enable_msg.buf[i] = 0;
  
  Can0.write(enable_msg);

  Serial.printf("[Teensy] Motor %d Initialized & Enabled successfully!\r\n", motor_id);
}

// 모터 비활성화
void disableMotor(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4 << 24) | (HOST_ID << 8) | motor_id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;
  
  Can0.write(msg);
}

// 실시간 운전 제어
CAN_message_t operationControl(uint8_t motor_id, float feed_forward, float pos, float vel, float kp, float kd) {
  uint16_t p_int  = floatToUint(pos,          P_MIN, P_MAX,  16);
  uint16_t v_int  = floatToUint(vel,          V_MIN, V_MAX,  16);
  uint16_t kp_int = floatToUint(kp,           0.0f,  KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd,           0.0f,  KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX,  16);

  CAN_message_t msg;
  msg.flags.extended = 1;
  
  msg.id = (1 << 24) | (t_int << 8) | motor_id;
  msg.len = 8;
  
  msg.buf[0] = (p_int >> 8) & 0xFF;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;
  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF;
  msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF;
  msg.buf[7] = kd_int & 0xFF;

  Can0.write(msg);
  return msg;
}

// -------------------------------------------------------------
// 7. CAN 수신 인터럽트 콜백
// -------------------------------------------------------------
void rxCallback(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;
  
  if (mode == 2) {
    uint8_t motor_id = (msg.id >> 8) & 0xFF; 
    uint16_t p_raw = (msg.buf[0] << 8) | msg.buf[1];
    uint16_t t_raw = (msg.buf[4] << 8) | msg.buf[5]; // 토크 Raw 데이터

    if (motor_id == ACT_ID_2) {       // 마스터(ID 127)
      master_pos = uintToFloat(p_raw, P_MIN, P_MAX);
    } 
    else if (motor_id == ACT_ID_1) {  // 슬레이브(ID 1)
      slave_pos = uintToFloat(p_raw, P_MIN, P_MAX);
      slave_trq = uintToFloat(t_raw, T_MIN, T_MAX); // 슬레이브 토크 읽기
    }
  }
}

// -------------------------------------------------------------
// 8. 초기 위치 오프셋 측정 헬퍼 함수
// -------------------------------------------------------------
void setupOffset() {
  // 모터 피드백 유도를 위한 Dummy 명령 전송
  operationControl(ACT_ID_2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  operationControl(ACT_ID_1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  
  // 피드백 데이터 수신 대기 (100ms)
  uint32_t waitStart = millis();
  while (millis() - waitStart < 100) {
    Can0.events();
  }

  // 초기 오프셋 계산 (pos_offset = slave - master)
  pos_offset = slave_pos - master_pos;
  Serial.printf("[SETUP] Initial Offset Calculated: %.3f rad (Master: %.3f, Slave: %.3f)\r\n", 
                pos_offset, master_pos, slave_pos);
}

// -------------------------------------------------------------
// 9. 메인 루프 구조
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); 

  Serial.println("=== Robstride Teleoperation (Position-Torque with Initial Offset) ===");

  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps
  Can0.setMaxMB(64);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();
  Can0.enableMBInterrupts();
  Can0.onReceive(rxCallback);

  Serial.println("Teensy CAN initialized.");
  delay(1000);

  // 마스터(ID 127) 및 슬레이브(ID 1) 모터 모두 활성화
  enableMotor(ACT_ID_2); // ID 127
  delay(100);
  enableMotor(ACT_ID_1); // ID 1

  // 초기 위치 오프셋 계산 실행
  setupOffset();

  controlTimer = 0;
}

void loop() {
  Can0.events(); // CAN 수신 이벤트 처리

  // 2ms 주기 제어 루프 (500Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // 1. 슬레이브 모터(ID 1) : 오프셋이 적용된 마스터 위치 추종
    float slave_target_pos = master_pos + pos_offset;
    float slave_kp = 25.0f; // 슬레이브 위치 추종 강도
    float slave_kd = 1.0f;
    operationControl(ACT_ID_1, 0.0f, slave_target_pos, 0.0f, slave_kp, slave_kd);

    // 2. 마스터 모터(ID 127) : 위치 저항(Kp=0)은 끄고, 슬레이브의 토크만 손으로 반력 피드백
    // (슬레이브 토크 반대 방향으로 피드백 토크 설정, 스케일 0.5f 적용)
    float filtered_trq = slave_trq;
    if (fabsf(filtered_trq) < 0.15f) {
      filtered_trq = 0.0f;
    } else if (filtered_trq > 0) {
      filtered_trq -= 0.15f;
    } else {
      filtered_trq += 0.15f;
    }

    float feedback_torque = -1.0f * filtered_trq * 0.5f;
    float master_kd = 0.0f; // 최소한의 손떨림 방지 댐핑만 유지
    operationControl(ACT_ID_2, feedback_torque, 0.0f, 0.0f, 0.0f, master_kd);

    // 500ms마다 상태 모니터링 출력
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      
      Serial.printf("[P-T TELEOP] Master Pos: %.3f rad | Slave Pos: %.3f rad (Target: %.3f) | Slave Trq: %.2f Nm | FB Trq: %.2f Nm\r\n", 
                    master_pos, slave_pos, slave_target_pos, slave_trq, feedback_torque);
    }
  }
}

void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd' || ch == 'D') {
      disableMotor(ACT_ID_1);
      delay(100);
      disableMotor(ACT_ID_2);
      Serial.println("[Teensy] Motors Disabled.");
    } else if (ch == 'e' || ch == 'E') {
      enableMotor(ACT_ID_2);
      delay(100);
      enableMotor(ACT_ID_1);
      setupOffset(); // 재활성화 시 오프셋 재계산
      Serial.println("[Teensy] Motors Enabled & Offset Reset.");
    }
  }
}