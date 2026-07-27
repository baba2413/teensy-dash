#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// -------------------------------------------------------------
// 1. 모터 및 통신 설정 파라미터
// -------------------------------------------------------------
const uint8_t ACT_ID_1 = 1;   // 슬레이브 모터 (따라 움직이는 모터)
const uint8_t ACT_ID_2 = 127;     // 마스터 모터 (사람이 손으로 움직이는 모터)
const uint8_t HOST_ID = 253;    // Teensy(호스트) ID

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

// 마스터 모터(ID 1)의 현재 위치를 실시간 저장하는 전역 변수
volatile float master_pos = 0.0f;

// -------------------------------------------------------------
// 4. 데이터 스케일링 및 가상 시리얼 변환 헬퍼 함수
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

float uintToFloat(uint16_t x, float x_min, float x_max) {
  return x_min + (float)x * (x_max - x_min) / 65535.0f;
}

void printAsSerialPacket(const CAN_message_t &msg, const char* prefix) {
  uint32_t header_shifted = (msg.id << 3) | 4;
  
  uint8_t h0 = (header_shifted >> 24) & 0xFF;
  uint8_t h1 = (header_shifted >> 16) & 0xFF;
  uint8_t h2 = (header_shifted >> 8) & 0xFF;
  uint8_t h3 = header_shifted & 0xFF;

  Serial.printf("[%s] 41 54 %02X %02X %02X %02X 08 %02X %02X %02X %02X %02X %02X %02X %02X 0D 0A\r\n",
                prefix, h0, h1, h2, h3,
                msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3],
                msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
}

// -------------------------------------------------------------
// 5. Robstride CAN 송신 제어 함수군
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
// 6. CAN 수신 인터럽트 콜백 (마스터 모터 위치 수신)
// -------------------------------------------------------------
void rxCallback(const CAN_message_t &msg) {
  uint8_t mode = (msg.id >> 24) & 0x1F;
  
  if (mode == 2) { // 모터 상태 피드백 메시지
    // [수정] 모터 ID는 Bit 8~15에 위치합니다.
    uint8_t motor_id = (msg.id >> 8) & 0xFF; 
    
    // 마스터 모터(ID 1)의 피드백인 경우 위치 업데이트
    if (motor_id == ACT_ID_2) {
      uint16_t p_raw = (msg.buf[0] << 8) | msg.buf[1];
      master_pos = uintToFloat(p_raw, P_MIN, P_MAX);
    }
  }
}

// -------------------------------------------------------------
// 7. 메인 루프 구조
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); 

  Serial.println("=== Robstride Teleoperation (Master ID 1 -> Slave ID 127) ===");

  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps
  Can0.setMaxMB(64);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();
  Can0.enableMBInterrupts();
  Can0.onReceive(rxCallback);

  Serial.println("Teensy CAN initialized.");
  delay(1000);

  // 마스터(ID 1) 및 슬레이브(ID 127) 모터 모두 활성화
  enableMotor(ACT_ID_2); // ID 1
  delay(100);
  enableMotor(ACT_ID_1); // ID 127

  controlTimer = 0;
}

void loop() {
  Can0.events(); // CAN 수신 이벤트 처리

  // 2ms 주기 제어 루프 (500Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    // 1. 마스터 모터(ID 1) : Kp=0, Kd=0 으로 설정해 손으로 자유롭게 돌릴 수 있게 함
    // (이 패킷을 전송해야 마스터 모터가 위치 피드백 응답 패킷을 보냅니다)
    operationControl(ACT_ID_2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // 2. 슬레이브 모터(ID 127) : 마스터의 현재 위치(master_pos)로 추종 명령 전송
    float slave_kp = 25.0f; // 슬레이브 추종 강도 (상황에 따라 10.0 ~ 50.0 조절)
    float slave_kd = 1.0f;  // 감쇠 계수
    CAN_message_t msg_slave = operationControl(ACT_ID_1, 0.0f, master_pos, 0.0f, slave_kp, slave_kd);

    // 500ms마다 시리얼 모니터에 상태 출력
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      
      Serial.printf("[TELEOP] Master(ID %d) Pos: %.3f rad  -->  Slave(ID %d) Target Pos: %.3f rad\r\n", 
                    ACT_ID_2, master_pos, ACT_ID_1, master_pos);
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
      Serial.println("[Teensy] Motors Enabled.");
    }
  }
}