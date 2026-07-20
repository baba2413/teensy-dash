#include <Arduino.h>
#include <QNEthernet.h> 
#include <FlexCAN_T4.h>
#include <math.h>

using namespace qindesign::network;

// -------------------------------------------------------------
// 1. 테스트할 모터 ID 배열 및 호스트 설정
// -------------------------------------------------------------
const uint8_t NUM_MOTORS = 2;
const uint8_t TEST_MOTOR_IDS[NUM_MOTORS] = {1, 11};  // 1번과 11번 모터 ID 설정
const uint8_t HOST_ID = 253;                         // Teensy 호스트 ID

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// -------------------------------------------------------------
// 2. Robstride 프로토콜 물리적 제한 한계값 (MIT 모드 패킹용)
// -------------------------------------------------------------
const float P_MIN = -12.5f;   const float P_MAX = 12.5f;
const float V_MIN = -45.0f;   const float V_MAX = 45.0f;
const float KP_MAX = 500.0f;  const float KD_MAX = 5.0f;
const float T_MIN = -18.0f;   const float T_MAX = 18.0f;

// -------------------------------------------------------------
// 3. 네트워크 및 UDP 제어 주기 설정
// -------------------------------------------------------------
const uint16_t UDP_PORT = 5005; 
EthernetUDP udp;

IPAddress staticIP(192, 168, 1, 15); // Teensy 고정 IP
IPAddress subnet(255, 255, 255, 0);

const uint32_t CONTROL_PERIOD_US = 20000; // 50Hz (20ms)
elapsedMicros controlTimer;

// -------------------------------------------------------------
// 4. 모터별 입력 버퍼 설정 (배열 구조)
// -------------------------------------------------------------
float ext_target_pos[NUM_MOTORS] = {0.0f, 0.0f}; // [0]: 1번 모터, [1]: 11번 모터 목표 각도
bool ext_control_active = false;
uint32_t last_packet_time = 0;
const uint32_t WATCHDOG_TIMEOUT_MS = 500; // 0.5초 대기

// -------------------------------------------------------------
// 5. 데이터 정수화 및 모터 제어 명령 함수군
// -------------------------------------------------------------
uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

void enableMotor(uint8_t motor_id) {
  CAN_message_t mode_msg;
  mode_msg.flags.extended = 1;
  mode_msg.id = (0x12 << 24) | (HOST_ID << 8) | motor_id;
  mode_msg.len = 8;
  mode_msg.buf[0] = 0x05; mode_msg.buf[1] = 0x70; // Run Mode 주소
  mode_msg.buf[4] = 0x00; // MIT 모드 (0)
  Can0.write(mode_msg);
  delay(50); 

  CAN_message_t enable_msg;
  enable_msg.flags.extended = 1;
  enable_msg.id = (3 << 24) | (HOST_ID << 8) | motor_id;
  enable_msg.len = 8;
  for (int i = 0; i < 8; i++) enable_msg.buf[i] = 0;
  Can0.write(enable_msg);
  Serial.printf("[Teensy] Motor ID %d Enabled.\r\n", motor_id);
}

void disableMotor(uint8_t motor_id) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (4 << 24) | (HOST_ID << 8) | motor_id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;
  Can0.write(msg);
  Serial.printf("[Teensy] Motor ID %d Disabled.\r\n", motor_id);
}

void operationControl(uint8_t motor_id, float feed_forward, float pos, float vel, float kp, float kd) {
  uint16_t p_int  = floatToUint(pos,          P_MIN, P_MAX,  16);
  uint16_t v_int  = floatToUint(vel,          V_MIN, V_MAX,  16);
  uint16_t kp_int = floatToUint(kp,           0.0f,  KP_MAX, 16);
  uint16_t kd_int = floatToUint(kd,           0.0f,  KD_MAX, 16);
  uint16_t t_int  = floatToUint(feed_forward, T_MIN, T_MAX,  16);

  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = (1 << 24) | (t_int << 8) | motor_id;
  msg.len = 8;
  msg.buf[0] = (p_int >> 8) & 0xFF;  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = (v_int >> 8) & 0xFF;  msg.buf[3] = v_int & 0xFF;
  msg.buf[4] = (kp_int >> 8) & 0xFF; msg.buf[5] = kp_int & 0xFF;
  msg.buf[6] = (kd_int >> 8) & 0xFF; msg.buf[7] = kd_int & 0xFF;
  Can0.write(msg);
}

// -------------------------------------------------------------
// 6. 메인 루프
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps
  Can0.setMaxMB(64);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();

  Ethernet.begin(staticIP, subnet, IPAddress(0, 0, 0, 0)); 
  udp.begin(UDP_PORT);

  IPAddress ip = Ethernet.localIP();
  Serial.println("==================================================");
  Serial.println("[Teensy 4.1] Dual Motor Test Mode (Ethernet-CAN)");
  Serial.printf("Target Motors   : ID %d, ID %d\r\n", TEST_MOTOR_IDS[0], TEST_MOTOR_IDS[1]);
  Serial.printf("Static IP       : %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
  Serial.printf("UDP Port        : %d\r\n", UDP_PORT);
  Serial.println("==================================================");
  
  controlTimer = 0;
}

void loop() {
  Can0.events();

  // UDP 수신 처리
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[64];
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      
      // 포맷 파싱: P,값1,값2 (예: P,0.2,-0.1)
      if (packetBuffer[0] == 'P') {
        char* token = strtok(packetBuffer, ",");
        int i = 0;
        while (token != NULL && i < NUM_MOTORS) {
          token = strtok(NULL, ","); // 다음 데이터 추출
          if (token != NULL) {
            ext_target_pos[i] = atof(token);
            i++;
          }
        }
        ext_control_active = true;
        last_packet_time = millis(); // 왓치독 리셋
      }
    }
  }

  // 20ms 제어 루프 (50Hz)
  if (controlTimer >= CONTROL_PERIOD_US) {
    controlTimer -= CONTROL_PERIOD_US;

    if (ext_control_active && (millis() - last_packet_time < WATCHDOG_TIMEOUT_MS)) {
      
      float target_vel = 0.0f;
      float feed_forward_torque = 0.0f;
      float kp = 15.0f; 
      float kd = 1.0f;

      // 두 개의 모터에 각각 명령 순차 전송
      for (int i = 0; i < NUM_MOTORS; i++) {
        operationControl(TEST_MOTOR_IDS[i], feed_forward_torque, ext_target_pos[i], target_vel, kp, kd);
      }

    } else {
      // 왓치독 타임아웃 처리
      if (ext_control_active) {
        Serial.println("[EMERGENCY] UDP Timeout! Disabling all test motors.");
        ext_control_active = false;
        for (int i = 0; i < NUM_MOTORS; i++) {
          disableMotor(TEST_MOTOR_IDS[i]);
        }
      }
    }
  }
}

// 시리얼 명령 수동 제어 (E: 전체 켜기, D: 전체 끄기)
void serialEvent() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd' || ch == 'D') {
      for (int i = 0; i < NUM_MOTORS; i++) {
        disableMotor(TEST_MOTOR_IDS[i]);
      }
    } else if (ch == 'e' || ch == 'E') {
      for (int i = 0; i < NUM_MOTORS; i++) {
        enableMotor(TEST_MOTOR_IDS[i]);
        delay(20); // CAN 버스 연속 명령 안정성을 위한 미세 딜레이
      }
    }
  }
}