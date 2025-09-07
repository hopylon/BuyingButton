// SYN115 + ESP32 (Arduino core) — OOK packet to match Si4463(WDS) config
// Packet format: [Preamble x8B, LSB-first] [Sync 0xB4,0x2B (LSB-first)] [Payload 4B MSB-first] [CRC16-IBM 2B, MSB-first]

#include <Arduino.h>

// ==== User pins / params ====
static const int   TX_PIN        = 5;   // SYN115 DATA 입력에 연결
static const bool  DATA_ON_HIGH  = true; // 대부분의 SYN115 모듈은 HIGH=carrier ON
static const uint32_t BIT_RATE   = 2000; // 2 kbps
static const uint32_t BIT_US     = 1000000UL / BIT_RATE;

// 전송 시 비트 단위 로그도 보고 싶으면 true
static const bool  LOG_BITS      = false;

// WDS 헤더의 CUSTOM_PAYLOAD (길이=4)
uint8_t payload[4] = { 0xAB, 0xCD, 0x12, 0x34 };

// ==== Low-level OOK bit sender ====
inline void ookWrite(bool one) {
  if (DATA_ON_HIGH) digitalWrite(TX_PIN, one ? HIGH : LOW);
  else              digitalWrite(TX_PIN, one ? LOW  : HIGH);
}

inline void sendBit(bool b) {
  if (LOG_BITS) Serial.print(b ? '1' : '0');
  ookWrite(b);
  delayMicroseconds(BIT_US);
}

// LSB-first (프리앰블/싱크용)
void sendByteLSB(uint8_t v) {
  for (int i = 0; i < 8; ++i) {
    sendBit((v >> i) & 0x01);
  }
  if (LOG_BITS) Serial.print(' ');
}

// MSB-first (데이터/CRC용, BIT_ORDER=0)
void sendByteMSB(uint8_t v) {
  for (int i = 7; i >= 0; --i) {
    sendBit((v >> i) & 0x01);
  }
  if (LOG_BITS) Serial.print(' ');
}

// CRC16(IBM) MSB-first 구현 (poly 0x8005, init 0xFFFF, no XORout)
uint16_t crc16_ibm_msb(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t idx = 0; idx < len; ++idx) {
    uint8_t b = data[idx];
    for (int i = 7; i >= 0; --i) {
      bool bit = (b >> i) & 1;           // MSB-first 입력
      bool c15 = (crc >> 15) & 1;
      crc <<= 1;
      if (c15 ^ bit) crc ^= 0x8005;
    }
  }
  return crc;
}

// 헥사 출력 유틸
void printHexByte(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

// 프레임 송신: PREAMBLE(8B, 0xAA LSB-first) + SYNC(0xB4, 0x2B LSB-first) + PAYLOAD(4B, MSB-first) + CRC(2B, MSB-first)
void sendFrame(const uint8_t* pl, size_t len) {
  // ---- 시리얼 로그: 페이로드/CRC 정보 ----
  uint16_t crc = crc16_ibm_msb(pl, len);
  uint8_t crc_hi = (crc >> 8) & 0xFF;
  uint8_t crc_lo = crc & 0xFF;

  Serial.print("[TX] t(ms)=");
  Serial.print(millis());
  Serial.print("  Payload(");
  Serial.print(len);
  Serial.print("B)=");

  for (size_t i = 0; i < len; ++i) {
    printHexByte(pl[i]);
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.print("  CRC16-IBM=");
  printHexByte(crc_hi);
  Serial.print(' ');
  printHexByte(crc_lo);
  Serial.print("  Sync(LSB)=B4 2B  OTA=> 2D D4");
  Serial.println();

  if (LOG_BITS) {
    Serial.print("[BITS] PREAMBLE/SYNC/PAYLOAD/CRC -> ");
  }

  // Optional: 인터럽트로 인한 지터를 줄이고 싶으면 아래 두 줄 사용
  // noInterrupts();

  // 1) Preamble (8 bytes of 0xAA, LSB-first)
  for (int i = 0; i < 8; ++i) sendByteLSB(0xAA);

  // 2) Sync (register bytes 0xB4, 0x2B sent LSB-first -> OTA 0x2D,0xD4)
  sendByteLSB(0xB4);
  sendByteLSB(0x2B);

  // 3) Payload (MSB-first)
  for (size_t i = 0; i < len; ++i) sendByteMSB(pl[i]);

  // 4) CRC16-IBM MSB-first, High byte 먼저
  sendByteMSB(crc_hi);
  sendByteMSB(crc_lo);

  if (LOG_BITS) Serial.println();

  // 5) Idle: carrier OFF
  ookWrite(false);

  // interrupts(); // 다시 허용
}

void setup() {
  Serial.begin(115200);
  // ESP32는 Serial가 즉시 연결되지만, 초기 부트 메시지 이후 안정화 대기
  delay(100);

  pinMode(TX_PIN, OUTPUT);
  ookWrite(false); // idle=OFF
  delay(50);

  Serial.println("OOK TX init @2 kbps (SYN115) — ready.");
}

void loop() {
  // 예제: 1초마다 한 번 패킷 전송
  sendFrame(payload, sizeof(payload));
  delay(1000);
}
