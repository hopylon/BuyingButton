/*
  ESP32 + Si4463 OOK RX @433.92MHz — fixed 6B field (4B payload + 2B CRC)
  - TX 규격과 매칭:
    * PREAMBLE/SYNC: LSB-first
    * PAYLOAD 4B:   MSB-first
    * CRC16-IBM 2B: MSB-first, 전송 순서 Hi,Lo
  - 하드웨어 CRC OFF, 소프트웨어로 CRC 검증
  - 시리얼 출력: "PAYLOAD=AA BB CC DD" (정상 수신 시에만)
*/

#include <Arduino.h>
#include <SPI.h>
#include "radio_config_Si4463_RX.h"

//=================== User knobs ===================
#define EXPECT_PAYLOAD_LEN        4    // 페이로드 4B
#define EXPECT_CRC_BYTES          2    // CRC 2B 포함
#define CRC_MODE_LSBFIRST         0    // TX는 MSB-first CRC → 0
//==================================================

// ---------- Pins ----------
static const int PIN_SCLK = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;
static const int PIN_CS   = 5;   // nSEL
static const int PIN_SDN  = 21;  // SDN
static const int PIN_NIRQ = 4;   // nIRQ

// ---------- SPI ----------
static SPIClass& spi = SPI;
static SPISettings spiSet(4000000, MSBFIRST, SPI_MODE0);

// ---------- Si446x opcodes ----------
#define SI446X_CMD_PART_INFO             0x01
#define SI446X_CMD_PACKET_INFO           0x16
#define SI446X_CMD_FIFO_INFO             0x15
#define SI446X_CMD_GET_INT_STATUS        0x20
#define SI446X_CMD_GET_PH_STATUS         0x21
#define SI446X_CMD_GET_MODEM_STATUS      0x22
#define SI446X_CMD_GET_CHIP_STATUS       0x23
#define SI446X_CMD_REQUEST_DEVICE_STATE  0x33
#define SI446X_CMD_START_RX              0x32
#define SI446X_CMD_READ_CMD_BUFF         0x44
#define SI446X_CMD_READ_RX_FIFO          0x77

// ---------- Helpers ----------
#define CS_LOW()   digitalWrite(PIN_CS, LOW)
#define CS_HIGH()  digitalWrite(PIN_CS, HIGH)
static const uint8_t Radio_Configuration_Data_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;

volatile bool g_irq_flag = false;
void IRAM_ATTR onNirqFalling() { g_irq_flag = true; }

// ----- Low-level SPI -----
void si446x_write(const uint8_t* data, uint8_t len) {
  spi.beginTransaction(spiSet);
  CS_LOW();
  for (uint8_t i=0;i<len;++i) spi.transfer(data[i]);
  CS_HIGH();
  spi.endTransaction();
}

bool si446x_wait_cts(uint32_t timeout_us = 20000) {
  uint32_t start = micros();
  uint8_t cts = 0x00;
  spi.beginTransaction(spiSet);
  do {
    CS_LOW(); spi.transfer(SI446X_CMD_READ_CMD_BUFF); cts = spi.transfer(0x00); CS_HIGH();
    if (cts == 0xFF) { spi.endTransaction(); return true; }
    delayMicroseconds(50);
  } while ((micros()-start) < timeout_us);
  spi.endTransaction();
  return false;
}

bool si446x_cmd(const uint8_t* cmd, uint8_t len) {
  si446x_write(cmd, len);
  return si446x_wait_cts();
}

// RESP (CTS 버리고 응답만)
bool si446x_get_resp(uint8_t* resp, uint8_t len) {
  if (!si446x_wait_cts()) return false;
  spi.beginTransaction(spiSet);
  CS_LOW();
  spi.transfer(SI446X_CMD_READ_CMD_BUFF);
  (void)spi.transfer(0x00); // discard CTS(0xFF)
  for (uint8_t i = 0; i < len; ++i) resp[i] = spi.transfer(0x00);
  CS_HIGH();
  spi.endTransaction();
  return true;
}

// ---------- PROPERTY helpers ----------
bool si446x_get_property(uint8_t group, uint8_t num_props, uint8_t start_id, uint8_t* out) {
  uint8_t cmd[4] = { 0x12, group, num_props, start_id }; // GET_PROPERTY=0x12
  if (!si446x_cmd(cmd, sizeof(cmd))) return false;
  return si446x_get_resp(out, num_props);
}
bool si446x_set_property(uint8_t group, uint8_t num_props, uint8_t start_id, const uint8_t* vals) {
  uint8_t buf[4+16];
  buf[0]=0x11; buf[1]=group; buf[2]=num_props; buf[3]=start_id;
  for (uint8_t i=0;i<num_props;i++) buf[4+i]=vals[i];
  return si446x_cmd(buf, 4+num_props);
}

// FIFO_INFO
bool si446x_fifo_info(uint8_t reset_flags, uint8_t& rx_cnt, uint8_t& tx_space) {
  uint8_t cmd[2] = { SI446X_CMD_FIFO_INFO, reset_flags };
  if (!si446x_cmd(cmd, sizeof(cmd))) return false;
  uint8_t resp[2] = {0};
  if (!si446x_get_resp(resp, sizeof(resp))) return false;
  rx_cnt = resp[0]; tx_space = resp[1];
  return true;
}

// STATUS (필요 최소한만 사용)
bool si446x_get_int_status(uint8_t& INT_PEND, uint8_t& INT_STATUS,
                           uint8_t& PH_PEND,  uint8_t& PH_STATUS,
                           uint8_t& MD_PEND,  uint8_t& MD_STATUS,
                           uint8_t& CH_PEND,  uint8_t& CH_STATUS) {
  uint8_t cmd[4] = { SI446X_CMD_GET_INT_STATUS, 0x00, 0x00, 0x00 };
  if (!si446x_cmd(cmd, sizeof(cmd))) return false;
  uint8_t resp[8] = {0};
  if (!si446x_get_resp(resp, sizeof(resp))) return false;
  INT_PEND=resp[0]; INT_STATUS=resp[1];
  PH_PEND =resp[2]; PH_STATUS =resp[3];
  MD_PEND =resp[4]; MD_STATUS =resp[5];
  CH_PEND =resp[6]; CH_STATUS =resp[7];
  return true;
}

// START_RX
bool si446x_start_rx() {
  uint8_t cmd[8] = { SI446X_CMD_START_RX, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08 };
  return si446x_cmd(cmd, sizeof(cmd));
}

// READ_RX_FIFO
bool si446x_read_rx_fifo(uint8_t* buf, uint8_t len) {
  spi.beginTransaction(spiSet);
  CS_LOW();
  spi.transfer(SI446X_CMD_READ_RX_FIFO);
  for (uint8_t i=0;i<len;++i) buf[i]=spi.transfer(0x00);
  CS_HIGH();
  spi.endTransaction();
  return true;
}

// Apply WDS configuration array
bool si446x_apply_config() {
  const uint8_t* p = Radio_Configuration_Data_Array;
  while (true) {
    uint8_t block_len = *p++;
    if (block_len == 0x00) break;
    if (!si446x_cmd(p, block_len)) return false;
    p += block_len;
  }
  return true;
}

// Reset via SDN
void si446x_hard_reset() {
  digitalWrite(PIN_SDN, HIGH); delay(10);
  digitalWrite(PIN_SDN, LOW);  delay(10);
}

// 헥사 출력 (payload 4바이트만)
void printHex4(const uint8_t* d) {
  for (int i=0;i<4;++i){ if (d[i]<16) Serial.print('0'); Serial.print(d[i],HEX); if(i<3) Serial.print(' '); }
}

// CRC16-IBM (MSB-first)
uint16_t crc16_ibm_msb(const uint8_t* data, size_t len) {
  uint16_t crc=0xFFFF;
  for (size_t idx=0; idx<len; ++idx) {
    uint8_t b=data[idx];
    for(int i=7;i>=0;--i){
      bool bit=(b>>i)&1, c15=(crc>>15)&1;
      crc<<=1; if (c15 ^ bit) crc ^= 0x8005;
    }
  } return crc;
}

// === RAW OFF + OOK threshold relax (optional, one-time tune) ===
void force_disable_RAW_and_relax_thresh() {
  // RF_MODEM_OOK_CNT1_9 (group 0x20, start 0x42)
  uint8_t p[9]={0};
  if (si446x_get_property(0x20, 9, 0x42, p)) {
    p[2] = 0x00; // RAW_SEARCH off
    p[3] = 0x00; // RAW_CONTROL off
    if (p[8] == 0xFF) p[8] = 0x40; // threshold 완화
    si446x_set_property(0x20, 9, 0x42, p);
  }
}

// === 고정 6바이트 필드(4B payload + 2B CRC), 하드웨어 CRC 비활성 ===
void apply_fixed6_nohwcrc_profile() {
  // 1) PH CRC 엔진 OFF
  {
    uint8_t blk[7]={0};
    si446x_get_property(0x12, 7, 0x00, blk);
    blk[0] = 0x00; // CRC DISABLE
    si446x_set_property(0x12, 7, 0x00, blk);
  }
  // 2) RX_FIELD1: LEN=6, Whitening/Manchester OFF, PH CRC OFF
  {
    uint8_t p[12]={0};
    si446x_get_property(0x12, 12, 0x20, p);
    p[1]=0x00; p[2]=0x06; // LEN = 6
    p[3]=0x00;            // no whitening/manchester
    p[4]=0x00;            // PH CRC off
    si446x_set_property(0x12, 12, 0x20, p);
  }
  // 3) TX_FIELD1도 맞춤(참고)
  {
    uint8_t p[12]={0};
    si446x_get_property(0x12, 12, 0x0C, p);
    p[4]=0x00; p[5]=0x06; // LEN = 6
    p[6]=0x00;            // config
    p[7]=0x00;            // CRC off
    si446x_set_property(0x12, 12, 0x0C, p);
  }
}

// 간단 재장전
void rearm_rx() {
  uint8_t rx, tx;
  si446x_fifo_info(0x03, rx, tx); // both reset
  si446x_start_rx();
}

void setup() {
  Serial.begin(115200); delay(80);

  pinMode(PIN_CS, OUTPUT); pinMode(PIN_SDN, OUTPUT); pinMode(PIN_NIRQ, INPUT_PULLUP);
  digitalWrite(PIN_CS, HIGH); digitalWrite(PIN_SDN, HIGH);

  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  si446x_hard_reset(); delay(6);
  si446x_wait_cts();

  if (!si446x_apply_config()) { while(1) delay(1000); }

  // 수신 감도/임계 튜닝 및 6바이트 고정 필드 적용
  force_disable_RAW_and_relax_thresh();
  apply_fixed6_nohwcrc_profile();

  // Arm
  uint8_t rx,tx; si446x_fifo_info(0x03, rx, tx);
  uint8_t a,b,c,d,e,f,g,h; si446x_get_int_status(a,b,c,d,e,f,g,h);
  si446x_start_rx();

  attachInterrupt(digitalPinToInterrupt(PIN_NIRQ), onNirqFalling, FALLING);
}

void loop() {
  if (!g_irq_flag) { delay(1); return; }
  g_irq_flag = false;

  // 인터럽트 읽고 클리어
  uint8_t INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MD_PEND, MD_STATUS, CH_PEND, CH_STATUS;
  if (!si446x_get_int_status(INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MD_PEND, MD_STATUS, CH_PEND, CH_STATUS)) {
    rearm_rx(); return;
  }

  // RX FIFO 상태 확인
  uint8_t rx_cnt_raw=0, tx_space=0;
  if (!si446x_fifo_info(0x00, rx_cnt_raw, tx_space)) { rearm_rx(); return; }

  bool rx_overflow = (rx_cnt_raw & 0x80);
  uint8_t rx_cnt = rx_cnt_raw & 0x7F;
  if (rx_overflow) { rearm_rx(); return; }

  const uint8_t need = EXPECT_PAYLOAD_LEN + EXPECT_CRC_BYTES; // = 6
  if (rx_cnt < need) return;

  // 6바이트 획득
  uint8_t buf[6];
  si446x_read_rx_fifo(buf, need);

  // CRC 검사 (TX: MSB-first → Hi,Lo)
  uint16_t calc = CRC_MODE_LSBFIRST ? 0 /*unused*/ : crc16_ibm_msb(buf, EXPECT_PAYLOAD_LEN);
  uint8_t hi = (calc >> 8) & 0xFF, lo = calc & 0xFF;
  bool crc_ok = (buf[EXPECT_PAYLOAD_LEN]==hi) && (buf[EXPECT_PAYLOAD_LEN+1]==lo);

  // ✅ 정상 수신 시에만 출력 (payload 4B만 + 타임스탬프)
  if (crc_ok) {
    Serial.print("[t=");
    Serial.print(millis());  // 부팅 후 경과 ms
    Serial.print(" ms] PAYLOAD=");
    printHex4(buf);          // buf[0..3]
    Serial.println();
  }

  // 다음 패킷 대기
  rearm_rx();
}
