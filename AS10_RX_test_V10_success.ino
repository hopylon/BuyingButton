/*
  ESP32 + Si4463 OOK RX @433.92MHz — fixed 6B field (4B payload + 2B CRC), SW CRC verify

  TX 규격(네 SYN115 코드)에 맞춘 수신:
    - PREAMBLE/SYNC: LSB-first (WDS 설정 유지)
    - PAYLOAD 4B:   MSB-first
    - CRC16-IBM 2B: MSB-first, 전송 순서 Hi,Lo
  → RX는 Field1=6바이트(CRC 포함)를 그대로 FIFO로 받고, 소프트웨어로 CRC 검증

  변경 요약:
    - READ_CMD_BUFF 응답에서 CTS(0xFF) 1바이트 버리고 실제 응답만 읽음
    - GET_PROPERTY(0x12) 사용
    - RAW 비활성 + OOK 임계 완화
    - RX_FIELD1 len=6, CRC off (하드웨어 CRC 비사용)
*/

#include <Arduino.h>
#include <SPI.h>
#include "radio_config_Si4463_RX.h"

//=================== User knobs ===================
#define EXPECT_PAYLOAD_LEN        4    // 페이로드 4B
#define EXPECT_CRC_BYTES          2    // CRC 2B 포함
#define CRC_MODE_LSBFIRST         0    // TX는 MSB-first CRC → 0

// PH interrupt bits (PEND 기준)
#define PH_BIT_PACKET_RX_PEND     0x10
#define PH_BIT_CRC_ERROR_PEND     0x20
#define PH_BIT_PREAMBLE_PEND      0x02
#define PH_BIT_SYNC_PEND          0x04

// Stage timeouts (optional)
#define TMO_PREAMBLE_TO_SYNC_MS   80
#define TMO_SYNC_TO_RESULT_MS     80
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

// Debug counters
volatile uint32_t cnt_irq = 0;
volatile uint32_t cnt_ok  = 0;
volatile uint32_t cnt_err = 0;
volatile uint32_t cnt_emp = 0;

// Stage (optional)
enum { ST_IDLE = 0, ST_PREAMBLE = 1, ST_SYNC = 2, ST_RESULT = 3 };
static uint8_t  g_stage = ST_IDLE;
static uint32_t stage_deadline_ms = 0;
static uint32_t c_preamble=0, c_sync=0, c_pktok=0, c_crcerr=0, c_streamdrop=0, c_overflow=0;

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

// ===== FIX: discard CTS byte before reading response bytes =====
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
// ===== FIX: GET_PROPERTY uses 0x12 =====
bool si446x_get_property(uint8_t group, uint8_t num_props, uint8_t start_id, uint8_t* out) {
  uint8_t cmd[4] = { 0x12, group, num_props, start_id };
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

// STATUS
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
bool si446x_get_ph_status(uint8_t& pend, uint8_t& status) {
  uint8_t cmd = SI446X_CMD_GET_PH_STATUS;
  if (!si446x_cmd(&cmd, 1)) return false;
  uint8_t resp[2]={0};
  if (!si446x_get_resp(resp,2)) return false;
  pend=resp[0]; status=resp[1]; return true;
}
bool si446x_get_modem_status(uint8_t* resp8) {
  uint8_t cmd = SI446X_CMD_GET_MODEM_STATUS;
  if (!si446x_cmd(&cmd, 1)) return false;
  return si446x_get_resp(resp8, 8);
}
bool si446x_get_chip_status(uint8_t* resp3) {
  uint8_t cmd = SI446X_CMD_GET_CHIP_STATUS;
  if (!si446x_cmd(&cmd, 1)) return false;
  return si446x_get_resp(resp3, 3);
}
bool si446x_request_device_state(uint8_t& state, uint8_t& chan) {
  uint8_t cmd = SI446X_CMD_REQUEST_DEVICE_STATE;
  if (!si446x_cmd(&cmd, 1)) return false;
  uint8_t resp[2]={0};
  if (!si446x_get_resp(resp,2)) return false;
  state=resp[0]; chan=resp[1]; return true;
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

// PACKET_INFO (optional)
bool si446x_packet_info(uint8_t& len_field, uint16_t& len_remain) {
  uint8_t cmd[2] = { SI446X_CMD_PACKET_INFO, 0x00 };
  if (!si446x_cmd(cmd, sizeof(cmd))) return false;
  uint8_t resp[3]={0};
  if (!si446x_get_resp(resp,3)) return false;
  len_field  = resp[0];
  len_remain = ((uint16_t)resp[1] << 8) | resp[2];
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

// Pretty printers
void printHex(const uint8_t* d, size_t n) {
  for (size_t i=0;i<n;++i){ if (d[i]<16) Serial.print('0'); Serial.print(d[i],HEX); if(i+1<n) Serial.print(' ');}
}
void printAscii(const uint8_t* d, size_t n) {
  for (size_t i=0;i<n;++i){ char c=(d[i]>=32&&d[i]<=126)?(char)d[i]:'.'; Serial.print(c); }
}

// CRC16-IBM
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
uint16_t crc16_ibm_lsb(const uint8_t* d, size_t n) {
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<n;++i){ uint8_t b=d[i];
    for(int k=0;k<8;++k){ uint16_t mix=(crc^b)&1; crc>>=1; if(mix) crc^=0xA001; b>>=1; }
  } return crc;
}

// Stage helpers
void stage_enter(uint8_t st, uint32_t timeout_ms) {
  g_stage = st; stage_deadline_ms = millis() + timeout_ms;
}
void rearm_rx(const __FlashStringHelper* reason) {
  uint8_t rx, tx;
  si446x_fifo_info(0x03, rx, tx); // both reset
  si446x_start_rx();
  uint8_t st, ch; si446x_request_device_state(st, ch);
  Serial.print(F("[Rearm] ")); Serial.print(reason);
  Serial.print(F("  STATE=0x")); Serial.print(st, HEX);
  Serial.print(F(" CH=")); Serial.println(ch);
  g_stage = ST_IDLE; stage_deadline_ms = 0;
}
void stage_check_timeout() {
  if (!stage_deadline_ms) return;
  if ((int32_t)(millis()-stage_deadline_ms) >= 0) {
    if (g_stage==ST_PREAMBLE) Serial.println(F("[STAGE TMO] PREAMBLE->SYNC 미도달"));
    else if (g_stage==ST_SYNC) Serial.println(F("[STAGE TMO] SYNC->RESULT 미도달"));
    else Serial.println(F("[STAGE TMO] 예기치 않은 상태"));
    rearm_rx(F("stage_timeout"));
  }
}

// Device / Modem dumps
void dump_device_info() {
  uint8_t cmd = SI446X_CMD_PART_INFO;
  if (si446x_cmd(&cmd,1)) { uint8_t r[9]={0}; if (si446x_get_resp(r,9)) { Serial.print(F("PART_INFO: ")); printHex(r,9); Serial.println(); } }
  uint8_t st,ch; if (si446x_request_device_state(st,ch)) { Serial.print(F("DEVICE_STATE: ")); Serial.print(st,HEX); Serial.print(F(" CH=")); Serial.println(ch); }
}
void dump_modem_status() {
  uint8_t m[8]={0};
  if (si446x_get_modem_status(m)) {
    Serial.print(F("MODEM: PEND=")); Serial.print(m[0],HEX);
    Serial.print(F(" STAT=")); Serial.print(m[1],HEX);
    Serial.print(F(" RSSI[c/l/a1/a2]="));
    Serial.print((int8_t)m[2]); Serial.print('/'); Serial.print((int8_t)m[3]);
    Serial.print('/'); Serial.print((int8_t)m[4]); Serial.print('/'); Serial.print((int8_t)m[5]);
    Serial.print(F(" AFC=")); int16_t afc=(int16_t)((m[6]<<8)|m[7]); Serial.println(afc);
  }
}

// === RAW OFF + OOK threshold relax (optional) ===
void force_disable_RAW_and_relax_thresh() {
  // RF_MODEM_OOK_CNT1_9 (group 0x20, start 0x42)
  uint8_t p[9]={0};
  if (si446x_get_property(0x20, 9, 0x42, p)) {
    Serial.print(F("OOK_CNT1[0x20:0x42..] before: ")); printHex(p,9); Serial.println();
    // RAW off
    p[2] = 0x00; // RAW_SEARCH
    p[3] = 0x00; // RAW_CONTROL
    // relax threshold if extreme
    if (p[8] == 0xFF) p[8] = 0x40;
    if (si446x_set_property(0x20, 9, 0x42, p)) {
      Serial.print(F("OOK_CNT1[0x20:0x42..] patched: ")); printHex(p,9); Serial.println();
    } else {
      Serial.println(F("[ERR] set OOK_CNT1 failed"));
    }
  }
}

// === 고정 6바이트 필드(4B payload + 2B CRC), 하드웨어 CRC 비활성 ===
void apply_fixed6_nohwcrc_profile() {
  // 1) PKT_CRC_CONFIG 블록: CRC 엔진 비활성(하드웨어 CRC 안씀)
  {
    uint8_t blk[7]={0};
    si446x_get_property(0x12, 7, 0x00, blk);
    blk[0] = 0x00; // CRC DISABLE
    si446x_set_property(0x12, 7, 0x00, blk);
    Serial.println(F("[PROFILE] PKT_CRC_CONFIG: CRC disabled (SW verify)"));
  }
  // 2) RX_FIELD1: len=6, config=0x00(화이트닝/맨체스터 없음), CRC_CONFIG=0x00(하드웨어 CRC 안씀)
  {
    uint8_t p[12]={0};
    si446x_get_property(0x12, 12, 0x20, p);
    p[1]=0x00; p[2]=0x06; // LEN = 6
    p[3]=0x00;            // no whitening/manchester
    p[4]=0x00;            // CRC not handled by PH
    si446x_set_property(0x12, 12, 0x20, p);
    Serial.println(F("[PROFILE] RX_FIELD1 len=6, no whitening/manchester, PH CRC off"));
  }
  // 3) (참고) TX_FIELD1도 맞춰둠 (수신만 테스트 시 필수는 아님)
  {
    uint8_t p[12]={0};
    si446x_get_property(0x12, 12, 0x0C, p);
    p[4]=0x00; p[5]=0x06; // LEN = 6
    p[6]=0x00;            // config
    p[7]=0x00;            // CRC off
    si446x_set_property(0x12, 12, 0x0C, p);
  }
  // 4) PKT_LEN 블록(0x12:0x08..)은 기본값 유지(가변길이 사용 안 함)
}

// ---- Setup & Loop ----
void setup() {
  Serial.begin(115200); delay(80);
  Serial.println(F("\nSi4463 OOK RX (fixed 6B field, SW CRC verify)"));

  pinMode(PIN_CS, OUTPUT); pinMode(PIN_SDN, OUTPUT); pinMode(PIN_NIRQ, INPUT_PULLUP);
  digitalWrite(PIN_CS, HIGH); digitalWrite(PIN_SDN, HIGH);

  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  si446x_hard_reset(); delay(6);
  if (!si446x_wait_cts()) Serial.println(F("CTS timeout after reset"));

  if (!si446x_apply_config()) { Serial.println(F("Config apply failed")); while(1) delay(1000); }
  Serial.println(F("Config applied"));
  dump_device_info();

  // RAW OFF + 임계 완화
  force_disable_RAW_and_relax_thresh();

  // 고정 6바이트 프로파일 적용
  apply_fixed6_nohwcrc_profile();

  // 확인용 덤프
  {
    uint8_t p[16]={0};
    if (si446x_get_property(0x12,12,0x20,p)){
      Serial.print(F("PKT_RX_FIELD1[0x12:0x20..]: ")); printHex(p,12); Serial.println();
      uint16_t rx_f1_len = ((uint16_t)(p[1]&0x01)<<8) | p[2];
      Serial.print(F("  RX_FIELD1_LEN=")); Serial.println(rx_f1_len);
      Serial.print(F("  RX_FIELD1_CFG=0x")); Serial.println(p[3],HEX);
      Serial.print(F("  RX_FIELD1_CRC=0x")); Serial.println(p[4],HEX);
    }
    if (si446x_get_property(0x12,7,0x00,p)){ Serial.print(F("PKT_CRC_CFG[0x12:0x00..7]: ")); printHex(p,7); Serial.println(); }
  }

  // Arm
  uint8_t rx,tx; si446x_fifo_info(0x03, rx, tx);
  uint8_t a,b,c,d,e,f,g,h; si446x_get_int_status(a,b,c,d,e,f,g,h);
  si446x_start_rx();

  attachInterrupt(digitalPinToInterrupt(PIN_NIRQ), onNirqFalling, FALLING);
  Serial.println(F("RX armed"));

  uint8_t st, ch; si446x_request_device_state(st,ch);
  Serial.print(F("Init STATE=0x")); Serial.print(st,HEX); Serial.print(F(" CH=")); Serial.println(ch);
}

void loop() {
  static uint32_t t_last = 0;
  if (millis()-t_last > 200) { t_last = millis(); dump_modem_status(); stage_check_timeout(); }

  if (!g_irq_flag) { delay(1); return; }
  g_irq_flag = false; cnt_irq++;

  // Read & clear interrupts
  uint8_t INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MD_PEND, MD_STATUS, CH_PEND, CH_STATUS;
  if (!si446x_get_int_status(INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MD_PEND, MD_STATUS, CH_PEND, CH_STATUS)) {
    Serial.println(F("GET_INT_STATUS fail")); return;
  }
  uint8_t ph_p,ph_s; si446x_get_ph_status(ph_p,ph_s);
  uint8_t chip3[3]={0}; si446x_get_chip_status(chip3);

  Serial.print(F("\n[IRQ#")); Serial.print(cnt_irq);
  Serial.print(F("] INT{P,S}=")); Serial.print(INT_PEND,HEX); Serial.print('/'); Serial.print(INT_STATUS,HEX);
  Serial.print(F("  PH{P,S}="));  Serial.print(PH_PEND,HEX);  Serial.print('/'); Serial.print(PH_STATUS,HEX);
  Serial.print(F(" (snap ")); Serial.print(ph_p,HEX); Serial.print('/'); Serial.print(ph_s,HEX); Serial.print(')');
  Serial.print(F("  MD{P,S}=")); Serial.print(MD_PEND,HEX);  Serial.print('/'); Serial.print(MD_STATUS,HEX);
  Serial.print(F("  CH{P,S}=")); Serial.print(CH_PEND,HEX);  Serial.print('/'); Serial.print(CH_STATUS,HEX);
  Serial.print(F("  CHIP="));    printHex(chip3,3); Serial.println();

  // PRE
  if (PH_PEND & PH_BIT_PREAMBLE_PEND) {
    if (g_stage == ST_IDLE) { c_preamble++; Serial.println(F("[EVT] PREAMBLE_DETECT")); stage_enter(ST_PREAMBLE, TMO_PREAMBLE_TO_SYNC_MS); }
  }

  // SYNC
  if (PH_PEND & PH_BIT_SYNC_PEND) {
    if (g_stage == ST_PREAMBLE || g_stage == ST_IDLE) {
      c_sync++; Serial.println(F("[EVT] SYNC_DETECT"));
      // 경계 정렬(선택)
      uint8_t rx,tx; si446x_fifo_info(0x03, rx, tx);
      si446x_start_rx();
      Serial.println(F("[SYNC] Both FIFOs reset + START_RX"));
      stage_enter(ST_SYNC, TMO_SYNC_TO_RESULT_MS);
    }
  }

  // FIFO 상태
  uint8_t rx_cnt_raw=0, tx_space=0;
  if (!si446x_fifo_info(0x00, rx_cnt_raw, tx_space)) { Serial.println(F("FIFO_INFO fail")); rearm_rx(F("fifo_info_fail")); return; }
  bool rx_overflow = (rx_cnt_raw & 0x80);
  uint8_t rx_cnt = rx_cnt_raw & 0x7F;

  if (rx_overflow) {
    c_overflow++;
    Serial.print(F("[WARN] RX FIFO overflow! raw=0x")); Serial.println(rx_cnt_raw, HEX);
    rearm_rx(F("rx_overflow"));
    return;
  }

  // 결과 처리: PH가 패킷 완료를 못 줘도, 6바이트가 모이면 직접 처리
  bool pkt_ok_pend  = (PH_PEND & PH_BIT_PACKET_RX_PEND);
  bool pkt_err_pend = (PH_PEND & PH_BIT_CRC_ERROR_PEND); // 하드웨어 CRC off라면 항상 0일 수 있음

  if (pkt_ok_pend || rx_cnt >= (EXPECT_PAYLOAD_LEN + EXPECT_CRC_BYTES)) {
    const uint8_t need = EXPECT_PAYLOAD_LEN + EXPECT_CRC_BYTES;
    if (rx_cnt < need) {
      Serial.print(F("[INFO] waiting more bytes: ")); Serial.print(rx_cnt); Serial.print('/'); Serial.println(need);
      return;
    }

    uint8_t buf[64];
    uint8_t n = (need <= sizeof(buf)) ? need : sizeof(buf);
    si446x_read_rx_fifo(buf, n);

    Serial.print(F("PKT bytes(")); Serial.print(n); Serial.println(F("):"));
    Serial.print(F("  HEX : ")); printHex(buf, n); Serial.println();
    Serial.print(F("  ASCII: ")); printAscii(buf, n); Serial.println();

    // SW CRC 검증
    bool crc_ok_flag = true;
    if (EXPECT_CRC_BYTES == 2) {
      uint16_t calc = CRC_MODE_LSBFIRST ? crc16_ibm_lsb(buf, EXPECT_PAYLOAD_LEN)
                                        : crc16_ibm_msb(buf, EXPECT_PAYLOAD_LEN);
      uint8_t hi = (calc >> 8) & 0xFF, lo = calc & 0xFF;
      // TX는 MSB-first로 CRC 전송: Hi, Lo 순서로 온다고 가정
      crc_ok_flag = (buf[EXPECT_PAYLOAD_LEN]==hi) && (buf[EXPECT_PAYLOAD_LEN+1]==lo);
      Serial.print(F("  CRC(SW) => ")); Serial.println(crc_ok_flag?F("OK"):F("FAIL"));
    } else {
      Serial.println(F("  CRC: N/A"));
    }

    if (crc_ok_flag) { c_pktok++; cnt_ok++; Serial.print(F("[RESULT] PACKET_OK (total=")); Serial.print(c_pktok); Serial.println(')'); }
    else { c_crcerr++; cnt_err++; Serial.print(F("[RESULT] CRC_ERROR (total=")); Serial.print(c_crcerr); Serial.println(')'); }

    rearm_rx(F("after_packet"));
    return;
  }

  // 비패킷 스트림(가드)
  if (!pkt_ok_pend && rx_cnt > 0) {
    // 지나치게 커지면 드롭
    if (rx_cnt >= 60) {
      c_streamdrop++;
      Serial.print(F("[STREAM] Non-packet FIFO (rx_cnt=")); Serial.print(rx_cnt);
      Serial.print(F(") → drop & rearm. (drops=")); Serial.print(c_streamdrop); Serial.println(')');
      uint8_t tmp[64]; uint8_t toread = rx_cnt > sizeof(tmp) ? sizeof(tmp) : rx_cnt;
      si446x_read_rx_fifo(tmp, toread);
      rearm_rx(F("stream_drop"));
      return;
    }
    // 아직 6B 안 됐으면 대기
  } else {
    cnt_emp++;
  }

  // Counters
  Serial.print(F("Counters: PRE=")); Serial.print(c_preamble);
  Serial.print(F(" SYNC=")); Serial.print(c_sync);
  Serial.print(F(" OK="));   Serial.print(c_pktok);
  Serial.print(F(" ERR="));  Serial.print(c_crcerr);
  Serial.print(F(" DROP=")); Serial.print(c_streamdrop);
  Serial.print(F(" EMPTY="));Serial.print(cnt_emp);
  Serial.print(F(" IRQ="));  Serial.println(cnt_irq);
}
