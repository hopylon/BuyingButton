/*
  ESP32 + Si4463 (AS10-M4463D-SMA) OOK RX @433.92MHz with rich debugging

  - Loads WDS radio_config.h as-is (Rev B1, OOK 2 kbps, preamble/sync/CRC config).
  - Dumps device info at boot.
  - Periodically prints RSSI and state.
  - On packet IRQ, prints detailed INT/PH/MODEM/CHIP status and RX FIFO contents.
  - Tries to validate CRC (if CRC bytes are present in FIFO).

  Wiring (change to your pins):
    ESP32   ->  Si4463
    18(SCLK)->  SCLK
    23(MOSI)->  SDI
    19(MISO)->  SDO
    5 (CS)  ->  nSEL
    21(SDN) ->  SDN
    4 (INT) ->  nIRQ  (active low)
    GND     ->  GND
    3V3     ->  VDD

  Build: Arduino-ESP32 core 2.x
*/

#include <Arduino.h>
#include <SPI.h>
#include "radio_config_Si4463_RX.h" // <-- include your WDS header

// ---------- Pin assignment ----------
static const int PIN_SCLK = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;
static const int PIN_CS   = 5;   // nSEL
static const int PIN_SDN  = 21;  // SDN
static const int PIN_NIRQ = 4;   // nIRQ (interrupt)

// ---------- SPI settings ----------
static SPIClass& spi = SPI;
static SPISettings spiSet(4000000, MSBFIRST, SPI_MODE0); // 4 MHz, MODE0

// ---------- Si446x opcodes ----------
#define SI446X_CMD_PART_INFO             0x01
#define SI446X_CMD_POWER_UP              0x02
#define SI446X_CMD_GPIO_PIN_CFG          0x13
#define SI446X_CMD_PACKET_INFO           0x16
#define SI446X_CMD_FIFO_INFO             0x15
#define SI446X_CMD_GET_INT_STATUS        0x20
#define SI446X_CMD_GET_PH_STATUS         0x21
#define SI446X_CMD_GET_MODEM_STATUS      0x22
#define SI446X_CMD_GET_CHIP_STATUS       0x23
#define SI446X_CMD_REQUEST_DEVICE_STATE  0x33
#define SI446X_CMD_CHANGE_STATE          0x34
#define SI446X_CMD_START_RX              0x32
#define SI446X_CMD_READ_CMD_BUFF         0x44
#define SI446X_CMD_READ_RX_FIFO          0x77

// ---------- Helpers ----------
#define CS_LOW()   digitalWrite(PIN_CS, LOW)
#define CS_HIGH()  digitalWrite(PIN_CS, HIGH)

static const uint8_t Radio_Configuration_Data_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;

// ISR flag
volatile bool g_irq_flag = false;
void IRAM_ATTR onNirqFalling() { g_irq_flag = true; }

// Debug counters
volatile uint32_t cnt_irq = 0;
volatile uint32_t cnt_ok  = 0;
volatile uint32_t cnt_err = 0;
volatile uint32_t cnt_emp = 0;

// ----- Low-level SPI transfers -----
void si446x_write(const uint8_t* data, uint8_t len) {
  spi.beginTransaction(spiSet);
  CS_LOW();
  for (uint8_t i = 0; i < len; ++i) spi.transfer(data[i]);
  CS_HIGH();
  spi.endTransaction();
}

// Poll CTS (0xFF) using READ_CMD_BUFF
bool si446x_wait_cts(uint32_t timeout_us = 20000) {
  uint32_t start = micros();
  uint8_t cts = 0x00;

  spi.beginTransaction(spiSet);
  do {
    CS_LOW();
    spi.transfer(SI446X_CMD_READ_CMD_BUFF);
    cts = spi.transfer(0x00);
    CS_HIGH();
    if (cts == 0xFF) {
      spi.endTransaction();
      return true;
    }
    delayMicroseconds(50);
  } while ((micros() - start) < timeout_us);

  spi.endTransaction();
  return false;
}

// Send command then wait CTS (no response expected)
bool si446x_cmd(const uint8_t* cmd, uint8_t len) {
  si446x_write(cmd, len);
  return si446x_wait_cts();
}

// Get response after CTS ready
bool si446x_get_resp(uint8_t* resp, uint8_t len) {
  if (!si446x_wait_cts()) return false;

  spi.beginTransaction(spiSet);
  CS_LOW();
  spi.transfer(SI446X_CMD_READ_CMD_BUFF);
  for (uint8_t i = 0; i < len; ++i) resp[i] = spi.transfer(0x00);
  CS_HIGH();
  spi.endTransaction();
  return true;
}

// FIFO_INFO (option: reset flags) -> returns rx_count, tx_space
bool si446x_fifo_info(uint8_t reset_flags, uint8_t& rx_cnt, uint8_t& tx_space) {
  uint8_t cmd[2] = { SI446X_CMD_FIFO_INFO, reset_flags };
  if (!si446x_cmd(cmd, sizeof(cmd))) return false;

  uint8_t resp[2] = {0};
  if (!si446x_get_resp(resp, sizeof(resp))) return false;
  rx_cnt = resp[0];
  tx_space = resp[1];
  return true;
}

// GET_INT_STATUS (clear pendings) -> returns 8-byte block
bool si446x_get_int_status(uint8_t& INT_PEND, uint8_t& INT_STATUS,
                           uint8_t& PH_PEND,  uint8_t& PH_STATUS,
                           uint8_t& MD_PEND,  uint8_t& MD_STATUS,
                           uint8_t& CH_PEND,  uint8_t& CH_STATUS) {
  uint8_t cmd[4] = { SI446X_CMD_GET_INT_STATUS, 0x00, 0x00, 0x00 }; // clear all
  if (!si446x_cmd(cmd, sizeof(cmd))) return false;

  uint8_t resp[8] = {0};
  if (!si446x_get_resp(resp, sizeof(resp))) return false;

  INT_PEND = resp[0]; INT_STATUS = resp[1];
  PH_PEND  = resp[2]; PH_STATUS  = resp[3];
  MD_PEND  = resp[4]; MD_STATUS  = resp[5];
  CH_PEND  = resp[6]; CH_STATUS  = resp[7];
  return true;
}

// GET_PH_STATUS (does not clear GET_INT_STATUS groups)
bool si446x_get_ph_status(uint8_t& pend, uint8_t& status) {
  uint8_t cmd = SI446X_CMD_GET_PH_STATUS;
  if (!si446x_cmd(&cmd, 1)) return false;
  uint8_t resp[2] = {0};
  if (!si446x_get_resp(resp, 2)) return false;
  pend = resp[0]; status = resp[1];
  return true;
}

// GET_MODEM_STATUS -> returns 8 bytes (pend/status + rssi etc.)
bool si446x_get_modem_status(uint8_t* resp8) {
  uint8_t cmd = SI446X_CMD_GET_MODEM_STATUS;
  if (!si446x_cmd(&cmd, 1)) return false;
  return si446x_get_resp(resp8, 8);
}

// GET_CHIP_STATUS -> returns 3 bytes (pend/status and cmd_err)
bool si446x_get_chip_status(uint8_t* resp3) {
  uint8_t cmd = SI446X_CMD_GET_CHIP_STATUS;
  if (!si446x_cmd(&cmd, 1)) return false;
  return si446x_get_resp(resp3, 3);
}

// REQUEST_DEVICE_STATE -> returns 2 bytes (curr_state, curr_channel)
bool si446x_request_device_state(uint8_t& state, uint8_t& chan) {
  uint8_t cmd = SI446X_CMD_REQUEST_DEVICE_STATE;
  if (!si446x_cmd(&cmd, 1)) return false;
  uint8_t resp[2] = {0};
  if (!si446x_get_resp(resp, 2)) return false;
  state = resp[0]; chan = resp[1];
  return true;
}

// START_RX on channel 0; rx_len=0 (PH-managed); next states READY
bool si446x_start_rx() {
  uint8_t cmd[8] = {
    SI446X_CMD_START_RX,
    0x00,       // channel
    0x00,       // condition
    0x00, 0x00, // rx_len (PH managed)
    0x08,       // next_state1 (RXTIMEOUT->READY)
    0x08,       // next_state2 (RXVALID->READY)
    0x08        // next_state3 (RXINVALID->READY)
  };
  return si446x_cmd(cmd, sizeof(cmd));
}

// READ_RX_FIFO n bytes
bool si446x_read_rx_fifo(uint8_t* buf, uint8_t len) {
  spi.beginTransaction(spiSet);
  CS_LOW();
  spi.transfer(SI446X_CMD_READ_RX_FIFO);
  for (uint8_t i = 0; i < len; ++i) buf[i] = spi.transfer(0x00);
  CS_HIGH();
  spi.endTransaction();
  return true;
}

// Apply WDS configuration array (POWER_UP 포함)
bool si446x_apply_config() {
  const uint8_t* p = Radio_Configuration_Data_Array;
  while (true) {
    uint8_t block_len = *p++;
    if (block_len == 0x00) break; // end
    if (!si446x_cmd(p, block_len)) return false;
    p += block_len;
  }
  return true;
}

// Hard reset via SDN pin
void si446x_hard_reset() {
  digitalWrite(PIN_SDN, HIGH);
  delay(10);
  digitalWrite(PIN_SDN, LOW);
  delay(10);
}

// Pretty hex print
void printHex(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    if (d[i] < 16) Serial.print('0');
    Serial.print(d[i], HEX);
    if (i + 1 < n) Serial.print(' ');
  }
}

// ASCII dump (print only printable)
void printAscii(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    char c = (d[i] >= 32 && d[i] <= 126) ? (char)d[i] : '.';
    Serial.print(c);
  }
}

// CRC16-IBM (MSB-first input), poly 0x8005, init 0xFFFF, no XORout
uint16_t crc16_ibm_msb(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t idx = 0; idx < len; ++idx) {
    uint8_t b = data[idx];
    for (int i = 7; i >= 0; --i) {
      bool bit = (b >> i) & 1;
      bool c15 = (crc >> 15) & 1;
      crc <<= 1;
      if (c15 ^ bit) crc ^= 0x8005;
    }
  }
  return crc;
}

// Dump device info once
void dump_device_info() {
  // PART_INFO (9 bytes)
  {
    uint8_t cmd = SI446X_CMD_PART_INFO;
    if (si446x_cmd(&cmd, 1)) {
      uint8_t r[9] = {0};
      if (si446x_get_resp(r, sizeof(r))) {
        Serial.print(F("PART_INFO: "));
        printHex(r, sizeof(r));
        Serial.println();
      }
    }
  }
  // GET_CHIP_STATUS
  {
    uint8_t r3[3] = {0};
    if (si446x_get_chip_status(r3)) {
      Serial.print(F("CHIP_STATUS: "));
      printHex(r3, 3);
      Serial.println();
    }
  }
  // DEVICE STATE
  {
    uint8_t st, ch;
    if (si446x_request_device_state(st, ch)) {
      Serial.print(F("DEVICE_STATE: "));
      Serial.print(st, HEX);
      Serial.print(F("  CH="));
      Serial.println(ch);
    }
  }
}

// Periodic modem status & RSSI
void dump_modem_status() {
  uint8_t m[8] = {0};
  if (si446x_get_modem_status(m)) {
    // m[0]=MD_PEND, m[1]=MD_STATUS, m[2]=CURR_RSSI, m[3]=LATCH_RSSI, m[4]=ANT1_RSSI, m[5]=ANT2_RSSI, m[6..7]=AFC_OFFSET
    Serial.print(F("MODEM: PEND="));
    Serial.print(m[0], HEX);
    Serial.print(F(" STAT="));
    Serial.print(m[1], HEX);
    Serial.print(F(" RSSI[c/l/a1/a2]="));
    Serial.print((int8_t)m[2]); Serial.print('/');
    Serial.print((int8_t)m[3]); Serial.print('/');
    Serial.print((int8_t)m[4]); Serial.print('/');
    Serial.print((int8_t)m[5]);
    Serial.print(F(" AFC="));
    int16_t afc = (int16_t)((m[6] << 8) | m[7]);
    Serial.println(afc);
  }
}

// Restart RX with small log
void rearm_rx(const __FlashStringHelper* reason) {
  uint8_t rx, tx;
  si446x_fifo_info(0x02, rx, tx); // reset RX only
  si446x_start_rx();
  uint8_t st, ch;
  si446x_request_device_state(st, ch);
  Serial.print(F("[Rearm] "));
  Serial.print(reason);
  Serial.print(F("  STATE=0x"));
  Serial.print(st, HEX);
  Serial.print(F(" CH="));
  Serial.println(ch);
}

// ----- Setup & Loop -----
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("\nSi4463 OOK RX (debug) start"));

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_SDN, OUTPUT);
  pinMode(PIN_NIRQ, INPUT_PULLUP);

  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_SDN, HIGH); // keep reset until SPI is up

  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  // Reset & bring-up
  si446x_hard_reset();

  // Initial CTS sync
  delay(6);
  if (!si446x_wait_cts()) {
    Serial.println(F("CTS timeout after reset"));
  }

  // Load WDS config
  if (!si446x_apply_config()) {
    Serial.println(F("Config apply failed"));
    while (1) delay(1000);
  }
  Serial.println(F("Config applied"));

  dump_device_info();

  // Clear pending and start RX
  uint8_t rx, tx;
  si446x_fifo_info(0x03, rx, tx); // reset both FIFOs
  uint8_t a,b,c,d,e,f,g,h;
  si446x_get_int_status(a,b,c,d,e,f,g,h); // clear pendings
  si446x_start_rx();

  // IRQ
  attachInterrupt(digitalPinToInterrupt(PIN_NIRQ), onNirqFalling, FALLING);
  Serial.println(F("RX armed"));

  // Initial state
  uint8_t st, ch;
  si446x_request_device_state(st, ch);
  Serial.print(F("Init STATE=0x")); Serial.print(st, HEX);
  Serial.print(F(" CH=")); Serial.println(ch);
}

void loop() {
  static uint32_t t_last_rssi = 0;

  // Periodic RSSI & modem status (every ~200ms)
  if (millis() - t_last_rssi > 200) {
    t_last_rssi = millis();
    dump_modem_status();
  }

  if (!g_irq_flag) {
    delay(1);
    return;
  }
  g_irq_flag = false;
  cnt_irq++;

  // Read & clear interrupts
  uint8_t INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MD_PEND, MD_STATUS, CH_PEND, CH_STATUS;
  if (!si446x_get_int_status(INT_PEND, INT_STATUS, PH_PEND, PH_STATUS, MD_PEND, MD_STATUS, CH_PEND, CH_STATUS)) {
    Serial.println(F("GET_INT_STATUS fail"));
    return;
  }

  // Extra snapshots
  uint8_t ph_p, ph_s;
  si446x_get_ph_status(ph_p, ph_s);
  uint8_t chip3[3] = {0};
  si446x_get_chip_status(chip3);

  Serial.print(F("\n[IRQ#")); Serial.print(cnt_irq);
  Serial.print(F("] INT{P,S}="));
  Serial.print(INT_PEND, HEX); Serial.print('/');
  Serial.print(INT_STATUS, HEX);
  Serial.print(F("  PH{P,S}="));
  Serial.print(PH_PEND, HEX); Serial.print('/');
  Serial.print(PH_STATUS, HEX);
  Serial.print(F(" (snap "));
  Serial.print(ph_p, HEX); Serial.print('/');
  Serial.print(ph_s, HEX); Serial.print(')');
  Serial.print(F("  MD{P,S}="));
  Serial.print(MD_PEND, HEX); Serial.print('/');
  Serial.print(MD_STATUS, HEX);
  Serial.print(F("  CH{P,S}="));
  Serial.print(CH_PEND, HEX); Serial.print('/');
  Serial.print(CH_STATUS, HEX);
  Serial.print(F("  CHIP="));
  printHex(chip3, 3);
  Serial.println();

  bool pkt_ok  = (PH_PEND & 0x10); // PACKET_RX_PEND (common mapping)
  bool pkt_err = (PH_PEND & 0x20); // CRC_ERROR (common mapping)

  // Read whatever is in RX FIFO
  uint8_t rx_cnt = 0, tx_space = 0;
  if (!si446x_fifo_info(0x00, rx_cnt, tx_space)) {
    Serial.println(F("FIFO_INFO fail"));
    rearm_rx(F("fifo_info_fail"));
    return;
  }

  if (rx_cnt > 0) {
    uint8_t buf[128];
    uint8_t n = rx_cnt > sizeof(buf) ? sizeof(buf) : rx_cnt;
    si446x_read_rx_fifo(buf, n);

    Serial.print(F("RX bytes(")); Serial.print(n); Serial.println(F("):"));
    Serial.print(F("  HEX : ")); printHex(buf, n); Serial.println();
    Serial.print(F("  ASCII: ")); printAscii(buf, n); Serial.println();

    // Try CRC check if length suggests CRC present (payload 4B + CRC 2B = 6)
    if (n >= 4) {
      uint16_t calc = crc16_ibm_msb(buf, 4); // CRC over payload only (Field1)
      uint8_t calc_hi = (calc >> 8) & 0xFF;
      uint8_t calc_lo = calc & 0xFF;

      if (n >= 6) {
        bool crc_ok = (buf[4] == calc_hi) && (buf[5] == calc_lo);
        Serial.print(F("  CRC (calc HI/LO="));
        Serial.print(calc_hi, HEX); Serial.print('/');
        Serial.print(calc_lo, HEX); Serial.print(F(") => "));
        Serial.println(crc_ok ? F("OK") : F("FAIL"));
        if (crc_ok) cnt_ok++; else cnt_err++;
      } else {
        Serial.print(F("  CRC: ")); Serial.println(F("UNKNOWN (CRC not in FIFO)"));
        if (pkt_ok && !pkt_err) cnt_ok++; else if (pkt_err) cnt_err++; // infer from PH flags
      }
    }

    // Re-arm RX
    rearm_rx(F("after_rx"));
  } else {
    cnt_emp++;
    Serial.println(F("RX IRQ but FIFO empty"));
    rearm_rx(F("empty_irq"));
  }

  // Print quick counters
  Serial.print(F("Counters: OK=")); Serial.print(cnt_ok);
  Serial.print(F(" ERR=")); Serial.print(cnt_err);
  Serial.print(F(" EMPTY=")); Serial.print(cnt_emp);
  Serial.print(F(" IRQ=")); Serial.println(cnt_irq);
}
