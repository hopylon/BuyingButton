#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <MFRC522.h>

// --- 설정값 ---
#define EEPROM_SIZE 128
#define SSID_ADDR 0
#define PW_ADDR 64
#define BOOT_BUTTON 0

#define RST_PIN 22
#define SS_PIN 5

const char* ap_ssid = "BuyingButtonScanner";
const char* ap_password = "12345678";

const char* supabaseUrl = "https://ddmlsstiqushllizfkxk.supabase.co";
const char* supabaseKey = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImRkbWxzc3RpcXVzaGxsaXpma3hrIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDQxNjQxMzQsImV4cCI6MjA1OTc0MDEzNH0.gEt-GcHdM5frZO8DMLyJMYLEik5oJXK5AyBsb5luAnc";  // 실제 서비스키 입력
const String user_id = "d9272e61-55eb-4573-a3a4-6cf98c9a2e24"; // 기기 고유 user_id

WebServer server(80);
MFRC522 rfid(SS_PIN, RST_PIN);

// --- EEPROM 처리 ---
void saveCredentialsToEEPROM(String ssid, String password) {
  for (int i = 0; i < 32; i++) {
    EEPROM.write(SSID_ADDR + i, i < ssid.length() ? ssid[i] : 0);
    EEPROM.write(PW_ADDR + i, i < password.length() ? password[i] : 0);
  }
  EEPROM.commit();
}

void readCredentialsFromEEPROM(String &ssid, String &password) {
  char ssid_buf[33] = {0};
  char pw_buf[33] = {0};
  for (int i = 0; i < 32; i++) {
    ssid_buf[i] = EEPROM.read(SSID_ADDR + i);
    pw_buf[i] = EEPROM.read(PW_ADDR + i);
  }
  ssid = String(ssid_buf);
  password = String(pw_buf);
}

void checkForResetRequest() {
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  delay(1000);
  if (digitalRead(BOOT_BUTTON) == LOW) {
    Serial.println("[!] BOOT 버튼 눌림: EEPROM 초기화 중...");
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
    ESP.restart();
  }
}

// --- Supabase 전송 ---
void sendToSupabase(String barcode, String unique_code) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(String(supabaseUrl) + "/rest/v1/consumed_items");
  http.addHeader("apikey", supabaseKey);
  http.addHeader("Authorization", "Bearer " + String(supabaseKey));
  http.addHeader("Content-Type", "application/json");

  String payload = "{\"user_id\": \"" + user_id + "\", \"barcode\": \"" + barcode + "\", \"unique_code\": \"" + unique_code + "\"}";

  int httpResponseCode = http.POST(payload);
  Serial.print("[Supabase] 응답 코드: ");
  Serial.println(httpResponseCode);
  if (httpResponseCode > 0) {
    Serial.println(http.getString());
  }

  http.end();
}

// --- RFID 블록 읽기 함수 ---
String readBlock(byte blockAddr) {
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  byte buffer[18];
  byte size = sizeof(buffer);

  MFRC522::StatusCode status = rfid.PCD_Authenticate(
    MFRC522::PICC_CMD_MF_AUTH_KEY_A,
    blockAddr,
    &key,
    &(rfid.uid)
  );
  if (status != MFRC522::STATUS_OK) {
    Serial.print("❌ 인증 실패: ");
    Serial.println(rfid.GetStatusCodeName(status));
    return "";
  }

  status = rfid.MIFARE_Read(blockAddr, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("❌ 블록 읽기 실패: ");
    Serial.println(rfid.GetStatusCodeName(status));
    return "";
  }

  String result = "";
  for (int i = 0; i < 16; i++) {
    if (buffer[i] >= 32 && buffer[i] <= 126) {
      result += (char)buffer[i];
    }
  }

  return result;
}

// --- RFID 태그 리딩 ---
void readRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  String barcode = readBlock(4);
  String unique_code = readBlock(5);

  if (barcode.length() > 0 && unique_code.length() > 0) {
    Serial.println("✅ 태그 감지됨");
    Serial.println(" → barcode: " + barcode);
    Serial.println(" → unique_code: " + unique_code);
    sendToSupabase(barcode, unique_code);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// --- HTTP 처리 ---
void handleRoot() {
  server.send(200, "text/plain", "ESP32 WiFi Setup Ready");
}

void handleWiFiConfig() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String received_ssid = server.arg("ssid");
    String received_password = server.arg("password");

    server.send(200, "text/plain", "WiFi 정보 수신 완료");

    Serial.println("[*] 수신된 SSID: " + received_ssid);
    Serial.println("[*] 수신된 PW: " + received_password);

    saveCredentialsToEEPROM(received_ssid, received_password);

    WiFi.softAPdisconnect(true);
    WiFi.begin(received_ssid.c_str(), received_password.c_str());
    Serial.print("[*] WiFi 연결 시도 중");

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[+] WiFi 연결 성공!");
      Serial.print("IP 주소: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\n[!] WiFi 연결 실패");
    }
  } else {
    server.send(400, "text/plain", "ssid 또는 password 누락");
  }
}

// --- 초기화 ---
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  checkForResetRequest();

  SPI.begin();
  rfid.PCD_Init();
  Serial.println("[+] RFID 리더 초기화 완료");

  String ssid, pw;
  readCredentialsFromEEPROM(ssid, pw);

  if (ssid.length() > 0 && pw.length() > 0) {
    WiFi.begin(ssid.c_str(), pw.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid, ap_password);
      Serial.print("SoftAP IP: ");
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("\n[+] WiFi 연결 성공");
    }
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password);
    Serial.print("SoftAP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  server.on("/", handleRoot);
  server.on("/setwifi", HTTP_POST, handleWiFiConfig);
  server.begin();
  Serial.println("[+] HTTP 서버 시작됨");
}

// --- 루프 ---
void loop() {
  server.handleClient();
  readRFID();
}
