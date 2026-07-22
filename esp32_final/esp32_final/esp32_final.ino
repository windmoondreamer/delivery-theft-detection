/*
 * ESP32 - 최종 수신기
 * - Mega로부터 UART 명령 수신
 * - ALERT_ARRIVE: 텍스트 카톡
 * - ALERT_THEFT:  Arducam으로 사진 찍어서 사진 카톡
 *
 * 핀:
 *   UART2:  RX=16, TX=17 (Mega 통신)
 *   Arducam SPI: SCK=18, MISO=19, MOSI=23, CS=5
 */
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Arducam_Mega.h>

const char* WIFI_SSID     = "여기에_WIFI_이름";
const char* WIFI_PASSWORD = "여기에_WIFI_비밀번호";
const char* WORKER_URL    = "https://delivery-alert.jjw020716.workers.dev";

const int CAM_CS = 5;
Arducam_Mega myCAM(CAM_CS);

#define PHOTO_BUF_SIZE 60000
uint8_t photoBuf[PHOTO_BUF_SIZE];
bool cameraReady = false;

// ----- WiFi -----
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi FAIL");
  }
}

// ----- 카메라 초기화 + 워밍업 -----
void initCamera() {
  Serial.print("Initializing camera... ");
  CamStatus status = myCAM.begin();
  if (status != CAM_ERR_SUCCESS) {
    Serial.print("FAIL ");
    Serial.println(status);
    cameraReady = false;
    return;
  }
  Serial.println("OK");

  delay(2000);

  // 워밍업 한 장 (버림)
  Serial.println("Camera warmup...");
  myCAM.takePicture(CAM_IMAGE_MODE_QVGA, CAM_IMAGE_PIX_FMT_JPG);
  uint32_t warmupLen = myCAM.getTotalLength();
  uint32_t startWait = millis();
  uint32_t discardCount = 0;
  while (discardCount < warmupLen && (millis() - startWait) < 5000) {
    if (myCAM.getReceivedLength() > 0) {
      myCAM.readByte();
      discardCount++;
    } else {
      delay(1);
    }
  }
  Serial.print("Warmup discarded: ");
  Serial.println(discardCount);

  cameraReady = true;
}

// ----- 사진 캡처 -----
// 성공하면 받은 바이트 수 반환 (FF D9까지 포함), 실패하면 0
size_t capturePhoto() {
  if (!cameraReady) return 0;

  Serial.println("Taking real picture...");
  myCAM.takePicture(CAM_IMAGE_MODE_QVGA, CAM_IMAGE_PIX_FMT_JPG);
  delay(1000);

  uint32_t totalLen = myCAM.getTotalLength();
  Serial.print("getTotalLength: ");
  Serial.println(totalLen);

  uint8_t prev = 0, lastByte = 0;
  bool sawFFD9 = false;
  size_t readCount = 0;
  uint32_t lastByteTime = millis();
  const uint32_t TIMEOUT_MS = 5000;

  while (!sawFFD9 && readCount < PHOTO_BUF_SIZE && (millis() - lastByteTime) < TIMEOUT_MS) {
    if (myCAM.getReceivedLength() > 0) {
      uint8_t b = myCAM.readByte();
      photoBuf[readCount++] = b;
      if (lastByte == 0xFF && b == 0xD9) sawFFD9 = true;
      prev = lastByte;
      lastByte = b;
      lastByteTime = millis();
    } else {
      delay(1);
    }
  }

  Serial.print("Bytes read: ");
  Serial.println(readCount);
  Serial.print("FF D9 found? ");
  Serial.println(sawFFD9 ? "YES" : "NO");

  if (sawFFD9) return readCount;
  return 0;
}

// ----- 텍스트 알림 -----
void sendTextAlert(const char* msg) {
  Serial.print("[sendTextAlert] ");
  Serial.println(msg);
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, WORKER_URL);
  http.addHeader("Content-Type", "application/json");
  String body = String("{\"text\":\"") + msg + "\"}";
  int code = http.POST(body);
  Serial.print("HTTP code: "); Serial.println(code);
  if (code > 0) Serial.println(http.getString());
  http.end();
}

// ----- 사진 알림 -----
void sendPhotoAlert(uint8_t* buf, size_t len, const char* msg) {
  Serial.print("[sendPhotoAlert] len=");
  Serial.println(len);
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, WORKER_URL);
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("X-Alert-Text", msg);
  int code = http.POST(buf, len);
  Serial.print("HTTP code: "); Serial.println(code);
  if (code > 0) Serial.println(http.getString());
  http.end();
}

// ----- Mega로부터 한 줄 읽기 -----
String readLineFromMega(uint32_t timeoutMs) {
  String line = "";
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n') return line;
      if (c != '\r') line += c;
    }
  }
  return "";
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(1000);

  Serial.println();
  Serial.println("==================================");
  Serial.println(" ESP32 Final Receiver");
  Serial.println("==================================");

  connectWiFi();
  initCamera();

  Serial.println("Waiting for Mega commands...");
}

void loop() {
  if (!Serial2.available()) return;

  String line = readLineFromMega(2000);
  if (line.length() == 0) return;

  Serial.print("RX: '");
  Serial.print(line);
  Serial.println("'");

  if (line == "ALERT_ARRIVE") {
    sendTextAlert("📦 택배가 도착했습니다!");
  }
  else if (line == "ALERT_THEFT") {
    size_t photoLen = capturePhoto();
    if (photoLen > 100) {
      sendPhotoAlert(photoBuf, photoLen, "🚨 택배 도난 감지!");
    } else {
      Serial.println("Photo capture FAILED, sending text only");
      sendTextAlert("🚨 택배 도난 감지! (사진 실패)");
    }
  }
}