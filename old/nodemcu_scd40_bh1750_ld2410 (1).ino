/*
 * NodeMCU-02 + SCD40 + BH1750 + LD2410 → Supabase 로거
 *
 * 필요 라이브러리 (Arduino IDE > 라이브러리 관리자):
 *   - SparkFun SCD4x Arduino Library (SparkFun)
 *   - BH1750 (Christopher Laws)
 *   - ArduinoJson (v7)
 *
 * 배선:
 *   SCD40  SDA → D2   SCL → D1   VCC → 3.3V   GND → GND
 *   BH1750 SDA → D2   SCL → D1   VCC → 3.3V   GND → GND
 *   LD2410 TX  → D7   RX  → D8   VCC → VIN(5V) GND → GND
 *   내장 LED   → D4 (경고 표시용, 별도 배선 불필요)
 *
 *   ※ SCD40, BH1750은 I2C 버스 공유 (SDA/SCL 같은 핀)
 *   ※ LD2410 VCC는 반드시 5V (VIN핀), 3.3V 연결 시 오동작
 */

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <Wire.h>
#include <SensirionI2cScd4x.h>
#include <BH1750.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// ═════════════════════════════════════════
//  ★ 여기만 수정하세요
// ═════════════════════════════════════════
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* SUPABASE_URL  = "https://xxxxxxxxxxx.supabase.co";
const char* SUPABASE_KEY  = "your-anon-key";
const char* DEVICE_ID     = "nodemcu-02";

#define LED_PIN D4          // 내장 LED (LOW=켜짐, HIGH=꺼짐)

// CO2 경고 임계값
const int   CO2_WARN_YELLOW = 1000;  // 주의 (환기 권장)
const int   CO2_WARN_RED    = 1500;  // 경고 (즉시 환기)

// 측정/전송 간격
const unsigned long SAMPLE_INTERVAL = 5000;   // 5초마다 샘플
const unsigned long SEND_INTERVAL   = 60000;  // 1분마다 전송
// ═════════════════════════════════════════

// WiFi 설정
const int    WIFI_MAX_RETRY       = 20;
const int    WIFI_RETRY_DELAY     = 500;
const unsigned long WIFI_CHECK_INTERVAL = 10000;

// 전송 실패 재시도
const int MAX_SEND_RETRY = 3;

// Watchdog
const unsigned long MAX_NO_SEND = 600000;  // 10분간 전송 없으면 재시작

// ─── 센서 객체 ───────────────────────────
SensirionI2cScd4x scd40;
BH1750            lightMeter;
SoftwareSerial    ld2410Serial(D7, D8);  // RX=D7, TX=D8

bool scd40Ok  = false;
bool bh1750Ok = false;

// ─── 누적 변수 ───────────────────────────
float sumTemp = 0, sumHumi = 0, sumLux = 0;
long  sumCo2  = 0;
int   sampleCount = 0;
int   sampleFail  = 0;

// LD2410는 평균 내지 않고 최신값 사용
bool occupied    = false;
int  distanceCm  = 0;

// ─── 타이머 ──────────────────────────────
unsigned long lastSampleTime  = 0;
unsigned long lastSendTime    = 0;
unsigned long lastWifiCheck   = 0;
unsigned long lastSuccessTime = 0;

// ─── 로그 헬퍼 ───────────────────────────
void logf(const char* tag, const char* fmt, ...) {
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.printf("[%s] %s\n", tag, buf);
}
void log(const char* tag, const char* msg) {
  Serial.printf("[%s] %s\n", tag, msg);
}

// ─── LED ────────────────────────────────
void ledBlink(int times, int ms = 300) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, LOW);  delay(ms);
    digitalWrite(LED_PIN, HIGH); delay(ms);
  }
}
void ledOff() { digitalWrite(LED_PIN, HIGH); }

// ─── WiFi ────────────────────────────────
bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  logf("WiFi", "%s 연결 중...", WIFI_SSID);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < WIFI_MAX_RETRY; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      logf("WiFi", "연결됨! IP: %s  RSSI: %ddBm",
           WiFi.localIP().toString().c_str(), WiFi.RSSI());
      ledBlink(2, 100);
      return true;
    }
    delay(WIFI_RETRY_DELAY);
    Serial.print(".");
  }
  Serial.println();
  log("WiFi", "연결 실패 — 다음 체크까지 대기");
  return false;
}

// ─── SCD40 초기화 ────────────────────────
bool initScd40() {
  scd40.begin(Wire);
  uint16_t err;

  // 혹시 이미 측정 중이면 중지
  err = scd40.stopPeriodicMeasurement();
  delay(500);

  // 시리얼 번호 확인 (통신 체크용)
  uint16_t sn[3];
  err = scd40.getSerialNumber(sn[0], sn[1], sn[2]);
  if (err) {
    logf("SCD40", "초기화 실패 (err=%d) — 배선 확인", err);
    return false;
  }
  logf("SCD40", "SN: %04X-%04X-%04X", sn[0], sn[1], sn[2]);

  // 주기 측정 시작
  err = scd40.startPeriodicMeasurement();
  if (err) {
    logf("SCD40", "측정 시작 실패 (err=%d)", err);
    return false;
  }

  log("SCD40", "초기화 성공 — 첫 측정까지 5초 대기");
  delay(5000);  // SCD40 첫 샘플 준비 대기
  return true;
}

// ─── 센서 전체 초기화 ────────────────────
void initSensors() {
  Wire.begin(D2, D1);

  scd40Ok  = initScd40();
  bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  log("BH1750", bh1750Ok ? "초기화 성공" : "초기화 실패 — 배선 확인");

  ld2410Serial.begin(115200);  // SoftwareSerial 안정 최대치 (256000은 WiFi와 충돌 위험)
  log("LD2410", "UART 초기화 완료");
}

// ─── LD2410 파싱 ─────────────────────────
// LD2410 기본 출력 프레임 형식:
// F4 F3 F2 F1 [데이터] F8 F7 F6 F5
// 데이터[0]: 상태 (0=없음, 1=이동, 2=정지, 3=이동+정지)
// 데이터[1-2]: 이동 감지 거리 (little-endian, cm)
// 데이터[3]: 이동 에너지
// 데이터[4-5]: 정지 감지 거리 (little-endian, cm)
// 데이터[6]: 정지 에너지
// 데이터[7-8]: 감지 거리 (little-endian, cm)
void readLd2410() {
  // LD2410 기본 출력 프레임 (26바이트):
  // [0-3]  헤더: F4 F3 F2 F1
  // [4-5]  데이터 길이: 0D 00 (13)
  // [6]    0x02 (데이터 타입)
  // [7]    상태: 0=없음 1=이동 2=정지 3=이동+정지
  // [8-9]  이동 감지 거리 (cm, little-endian)
  // [10]   이동 에너지
  // [11-12] 정지 감지 거리 (cm, little-endian)
  // [13]   정지 에너지
  // [14-15] 감지 거리 (cm, little-endian)
  // [16]   예약
  // [17-18] 체크섬
  // [19]   0x02
  // [20-23] 푸터: F8 F7 F6 F5
  // [24-25] 예약

  const int FRAME_SIZE = 26;

  while (ld2410Serial.available() >= FRAME_SIZE) {
    uint8_t b = ld2410Serial.read();
    if (b != 0xF4) continue;

    uint8_t frame[FRAME_SIZE];
    frame[0] = b;
    int readBytes = ld2410Serial.readBytes(frame + 1, FRAME_SIZE - 1);
    if (readBytes < FRAME_SIZE - 1) break;

    // 헤더 검증
    if (frame[1] != 0xF3 || frame[2] != 0xF2 || frame[3] != 0xF1) continue;

    // 푸터 검증 (frame[20]~[23])
    if (frame[20] != 0xF8 || frame[21] != 0xF7 ||
        frame[22] != 0xF6 || frame[23] != 0xF5) {
      log("LD2410", "푸터 불일치 — 프레임 버림");
      continue;
    }

    // 데이터 길이 확인
    uint16_t dataLen = frame[4] | (frame[5] << 8);
    if (dataLen != 13) continue;  // 표준 보고 프레임은 13바이트

    uint8_t status = frame[7];
    occupied   = (status > 0);
    distanceCm = (int)(frame[14] | (frame[15] << 8));

    break;
  }
}

// ─── setup ───────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n[시작] NodeMCU-02: SCD40 + BH1750 + LD2410 → Supabase");

  pinMode(LED_PIN, OUTPUT);
  ledOff();

  initSensors();
  connectWiFi();

  lastSampleTime  = millis() - SAMPLE_INTERVAL;
  lastSendTime    = millis() - SEND_INTERVAL;
  lastSuccessTime = millis();
}

// ─── loop ────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── WiFi 상태 체크 (10초마다) ──
  if (now - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      log("WiFi", "연결 끊김 감지 — 재연결 시도");
      connectWiFi();
    }
  }

  // ── Watchdog: 10분간 전송 없으면 재시작 ──
  if (lastSuccessTime > 0 && now - lastSuccessTime > MAX_NO_SEND) {
    log("Watchdog", "10분간 전송 없음 → 재시작");
    delay(500);
    ESP.restart();
  }

  // ── LD2410 상시 읽기 (루프마다) ──
  readLd2410();

  // ── 5초마다 샘플 수집 ──
  if (now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;

    // SCD40 읽기
    bool dataReady = false;
    uint16_t rawCo2 = 0;
    float    t = NAN, h = NAN;

    if (scd40Ok) {
      uint16_t err = scd40.getDataReadyStatus(dataReady);
      if (err) {
        // I2C 통신 오류 → 진짜 실패
        logf("SCD40", "통신 오류 (err=%d)", err);
        sampleFail++;
      } else if (dataReady) {
        err = scd40.readMeasurement(rawCo2, t, h);
        if (err || rawCo2 == 0) {
          logf("SCD40", "측정값 오류 (err=%d, co2=%d)", err, rawCo2);
          t = NAN; h = NAN; rawCo2 = 0;
          sampleFail++;
        }
      }
      // dataReady=false는 타이밍 문제일 뿐 — sampleFail 카운트 안 함
    }

    // BH1750 읽기
    float l = bh1750Ok ? lightMeter.readLightLevel() : 0;

    // 유효성 검사 후 누적
    bool tempHumiOk = !isnan(t) && !isnan(h) && t > -10 && t < 60 && h >= 0 && h <= 100;
    bool co2Ok      = rawCo2 > 400 && rawCo2 < 5000;  // 실내 유효 범위

    if (tempHumiOk && co2Ok) {
      sumTemp += t;
      sumHumi += h;
      sumCo2  += rawCo2;
      sumLux  += (l >= 0 ? l : 0);
      sampleCount++;
      sampleFail = 0;
      logf("샘플", "#%d  CO2:%dppm  온도:%.1f°C  습도:%.1f%%  조도:%.0flux  재실:%s(%dcm)",
           sampleCount, rawCo2, t, h, l, occupied ? "O" : "X", distanceCm);
    } else if (!isnan(t) || rawCo2 > 0) {
      // 값은 왔는데 범위 벗어난 경우만 실패 카운트
      sampleFail++;
      logf("경고", "범위 초과 (%d회)  tempHumi:%s  co2:%s",
           sampleFail, tempHumiOk ? "OK" : "NG", co2Ok ? "OK" : "NG");

      if (sampleFail >= 5) {
        log("복구", "연속 실패 → 센서 재초기화");
        initSensors();
        sampleFail = 0;
      }
    }
  }

  // ── 1분마다 평균값 전송 ──
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;

    if (sampleCount == 0) {
      log("전송", "유효 샘플 없음 — 건너뜀");
    } else {
      float avgTemp = sumTemp / sampleCount;
      float avgHumi = sumHumi / sampleCount;
      float avgLux  = sumLux  / sampleCount;
      int   avgCo2  = (int)(sumCo2 / sampleCount);

      logf("전송", "평균 (샘플 %d개)  CO2:%dppm  온도:%.1f°C  습도:%.1f%%  조도:%.0flux  재실:%s(%dcm)",
           sampleCount, avgCo2, avgTemp, avgHumi, avgLux, occupied ? "O" : "X", distanceCm);

      // 전송 (실패 시 재시도) — 성공 후에만 누적값 초기화
      bool sent = false;
      for (int retry = 0; retry < MAX_SEND_RETRY && !sent; retry++) {
        if (retry > 0) {
          logf("전송", "재시도 %d/%d ...", retry, MAX_SEND_RETRY - 1);
          delay(2000);
          if (WiFi.status() != WL_CONNECTED) connectWiFi();
        }
        sent = sendToSupabase(avgTemp, avgHumi, avgCo2, avgLux, occupied, distanceCm);
      }

      if (!sent) {
        log("전송", "최종 실패 — 다음 주기에 재사용 가능하도록 누적값 유지");
        ledBlink(10, 100);
        // 누적값 복원 (평균으로 근사 복원)
        sumTemp = avgTemp * sampleCount;
        sumHumi = avgHumi * sampleCount;
        sumLux  = avgLux  * sampleCount;
        sumCo2  = (long)(avgCo2 * sampleCount);
        // sampleCount는 그대로 유지
      } else {
        // 전송 성공 후 초기화
        sumTemp = sumHumi = sumLux = 0;
        sumCo2  = 0;
        sampleCount = 0;
        // ── CO2 경고 LED ──
        if (avgCo2 >= CO2_WARN_RED) {
          logf("경고", "CO2 위험! %dppm → LED 경고", avgCo2);
          ledBlink(5, 200);
        } else if (avgCo2 >= CO2_WARN_YELLOW) {
          logf("경고", "CO2 주의 %dppm → LED 경고", avgCo2);
          ledBlink(3, 400);
        } else {
          ledOff();
        }
      }
    }
  }
}

// ─── Supabase 전송 ───────────────────────
bool sendToSupabase(float temp, float humi, int co2, float lux,
                    bool occ, int dist) {
  if (WiFi.status() != WL_CONNECTED) return false;

  JsonDocument doc;
  doc["device_id"]   = DEVICE_ID;
  doc["temperature"] = round(temp * 10) / 10.0;
  doc["humidity"]    = round(humi * 10) / 10.0;
  doc["co2"]         = co2;
  doc["lux"]         = round(lux);
  doc["occupied"]    = occ;
  doc["distance"]    = dist;

  String payload;
  serializeJson(doc, payload);

  std::unique_ptr<BearSSL::WiFiClientSecure> client(new BearSSL::WiFiClientSecure);
  client->setInsecure();
  client->setTimeout(10);

  HTTPClient https;
  String url = String(SUPABASE_URL) + "/rest/v1/study_env_logs";

  if (!https.begin(*client, url)) {
    log("Supabase", "연결 실패");
    return false;
  }

  https.addHeader("Content-Type", "application/json");
  https.addHeader("apikey",        SUPABASE_KEY);
  https.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  https.addHeader("Prefer",        "return=minimal");
  https.setTimeout(10000);

  int  httpCode = https.POST(payload);
  bool success  = (httpCode == 201);

  if (success) {
    logf("Supabase", "✓ 전송 성공 (RSSI: %ddBm)", WiFi.RSSI());
    lastSuccessTime = millis();
  } else {
    logf("Supabase", "✗ 오류: %d  %s", httpCode, https.getString().c_str());
  }

  https.end();
  return success;
}
