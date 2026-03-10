/*
 * NodeMCU-02 + SCD40 + BH1750 + LD2410 → Supabase 로거
 *
 * 필요 라이브러리 (Arduino IDE > 라이브러리 관리자):
 *   - SparkFun SCD4x Arduino Library (SparkFun)
 *   - BH1750 (Christopher Laws)
 *   - MyLD2410 (iavorvel)
 *   - ArduinoJson (v7)
 *
 * 배선:
 *   SCD40  SDA → D2   SCL → D1   VCC → 3.3V    GND → GND
 *   BH1750 SDA → D2   SCL → D1   VCC → 3.3V    GND → GND
 *   LD2410 TX  → D5   RX  → D6   VCC → VIN(5V) GND → GND
 *   내장 LED   → D4 (경고 표시용, 별도 배선 불필요)
 *
 *   ※ SCD40, BH1750은 I2C 버스 공유 (SDA/SCL 같은 핀)
 *   ※ LD2410 VCC는 반드시 5V (VIN핀), 3.3V 연결 시 오동작
 *   ※ LD2410 보드레이트: HLKRadarTool로 115200으로 변경 필요
 */

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <Wire.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
#include <BH1750.h>
#include <SoftwareSerial.h>
#include <MyLD2410.h>
#include <ArduinoJson.h>

// ═════════════════════════════════════════
//  ★ 여기만 수정하세요
// ═════════════════════════════════════════
// WiFi AP 목록 (접속 가능한 것에 자동 연결)
struct WifiAP { const char* ssid; const char* password; };
const WifiAP WIFI_APS[] = {
  { "doyoon1", "20080111" },
  { "doyoon3", "20080111" },
  { "doogie69", "20080111" },
  // 필요하면 더 추가 가능
};

const char* SUPABASE_URL  = "https://dflyecjzeulllamfpaoe.supabase.co";
const char* SUPABASE_KEY  = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImRmbHllY2p6ZXVsbGxhbWZwYW9lIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzMwNDY5NjEsImV4cCI6MjA4ODYyMjk2MX0.tRc1bXQjMJE4yy24EIFjLgAPWjXiPhsyKmV6C2RGBAY";
const char* DEVICE_ID     = "nodemcu-02";

#define LED_PIN D4          // 내장 LED (LOW=켜짐, HIGH=꺼짐)

// CO2 경고 임계값
const int   CO2_WARN_YELLOW = 1000;  // 주의 (환기 권장)
const int   CO2_WARN_RED    = 1500;  // 경고 (즉시 환기)

// 측정/전송 간격
const unsigned long SAMPLE_INTERVAL = 5000;   // 5초마다 샘플
const unsigned long SEND_INTERVAL   = 30000;  // 1분마다 전송
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
ESP8266WiFiMulti wifiMulti;
SCD4x            scd40;
BH1750           lightMeter;
SoftwareSerial   radarSerial(D5, D6);  // RX=D5(LD2410 TX), TX=D6(LD2410 RX)
MyLD2410         radar(radarSerial);

bool scd40Ok  = false;
bool bh1750Ok = false;

// ─── 누적 변수 ───────────────────────────
float sumTemp = 0, sumHumi = 0, sumLux = 0;
long  sumCo2  = 0;
int   sampleCount = 0;
int   sampleFail  = 0;

// ─── 급변 감지 (방향성) ──────────────────
// 연속 N회 같은 방향으로 변화하면 급변으로 판단
const int   TREND_COUNT   = 3;      // 연속 몇 회 같은 방향이면 급변
const int   TREND_CO2_MIN = 20;     // CO2 방향 감지 최소 변화량 (ppm)
const float TREND_TEMP_MIN = 0.3;   // 온도 방향 감지 최소 변화량 (°C)
const float TREND_HUMI_MIN = 1.0;   // 습도 방향 감지 최소 변화량 (%)

float prevCo2  = 0, prevTemp = NAN, prevHumi = NAN;
int   trendCo2 = 0, trendTemp = 0,  trendHumi = 0;
// 양수 = 상승 중, 음수 = 하강 중, 0 = 방향 없음

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

  // AP 목록 등록 (중복 등록 방지)
  static bool apsAdded = false;
  if (!apsAdded) {
    WiFi.mode(WIFI_STA);
    for (auto& ap : WIFI_APS) wifiMulti.addAP(ap.ssid, ap.password);
    apsAdded = true;
  }

  log("WiFi", "주변 AP 스캔 중...");
  int attempts = 0;
  while (wifiMulti.run() != WL_CONNECTED && attempts < WIFI_MAX_RETRY) {
    delay(WIFI_RETRY_DELAY);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    logf("WiFi", "연결됨! SSID: %s  IP: %s  RSSI: %ddBm",
         WiFi.SSID().c_str(),
         WiFi.localIP().toString().c_str(),
         WiFi.RSSI());
    ledBlink(2, 100);
    return true;
  }

  log("WiFi", "연결 실패 — 다음 체크까지 대기");
  return false;
}

// ─── SCD40 초기화 ────────────────────────
bool initScd40() {
  // begin()이 내부적으로 stopPeriodicMeasurement → getSerialNumber → startPeriodicMeasurement 처리
  if (!scd40.begin()) {
    log("SCD40", "초기화 실패 — 배선 확인");
    return false;
  }
  log("SCD40", "초기화 성공 — 첫 측정까지 5초 대기");
  delay(5000);
  return true;
}

// ─── 센서 전체 초기화 ────────────────────
void initSensors() {
  Wire.begin(D2, D1);

  scd40Ok  = initScd40();
  bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  log("BH1750", bh1750Ok ? "초기화 성공" : "초기화 실패 — 배선 확인");

  radarSerial.begin(115200);
  if (radar.begin()) {
    log("LD2410", "초기화 성공");
  } else {
    log("LD2410", "초기화 실패 — 배선 확인 (TX→D5, RX→D6, VCC→5V)");
  }
}

// ─── LD2410 읽기 ─────────────────────────
void readLd2410() {
  radar.check();  // 프레임 처리 — 루프마다 호출

  occupied   = radar.presenceDetected();
  distanceCm = radar.movingTargetDetected()
               ? radar.movingTargetDistance()
               : radar.stationaryTargetDetected()
                 ? radar.stationaryTargetDistance()
                 : 0;
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

    // SCD40 읽기 (SparkFun 라이브러리)
    uint16_t rawCo2 = 0;
    float    t = NAN, h = NAN;

    if (scd40Ok) {
      if (scd40.readMeasurement()) {
        // readMeasurement()가 true면 새 데이터 있음
        rawCo2 = scd40.getCO2();
        t      = scd40.getTemperature();
        h      = scd40.getHumidity();
        if (rawCo2 == 0) {
          logf("SCD40", "측정값 0 — 건너뜀");
          t = NAN; h = NAN; rawCo2 = 0;
          sampleFail++;
        }
      }
      // readMeasurement()가 false면 데이터 미준비 — sampleFail 카운트 안 함
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

      // ── 방향성 감지 ──
      if (prevCo2 > 0) {
        int diffCo2 = (int)rawCo2 - (int)prevCo2;
        if      (abs(diffCo2) >= TREND_CO2_MIN && diffCo2 > 0) trendCo2 = max(trendCo2 + 1, 1);
        else if (abs(diffCo2) >= TREND_CO2_MIN && diffCo2 < 0) trendCo2 = min(trendCo2 - 1, -1);
        else                                                     trendCo2 = 0;
      }
      if (!isnan(prevTemp)) {
        float diffTemp = t - prevTemp;
        if      (abs(diffTemp) >= TREND_TEMP_MIN && diffTemp > 0) trendTemp = max(trendTemp + 1, 1);
        else if (abs(diffTemp) >= TREND_TEMP_MIN && diffTemp < 0) trendTemp = min(trendTemp - 1, -1);
        else                                                        trendTemp = 0;
      }
      if (!isnan(prevHumi)) {
        float diffHumi = h - prevHumi;
        if      (abs(diffHumi) >= TREND_HUMI_MIN && diffHumi > 0) trendHumi = max(trendHumi + 1, 1);
        else if (abs(diffHumi) >= TREND_HUMI_MIN && diffHumi < 0) trendHumi = min(trendHumi - 1, -1);
        else                                                        trendHumi = 0;
      }

      prevCo2  = rawCo2;
      prevTemp = t;
      prevHumi = h;

      logf("방향", "CO2:%+d  온도:%+d  습도:%+d",
           trendCo2, trendTemp, trendHumi);

      // ── 급변 감지 → 즉시 전송 ──
      bool surge =
        abs(trendCo2)  >= TREND_COUNT ||
        abs(trendTemp) >= TREND_COUNT ||
        abs(trendHumi) >= TREND_COUNT;

      if (surge && sampleCount > 0) {
        logf("급변", "방향 연속 %d회 감지 → 즉시 전송 (CO2:%+d 온도:%+d 습도:%+d)",
             TREND_COUNT, trendCo2, trendTemp, trendHumi);

        float urgTemp = sumTemp / sampleCount;
        float urgHumi = sumHumi / sampleCount;
        float urgLux  = sumLux  / sampleCount;
        int   urgCo2  = (int)(sumCo2 / sampleCount);

        bool sent = sendToSupabase(urgTemp, urgHumi, urgCo2, urgLux, occupied, distanceCm);
        if (sent) {
          sumTemp = sumHumi = sumLux = 0;
          sumCo2  = 0;
          sampleCount = 0;
          lastSendTime = now;   // 1분 타이머 리셋
          trendCo2 = trendTemp = trendHumi = 0;  // 방향 카운터 리셋
          log("급변", "즉시 전송 성공 — 타이머 리셋");
        }
      }

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
