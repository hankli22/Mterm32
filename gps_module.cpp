#include "gps_module.h"
#include "config.h"
#include "ui.h"
#include "hardwareLayer.h"

bool GPSCalc::isRunning = false;
float GPSCalc::totalDistance = 0;
float GPSCalc::currentSpeed = 0;
float GPSCalc::slidingPace = 0;
int GPSCalc::paceMin = 0;
int GPSCalc::paceSec = 0;
int GPSCalc::laps = 0;
int GPSCalc::durationSec = 0;

int GPSCalc::satellites = 0;
int GPSCalc::satsInView = 0;
int GPSCalc::accuracyPct = 0;

double GPSCalc::altitude = 0;
double GPSCalc::course = 0;
bool GPSCalc::homeSet = false;
double GPSCalc::homeLat = 0, GPSCalc::homeLng = 0;
double GPSCalc::lastLat = 0, GPSCalc::lastLng = 0;

LapInfo GPSCalc::lapHistory[MAX_LAPS];
float GPSCalc::trackX[MAX_TRACK_POINTS];
float GPSCalc::trackY[MAX_TRACK_POINTS];
int GPSCalc::trackPointsCount = 0;

float GPSCalc::paceBuffer[PACE_WINDOW_SIZE] = { 0 };
int GPSCalc::paceBufIdx = 0;
uint32_t GPSCalc::lastPaceUpdate = 0;
float GPSCalc::lastDistForPace = 0;
uint32_t GPSCalc::runStartTime = 0;
uint32_t GPSCalc::lastLapTime = 0;
uint32_t GPSCalc::lastTrackSaveTime = 0;
float GPSCalc::distanceAtLastLap = 0;

SatData GPSCalc::sats[40];
int GPSCalc::satCount = 0;
int GPSCalc::sysTracked[4] = { 0 };
int GPSCalc::sysInView[4] = { 0 };

double GPSCalc::rawLat = 0;
double GPSCalc::rawLng = 0;
double GPSCalc::filtLat = 0;
double GPSCalc::filtLng = 0;
float GPSCalc::maxSpeed = 0;
int GPSCalc::calories = 0;
KalmanFilter4D GPSCalc::kalman;
bool GPSCalc::useKalman = true;

static TinyGPSPlus gps;
static TinyGPSCustom satsInViewCustom(gps, "GPGSV", 3);

SemaphoreHandle_t GPSCalc::mutex = nullptr;

void GPSCalc::lock() {
  if (mutex) xSemaphoreTake(mutex, portMAX_DELAY);
}

void GPSCalc::unlock() {
  if (mutex) xSemaphoreGive(mutex);
}

// ===================== KalmanFilter4D =====================

static constexpr float DEG2RAD = 0.01745329252f;
static constexpr float METER_PER_DEG = 111320.0f;

static inline float latToM(float dlat) { return dlat * METER_PER_DEG; }
static inline float lngToM(float dlng, float cosLat) { return dlng * METER_PER_DEG * cosLat; }

KalmanFilter4D::KalmanFilter4D() : ready_(false) {
  memset(x_, 0, sizeof(x_));
  memset(P_, 0, sizeof(P_));
}

void KalmanFilter4D::reset(double lat, double lng) {
  originLat_ = (float)lat;
  originLng_ = (float)lng;
  cosOrigin_ = cosf(originLat_ * DEG2RAD);
  x_[0] = 0;  x_[1] = 0;  x_[2] = 0;  x_[3] = 0;
  memset(P_, 0, sizeof(P_));
  P_[0][0] = R_POS;  P_[1][1] = R_POS;
  P_[2][2] = 4.0f;   P_[3][3] = 4.0f;
  ready_ = true;
}

void KalmanFilter4D::predict(float dt) {
  if (!ready_ || dt <= 0) return;

  x_[0] += x_[2] * dt;
  x_[1] += x_[3] * dt;

  // P = F*P*F' + Q
  // F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]
  // Step 1: tmp = P * F'
  float tmp[4][4];
  for (int i = 0; i < 4; i++) {
    tmp[i][0] = P_[i][0] + P_[i][2] * dt;
    tmp[i][1] = P_[i][1] + P_[i][3] * dt;
    tmp[i][2] = P_[i][2];
    tmp[i][3] = P_[i][3];
  }

  // Step 2: P_new = F * tmp + Q
  // Row i of P_new = tmp[i] + dt*tmp[i+2] (for i=0,1) or tmp[i] (for i=2,3), plus Q
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;

  P_[0][0] = tmp[0][0] + dt * tmp[2][0] + Q_ACCEL * dt4 / 4;
  P_[0][1] = tmp[0][1] + dt * tmp[2][1];
  P_[0][2] = tmp[0][2] + dt * tmp[2][2] + Q_ACCEL * dt3 / 2;
  P_[0][3] = tmp[0][3] + dt * tmp[2][3];

  P_[1][0] = tmp[1][0] + dt * tmp[3][0];
  P_[1][1] = tmp[1][1] + dt * tmp[3][1] + Q_ACCEL * dt4 / 4;
  P_[1][2] = tmp[1][2] + dt * tmp[3][2];
  P_[1][3] = tmp[1][3] + dt * tmp[3][3] + Q_ACCEL * dt3 / 2;

  P_[2][0] = tmp[2][0] + Q_ACCEL * dt3 / 2;
  P_[2][1] = tmp[2][1];
  P_[2][2] = tmp[2][2] + Q_ACCEL * dt2;
  P_[2][3] = tmp[2][3];

  P_[3][0] = tmp[3][0];
  P_[3][1] = tmp[3][1] + Q_ACCEL * dt3 / 2;
  P_[3][2] = tmp[3][2];
  P_[3][3] = tmp[3][3] + Q_ACCEL * dt2;

  // Enforce symmetry (float rounding may introduce tiny asymmetry)
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      P_[i][j] = P_[j][i] = (P_[i][j] + P_[j][i]) * 0.5f;
    }
  }
}

void KalmanFilter4D::update(double measLat, double measLng) {
  if (!ready_) return;

  float zx = lngToM((float)(measLng - originLng_), cosOrigin_);
  float zy = latToM((float)(measLat - originLat_));

  float innov[2];
  innov[0] = zx - x_[0];
  innov[1] = zy - x_[1];

  float innovDist = sqrtf(innov[0] * innov[0] + innov[1] * innov[1]);
  if (innovDist > INNOV_GATE) {
    reset(measLat, measLng);
    return;
  }

  // S = H*P*H' + R
  float s00 = P_[0][0] + R_POS;
  float s01 = P_[0][1];
  float s11 = P_[1][1] + R_POS;
  float det = s00 * s11 - s01 * s01;
  if (fabsf(det) < 1e-3f) return;

  // K = P * H' * inv(S)
  float K[4][2];
  float inv00 = s11 / det;
  float inv01 = -s01 / det;
  float inv11 = s00 / det;
  for (int i = 0; i < 4; i++) {
    K[i][0] = P_[i][0] * inv00 + P_[i][1] * inv01;
    K[i][1] = P_[i][0] * inv01 + P_[i][1] * inv11;
  }

  for (int i = 0; i < 4; i++) {
    x_[i] += K[i][0] * innov[0] + K[i][1] * innov[1];
  }

  // P = (I - K*H) * P, using temp to avoid read-after-write on rows 0,1
  float P_new[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P_new[i][j] = P_[i][j] - (K[i][0] * P_[0][j] + K[i][1] * P_[1][j]);
    }
  }
  memcpy(P_, P_new, sizeof(P_));

  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      P_[i][j] = P_[j][i] = (P_[i][j] + P_[j][i]) * 0.5f;
    }
  }
}

double KalmanFilter4D::getLat() const {
  return originLat_ + x_[1] / METER_PER_DEG;
}

double KalmanFilter4D::getLng() const {
  return originLng_ + x_[0] / (METER_PER_DEG * cosOrigin_);
}

float KalmanFilter4D::speedMps() const {
  return sqrtf(x_[2] * x_[2] + x_[3] * x_[3]);
}

float KalmanFilter4D::courseDeg() const {
  if (fabsf(x_[2]) < 0.01f && fabsf(x_[3]) < 0.01f) return 0;
  float deg = atan2f(x_[2], x_[3]) * 57.2957795f;
  if (deg < 0) deg += 360;
  return deg;
}

// ===================== GPSCalc =====================

void GPSCalc::init() {
  mutex = xSemaphoreCreateMutex();
  Serial1.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);
  Serial1.setRxBufferSize(1024);
  if (sysCfg.record_freq >= 5.0) Serial1.println("$PCAS02,200*1D");
  else if (sysCfg.record_freq >= 2.0) Serial1.println("$PCAS02,500*1A");
  else Serial1.println("$PCAS02,1000*2E");
}

void GPSCalc::startRun() {
  isRunning = true;
  totalDistance = 0;
  laps = 0;
  trackPointsCount = 0;
  homeSet = false;
  homeLat = 0;
  durationSec = 0;
  maxSpeed = 0;
  calories = 0;
  kalman = KalmanFilter4D();
  runStartTime = millis();
  lastLapTime = millis();
  for (int i = 0; i < PACE_WINDOW_SIZE; i++) paceBuffer[i] = 0;
  lastDistForPace = 0;
  distanceAtLastLap = 0;
  lastPaceUpdate = millis();
  lapHistory[0].trackStartIdx = 0;
}

void GPSCalc::stopRun() {
  isRunning = false;
  if (laps < MAX_LAPS) lapHistory[laps].trackEndIdx = trackPointsCount - 1;
}

static float calcDist(double lat1, double lng1, double lat2, double lng2) {
  return (float)TinyGPSPlus::distanceBetween(lat1, lng1, lat2, lng2);
}

String GPSCalc::getDateTime() {
  if (!gps.time.isValid() || gps.date.year() < 2020) {
    if (gps.time.isValid()) {
      char tBuf[32];
      int h = (gps.time.hour() + 8) % 24;
      sprintf(tBuf, "TIME: %02d:%02d:%02d", h, (int)gps.time.minute(), (int)gps.time.second());
      return String(tBuf);
    }
    return "WAITING SATELLITES";
  }
  char timeBuf[32];
  int localHour = gps.time.hour() + 8;
  int localDay = gps.date.day();
  if (localHour >= 24) {
    localHour -= 24;
    localDay += 1;
  }
  sprintf(timeBuf, "%02d/%02d/%02d %02d:%02d:%02d",
          (int)(gps.date.year() % 100), (int)gps.date.month(), (int)localDay,
          (int)localHour, (int)gps.time.minute(), (int)gps.time.second());
  return String(timeBuf);
}


void GPSCalc::process() {
  lock();
  static char nmeaBuf[128];
  static int nmeaPos = 0;

  if (!MenuManager::usbBridgeActive) {
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      gps.encode(c);

      if (c == '$') {
        nmeaPos = 0;
        nmeaBuf[nmeaPos++] = c;
      } else if (c == '\n' || c == '\r') {
        nmeaBuf[nmeaPos] = 0;
        if (nmeaPos > 10) parseGSV(nmeaBuf);
        nmeaPos = 0;
      } else if (nmeaPos < (int)sizeof(nmeaBuf) - 1) {
        nmeaBuf[nmeaPos++] = c;
      }
    }
  }

  cleanupSats();

  satellites = gps.satellites.value();
  if (satsInViewCustom.isUpdated() && satsInViewCustom.isValid()) {
    satsInView = atoi(satsInViewCustom.value());
  }

  double h = gps.hdop.hdop();
  if (h > 0) {
    int acc = 100 - (int)((h - 1.0) * 15);
    accuracyPct = constrain(acc, 0, 100);
  } else accuracyPct = 0;

  if (gps.location.isValid()) {
    rawLat = gps.location.lat();
    rawLng = gps.location.lng();
    altitude = gps.altitude.meters();
    course = gps.course.deg();
    currentSpeed = gps.speed.kmph();

    if (useKalman) {
      static uint32_t lastKalmanMs = 0;
      uint32_t nowMs = millis();
      if (!kalman.ready()) {
        kalman.reset(rawLat, rawLng);
        lastKalmanMs = nowMs;
      } else {
        float dt = (nowMs - lastKalmanMs) / 1000.0f;
        lastKalmanMs = nowMs;
        if (dt > 0.01f) {
          if (dt > KalmanFilter4D::DT_MAX) dt = KalmanFilter4D::DT_MAX;
          kalman.predict(dt);
        }
        kalman.update(rawLat, rawLng);
      }
      filtLat = kalman.lat();
      filtLng = kalman.lng();
    } else {
      filtLat = rawLat;
      filtLng = rawLng;
    }
  }

  if (!isRunning || !gps.location.isValid() || gps.location.age() > 2000) { unlock(); return; }

  uint32_t now = millis();
  durationSec = (now - runStartTime) / 1000;

  if (now - lastPaceUpdate >= 1000) {
    float distThisSec = totalDistance - lastDistForPace;
    lastDistForPace = totalDistance;

    paceBuffer[paceBufIdx] = distThisSec;
    paceBufIdx = (paceBufIdx + 1) % PACE_WINDOW_SIZE;
    lastPaceUpdate = now;

    float distInWindow = 0;
    for (int i = 0; i < PACE_WINDOW_SIZE; i++) distInWindow += paceBuffer[i];

    if (distInWindow > 0.5f) {
      float speedMps = distInWindow / (float)PACE_WINDOW_SIZE;
      slidingPace = 16.6667f / speedMps;
      paceMin = (int)slidingPace;
      paceSec = (int)((slidingPace - paceMin) * 60);
    } else {
      slidingPace = 0;
      paceMin = 0;
      paceSec = 0;
    }

    if (currentSpeed > maxSpeed) maxSpeed = currentSpeed;
    calories = (int)((totalDistance / 1000.0f) * 60.0f * 1.036f);
  }

  if (!homeSet) {
    if (satellites > 4 && gps.hdop.hdop() < 2.5) {
      homeSet = true;
      homeLat = filtLat;
      homeLng = filtLng;
      lastLat = filtLat;
      lastLng = filtLng;
      lastLapTime = now;
      if (laps < MAX_LAPS) lapHistory[laps].trackStartIdx = trackPointsCount;
    }
    unlock(); return;
  }

  float d = calcDist(filtLat, filtLng, lastLat, lastLng);
  if (d >= 2.0f && d < 35.0f) {
    totalDistance += d;
    lastLat = filtLat;
    lastLng = filtLng;

    if (now - lastTrackSaveTime > 2000 && trackPointsCount < MAX_TRACK_POINTS) {
      trackX[trackPointsCount] = lngToM((float)(filtLng - homeLng), cosf((float)homeLat * DEG2RAD));
      trackY[trackPointsCount] = latToM((float)(filtLat - homeLat));
      trackPointsCount++;
      lastTrackSaveTime = now;
    }
  }

  float distToHome = calcDist(filtLat, filtLng, homeLat, homeLng);
  // 回到起点 15 米内，且本圈已经跑了至少 200 米
  if (distToHome < 15.0f && (totalDistance - distanceAtLastLap) > 200.0f) {
    if (laps < MAX_LAPS) {
      lapHistory[laps].timeSec = (now - lastLapTime) / 1000;
      if (lapHistory[laps].timeSec > 0) {
        lapHistory[laps].pace = (lapHistory[laps].timeSec / 0.25f) / 60.0f;
      }
      lapHistory[laps].trackEndIdx = (trackPointsCount > 0) ? trackPointsCount - 1 : 0;
      laps++;
      distanceAtLastLap = totalDistance;
      if (laps < MAX_LAPS) {
        lapHistory[laps].trackStartIdx = trackPointsCount;
      }
    }
    lastLapTime = now;
  }
  unlock();
}

void GPSCalc::parseGSV(const char* nmea) {
  int len = strlen(nmea);
  if (len < 6 || strncmp(nmea + 3, "GSV", 3) != 0) return;

  uint8_t sys = 3;                                     // 默认 SBS/其他
  if (nmea[1] == 'G' && nmea[2] == 'P') sys = 0;       // GPS
  else if (nmea[1] == 'B' && nmea[2] == 'D') sys = 1;  // 北斗 BDS
  else if (nmea[1] == 'G' && nmea[2] == 'B') sys = 1;  // 北斗兼容名
  else if (nmea[1] == 'G' && nmea[2] == 'L') sys = 2;  // GLONASS

  int toks[20];
  int tCnt = 0;
  toks[tCnt++] = 0;
  for (int i = 0; nmea[i]; i++) {
    if (nmea[i] == ',') toks[tCnt++] = i + 1;
    else if (nmea[i] == '*') {
      toks[tCnt++] = i + 1;
      break;
    }
  }
  if (tCnt < 4) return;
  sysInView[sys] = atoi(nmea + toks[3]);

  // 解析每颗卫星 PRN, ELE, AZI, SNR
  for (int i = 4; i + 3 < tCnt; i += 4) {
    int prn = atoi(nmea + toks[i]);
    if (prn == 0) continue;
    int ele = atoi(nmea + toks[i + 1]);
    int azi = atoi(nmea + toks[i + 2]);
    int snr = atoi(nmea + toks[i + 3]);

    int idx = -1;
    for (int j = 0; j < satCount; j++) {
      if (sats[j].sys == sys && sats[j].prn == prn) {
        idx = j;
        break;
      }
    }
    if (idx == -1 && satCount < 40) idx = satCount++;
    if (idx != -1) {
      sats[idx].sys = sys;
      sats[idx].prn = prn;
      sats[idx].ele = ele;
      sats[idx].azi = azi;
      sats[idx].snr = snr;
      sats[idx].lastSeen = millis();
    }
  }
}

void GPSCalc::cleanupSats() {
  uint32_t now = millis();
  for (int i = 0; i < satCount;) {
    if (now - sats[i].lastSeen > 5000) {
      sats[i] = sats[satCount - 1];
      satCount--;
    } else i++;
  }
  for (int i = 0; i < 4; i++) sysTracked[i] = 0;
  for (int i = 0; i < satCount; i++) {
    if (sats[i].snr > 15) sysTracked[sats[i].sys]++;
  }
}


bool GPSCalc::isGpsReady() {
  return gps.charsProcessed() > 0;
}