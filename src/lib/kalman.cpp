#include "kalman.h"
#include <math.h>
#include <string.h>

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

  float tmp[4][4];
  for (int i = 0; i < 4; i++) {
    tmp[i][0] = P_[i][0] + P_[i][2] * dt;
    tmp[i][1] = P_[i][1] + P_[i][3] * dt;
    tmp[i][2] = P_[i][2];
    tmp[i][3] = P_[i][3];
  }

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

  float s00 = P_[0][0] + R_POS;
  float s01 = P_[0][1];
  float s11 = P_[1][1] + R_POS;
  float det = s00 * s11 - s01 * s01;
  if (fabsf(det) < 1e-3f) return;

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
