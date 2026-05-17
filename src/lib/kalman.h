#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <math.h>

static constexpr float DEG2RAD       = 0.01745329252f;
static constexpr float METER_PER_DEG = 111320.0f;

inline float latToM(float dlat)              { return dlat * METER_PER_DEG; }
inline float lngToM(float dlng, float cosLat) { return dlng * METER_PER_DEG * cosLat; }

// 4-state Kalman filter: east, north (m from origin), v_east, v_north (m/s)
class KalmanFilter4D {
public:
  KalmanFilter4D();

  static constexpr float Q_ACCEL = 0.25f;
  static constexpr float R_POS   = 25.0f;
  static constexpr float INNOV_GATE = 50.0f;
  static constexpr float DT_MAX  = 2.0f;

  void reset(double lat, double lng);
  void predict(float dt);
  void update(double measLat, double measLng);
  bool ready() const { return ready_; }
  double getLat() const;
  double getLng() const;
  double lat() const { return getLat(); }
  double lng() const { return getLng(); }
  float speedMps() const;
  float courseDeg() const;

private:
  float x_[4];
  float P_[4][4];
  float originLat_, originLng_;
  float cosOrigin_;
  bool ready_;
};

#endif
