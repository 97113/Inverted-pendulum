#include "attitude.h"

#include <M5Unified.h>

#include <cmath>

#include "config.h"
#include "i2c_bus.h"

namespace attitude {
namespace {

constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

float pickAccel(const Raw& r, int axis, float sign) {
  return r.accel[axis] * sign;
}

}  // namespace

void Estimator::startCalibration() {
  calibRemaining_ = cfg::kGyroCalibSamples;
  calibTotal_     = cfg::kGyroCalibSamples;
  calibAccum_[0] = calibAccum_[1] = calibAccum_[2] = 0.0f;
}

float Estimator::calibrationProgress() const {
  if (calibTotal_ == 0) return 1.0f;
  return 1.0f - static_cast<float>(calibRemaining_) /
                    static_cast<float>(calibTotal_);
}

void Estimator::reset() {
  initialized_ = false;
}

bool Estimator::update(float dt) {
  {
    // 内蔵 I2C は低速タスクとの共有なので、読み出しの間だけロックする。
    i2cbus::I2cLock lock;
    M5.Imu.update();
    if (!M5.Imu.getAccel(&raw_.accel[0], &raw_.accel[1], &raw_.accel[2])) {
      return false;
    }
    if (!M5.Imu.getGyro(&raw_.gyro[0], &raw_.gyro[1], &raw_.gyro[2])) {
      return false;
    }
  }

  // ---- ジャイロバイアス較正 ----
  if (calibRemaining_ > 0) {
    for (int i = 0; i < 3; ++i) calibAccum_[i] += raw_.gyro[i];
    if (--calibRemaining_ == 0) {
      const float n = static_cast<float>(calibTotal_);
      for (int i = 0; i < 3; ++i) gyroBias_[i] = calibAccum_[i] / n;
      initialized_ = false;  // 較正後にフィルタを張り直す
    }
  }

  // ---- 加速度から傾きを求める ----
  const float num = pickAccel(raw_, cfg::kAccelPitchNumAxis, cfg::kAccelPitchNumSign);
  const float den = pickAccel(raw_, cfg::kAccelPitchDenAxis, cfg::kAccelPitchDenSign);
  accelPitch_ = std::atan2(num, den);

  // ---- ジャイロ (バイアス補正済み, rad/s) ----
  const float gyroPitch =
      (raw_.gyro[cfg::kGyroPitchAxis] - gyroBias_[cfg::kGyroPitchAxis]) *
      cfg::kGyroPitchSign * kDegToRad;
  const float gyroYaw =
      (raw_.gyro[cfg::kGyroYawAxis] - gyroBias_[cfg::kGyroYawAxis]) *
      cfg::kGyroYawSign * kDegToRad;

  pitchRate_ = gyroPitch;
  yawRate_   = gyroYaw;

  if (!initialized_) {
    pitch_ = accelPitch_;
    initialized_ = true;
    return true;
  }

  // ---- 相補フィルタ ----
  const float alpha = cfg::kCompFilterTau / (cfg::kCompFilterTau + dt);
  pitch_ = alpha * (pitch_ + gyroPitch * dt) + (1.0f - alpha) * accelPitch_;

  return true;
}

}  // namespace attitude
