// =============================================================================
//  attitude.h - Core2 内蔵 IMU による姿勢推定
// =============================================================================
//  加速度から求めた傾き (低周波は正しいがノイズと並進加速度に弱い) と、
//  ジャイロ積分 (高周波は正しいがドリフトする) を相補フィルタで統合する。
// =============================================================================
#pragma once

#include <cstdint>

namespace attitude {

struct Raw {
  float accel[3] = {0.0f, 0.0f, 0.0f};  // [g]
  float gyro[3]  = {0.0f, 0.0f, 0.0f};  // [deg/s]
};

class Estimator {
 public:
  // ジャイロのゼロ点較正を開始する。機体を静止させた状態で呼ぶこと。
  void startCalibration();
  bool isCalibrating() const { return calibRemaining_ > 0; }
  float calibrationProgress() const;

  // 制御周期ごとに呼ぶ。IMU の読み出しに失敗したら false。
  bool update(float dt);

  float pitch() const { return pitch_; }           // [rad] 前傾が正
  float pitchRate() const { return pitchRate_; }   // [rad/s]
  float yawRate() const { return yawRate_; }       // [rad/s] 左旋回が正

  // 加速度のみから求めた傾き。ARM 判定と収束確認に使う。
  float pitchFromAccel() const { return accelPitch_; }

  const Raw& raw() const { return raw_; }

  // フィルタの内部状態を、いまの加速度から求めた角度で初期化する。
  void reset();

 private:
  Raw   raw_;
  float pitch_      = 0.0f;
  float pitchRate_  = 0.0f;
  float yawRate_    = 0.0f;
  float accelPitch_ = 0.0f;

  float gyroBias_[3] = {0.0f, 0.0f, 0.0f};
  float calibAccum_[3] = {0.0f, 0.0f, 0.0f};
  uint32_t calibRemaining_ = 0;
  uint32_t calibTotal_     = 0;

  bool initialized_ = false;
};

}  // namespace attitude
