#include "balance.h"

#include <algorithm>
#include <cmath>

#include "config.h"

namespace balance {
namespace {

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

float normalizeStick(uint16_t raw, uint16_t maxRaw, bool invert) {
  const float center = static_cast<float>(maxRaw) * 0.5f;
  float v = (static_cast<float>(raw) - center) / center;  // −1..+1
  if (invert) v = -v;

  const float dz = cfg::kStickDeadzone;
  if (std::fabs(v) <= dz) return 0.0f;
  // デッドゾーンの外側を −1..+1 に線形に張り直す (段差をなくす)
  v = (v - std::copysign(dz, v)) / (1.0f - dz);
  return clampf(v, -1.0f, 1.0f);
}

float slew(float current, float target, float ratePerSec, float dt) {
  const float maxStep = ratePerSec * dt;
  const float diff = target - current;
  if (diff > maxStep) return current + maxStep;
  if (diff < -maxStep) return current - maxStep;
  return target;
}

void Controller::reset(float wheelPos) {
  trim_       = 0.0f;
  posRef_     = wheelPos;
  lastCommon_ = 0.0f;
  speedState_ = 0.0f;
}

Output Controller::update(const Measurement& m, const Command& cmd, float dt) {
  // ---- 目標値 ----
  const float velRef = cmd.forward * cfg::kMaxSpeed;      // [m/s]
  const float yawRef = cmd.turn * cfg::kMaxYawRate;       // [rad/s]

  // 前進指令中は位置項を効かせない (位置追従ではなく速度追従にする)。
  // スティックを戻したその場所が、そのまま新しい保持位置になる。
  if (cmd.forward != 0.0f) {
    posRef_ = m.wheelPos;
  }

  // ---- 重心オフセットの自動学習 ----
  if (cfg::kTrimIntegralGain > 0.0f) {
    trim_ += cfg::kTrimIntegralGain * (m.wheelVel - velRef) * dt;
    trim_ = clampf(trim_, -cfg::kTrimLimit, cfg::kTrimLimit);
  }

  // ---- 状態フィードバック ----
  float u = cfg::kGainPitch     * (m.pitch - trim_)
          + cfg::kGainPitchRate * m.pitchRate
          - cfg::kGainPos       * (m.wheelPos - posRef_)
          - cfg::kGainVel       * (m.wheelVel - velRef);
  lastCommon_ = u;

  Output out;

  if (cfg::kMotorMode == cfg::MotorMode::Torque) {
    // ---- トルクモード ----
    // 旋回はヨーレートの P 制御による差分トルク。
    const float yawTorque = cfg::kGainYawRate * (yawRef - m.yawRate);

    float left  = u - yawTorque;
    float right = u + yawTorque;

    out.left  = clampf(left,  -cfg::kTorqueLimit, cfg::kTorqueLimit);
    out.right = clampf(right, -cfg::kTorqueLimit, cfg::kTorqueLimit);
  } else {
    // ---- 速度モード ----
    // u を車輪の角加速度 [rad/s^2] とみなして積分し、速度指令を作る。
    // トルクモードとは次元が違うため、ゲインは別途調整が必要。
    speedState_ += u * dt;
    speedState_ = clampf(speedState_, -cfg::kVelMax, cfg::kVelMax);

    // 旋回は左右の速度差で与える (ヨーレート指令をそのまま運動学で変換)。
    const float dOmega = yawRef * cfg::kTreadWidth * 0.5f / cfg::kWheelRadius;

    out.left  = clampf(speedState_ - dOmega, -cfg::kVelMax, cfg::kVelMax);
    out.right = clampf(speedState_ + dOmega, -cfg::kVelMax, cfg::kVelMax);
  }

  return out;
}

}  // namespace balance
