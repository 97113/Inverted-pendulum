// =============================================================================
//  balance.h - 倒立振子のバランス制御
// =============================================================================
//  状態フィードバック:
//    u = Kθ·(θ − θtrim) + Kθd·θ̇ − Kx·(x − x_ref) − Kv·(v − v_ref)
//
//  u は左右共通の車輪トルク [N·m]。旋回はこれに差分トルクを重ねる。
//  θtrim は速度誤差の積分から自動学習し、重心が前後にずれていても
//  その場に留まれるようにする。
// =============================================================================
#pragma once

#include <cstdint>

namespace balance {

// 操縦入力 (正規化済み)
struct Command {
  float forward = 0.0f;   // −1..+1  前進が正
  float turn    = 0.0f;   // −1..+1  左旋回が正
};

// 制御に必要な計測値
struct Measurement {
  float pitch       = 0.0f;  // [rad]
  float pitchRate   = 0.0f;  // [rad/s]
  float yawRate     = 0.0f;  // [rad/s]
  float wheelPos    = 0.0f;  // [m]   左右平均の走行距離
  float wheelVel    = 0.0f;  // [m/s] 左右平均の走行速度
};

// 制御出力
struct Output {
  float left  = 0.0f;   // 左車輪指令 (トルクモードなら [N·m])
  float right = 0.0f;   // 右車輪指令
};

class Controller {
 public:
  // ARM 時に呼ぶ。積分項と目標位置をリセットする。
  void reset(float wheelPos);

  Output update(const Measurement& m, const Command& cmd, float dt);

  float trim() const { return trim_; }
  float posRef() const { return posRef_; }
  float lastCommon() const { return lastCommon_; }

 private:
  float trim_       = 0.0f;   // [rad] 学習した姿勢オフセット
  float posRef_     = 0.0f;   // [m]   目標位置
  float lastCommon_ = 0.0f;
  float speedState_ = 0.0f;   // 速度モードで使う積分状態 [rad/s]
};

// スティック生値 (0..maxJoy) を −1..+1 に正規化し、デッドゾーンを適用する。
float normalizeStick(uint16_t raw, uint16_t maxRaw, bool invert);

// 指令のスルーレート制限
float slew(float current, float target, float ratePerSec, float dt);

}  // namespace balance
