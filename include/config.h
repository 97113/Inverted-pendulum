// =============================================================================
//  config.h - すべての調整パラメータをここに集約
// =============================================================================
//  機体: M5Stack Core2 + PwrCAN Module 13.2 + RobStride EduLite 02 x2
//
//  座標系 (機体基準):
//    X : 右          Y : 上          Z : 前
//    pitch (θ) : 前傾が正   [rad]
//    yaw   (ψ) : 左旋回が正 [rad/s]
//    車輪の「正転」= 機体が前進する向き
// =============================================================================
#pragma once

#include <cstdint>

namespace cfg {

// -----------------------------------------------------------------------------
//  CAN バス (PwrCAN Module 13.2)
// -----------------------------------------------------------------------------
// PwrCAN 13.2 は M-BUS 上の CAN_TX / CAN_RX をディップスイッチで選ぶ。
// Core2 の M-BUS 割り当ては以下の通りなので、干渉のない組み合わせを選ぶこと。
//
//   CAN_RX 候補 : pin2 = G35(入力専用) / pin14 = G1(USB-TXD0, 不可)
//                 pin22 = G19          / pin26 = G34(入力専用)
//   CAN_TX 候補 : pin16 = G14          / pin20 = G33(PORT.A SCL)
//                 pin24 = G0(ブート端子, 非推奨)
//
// 既定値は RX = M-BUS pin2 (G35) / TX = M-BUS pin16 (G14) の組み合わせ。
// ディップスイッチを別の位置にした場合は必ずここを合わせること。
constexpr int      kCanRxPin        = 35;
constexpr int      kCanTxPin        = 14;
constexpr uint32_t kCanBitrateKbps  = 1000;   // RobStride は 1 Mbps 固定

// -----------------------------------------------------------------------------
//  モータ (RobStride EduLite 02)
// -----------------------------------------------------------------------------
// モータ ID は RobStride の PC ツールであらかじめ別々の値に設定しておくこと
// (出荷時は両方とも 127 なので、そのままでは 2 台同時に使えない)。
constexpr uint8_t kMotorIdLeft  = 1;
constexpr uint8_t kMotorIdRight = 2;

// このコントローラ (ホスト) の CAN ID。モータ ID と衝突しない値にする。
constexpr uint8_t kHostCanId = 0xFD;

// 左右のモータは向かい合わせに取り付くため、回転方向が逆になる。
// 「車輪を浮かせた状態で前進指令 → 両輪が前に回る」まで調整すること。
constexpr float kMotorDirLeft  = +1.0f;
constexpr float kMotorDirRight = -1.0f;

// ---- スケーリング定数 ----------------------------------------------------
// !! 重要 !!  以下は RobStride 02 系の一般値。必ず手元のマニュアルの
//             「パラメータ範囲」の表と突き合わせて修正すること。
//             ここがずれるとトルク指令もフィードバックも定数倍ずれる。
constexpr float kPosMax   = 12.566370614f;  // ±4π [rad]  (マニュアル要確認)
constexpr float kVelMax   = 44.0f;          // ± [rad/s]  (マニュアル要確認)
constexpr float kTorqueMax= 17.0f;          // ± [N·m]    (マニュアル要確認)
constexpr float kKpMax    = 500.0f;
constexpr float kKdMax    = 5.0f;

// 実際に出してよいトルクの上限。まずは小さい値から始めること。
constexpr float kTorqueLimit = 4.0f;        // [N·m]

// -----------------------------------------------------------------------------
//  機体パラメータ
// -----------------------------------------------------------------------------
constexpr float kWheelRadius = 0.05f;       // 車輪半径 [m]
constexpr float kTreadWidth  = 0.20f;       // 左右車輪間の距離 [m]

// -----------------------------------------------------------------------------
//  制御方式
// -----------------------------------------------------------------------------
enum class MotorMode : uint8_t {
  Torque = 0,   // run_mode=0 (MIT/運動制御) を Kp=Kd=0 で使う純トルク制御
  Speed  = 2,   // run_mode=2 (速度モード)。制御出力を加速度とみなして積分する
};
constexpr MotorMode kMotorMode = MotorMode::Torque;

// 制御周期
constexpr uint32_t kControlHz     = 200;
constexpr float    kControlDt     = 1.0f / static_cast<float>(kControlHz);

// -----------------------------------------------------------------------------
//  姿勢推定
// -----------------------------------------------------------------------------
// Core2 を「画面が進行方向を向くように直立」させた場合の既定値。
// 実機が違う向きなら DISARMED 画面の生値表示を見ながら軸と符号を直すこと。
//
//   pitch = atan2(-az, ay)      前傾で正
//   pitchRate = +gx             前傾方向の角速度が正
//   yawRate   = +gy             左旋回で正
constexpr int   kAccelPitchNumAxis = 2;     // 0=x 1=y 2=z   分子に使う軸
constexpr float kAccelPitchNumSign = -1.0f;
constexpr int   kAccelPitchDenAxis = 1;     // 分母に使う軸
constexpr float kAccelPitchDenSign = +1.0f;
constexpr int   kGyroPitchAxis     = 0;
constexpr float kGyroPitchSign     = +1.0f;
constexpr int   kGyroYawAxis       = 1;
constexpr float kGyroYawSign       = +1.0f;

// 相補フィルタの時定数 [s]。大きいほどジャイロ寄り。
constexpr float kCompFilterTau = 1.0f;

// 起動時のジャイロバイアス較正 (この間は機体を完全に静止させておくこと)
constexpr uint32_t kGyroCalibSamples = 400;   // 200Hz なら 2 秒

// -----------------------------------------------------------------------------
//  バランス制御ゲイン  (u = Kθ·θ + Kθd·θ̇ − Kx·(x−x_ref) − Kv·(v−v_ref))
// -----------------------------------------------------------------------------
// すべて正の値。単位は出力 [N·m] に対して:
//   kGainPitch     [N·m / rad]
//   kGainPitchRate [N·m / (rad/s)]
//   kGainPos       [N·m / m]
//   kGainVel       [N·m / (m/s)]
//
// 調整順序は docs/tuning.md を参照。まずは kGainPos = kGainVel = 0 で
// 姿勢だけ立たせてから、位置・速度項を足していくこと。
constexpr float kGainPitch     = 18.0f;
constexpr float kGainPitchRate = 1.2f;
constexpr float kGainPos       = 1.5f;
constexpr float kGainVel       = 2.5f;

// 重心オフセットの自動学習。速度誤差を積分して目標姿勢角を補正する。
// 0 にすると無効。
constexpr float kTrimIntegralGain = 0.02f;    // [rad / (m/s) / s]
constexpr float kTrimLimit        = 0.10f;    // [rad] 補正量の上限 (約 5.7°)

// 旋回 (ヨーレート P 制御)
constexpr float kGainYawRate = 0.35f;         // [N·m / (rad/s)]

// -----------------------------------------------------------------------------
//  操縦入力
// -----------------------------------------------------------------------------
constexpr float kMaxSpeed    = 0.8f;          // 前後の最大速度 [m/s]
constexpr float kMaxYawRate  = 2.5f;          // 最大旋回速度 [rad/s]
constexpr float kStickDeadzone = 0.10f;       // −1..1 正規化後のデッドゾーン
constexpr float kCommandSlewPerSec = 2.0f;    // 指令のスルーレート [1/s]

// スティックの上方向が joyLVert=0 なので前進は反転が必要。
// 実機で前後が逆なら false にする。
constexpr bool kInvertStickVertical = true;
constexpr bool kInvertStickYaw      = false;

// コントローラが切れたとみなすまでの無通信時間 [ms]
constexpr uint32_t kGamepadTimeoutMs = 500;

// 特定のコントローラだけに繋ぎたい場合は MAC アドレスを書く (例 "11:22:33:aa:bb:cc")。
// 空文字なら見つかった Xbox コントローラに繋ぐ。
constexpr const char* kGamepadAddress = "";

// -----------------------------------------------------------------------------
//  安全装置
// -----------------------------------------------------------------------------
constexpr float kArmMaxPitchDeg  = 5.0f;      // この範囲でないと ARM できない
constexpr float kFallPitchDeg    = 35.0f;     // これを超えたら転倒とみなす
constexpr uint32_t kCanTimeoutMs = 100;       // モータ応答が途絶えたら停止

}  // namespace cfg
