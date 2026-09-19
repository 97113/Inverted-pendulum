// =============================================================================
//  robstride.h - RobStride / CyberGear 系アクチュエータの CAN ドライバ
// =============================================================================
//  フレーム形式: CAN 2.0B 拡張 ID (29bit), 1 Mbps, データ長 8 固定
//
//    bit 28..24 : 通信種別 (Communication Type)
//    bit 23.. 8 : データ領域 2 (種別ごとに意味が変わる)
//    bit  7.. 0 : 宛先 (モータ) の CAN ID
// =============================================================================
#pragma once

#include <cstdint>

#include "config.h"

namespace robstride {

// 通信種別
enum class CommType : uint8_t {
  GetDeviceId   = 0,
  MotionControl = 1,   // 運動制御指令 (MIT 形式)
  Feedback      = 2,   // モータ → ホストの状態フィードバック
  Enable        = 3,
  Stop          = 4,
  SetMechZero   = 6,
  SetCanId      = 7,
  ReadParam     = 17,
  WriteParam    = 18,
  FaultFeedback = 21,
};

// パラメータインデックス (書き込み/読み出しで使用)
enum class ParamIndex : uint16_t {
  RunMode     = 0x7005,  // uint8 : 0=運動制御 1=位置 2=速度 3=電流
  IqRef       = 0x7006,  // float : 電流モードの目標 iq [A]
  SpdRef      = 0x700A,  // float : 速度モードの目標速度 [rad/s]
  LimitTorque = 0x700B,  // float : トルク上限 [N·m]
  CurKp       = 0x7010,
  CurKi       = 0x7011,
  CurFiltGain = 0x7014,
  LocRef      = 0x7016,  // float : 位置モードの目標角 [rad]
  LimitSpd    = 0x7017,  // float : 位置モードの速度上限 [rad/s]
  LimitCur    = 0x7018,  // float : 電流上限 [A]
  MechPos     = 0x7019,
  Iqf         = 0x701A,
  MechVel     = 0x701B,
  VBus        = 0x701C,  // float : 母線電圧 [V]
  LocKp       = 0x701E,
  SpdKp       = 0x701F,
  SpdKi       = 0x7020,
};

enum class RunMode : uint8_t {
  Motion   = 0,
  Position = 1,
  Speed    = 2,
  Current  = 3,
};

// フィードバックフレームに載るモータ状態
struct MotorState {
  float    angle       = 0.0f;  // [rad] 機械原点基準 (±kPosMax に折り返す)
  float    velocity    = 0.0f;  // [rad/s]
  float    torque      = 0.0f;  // [N·m]
  float    temperature = 0.0f;  // [degC]
  uint8_t  faults      = 0;     // 故障ビット (bit0.. 下記)
  uint8_t  mode        = 0;     // 0=Reset 1=Cali 2=Motor
  uint32_t updatedAtMs = 0;
  bool     valid       = false;
};

// faults のビット定義
enum FaultBits : uint8_t {
  kFaultUncalibrated = 1 << 0,
  kFaultHallEncoder  = 1 << 1,
  kFaultMagEncoder   = 1 << 2,
  kFaultOverTemp     = 1 << 3,
  kFaultOverCurrent  = 1 << 4,
  kFaultUnderVoltage = 1 << 5,
};

// -----------------------------------------------------------------------------
//  CAN バス (ESP32 内蔵 TWAI)
// -----------------------------------------------------------------------------
class Bus {
 public:
  bool begin(int txPin, int rxPin, uint32_t bitrateKbps);
  void end();

  // 送信。成功したら true。
  bool send(uint32_t extId, const uint8_t data[8], uint32_t timeoutMs = 2);

  // 受信キューから 1 フレーム取り出す。無ければ false。
  bool receive(uint32_t* extId, uint8_t data[8], uint32_t timeoutMs = 0);

  // バスオフからの自動復帰。制御ループから毎周期呼ぶ。
  void poll();

  bool     isBusOff() const { return busOff_; }
  uint32_t recoveryCount() const { return recoveryCount_; }

 private:
  bool     started_       = false;
  bool     busOff_        = false;
  uint32_t recoveryCount_ = 0;
  uint32_t recoverAtMs_   = 0;
};

// -----------------------------------------------------------------------------
//  モータ 1 台
// -----------------------------------------------------------------------------
class Motor {
 public:
  Motor(Bus& bus, uint8_t motorId) : bus_(bus), id_(motorId) {}

  uint8_t id() const { return id_; }

  bool enable();
  bool stop(bool clearFault = false);
  bool setMechZero();

  bool writeParam(ParamIndex index, float value);
  bool writeParam(ParamIndex index, uint8_t value);
  bool requestParam(ParamIndex index);          // 応答は Feedback ではなく別フレーム
  bool setRunMode(RunMode mode);

  // 運動制御指令 (MIT 形式)。torque 以外を 0 にすれば純トルク制御になる。
  bool sendMotion(float torque, float angle, float velocity, float kp, float kd);

  // 速度モード用
  bool sendSpeed(float velocity);

  // 自分宛のフレームなら状態を取り込んで true を返す。
  bool consume(uint32_t extId, const uint8_t data[8]);

  const MotorState& state() const { return state_; }

  // 多回転を考慮した累積角 [rad]。consume() のたびに更新される。
  float accumulatedAngle() const { return accumAngle_; }
  void  resetAccumulatedAngle() { accumAngle_ = 0.0f; hasPrevAngle_ = false; }

  // 直近にパラメータ読み出し応答で得られた値
  float lastParamValue() const { return lastParamValue_; }
  ParamIndex lastParamIndex() const { return lastParamIndex_; }

 private:
  bool sendFrame(CommType type, uint16_t data2, const uint8_t payload[8]);

  Bus&       bus_;
  uint8_t    id_;
  MotorState state_;
  float      accumAngle_    = 0.0f;
  float      prevAngle_     = 0.0f;
  bool       hasPrevAngle_  = false;
  float      lastParamValue_ = 0.0f;
  ParamIndex lastParamIndex_ = ParamIndex::VBus;
};

}  // namespace robstride
