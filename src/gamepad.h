// =============================================================================
//  gamepad.h - Xbox Series X|S コントローラ (BLE) の薄いラッパ
// =============================================================================
//  BLE のスキャン/接続処理はブロックすることがあるため、制御ループとは
//  別タスクで onLoop() を回し、結果をスナップショットで受け渡す。
// =============================================================================
#pragma once

#include <cstdint>

#include "balance.h"

namespace gamepad {

struct Snapshot {
  balance::Command command;
  bool     connected     = false;
  bool     btnArm        = false;   // A
  bool     btnDisarm     = false;   // B
  bool     btnBoost      = false;   // RB (押している間だけ最大速度)
  uint8_t  battery       = 0;       // 0..100 [%]
  uint32_t updatedAtMs   = 0;
  uint8_t  failedConnects = 0;
};

// BLE を初期化する。setup() から 1 度だけ呼ぶ。
void begin();

// 専用タスクから繰り返し呼ぶ。接続処理とスナップショット更新を行う。
void serviceOnce();

// 制御ループから呼ぶ。直近のスナップショットを複製して返す (スレッドセーフ)。
Snapshot read();

}  // namespace gamepad
