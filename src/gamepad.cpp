#include "gamepad.h"

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include "config.h"

namespace gamepad {
namespace {

XboxSeriesXControllerESP32_asukiaaa::Core* g_xbox = nullptr;

Snapshot          g_snapshot;
portMUX_TYPE      g_mux = portMUX_INITIALIZER_UNLOCKED;

// 接続が繰り返し失敗したら BLE スタックを作り直す。
constexpr uint8_t kMaxFailedConnects = 5;

}  // namespace

void begin() {
  if (g_xbox != nullptr) return;
  g_xbox = new XboxSeriesXControllerESP32_asukiaaa::Core(cfg::kGamepadAddress);
  g_xbox->begin();
}

void serviceOnce() {
  if (g_xbox == nullptr) return;

  g_xbox->onLoop();

  Snapshot s;
  s.failedConnects = g_xbox->getCountFailedConnection();

  const bool live = g_xbox->isConnected() &&
                    !g_xbox->isWaitingForFirstNotification();

  if (live) {
    const auto& n = g_xbox->xboxNotif;
    constexpr uint16_t kMaxJoy = XboxControllerNotificationParser::maxJoy;

    s.command.forward =
        balance::normalizeStick(n.joyLVert, kMaxJoy, cfg::kInvertStickVertical);
    // 右スティック右倒しで値が増える。左旋回を正にするので符号を反転する。
    s.command.turn =
        balance::normalizeStick(n.joyRHori, kMaxJoy, !cfg::kInvertStickYaw);

    s.btnArm    = n.btnA;
    s.btnDisarm = n.btnB;
    s.btnBoost  = n.btnRB;
    s.connected = true;
    s.battery   = g_xbox->battery;
    s.updatedAtMs = g_xbox->getReceiveNotificationAt();
  }

  taskENTER_CRITICAL(&g_mux);
  g_snapshot = s;
  taskEXIT_CRITICAL(&g_mux);

  if (s.failedConnects >= kMaxFailedConnects) {
    // スキャンをやり直す。ここで復帰しない場合は本体を再起動すること。
    g_xbox->startScan();
  }
}

Snapshot read() {
  Snapshot s;
  taskENTER_CRITICAL(&g_mux);
  s = g_snapshot;
  taskEXIT_CRITICAL(&g_mux);

  // 通知が途絶えたら「接続なし」として扱い、指令を中立に落とす。
  if (s.connected &&
      (millis() - s.updatedAtMs) > cfg::kGamepadTimeoutMs) {
    s.connected = false;
    s.command = balance::Command{};
    s.btnArm = s.btnDisarm = s.btnBoost = false;
  }
  return s;
}

}  // namespace gamepad
