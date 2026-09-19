#include "ui.h"

#include <M5Unified.h>

#include <cmath>

#include "config.h"

namespace ui {
namespace {

M5Canvas* g_canvas = nullptr;

constexpr int kWidth  = 320;
constexpr int kHeight = 240;

struct StateStyle {
  const char* label;
  uint16_t    color;
};

StateStyle styleOf(RobotState s) {
  switch (s) {
    case RobotState::Booting:     return {"BOOTING",     TFT_DARKGREY};
    case RobotState::Calibrating: return {"CALIBRATING", TFT_CYAN};
    case RobotState::Disarmed:    return {"DISARMED",    TFT_YELLOW};
    case RobotState::Armed:       return {"ARMED",       TFT_GREEN};
    case RobotState::Fallen:      return {"FALLEN",      TFT_ORANGE};
    case RobotState::Fault:       return {"FAULT",       TFT_RED};
  }
  return {"?", TFT_WHITE};
}

// 傾きを横棒で表示する
void drawTiltBar(M5Canvas* c, int x, int y, int w, int h, float pitchDeg) {
  c->drawRect(x, y, w, h, TFT_DARKGREY);
  const int cx = x + w / 2;
  c->drawFastVLine(cx, y, h, TFT_DARKGREY);

  const float clamped =
      std::fmax(-cfg::kFallPitchDeg, std::fmin(cfg::kFallPitchDeg, pitchDeg));
  const int px = cx + static_cast<int>(clamped / cfg::kFallPitchDeg * (w / 2 - 2));
  const uint16_t col =
      std::fabs(pitchDeg) > cfg::kArmMaxPitchDeg ? TFT_ORANGE : TFT_GREEN;
  if (px >= cx) {
    c->fillRect(cx, y + 2, px - cx + 1, h - 4, col);
  } else {
    c->fillRect(px, y + 2, cx - px + 1, h - 4, col);
  }
}

}  // namespace

void begin() {
  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);

  g_canvas = new M5Canvas(&M5.Display);
  g_canvas->setColorDepth(8);
  g_canvas->createSprite(kWidth, kHeight);
  g_canvas->setTextSize(1);
}

void render(const Telemetry& t) {
  if (g_canvas == nullptr) return;
  M5Canvas* c = g_canvas;

  c->fillSprite(TFT_BLACK);

  // ---- ヘッダ ----
  const StateStyle st = styleOf(t.state);
  c->fillRect(0, 0, kWidth, 28, st.color);
  c->setTextColor(TFT_BLACK, st.color);
  c->setTextSize(2);
  c->setCursor(6, 6);
  c->print(st.label);

  c->setTextSize(1);
  c->setCursor(200, 4);
  c->printf("PAD %s", t.padConnected ? "OK" : "--");
  c->setCursor(200, 15);
  c->printf("CAN %s", t.canBusOff ? "OFF" : "OK");

  c->setTextColor(TFT_WHITE, TFT_BLACK);

  // ---- 傾き ----
  c->setTextSize(2);
  c->setCursor(6, 36);
  c->printf("%+6.2f deg", t.pitchDeg);
  c->setTextSize(1);
  c->setCursor(170, 42);
  c->printf("rate %+7.1f dps", t.pitchRateDps);

  drawTiltBar(c, 6, 58, kWidth - 12, 16, t.pitchDeg);

  // ---- 走行状態 ----
  int y = 82;
  c->setCursor(6, y);
  c->printf("vel %+5.2f m/s   pos %+6.2f m   trim %+5.2f deg",
            t.wheelVel, t.wheelPos, t.trimDeg);
  y += 12;
  c->setCursor(6, y);
  c->printf("cmd fwd %+4.2f  turn %+4.2f", t.cmdForward, t.cmdTurn);
  y += 12;
  c->setCursor(6, y);
  c->printf("out  L %+7.2f   R %+7.2f",  t.outLeft, t.outRight);

  // ---- モータ ----
  y += 16;
  c->drawFastHLine(0, y, kWidth, TFT_DARKGREY);
  y += 4;
  c->setCursor(6, y);
  c->setTextColor(t.motorAliveL ? TFT_WHITE : TFT_RED, TFT_BLACK);
  c->printf("M%u %s %4.1fC f=%02X", cfg::kMotorIdLeft,
            t.motorAliveL ? "ok" : "--", t.motorTempL, t.motorFaultL);
  c->setCursor(170, y);
  c->setTextColor(t.motorAliveR ? TFT_WHITE : TFT_RED, TFT_BLACK);
  c->printf("M%u %s %4.1fC f=%02X", cfg::kMotorIdRight,
            t.motorAliveR ? "ok" : "--", t.motorTempR, t.motorFaultR);
  c->setTextColor(TFT_WHITE, TFT_BLACK);

  y += 12;
  c->setCursor(6, y);
  c->printf("VBUS %5.1fV  Core2 %4.2fV  PAD %3u%%  overrun %lu",
            t.busVoltage, t.coreVoltage, t.padBattery,
            static_cast<unsigned long>(t.loopOverrun));

  // ---- DISARMED のときは IMU 生値を出す (軸の確認用) ----
  if (t.state == RobotState::Disarmed || t.state == RobotState::Calibrating) {
    y += 16;
    c->drawFastHLine(0, y, kWidth, TFT_DARKGREY);
    y += 4;
    c->setTextColor(TFT_CYAN, TFT_BLACK);
    c->setCursor(6, y);
    c->printf("acc %+5.2f %+5.2f %+5.2f [g]",
              t.accel[0], t.accel[1], t.accel[2]);
    y += 12;
    c->setCursor(6, y);
    c->printf("gyr %+7.1f %+7.1f %+7.1f [dps]",
              t.gyro[0], t.gyro[1], t.gyro[2]);
    c->setTextColor(TFT_WHITE, TFT_BLACK);

    if (t.state == RobotState::Calibrating) {
      y += 14;
      c->drawRect(6, y, kWidth - 12, 10, TFT_CYAN);
      c->fillRect(6, y, static_cast<int>((kWidth - 12) * t.calibProgress), 10,
                  TFT_CYAN);
    }
  }

  // ---- メッセージ ----
  if (t.message != nullptr && t.message[0] != '\0') {
    c->setTextColor(TFT_YELLOW, TFT_BLACK);
    c->setCursor(6, kHeight - 24);
    c->print(t.message);
    c->setTextColor(TFT_WHITE, TFT_BLACK);
  }

  // ---- 画面下のボタン割り当て ----
  c->setTextColor(TFT_DARKGREY, TFT_BLACK);
  c->setCursor(6, kHeight - 12);
  c->print("BtnA: ARM    BtnB: DISARM    BtnC: re-calibrate + zero wheels");

  c->pushSprite(0, 0);
}

}  // namespace ui
