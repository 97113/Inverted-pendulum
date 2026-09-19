// =============================================================================
//  ui.h - Core2 の画面表示
// =============================================================================
#pragma once

#include <cstdint>

namespace ui {

enum class RobotState : uint8_t {
  Booting,
  Calibrating,
  Disarmed,
  Armed,
  Fallen,
  Fault,
};

struct Telemetry {
  RobotState state = RobotState::Booting;

  float pitchDeg     = 0.0f;
  float pitchRateDps = 0.0f;
  float trimDeg      = 0.0f;
  float wheelVel     = 0.0f;   // [m/s]
  float wheelPos     = 0.0f;   // [m]
  float cmdForward   = 0.0f;
  float cmdTurn      = 0.0f;
  float outLeft      = 0.0f;
  float outRight     = 0.0f;

  float accel[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3]  = {0.0f, 0.0f, 0.0f};

  float   motorTempL = 0.0f;
  float   motorTempR = 0.0f;
  uint8_t motorFaultL = 0;
  uint8_t motorFaultR = 0;
  bool    motorAliveL = false;
  bool    motorAliveR = false;
  float   busVoltage  = 0.0f;  // モータの母線電圧 [V]

  bool    padConnected = false;
  uint8_t padBattery   = 0;     // [%]
  float   coreVoltage  = 0.0f;  // Core2 内蔵電池 [V]

  bool     canBusOff  = false;
  uint32_t loopOverrun = 0;
  float    calibProgress = 0.0f;

  const char* message = "";
};

void begin();
void render(const Telemetry& t);

}  // namespace ui
