// =============================================================================
//  車輪型倒立振子ラジコン
//    M5Stack Core2 + PwrCAN Module 13.2 + RobStride EduLite 02 x2
//    操縦: Xbox Series X|S コントローラ (BLE)
// =============================================================================
//  タスク構成
//    core1 : controlTask  200 Hz  姿勢推定 → バランス制御 → CAN 送信/受信
//    core0 : slowTask      50 Hz  BLE の接続処理、画面描画、ボタン処理
//
//  BLE のスキャンは内部で待ちが入るため、制御ループとは必ず別タスクにする。
// =============================================================================
#include <Arduino.h>
#include <M5Unified.h>

#include <cmath>

#include "attitude.h"
#include "balance.h"
#include "config.h"
#include "gamepad.h"
#include "i2c_bus.h"
#include "robstride.h"
#include "ui.h"

namespace {

// -----------------------------------------------------------------------------
//  グローバル
// -----------------------------------------------------------------------------
robstride::Bus   g_bus;
robstride::Motor g_motorL(g_bus, cfg::kMotorIdLeft);
robstride::Motor g_motorR(g_bus, cfg::kMotorIdRight);

attitude::Estimator g_imu;
balance::Controller g_ctrl;

ui::Telemetry g_tlm;
portMUX_TYPE  g_tlmMux = portMUX_INITIALIZER_UNLOCKED;

ui::RobotState g_state = ui::RobotState::Booting;
const char*    g_message = "";

// IMU や CAN の初期化に失敗した場合は、再起動しないと復帰できない。
bool g_bootFault = false;

// 別タスクから制御タスクへの要求
volatile bool g_reqArm     = false;
volatile bool g_reqDisarm  = false;
volatile bool g_reqRecalib = false;

balance::Command g_smoothCmd;

// 低速タスクが I2C 経由で読み、制御タスクがテレメトリに載せる
volatile float g_coreVoltage = 0.0f;

// -----------------------------------------------------------------------------
//  ヘルパ
// -----------------------------------------------------------------------------
constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;

// 車輪の累積角 [rad] から走行距離 [m] を求める。左右の平均を取る。
float wheelPosition() {
  const float l = g_motorL.accumulatedAngle() * cfg::kMotorDirLeft;
  const float r = g_motorR.accumulatedAngle() * cfg::kMotorDirRight;
  return (l + r) * 0.5f * cfg::kWheelRadius;
}

float wheelVelocity() {
  const float l = g_motorL.state().velocity * cfg::kMotorDirLeft;
  const float r = g_motorR.state().velocity * cfg::kMotorDirRight;
  return (l + r) * 0.5f * cfg::kWheelRadius;
}

bool motorAlive(const robstride::Motor& m) {
  const auto& s = m.state();
  return s.valid && (millis() - s.updatedAtMs) < cfg::kCanTimeoutMs;
}

bool motorFaulted(const robstride::Motor& m) {
  // 未較正ビットは電源投入直後に立つことがあるため、停止条件からは外す。
  constexpr uint8_t kCritical = robstride::kFaultHallEncoder |
                                robstride::kFaultMagEncoder |
                                robstride::kFaultOverTemp |
                                robstride::kFaultOverCurrent |
                                robstride::kFaultUnderVoltage;
  return (m.state().faults & kCritical) != 0;
}

void configureMotor(robstride::Motor& m) {
  m.stop(/*clearFault=*/true);
  delay(5);
  // 注意: 速度モードではパラメータ書き込みに対してモータが
  //       フィードバックフレームを返さない機種があり、その場合は
  //       画面上でモータが "--" のままになる。トルクモードを推奨。
  if (cfg::kMotorMode == cfg::MotorMode::Torque) {
    m.setRunMode(robstride::RunMode::Motion);
  } else {
    m.setRunMode(robstride::RunMode::Speed);
  }
  delay(5);
  m.writeParam(robstride::ParamIndex::LimitTorque, cfg::kTorqueLimit);
  delay(5);
}

// 指令を 0 にしたうえでモータを無効化する
void disableMotors() {
  if (cfg::kMotorMode == cfg::MotorMode::Torque) {
    g_motorL.sendMotion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    g_motorR.sendMotion(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  } else {
    g_motorL.sendSpeed(0.0f);
    g_motorR.sendSpeed(0.0f);
  }
  g_motorL.stop();
  g_motorR.stop();
}

void enableMotors() {
  configureMotor(g_motorL);
  configureMotor(g_motorR);
  g_motorL.enable();
  g_motorR.enable();
}

void drainCanRx() {
  uint32_t id;
  uint8_t  data[8];
  // 1 周期分の受信をまとめて処理する。取りこぼしを防ぐため上限は多めに取る。
  for (int i = 0; i < 32; ++i) {
    if (!g_bus.receive(&id, data, 0)) break;
    if (g_motorL.consume(id, data)) continue;
    g_motorR.consume(id, data);
  }
}

void sendMotorCommand(const balance::Output& out) {
  if (cfg::kMotorMode == cfg::MotorMode::Torque) {
    g_motorL.sendMotion(out.left * cfg::kMotorDirLeft, 0.0f, 0.0f, 0.0f, 0.0f);
    g_motorR.sendMotion(out.right * cfg::kMotorDirRight, 0.0f, 0.0f, 0.0f, 0.0f);
  } else {
    g_motorL.sendSpeed(out.left * cfg::kMotorDirLeft);
    g_motorR.sendSpeed(out.right * cfg::kMotorDirRight);
  }
}

// -----------------------------------------------------------------------------
//  制御タスク (core 1)
// -----------------------------------------------------------------------------
void controlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / cfg::kControlHz);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t overruns = 0;
  uint32_t lastVbusRequestMs = 0;
  uint32_t lastHeartbeatMs = 0;
  constexpr uint32_t kHeartbeatIntervalMs = 50;
  float vbus = 0.0f;

  // setup() で致命的な異常を検出している場合は、そちらの状態を保つ。
  if (g_state != ui::RobotState::Fault) {
    g_imu.startCalibration();
    g_state = ui::RobotState::Calibrating;
    g_message = "keep the robot still";
  }

  for (;;) {
    const TickType_t before = xTaskGetTickCount();

    g_bus.poll();
    drainCanRx();

    if (!g_imu.update(cfg::kControlDt)) {
      g_message = "IMU read failed";
    }

    // 停止中は運動制御フレームを送らないので、モータからのフィードバックも来ない。
    // そのままだと「モータが死んでいる」と判定されて ARM できなくなるため、
    // 必ず応答が返るパラメータ読み出しを心拍として流しておく。
    if (g_state != ui::RobotState::Armed &&
        millis() - lastHeartbeatMs > kHeartbeatIntervalMs) {
      lastHeartbeatMs = millis();
      g_motorL.requestParam(robstride::ParamIndex::MechPos);
      g_motorR.requestParam(robstride::ParamIndex::MechPos);
    }

    // 母線電圧を 1 秒ごとに問い合わせる
    if (millis() - lastVbusRequestMs > 1000) {
      lastVbusRequestMs = millis();
      g_motorL.requestParam(robstride::ParamIndex::VBus);
    }
    if (g_motorL.lastParamIndex() == robstride::ParamIndex::VBus) {
      vbus = g_motorL.lastParamValue();
    }

    const float pitchDeg = g_imu.pitch() * kRadToDeg;
    const bool  bothAlive = motorAlive(g_motorL) && motorAlive(g_motorR);
    const bool  faulted   = motorFaulted(g_motorL) || motorFaulted(g_motorR);

    // ---- 状態遷移 ----
    switch (g_state) {
      case ui::RobotState::Booting:
      case ui::RobotState::Calibrating:
        if (!g_imu.isCalibrating()) {
          g_state = ui::RobotState::Disarmed;
          g_message = "press A (or BtnA) to arm";
        }
        break;

      case ui::RobotState::Disarmed:
      case ui::RobotState::Fallen:
        if (g_reqArm) {
          if (std::fabs(pitchDeg) > cfg::kArmMaxPitchDeg) {
            g_message = "hold it upright before arming";
          } else if (!bothAlive) {
            g_message = "no motor feedback - check CAN wiring";
          } else {
            g_motorL.resetAccumulatedAngle();
            g_motorR.resetAccumulatedAngle();
            g_ctrl.reset(0.0f);
            g_smoothCmd = balance::Command{};
            enableMotors();
            g_state = ui::RobotState::Armed;
            g_message = "";
          }
        }
        break;

      case ui::RobotState::Armed:
        if (std::fabs(pitchDeg) > cfg::kFallPitchDeg) {
          disableMotors();
          g_state = ui::RobotState::Fallen;
          g_message = "fell over - stand it up and press A";
        } else if (g_reqDisarm) {
          disableMotors();
          g_state = ui::RobotState::Disarmed;
          g_message = "disarmed";
        } else if (!bothAlive || g_bus.isBusOff() || faulted) {
          disableMotors();
          g_state = ui::RobotState::Fault;
          g_message = faulted ? "motor fault - check the RobStride tool"
                              : "CAN link lost";
        }
        break;

      case ui::RobotState::Fault:
        // 復帰時はジャイロを較正し直してから DISARMED に戻す。
        if (g_reqArm && !g_bootFault && bothAlive && !g_bus.isBusOff() &&
            !faulted) {
          g_imu.startCalibration();
          g_imu.reset();
          g_state = ui::RobotState::Calibrating;
          g_message = "fault cleared - keep the robot still";
        }
        break;
    }
    // 要求は 1 周期で必ず消費する (取りこぼしても危険側には倒れない)
    g_reqArm = false;
    g_reqDisarm = false;

    // ARM 中の再較正はモータが暴れるので受け付けない。
    if (g_reqRecalib) {
      g_reqRecalib = false;
      if (g_state != ui::RobotState::Armed) {
        g_motorL.resetAccumulatedAngle();
        g_motorR.resetAccumulatedAngle();
        g_ctrl.reset(0.0f);
        g_imu.startCalibration();
        g_imu.reset();
        g_state = ui::RobotState::Calibrating;
        g_message = "keep the robot still";
      }
    }

    // ---- 制御 ----
    balance::Output out;
    const gamepad::Snapshot pad = gamepad::read();

    if (g_state == ui::RobotState::Armed) {
      balance::Command target = pad.command;
      if (!pad.btnBoost) {
        // 既定では最大速度の 50%。RB を押している間だけフルスケールにする。
        target.forward *= 0.5f;
        target.turn    *= 0.5f;
      }
      g_smoothCmd.forward = balance::slew(g_smoothCmd.forward, target.forward,
                                          cfg::kCommandSlewPerSec, cfg::kControlDt);
      g_smoothCmd.turn = balance::slew(g_smoothCmd.turn, target.turn,
                                       cfg::kCommandSlewPerSec, cfg::kControlDt);

      balance::Measurement meas;
      meas.pitch     = g_imu.pitch();
      meas.pitchRate = g_imu.pitchRate();
      meas.yawRate   = g_imu.yawRate();
      meas.wheelPos  = wheelPosition();
      meas.wheelVel  = wheelVelocity();

      out = g_ctrl.update(meas, g_smoothCmd, cfg::kControlDt);
      sendMotorCommand(out);
    } else {
      g_smoothCmd = balance::Command{};
    }

    // ---- テレメトリ更新 ----
    ui::Telemetry t;
    t.state        = g_state;
    t.pitchDeg     = pitchDeg;
    t.pitchRateDps = g_imu.pitchRate() * kRadToDeg;
    t.trimDeg      = g_ctrl.trim() * kRadToDeg;
    t.wheelVel     = wheelVelocity();
    t.wheelPos     = wheelPosition();
    t.cmdForward   = g_smoothCmd.forward;
    t.cmdTurn      = g_smoothCmd.turn;
    t.outLeft      = out.left;
    t.outRight     = out.right;
    for (int i = 0; i < 3; ++i) {
      t.accel[i] = g_imu.raw().accel[i];
      t.gyro[i]  = g_imu.raw().gyro[i];
    }
    t.motorTempL  = g_motorL.state().temperature;
    t.motorTempR  = g_motorR.state().temperature;
    t.motorFaultL = g_motorL.state().faults;
    t.motorFaultR = g_motorR.state().faults;
    t.motorAliveL = motorAlive(g_motorL);
    t.motorAliveR = motorAlive(g_motorR);
    t.busVoltage  = vbus;
    t.padConnected = pad.connected;
    t.padBattery   = pad.battery;
    t.coreVoltage  = g_coreVoltage;
    t.canBusOff    = g_bus.isBusOff();
    t.calibProgress = g_imu.calibrationProgress();
    t.message      = g_message;
    t.loopOverrun  = overruns;

    taskENTER_CRITICAL(&g_tlmMux);
    g_tlm = t;
    taskEXIT_CRITICAL(&g_tlmMux);

    if (xTaskGetTickCount() - before > period) ++overruns;

    // ARM 時のモータ設定などで大きく遅れた場合、そのまま vTaskDelayUntil に
    // 入ると遅れを取り戻そうとして無遅延で数周回ってしまう。基準を取り直す。
    if ((xTaskGetTickCount() - lastWake) > period * 4) {
      lastWake = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

// -----------------------------------------------------------------------------
//  低速タスク (core 0) : BLE + 画面 + 本体ボタン
// -----------------------------------------------------------------------------
void slowTask(void*) {
  gamepad::begin();

  const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastRenderMs = 0;
  bool prevArmBtn = false;
  bool prevDisarmBtn = false;

  for (;;) {
    gamepad::serviceOnce();

    {
      // ボタン/タッチと PMIC も内蔵 I2C なので IMU と排他にする。
      i2cbus::I2cLock lock;
      M5.update();
      g_coreVoltage = M5.Power.getBatteryVoltage() / 1000.0f;
    }
    const gamepad::Snapshot pad = gamepad::read();

    // コントローラの A/B は「押した瞬間」だけを拾う
    if (pad.btnArm && !prevArmBtn) g_reqArm = true;
    if (pad.btnDisarm && !prevDisarmBtn) g_reqDisarm = true;
    prevArmBtn = pad.btnArm;
    prevDisarmBtn = pad.btnDisarm;

    if (M5.BtnA.wasPressed()) g_reqArm = true;
    if (M5.BtnB.wasPressed()) g_reqDisarm = true;
    if (M5.BtnC.wasPressed()) g_reqRecalib = true;

    // 画面は 10 Hz で十分
    if (millis() - lastRenderMs >= 100) {
      lastRenderMs = millis();
      ui::Telemetry t;
      taskENTER_CRITICAL(&g_tlmMux);
      t = g_tlm;
      taskEXIT_CRITICAL(&g_tlmMux);
      ui::render(t);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

}  // namespace

// -----------------------------------------------------------------------------
void setup() {
  auto m5cfg = M5.config();
  m5cfg.internal_imu = true;
  M5.begin(m5cfg);

  Serial.begin(115200);
  i2cbus::initI2cMutex();
  ui::begin();

  if (M5.Imu.getType() == m5::imu_none) {
    g_message = "IMU not found - reboot required";
    g_state = ui::RobotState::Fault;
    g_bootFault = true;
  }

  if (!g_bus.begin(cfg::kCanTxPin, cfg::kCanRxPin, cfg::kCanBitrateKbps)) {
    g_message = "TWAI init failed - check the CAN pins";
    g_state = ui::RobotState::Fault;
    g_bootFault = true;
  } else {
    // 起動直後は必ず停止状態にしておく (run_mode を決めてから停止指令を送る)
    configureMotor(g_motorL);
    configureMotor(g_motorR);
    disableMotors();
  }

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 10, nullptr, 1);
  xTaskCreatePinnedToCore(slowTask,    "slow",    8192, nullptr,  3, nullptr, 0);
}

void loop() {
  // すべてタスクで処理しているのでここは空。
  vTaskDelay(pdMS_TO_TICKS(1000));
}
