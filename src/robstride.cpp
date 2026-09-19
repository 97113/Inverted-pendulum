#include "robstride.h"

#include <Arduino.h>
#include <driver/twai.h>

#include <cmath>
#include <cstring>

namespace robstride {
namespace {

// 物理量 <-> 16bit の相互変換 (CyberGear / RobStride 共通)
uint16_t floatToU16(float value, float min, float max) {
  if (!std::isfinite(value)) value = 0.0f;
  if (value < min) value = min;
  if (value > max) value = max;
  const float span = max - min;
  return static_cast<uint16_t>((value - min) * 65535.0f / span);
}

float u16ToFloat(uint16_t raw, float min, float max) {
  const float span = max - min;
  return static_cast<float>(raw) * span / 65535.0f + min;
}

void putU16BE(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}

uint16_t getU16BE(const uint8_t* p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t makeExtId(CommType type, uint16_t data2, uint8_t motorId) {
  return (static_cast<uint32_t>(type) << 24) |
         (static_cast<uint32_t>(data2) << 8) |
         static_cast<uint32_t>(motorId);
}

constexpr uint32_t kBusOffRetryMs = 200;

}  // namespace

// -----------------------------------------------------------------------------
//  Bus
// -----------------------------------------------------------------------------
bool Bus::begin(int txPin, int rxPin, uint32_t bitrateKbps) {
  if (started_) end();

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(txPin), static_cast<gpio_num_t>(rxPin),
      TWAI_MODE_NORMAL);
  g.tx_queue_len = 32;
  g.rx_queue_len = 64;
  g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                     TWAI_ALERT_RX_QUEUE_FULL;

  twai_timing_config_t t;
  switch (bitrateKbps) {
    case 1000: t = TWAI_TIMING_CONFIG_1MBITS();   break;
    case 500:  t = TWAI_TIMING_CONFIG_500KBITS(); break;
    case 250:  t = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 125:  t = TWAI_TIMING_CONFIG_125KBITS(); break;
    default:   return false;
  }

  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }

  started_ = true;
  busOff_  = false;
  return true;
}

void Bus::end() {
  if (!started_) return;
  twai_stop();
  twai_driver_uninstall();
  started_ = false;
}

bool Bus::send(uint32_t extId, const uint8_t data[8], uint32_t timeoutMs) {
  if (!started_ || busOff_) return false;

  twai_message_t msg = {};
  msg.extd = 1;
  msg.identifier = extId;
  msg.data_length_code = 8;
  std::memcpy(msg.data, data, 8);

  return twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

bool Bus::receive(uint32_t* extId, uint8_t data[8], uint32_t timeoutMs) {
  if (!started_) return false;

  twai_message_t msg;
  if (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) return false;
  if (!msg.extd) return false;

  *extId = msg.identifier;
  std::memset(data, 0, 8);
  std::memcpy(data, msg.data,
              msg.data_length_code > 8 ? 8 : msg.data_length_code);
  return true;
}

void Bus::poll() {
  if (!started_) return;

  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) {
      busOff_ = true;
      recoverAtMs_ = millis() + kBusOffRetryMs;
      twai_initiate_recovery();
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
      if (twai_start() == ESP_OK) {
        busOff_ = false;
        ++recoveryCount_;
      }
    }
  }

  // BUS_RECOVERED を取りこぼしても復帰できるように保険をかける。
  if (busOff_ && static_cast<int32_t>(millis() - recoverAtMs_) > 0) {
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK &&
        st.state == TWAI_STATE_STOPPED && twai_start() == ESP_OK) {
      busOff_ = false;
      ++recoveryCount_;
    }
    recoverAtMs_ = millis() + kBusOffRetryMs;
  }
}

// -----------------------------------------------------------------------------
//  Motor
// -----------------------------------------------------------------------------
bool Motor::sendFrame(CommType type, uint16_t data2, const uint8_t payload[8]) {
  return bus_.send(makeExtId(type, data2, id_), payload);
}

bool Motor::enable() {
  uint8_t payload[8] = {0};
  return sendFrame(CommType::Enable, cfg::kHostCanId, payload);
}

bool Motor::stop(bool clearFault) {
  uint8_t payload[8] = {0};
  // data[0] の bit0 を立てると故障クリアつきで停止する。
  payload[0] = clearFault ? 1 : 0;
  return sendFrame(CommType::Stop, cfg::kHostCanId, payload);
}

bool Motor::setMechZero() {
  uint8_t payload[8] = {0};
  payload[0] = 1;
  return sendFrame(CommType::SetMechZero, cfg::kHostCanId, payload);
}

bool Motor::writeParam(ParamIndex index, float value) {
  uint8_t payload[8] = {0};
  const uint16_t idx = static_cast<uint16_t>(index);
  payload[0] = static_cast<uint8_t>(idx & 0xFF);
  payload[1] = static_cast<uint8_t>(idx >> 8);
  std::memcpy(&payload[4], &value, 4);  // little endian float
  return sendFrame(CommType::WriteParam, cfg::kHostCanId, payload);
}

bool Motor::writeParam(ParamIndex index, uint8_t value) {
  uint8_t payload[8] = {0};
  const uint16_t idx = static_cast<uint16_t>(index);
  payload[0] = static_cast<uint8_t>(idx & 0xFF);
  payload[1] = static_cast<uint8_t>(idx >> 8);
  payload[4] = value;
  return sendFrame(CommType::WriteParam, cfg::kHostCanId, payload);
}

bool Motor::requestParam(ParamIndex index) {
  uint8_t payload[8] = {0};
  const uint16_t idx = static_cast<uint16_t>(index);
  payload[0] = static_cast<uint8_t>(idx & 0xFF);
  payload[1] = static_cast<uint8_t>(idx >> 8);
  return sendFrame(CommType::ReadParam, cfg::kHostCanId, payload);
}

bool Motor::setRunMode(RunMode mode) {
  return writeParam(ParamIndex::RunMode, static_cast<uint8_t>(mode));
}

bool Motor::sendMotion(float torque, float angle, float velocity, float kp,
                       float kd) {
  uint8_t payload[8];
  putU16BE(&payload[0], floatToU16(angle, -cfg::kPosMax, cfg::kPosMax));
  putU16BE(&payload[2], floatToU16(velocity, -cfg::kVelMax, cfg::kVelMax));
  putU16BE(&payload[4], floatToU16(kp, 0.0f, cfg::kKpMax));
  putU16BE(&payload[6], floatToU16(kd, 0.0f, cfg::kKdMax));

  // 運動制御では目標トルクを ID のデータ領域 2 に載せる。
  const uint16_t torqueRaw =
      floatToU16(torque, -cfg::kTorqueMax, cfg::kTorqueMax);
  return sendFrame(CommType::MotionControl, torqueRaw, payload);
}

bool Motor::sendSpeed(float velocity) {
  return writeParam(ParamIndex::SpdRef, velocity);
}

bool Motor::consume(uint32_t extId, const uint8_t data[8]) {
  const CommType type = static_cast<CommType>((extId >> 24) & 0x1F);
  const uint16_t data2 = static_cast<uint16_t>((extId >> 8) & 0xFFFF);

  switch (type) {
    case CommType::Feedback: {
      // フィードバックフレームでは、データ領域 2 の下位 8bit が送信元モータ ID。
      if (static_cast<uint8_t>(data2 & 0xFF) != id_) return false;

      state_.angle    = u16ToFloat(getU16BE(&data[0]), -cfg::kPosMax, cfg::kPosMax);
      state_.velocity = u16ToFloat(getU16BE(&data[2]), -cfg::kVelMax, cfg::kVelMax);
      state_.torque   = u16ToFloat(getU16BE(&data[4]), -cfg::kTorqueMax, cfg::kTorqueMax);
      state_.temperature = static_cast<float>(getU16BE(&data[6])) * 0.1f;
      state_.faults   = static_cast<uint8_t>((data2 >> 8) & 0x3F);
      state_.mode     = static_cast<uint8_t>((data2 >> 14) & 0x03);
      state_.updatedAtMs = millis();
      state_.valid    = true;

      // ±kPosMax で折り返すため、差分を取って多回転角を積算する。
      if (hasPrevAngle_) {
        float delta = state_.angle - prevAngle_;
        const float span = 2.0f * cfg::kPosMax;
        if (delta >  cfg::kPosMax) delta -= span;
        if (delta < -cfg::kPosMax) delta += span;
        accumAngle_ += delta;
      }
      prevAngle_ = state_.angle;
      hasPrevAngle_ = true;
      return true;
    }

    case CommType::ReadParam: {
      // 読み出し応答も宛先がホスト、データ領域 2 の下位 8bit が送信元モータ ID。
      if (static_cast<uint8_t>(data2 & 0xFF) != id_) return false;
      lastParamIndex_ = static_cast<ParamIndex>(
          static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
      std::memcpy(&lastParamValue_, &data[4], 4);
      // 停止中は運動制御フレームを送らないため、この応答が唯一の生存確認になる。
      state_.updatedAtMs = millis();
      state_.valid = true;
      return true;
    }

    case CommType::FaultFeedback: {
      if (static_cast<uint8_t>(data2 & 0xFF) != id_) return false;
      // 故障フレームが来た時点で「何か起きた」ことだけ記録する。
      // 詳細ビットはマニュアル参照。
      state_.faults |= kFaultOverCurrent;
      state_.updatedAtMs = millis();
      return true;
    }

    default:
      return false;
  }
}

}  // namespace robstride
