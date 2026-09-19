// =============================================================================
//  i2c_bus.h - 内蔵 I2C バスの排他制御
// =============================================================================
//  Core2 では IMU (MPU6886 / BMI270)、PMIC (AXP192)、タッチパネルが
//  同じ内蔵 I2C バスにぶら下がっている。
//  制御タスク (core1) が IMU を読む一方で、低速タスク (core0) が
//  M5.update() や電池電圧を読むため、必ずこのロックで囲むこと。
// =============================================================================
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace i2cbus {

void initI2cMutex();

// RAII ガード。スコープを抜けると自動で解放される。
class I2cLock {
 public:
  I2cLock();
  ~I2cLock();

  I2cLock(const I2cLock&) = delete;
  I2cLock& operator=(const I2cLock&) = delete;
};

}  // namespace i2cbus
