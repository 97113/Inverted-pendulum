#include "i2c_bus.h"

namespace i2cbus {
namespace {

SemaphoreHandle_t g_i2cMutex = nullptr;

}  // namespace

void initI2cMutex() {
  if (g_i2cMutex == nullptr) {
    g_i2cMutex = xSemaphoreCreateMutex();
  }
}

I2cLock::I2cLock() {
  if (g_i2cMutex != nullptr) {
    xSemaphoreTake(g_i2cMutex, portMAX_DELAY);
  }
}

I2cLock::~I2cLock() {
  if (g_i2cMutex != nullptr) {
    xSemaphoreGive(g_i2cMutex);
  }
}

}  // namespace i2cbus
