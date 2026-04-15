#pragma once

#include <driver/i2c_master.h>

#include "i2c_device.h"

class Pca9685 : public I2cDevice {
public:
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x40);
    void InitializeServoMode();
    void SetServoPulse(uint8_t channel, float pulse_us);

private:
    void WriteRegister(uint8_t reg, uint8_t value) { WriteReg(reg, value); }
};
