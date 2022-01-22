#pragma once

#include "pn532.h"

#include "driver/i2c.h"

class PN532_I2C : public PN532 {
public:
        PN532_I2C(i2c_port_t i2c_port, gpio_num_t irqGPIO, gpio_num_t resetGPIO, xSemaphoreHandle i2c_lock);

protected:
        bool read(uint8_t *buff, uint8_t n) override;
        void write(uint8_t *cmd, uint8_t cmdlen) override;

private:
        i2c_port_t i2c_port;
        xSemaphoreHandle i2c_lock;
};