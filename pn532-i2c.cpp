#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"

#include "pn532-i2c.h"
#include "pn532-debug.h"

PN532_I2C::PN532_I2C(i2c_port_t i2c_port, gpio_num_t irqGPIO, gpio_num_t resetGPIO) :
        PN532(irqGPIO, resetGPIO), i2c_port(i2c_port) {
        //Needed due to long wake up procedure on the first command on i2c bus. May be decreased
        i2c_set_timeout (i2c_port, 400000);
}

/**************************************************************************/
/*!
 @brief  Reads n bytes of data from the PN532 via SPI or I2C.

 @param  buff      Pointer to the buffer where data will be written
 @param  n         Number of bytes to be read
 @return true if read success, false otherwise
 */
/**************************************************************************/
bool PN532_I2C::read(uint8_t *buff, uint8_t n) {
        i2c_cmd_handle_t i2ccmd;
        uint8_t *buffer = (uint8_t *) malloc(n + 3);

        vTaskDelay(10 / portTICK_PERIOD_MS);
        bzero(buffer, n + 3);
        bzero(buff, n);

        i2ccmd = i2c_cmd_link_create();
        i2c_master_start(i2ccmd);
        i2c_master_write_byte(i2ccmd, PN532_I2C_READ_ADDRESS, true);
        for (uint8_t i = 0; i < (n + 2); i++)
                i2c_master_read_byte(i2ccmd, &buffer[i], I2C_MASTER_ACK);

        i2c_master_read_byte(i2ccmd, &buffer[n + 2], I2C_MASTER_LAST_NACK);
        i2c_master_stop(i2ccmd);

        if (i2c_master_cmd_begin(i2c_port, i2ccmd, I2C_READ_TIMEOUT / portTICK_RATE_MS) != ESP_OK) {
                //Reset i2c bus
                i2c_cmd_link_delete(i2ccmd);
                free(buffer);
                return false;
        };

        i2c_cmd_link_delete(i2ccmd);

        memcpy(buff, buffer + 1, n);

        // Start read (n+1 to take into account leading 0x01 with I2C)
        PN532_DEBUG_HEXDUMP("Reading:", buffer, n+3);
        free(buffer);

        return true;
}

/**************************************************************************/
/*!
 @brief  Writes a command to the PN532, automatically inserting the
 preamble and required frame details (checksum, len, etc.)

 @param  cmd       Pointer to the command buffer
 @param  cmdlen    Command length in bytes
 */
/**************************************************************************/
void PN532_I2C::write(uint8_t *cmd, uint8_t cmdlen) {
        // I2C command write.
        uint8_t checksum;

        //Create the command
        uint8_t *command = (uint8_t *) malloc(cmdlen + 9);
        bzero(command, cmdlen + 9);

        vTaskDelay(10 / portTICK_PERIOD_MS);
        checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;

        command[0] = PN532_I2C_ADDRESS;
        command[1] = PN532_PREAMBLE;
        command[2] = PN532_PREAMBLE;
        command[3] = PN532_STARTCODE2;
        command[4] = (cmdlen + 1);
        command[5] = ~(cmdlen + 1) + 1;
        command[6] = PN532_HOSTTOPN532;
        checksum += PN532_HOSTTOPN532;

        for (uint8_t i = 0; i < cmdlen; i++) {
                command[i + 7] = cmd[i];
                checksum += cmd[i];
        }

        command[(cmdlen - 1) + 8] = ~checksum;
        command[(cmdlen - 1) + 9] = PN532_POSTAMBLE;

        i2c_cmd_handle_t i2ccmd = i2c_cmd_link_create();
        i2c_master_start(i2ccmd);
        i2c_master_write_byte(i2ccmd, command[0], true);

        for (uint8_t i = 1; i < cmdlen + 9; i++)
                i2c_master_write_byte(i2ccmd, command[i], true);

        i2c_master_stop(i2ccmd);

        PN532_DEBUG_HEXDUMP("Sending:", command, cmdlen+9);

        esp_err_t result = ESP_OK;
        result = i2c_master_cmd_begin(i2c_port, i2ccmd, I2C_WRITE_TIMEOUT / portTICK_PERIOD_MS);

        if (result != ESP_OK) {
                const char *resultText = NULL;
                switch (result) {
                case ESP_ERR_INVALID_ARG:
                        resultText = "Parameter error";
                        break;
                case ESP_FAIL:
                        resultText = "Sending command error, slave doesn’t ACK the transfer.";
                        break;
                case ESP_ERR_INVALID_STATE:
                        resultText = "I2C driver not installed or not in master mode.";
                        break;
                case ESP_ERR_TIMEOUT:
                        resultText = "Operation timeout because the bus is busy. ";
                        break;
                }

                ESP_LOGE(TAG, "%s I2C write failed: %s", __func__, resultText);
        }

        i2c_cmd_link_delete(i2ccmd);

        free(command);
}
