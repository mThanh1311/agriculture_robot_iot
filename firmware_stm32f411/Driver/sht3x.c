/*
 * sht3x.c
 *
 *  Created on: Jan 1, 2026
 *      Author: AD
 */

#include "sht3x.h"

#define SHT3X_CMD_MEASURE_HIGHREP  0x2C06

static uint8_t crc8(uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

bool SHT3x_Init(SHT3x_t *dev)
{
    uint8_t cmd[2] = {0x30, 0xA2}; // soft reset
    return HAL_I2C_Master_Transmit(
        dev->hi2c, dev->address << 1,
        cmd, 2, 100
    ) == HAL_OK;
}

bool SHT3x_ReadTempHum(SHT3x_t *dev, float *temp, float *hum)
{
    uint8_t cmd[2] = {0x2C, 0x06};
    uint8_t rx[6];

    if (HAL_I2C_Master_Transmit(dev->hi2c, dev->address << 1, cmd, 2, 100) != HAL_OK)
        return false;

    HAL_Delay(15);

    if (HAL_I2C_Master_Receive(dev->hi2c, dev->address << 1, rx, 6, 100) != HAL_OK)
        return false;

    if (crc8(rx, 2) != rx[2] || crc8(&rx[3], 2) != rx[5])
        return false;

    uint16_t rawT = (rx[0] << 8) | rx[1];
    uint16_t rawH = (rx[3] << 8) | rx[4];

    *temp = -45.0f + 175.0f * rawT / 65535.0f;
    *hum  = 100.0f * rawH / 65535.0f;

    return true;
}


