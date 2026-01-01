/*
 * sht3x.h
 *
 *  Created on: Jan 1, 2026
 *      Author: AD
 */

#ifndef INC_SHT3X_H_
#define INC_SHT3X_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SHT3X_ADDR_LOW   0x44   // ADDR pin = GND
#define SHT3X_ADDR_HIGH  0x45   // ADDR pin = VCC

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
} SHT3x_t;

bool SHT3x_Init(SHT3x_t *dev);
bool SHT3x_ReadTempHum(SHT3x_t *dev, float *temp, float *hum);

#endif /* INC_SHT3X_H_ */
