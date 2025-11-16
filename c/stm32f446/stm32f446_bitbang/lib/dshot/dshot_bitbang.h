/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include "main.h"
#include <stdbool.h>


#define MOTOR_DSHOT600_SYMBOL_RATE     (600 * 1000)
#define MOTOR_DSHOT300_SYMBOL_RATE     (300 * 1000)
#define MOTOR_DSHOT150_SYMBOL_RATE     (150 * 1000)



#define MOTOR_DSHOT_SYMBOL_TIME_NS(rate)  (1000000000 / (rate))

#define MOTOR_DSHOT_BIT_PER_SYMBOL         1

#define MOTOR_DSHOT_STATE_PER_SYMBOL       3  // Initial high, 0/1, low
#define MOTOR_DSHOT_BIT_HOLD_STATES        3  // 3 extra states at the end of transmission required to allow ESC to sample the last bit correctly.

#define MOTOR_DSHOT_FRAME_BITS             16

#define MOTOR_DSHOT_FRAME_TIME_NS(rate)    ((MOTOR_DSHOT_FRAME_BITS / MOTOR_DSHOT_BIT_PER_SYMBOL) * MOTOR_DSHOT_SYMBOL_TIME_NS(rate))

#define DSHOT_BITBANG_INVERTED         true
#define DSHOT_BITBANG_NONINVERTED      false

typedef enum {
    DSHOT_BITBANG_OFF,
    DSHOT_BITBANG_ON,
    DSHOT_BITBANG_AUTO,
} dshotBitbangMode_e;

typedef enum {
    DSHOT_BITBANG_STATUS_OK,
    DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT,
    DSHOT_BITBANG_STATUS_NO_PACER,
    DSHOT_BITBANG_STATUS_TOO_MANY_PORTS,
} dshotBitbangStatus_e;


uint32_t bbOutputBuffer[48];

void bbOutputDataInit(uint32_t *buffer, uint16_t pin);
void bbOutputDataSet(uint32_t *buffer, int pin, uint16_t value);
void bbOutputDataClear(uint32_t *buffer);