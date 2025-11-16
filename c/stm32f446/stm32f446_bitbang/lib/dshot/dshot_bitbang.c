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

#include <stdint.h>
#include <math.h>
#include <string.h>
#include "dshot_bitbang.h"


#ifdef USE_DSHOT_BITBANG

uint32_t bbOutputBuffer[48];
void bbOutputDataInit(uint32_t *buffer, uint16_t pin)
{
    int bitpos;
    for (bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 0] |= pin; // Always set all ports
        buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
        buffer[bitpos * 3 + 2] |= pin<<16; // Always reset all ports
    }
}
void bbOutputDataSet(uint32_t *buffer, int pin, uint16_t value)
{
    uint32_t middleBit;
    middleBit = pin<<16;
    for (int pos = 0; pos < 16; pos++) {
        if (!(value & 0x8000)) {
            buffer[pos * 3 + 1] |= middleBit;  //只有0码需要操作，1码保持即可
        }
        value <<= 1;
    }
}
void bbOutputDataClear(uint32_t *buffer)
{    // Middle position to no change
    for (int bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 1] = 0;
    }
}
// DMA GPIO output buffer formatting

// void bbOutputDataInit(uint32_t *buffer, uint16_t portMask, bool inverted)
// {
//     uint32_t resetMask;
//     uint32_t setMask;
//     if (inverted) {
//         resetMask = portMask;
//         setMask = (portMask << 16);
//     } else {
//         resetMask = (portMask << 16);
//         setMask = portMask;
//     }
//     int bitpos;
//     for (bitpos = 0; bitpos < 16; bitpos++) {
//         buffer[bitpos * 3 + 0] |= setMask ; // Always set all ports
//         buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
//         buffer[bitpos * 3 + 2] |= resetMask; // Always reset all ports
//     }
// }

// void bbOutputDataSet(uint32_t *buffer, int pinNumber, uint16_t value, bool inverted)
// {
//     uint32_t middleBit;
//     if (inverted) {
//         middleBit = (1 << (pinNumber + 0));
//     } else {
//         middleBit = (1 << (pinNumber + 16));
//     }
//     for (int pos = 0; pos < 16; pos++) {
//         if (!(value & 0x8000)) {
//             buffer[pos * 3 + 1] |= middleBit;
//         }
//         value <<= 1;
//     }
// }
// void bbOutputDataClear(uint32_t *buffer)
// {    // Middle position to no change
//     for (int bitpos = 0; bitpos < 16; bitpos++) {
//         buffer[bitpos * 3 + 1] = 0;
//     }
// }


#endif // USE_DSHOT_BB
