/**
 * StepperServoCAN
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __TLE5012_H
#define __TLE5012_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "spi.h"

#define SPI_RX_ON  SPI_BiDirectionalLineConfig(TLE5012B_SPI, SPI_Direction_Rx)
#define SPI_RX_OFF   SPI_BiDirectionalLineConfig(TLE5012B_SPI, SPI_Direction_Tx)

// dummy variable used for receive. Each time this is sent, it is for the purposes of receiving using SPI transfer.
#define DUMMY                   0xFFFF

#define TLE012_CS_H			PIN_TLE5012B->BSRR = PIN_TLE5012B_CS;		//GPIO_SetBits(PIN_TLE5012B, PIN_TLE5012B_CS)
#define TLE012_CS_L			PIN_TLE5012B->BRR  = PIN_TLE5012B_CS;		//GPIO_ResetBits(PIN_TLE5012B, PIN_TLE5012B_CS)

/* SPI command for TLE5012 */
#define READ_STATUS				0x8001			//00h
#define READ_ANGLE_VALUE		0x8021			//02h
#define READ_SPEED_VALUE		0x8031			//03h

#define WRITE_MOD1_VALUE		0x5060
#define WRITE_MOD2_VALUE		0x5081
#define WRITE_MOD3_VALUE		0x5091
#define WRITE_MOD4_VALUE		0x50E0
#define WRITE_IFAB_VALUE		0x50B1

#define WRITE_READ_REG_OFFSET   0x3000

bool TLE5012_begin(void);
uint16_t TLE5012_ReadValue(uint16_t RegValue);
void TLE5012_WriteValue(uint16_t RegAdd,uint16_t RegValue);
bool TLE5012_WriteAndCheck(uint16_t RegAdd,uint16_t RegValue);
uint16_t TLE5012_ReadState(void);
uint16_t TLE5012_ReadAngle(void);
void CalibrateEncoder(void);


// Values used to calculate 15 bit signed int sent by the sensor
#define DELETE_BIT_15               0x7FFF
#define CHANGE_UINT_TO_INT_15       32768
#define CHECK_BIT_14                0x4000


#endif

