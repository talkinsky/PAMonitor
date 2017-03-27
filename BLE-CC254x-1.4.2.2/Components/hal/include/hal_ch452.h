/******************************************************************************

 @file  hal_ch452.h

 @brief This file contains the interface to the ch452 2 wire chip.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2005-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

#ifndef HAL_CH452_H
#define HAL_CH452_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_board.h"

/*********************************************************************
 * MACROS
 */

/* CH451?CH452?????? */
#define CH452_NOP		0x0000					// ???
#define CH452_RESET     0x0201					// ??
#define CH452_LEVEL		0x0100					// ?????,???7???
#define CH452_CLR_BIT	0x0180					// ???0,???6???
#define CH452_SET_BIT	0x01C0					// ???1,???6???
#define CH452_SLEEP		0x0202					// ??????
#define CH452_LEFTMOV   0x0300		            // ??????-??
#define CH452_LEFTCYC   0x0301		            // ??????-???
#define CH452_RIGHTMOV  0x0302		            // ??????-??
#define CH452_RIGHTCYC  0x0303		            // ??????-???	
#define CH452_SELF_BCD	0x0380					// ???BCD?,???7???
#define CH452_SYSOFF    0x0400					// ?????????
#define CH452_SYSON1    0x0401					// ????
#define CH452_SYSON2    0x0403					// ???????
#define CH452_SYSON2W   0x0423					// ???????, ??2???
#define CH452_NO_BCD    0x0500					// ????????,???3?????
#define CH452_BCD       0x0580					// ??BCD????,???3?????
#define CH452_TWINKLE   0x0600		            // ??????,???8???
#define CH452_GET_KEY	0x0700					// ????,??????
#define CH452_DIG0      0x0800					// ????0??,???8???
#define CH452_DIG1      0x0900		            // ????1??,???8???
#define CH452_DIG2      0x0a00		            // ????2??,???8???
#define CH452_DIG3      0x0b00		            // ????3??,???8???
#define CH452_DIG4      0x0c00		            // ????4??,???8???
#define CH452_DIG5      0x0d00					// ????5??,???8???
#define CH452_DIG6      0x0e00					// ????6??,???8???
#define CH452_DIG7      0x0f00		            // ????7??,???8???

// BCD??????????
#define		CH452_BCD_SPACE		0x10
#define		CH452_BCD_PLUS		0x11
#define		CH452_BCD_MINUS		0x12
#define		CH452_BCD_EQU		0x13
#define		CH452_BCD_LEFT		0x14
#define		CH452_BCD_RIGHT		0x15
#define		CH452_BCD_UNDER		0x16
#define		CH452_BCD_CH_H		0x17
#define		CH452_BCD_CH_L		0x18
#define		CH452_BCD_CH_P		0x19
#define		CH452_BCD_DOT		0x1A
#define		CH452_BCD_SELF		0x1E
#define		CH452_BCD_TEST		0x88
#define		CH452_BCD_DOT_X		0x80

// ??????
#define		CH452_KEY_MIN		0x40
#define		CH452_KEY_MAX		0x7F

// 2????CH452??
#define		CH452_I2C_ADDR0		0x40			// CH452?ADDR=0????
#define		CH452_I2C_ADDR1		0x60			// CH452?ADDR=1????,???
#define		CH452_I2C_MASK		0x3E			// CH452?2??????????


/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Initialize CH452 clock and data line
 */
extern void HalCH452Init( void );

/*
 * Read keyboard from ch452
 */
extern uint8 HalCH452Read( );

/*
 * write one byte data to ch452
 */
extern void HalCH452Write( uint8 data);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
