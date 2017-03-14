/******************************************************************************

 @file  PAMonitor.h

 @brief This file contains the Simple GATT profile definitions and prototypes.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2016, Texas Instruments Incorporated
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
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

#ifndef __PAMONITORPROFILE_H__
#define __PAMONITORPROFILE_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * MACROS
 */

enum{
  GAS_TYPE_CH4 = 0x01,
  GAS_TYPE_SO,
  GAS_TYPE_SH,
  GAS_TYPE_CO2,
  GAS_TYPE_REV,
  GAS_TYPE_REV_1,
};


/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define PAMONITORPROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value 
#define PAMONITORPROFILE_CHAR2                   1  // RW uint8 - Profile Characteristic 2 value
#define PAMONITORPROFILE_CHAR3                   2  // RW uint8 - Profile Characteristic 3 value
#define PAMONITORPROFILE_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value
#define PAMONITORPROFILE_CHAR5                   4  // RW uint8 - Profile Characteristic 4 value
#define PAMONITORPROFILE_CHAR6                   5  // RW uint8 - Profile Characteristic 4 value

  
// PAMonitor Service UUID
#define PAMONITORPROFILE_SERV_UUID               0xFFF0
    
// PAMonitor function character UUID
#define PAMONITORPROFILE_CHAR1_UUID            0xFFF1  //Used for timer control
#define PAMONITORPROFILE_CHAR2_UUID            0xFFF2  //used for timer read
#define PAMONITORPROFILE_CHAR3_UUID            0xFFF3  //used for gas control
#define PAMONITORPROFILE_CHAR4_UUID            0xFFF4  //used for gas read
#define PAMONITORPROFILE_CHAR5_UUID            0xFFF5  //used for alarm control
#define PAMONITORPROFILE_CHAR6_UUID            0xFFF6  //used for Device Name

  
// Simple Keys Profile Services bit fields
#define PAMONITORPROFILE_SERVICE               0x00000001


// Length of Characteristic 2 in bytes 
#define PAMONITORPROFILE_CHAR2_LEN			4

// Length of Characteristic 6 in bytes 
#define PAMONITORPROFILE_CHAR6_LEN			5

/*********************************************************************
 * TYPEDEFS
 */

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*PAMonitorProfileChange_t)( uint8 paramID );

typedef struct
{
  PAMonitorProfileChange_t        pfnPAMonitorProfileChange;  // Called when characteristic value changes
} PAMonitorProfileCBs_t;

    

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * PAMonitorProfile_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t PAMonitorProfile_AddService( uint32 services );

/*
 * PAMonitorProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t PAMonitorProfile_RegisterAppCBs( PAMonitorProfileCBs_t *appCallbacks );

/*
 * PAMonitorProfile_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t PAMonitorProfile_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * PAMonitorProfile_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t PAMonitorProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */
