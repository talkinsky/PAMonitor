/******************************************************************************

 @file  PAMonitor.c

 @brief This file contains the PAMonitor GATT profile sample GATT service profile
        for use with the BLE sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "pamonitor.h"
#include "timer.h"
#include "gas.h"

#include "hal_led.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        20

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// PAMonitor GATT Profile Service UUID: 0xFFF0
CONST uint8 PAMonitorProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PAMONITORPROFILE_SERV_UUID), HI_UINT16(PAMONITORPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 PAMonitorProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PAMONITORPROFILE_CHAR1_UUID), HI_UINT16(PAMONITORPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 PAMonitorProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PAMONITORPROFILE_CHAR2_UUID), HI_UINT16(PAMONITORPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 PAMonitorProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PAMONITORPROFILE_CHAR3_UUID), HI_UINT16(PAMONITORPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 PAMonitorProfilechar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PAMONITORPROFILE_CHAR4_UUID), HI_UINT16(PAMONITORPROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8 PAMonitorProfilechar5UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PAMONITORPROFILE_CHAR5_UUID), HI_UINT16(PAMONITORPROFILE_CHAR5_UUID)
};

// Characteristic 5 UUID: 0xFFF6
CONST uint8 PAMonitorProfilechar6UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(PAMONITORPROFILE_CHAR6_UUID), HI_UINT16(PAMONITORPROFILE_CHAR6_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static PAMonitorProfileCBs_t *PAMonitorProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// PAMonitor Profile Service attribute
static CONST gattAttrType_t PAMonitorProfileService = { ATT_BT_UUID_SIZE, PAMonitorProfileServUUID };


// PAMonitor Profile Characteristic 1 Properties
static uint8 PAMonitorProfileChar1Props = GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 PAMonitorProfileChar1 = 0x00;

// PAMonitor Profile Characteristic 1 User Description
static uint8 PAMonitorProfileChar1UserDesp[17] = "Start Timer Cont";


// PAMonitor Profile Characteristic 2 Properties
static uint8 PAMonitorProfileChar2Props = GATT_PROP_READ;

// Characteristic 2 Value
static uint8 PAMonitorProfileChar2[PAMONITORPROFILE_CHAR2_LEN] = {0x00, 0x00, 0x00, 0x00};

// PAMonitor Profile Characteristic 2 User Description
static uint8 PAMonitorProfileChar2UserDesp[17] = "Get Timer Count ";


// PAMonitor Profile Characteristic 3 Properties
static uint8 PAMonitorProfileChar3Props =  GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 PAMonitorProfileChar3 = 0;

// PAMonitor Profile Characteristic 3 User Description
static uint8 PAMonitorProfileChar3UserDesp[17] = "Start Gas Calibr";


// PAMonitor Profile Characteristic 4 Properties
static uint8 PAMonitorProfileChar4Props = GATT_PROP_READ;

// Characteristic 4 Value
static uint8 PAMonitorProfileChar4 = 0x00;

// Simple Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *PAMonitorProfileChar4Config;

										
// PAMonitor Profile Characteristic 4 User Description
static uint8 PAMonitorProfileChar4UserDesp[17] = "Cali Res Notify ";


// PAMonitor Profile Characteristic 5 Properties
static uint8 PAMonitorProfileChar5Props = GATT_PROP_WRITE;

// Characteristic 5 Value
static uint8 PAMonitorProfileChar5 = 0x00;

// PAMonitor Profile Characteristic 5 User Description
static uint8 PAMonitorProfileChar5UserDesp[17] = "Alarm control   ";


// PAMonitor Profile Characteristic 6 Properties
static uint8 PAMonitorProfileChar6Props = GATT_PROP_READ;

// Characteristic 6 Value
static uint8 PAMonitorProfileChar6[PAMONITORPROFILE_CHAR6_LEN] = {0x00};   //Device name T000001

// PAMonitor Profile Characteristic 6 User Description
static uint8 PAMonitorProfileChar6UserDesp[17] = "Device Name char";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t PAMonitorProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // PAMonitor Profile Service
	{ 
		{ ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
		GATT_PERMIT_READ,                         /* permissions */
		0,                                        /* handle */
		(uint8 *)&PAMonitorProfileService            /* pValue */
	},

    // Characteristic 1 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&PAMonitorProfileChar1Props 
	},
    // Characteristic Value 1
	{ 
		{ ATT_BT_UUID_SIZE, PAMonitorProfilechar1UUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
		0, 
		&PAMonitorProfileChar1 
	},
	// Characteristic 1 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		PAMonitorProfileChar1UserDesp 
	},      


	// Characteristic 2 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&PAMonitorProfileChar2Props 
	},
	// Characteristic Value 2
	{ 
		{ ATT_BT_UUID_SIZE, PAMonitorProfilechar2UUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
		0, 
		PAMonitorProfileChar2 
	},
	// Characteristic 2 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		PAMonitorProfileChar2UserDesp 
	},           


		
	// Characteristic 3 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&PAMonitorProfileChar3Props 
	},
	// Characteristic Value 3
	{ 
		{ ATT_BT_UUID_SIZE, PAMonitorProfilechar3UUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
		0, 
		&PAMonitorProfileChar3 
	},
	// Characteristic 3 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		PAMonitorProfileChar3UserDesp 
	},


	// Characteristic 4 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&PAMonitorProfileChar4Props 
	},
	// Characteristic Value 4
	{ 
		{ ATT_BT_UUID_SIZE, PAMonitorProfilechar4UUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
		0, 
		&PAMonitorProfileChar4 
	},

	// Characteristic 4 configuration
	  { 
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
		0, 
		(uint8 *)&PAMonitorProfileChar4Config 
	  },
		  
	// Characteristic 4 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		PAMonitorProfileChar4UserDesp 
	},


			

		
	// Characteristic 5 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&PAMonitorProfileChar5Props 
	},

	// Characteristic Value 5
	{ 
		{ ATT_BT_UUID_SIZE, PAMonitorProfilechar5UUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
		0, 
		&PAMonitorProfileChar5 
	},

	// Characteristic 5 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		PAMonitorProfileChar5UserDesp 
	},


	// Characteristic 6 Declaration
	{ 
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 
		0,
		&PAMonitorProfileChar6Props 
	},

	// Characteristic Value 6
	{ 
		{ ATT_BT_UUID_SIZE, PAMonitorProfilechar6UUID },
		GATT_PERMIT_READ, 
		0, 
		PAMonitorProfileChar6 
	},

	// Characteristic 6 User Description
	{ 
		{ ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 
		0, 
		PAMonitorProfileChar6UserDesp 
	},
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t PAMonitorProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method );
static bStatus_t PAMonitorProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// PAMonitor Profile Service Callbacks
CONST gattServiceCBs_t PAMonitorProfileCBs =
{
  PAMonitorProfile_ReadAttrCB,  // Read callback function pointer
  PAMonitorProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      PAMonitorProfile_AddService
 *
 * @brief   Initializes the PAMonitor Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t PAMonitorProfile_AddService( uint32 services )
{
    uint8 status;
  
  // Allocate Client Characteristic Configuration table
  PAMonitorProfileChar4Config = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                              linkDBNumConns );
  if ( PAMonitorProfileChar4Config == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, PAMonitorProfileChar4Config );
  
  if ( services & PAMONITORPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( PAMonitorProfileAttrTbl, 
                                          GATT_NUM_ATTRS( PAMonitorProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &PAMonitorProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      PAMonitorProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t PAMonitorProfile_RegisterAppCBs( PAMonitorProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    PAMonitorProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      PAMonitorProfile_SetParameter
 *
 * @brief   Set a PAMonitor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t PAMonitorProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case PAMONITORPROFILE_CHAR1:
		if ( len == sizeof ( uint8 ) ) 
		{
			PAMonitorProfileChar1 = *((uint8*)value);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;

    case PAMONITORPROFILE_CHAR2:
		if ( len == PAMONITORPROFILE_CHAR2_LEN ) 
		{
			VOID memcpy( PAMonitorProfileChar2, value, PAMONITORPROFILE_CHAR2_LEN );
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;

    case PAMONITORPROFILE_CHAR3:
		if ( len == sizeof ( uint8 ) ) 
		{
			PAMonitorProfileChar3 = *((uint8*)value);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
	  
	case PAMONITORPROFILE_CHAR4:
      if ( len == sizeof ( uint8 ) ) 
      {
        PAMonitorProfileChar4 = *((uint8*)value);
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( PAMonitorProfileChar4Config, &PAMonitorProfileChar4, FALSE,
                                    PAMonitorProfileAttrTbl, GATT_NUM_ATTRS( PAMonitorProfileAttrTbl ),
                                    INVALID_TASK_ID, PAMonitorProfile_ReadAttrCB );
		//PAMonitorProfileChar4
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case PAMONITORPROFILE_CHAR5:
		if ( len == sizeof ( uint8 ) ) 
		{
			PAMonitorProfileChar5 = *((uint8*)value);
		}
		else
		{
			ret = bleInvalidRange;
		}
		break;
      
    default:
		ret = INVALIDPARAMETER;
		break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      PAMonitorProfile_GetParameter
 *
 * @brief   Get a PAMonitor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t PAMonitorProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case PAMONITORPROFILE_CHAR1:
      *((uint8*)value) = PAMonitorProfileChar1;
      break;

    case PAMONITORPROFILE_CHAR2:
      VOID memcpy( value, PAMonitorProfileChar2, PAMONITORPROFILE_CHAR2_LEN );
      break;      

    case PAMONITORPROFILE_CHAR3:
      *((uint8*)value) = PAMonitorProfileChar3;
      break;  

    case PAMONITORPROFILE_CHAR4:
	  *((uint8*)value) = PAMonitorProfileChar4;
      break;

    case PAMONITORPROFILE_CHAR5:
      *((uint8*)value) = PAMonitorProfileChar5;
      break;      
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          PAMonitorProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t PAMonitorProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here

      case PAMONITORPROFILE_CHAR2_UUID:
		*pLen = PAMONITORPROFILE_CHAR2_LEN;
		get_timer1(PAMonitorProfileChar2);
		stop_timer1();
		VOID memcpy( pValue, pAttr->pValue, PAMONITORPROFILE_CHAR2_LEN );
		break;
      case PAMONITORPROFILE_CHAR4_UUID:
	  	*pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
	  case PAMONITORPROFILE_CHAR6_UUID:
	  	*pLen = PAMONITORPROFILE_CHAR6_LEN;
        VOID memcpy( pValue, pAttr->pValue, PAMONITORPROFILE_CHAR6_LEN );
        break;
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      PAMonitorProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t PAMonitorProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
	{
		case PAMONITORPROFILE_CHAR1_UUID:
			//Validate the value
			// Make sure it's not a blob oper
			if ( offset == 0 )
			{
				if ( len != 1 )
				{
					status = ATT_ERR_INVALID_VALUE_SIZE;
				}
			}
			else
			{
				status = ATT_ERR_ATTR_NOT_LONG;
			}

			//Write the value
			if ( status == SUCCESS )
			{
				uint8 *pCurValue = (uint8 *)pAttr->pValue;        
				*pCurValue = pValue[0];
				//notifyApp = PAMONITORPROFILE_CHAR1;    
				if(0x01 == PAMonitorProfileChar1)
				{
					start_timer1();
				}
				else if(0x0A == PAMonitorProfileChar1)
				{
					get_timer1(PAMonitorProfileChar2);
					stop_timer1();
				}
				else if(0x00 == PAMonitorProfileChar1)
				{
					stop_timer1();
				}

			
			}   
			break;
		case PAMONITORPROFILE_CHAR3_UUID:
			//Validate the value
			// Make sure it's not a blob oper
			if ( offset == 0 )
			{
				if ( len != 1 )
				{
					status = ATT_ERR_INVALID_VALUE_SIZE;
				}
			}
			else
			{
				status = ATT_ERR_ATTR_NOT_LONG;
			}

			//Write the value
			if ( status == SUCCESS )
			{
				uint8 *pCurValue = (uint8 *)pAttr->pValue;        
				*pCurValue = pValue[0];
				notifyApp = PAMONITORPROFILE_CHAR3;        
			}   
			break;
		case PAMONITORPROFILE_CHAR4_UUID:
			//Validate the value
			// Make sure it's not a blob oper
			if ( offset == 0 )
			{
				if ( len != 1 )
				{
					status = ATT_ERR_INVALID_VALUE_SIZE;
				}
			}
			else
			{
				status = ATT_ERR_ATTR_NOT_LONG;
			}

			//Write the value
			if ( status == SUCCESS )
			{
				uint8 *pCurValue = (uint8 *)pAttr->pValue;        
				*pCurValue = pValue[0];
				notifyApp = PAMONITORPROFILE_CHAR4;        
			}   
			break;
			
		case PAMONITORPROFILE_CHAR5_UUID:
			//Validate the value
			// Make sure it's not a blob oper
			if ( offset == 0 )
			{
				if ( len != 1 )
				{
					status = ATT_ERR_INVALID_VALUE_SIZE;
				}
			}
			else
			{
				status = ATT_ERR_ATTR_NOT_LONG;
			}

			//Write the value
			if ( status == SUCCESS )
			{
				uint8 *pCurValue = (uint8 *)pAttr->pValue;        
				*pCurValue = pValue[0];
				notifyApp = PAMONITORPROFILE_CHAR5;        
			}   
			break;

		case GATT_CLIENT_CHAR_CFG_UUID:
			status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
			                                 offset, GATT_CLIENT_CFG_NOTIFY );
			break;

		default:
			// Should never get here! (characteristics 2 and 4 do not have write permissions)
			status = ATT_ERR_ATTR_NOT_FOUND;
			break;
	}
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && PAMonitorProfile_AppCBs && PAMonitorProfile_AppCBs->pfnPAMonitorProfileChange )
  {
    PAMonitorProfile_AppCBs->pfnPAMonitorProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/
