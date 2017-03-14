/******************************************************************************

 @file  simpleBLECentral.c

 @brief This file contains the Simple BLE Central sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

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

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "pamonitor.h"
#include "simpleBLECentral.h"
#include "npi.h"
#include "pastation.h"
/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  80

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 500

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_GENERAL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  FALSE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE


//Application States
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTE_SIMPLE,
  BLE_STATE_DISC_SERVICE,
  BLE_STATE_DISC_SERVICE_SUCESS,
  BLE_STATE_DISC_CHARACTER,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

enum{
  BLE_PAMONITOR_CHAR_1,
  BLE_PAMONITOR_CHAR_2,
  BLE_PAMONITOR_CHAR_3,
  BLE_PAMONITOR_CHAR_4,
  BLE_PAMONITOR_CHAR_5,
  BLE_PAMONITOR_CHAR_6,
};


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
static uint16 PAStationConnHandle = GAP_CONNHANDLE_INIT;
static uint16 PAStationSvcStartHdl,PAStationSvcEndHdl = 0;
static uint16 PAStationChar_Hdl[6];
	

// Application state
static uint8 PAStation_State = BLE_STATE_IDLE;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLE_Conn_Index;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// RSSI polling state
static uint8 simpleBLERssi = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
static void SimpleBLECentral_Start_Discovery_Device();

char *bdAddr2Str ( uint8 *pAddr );
static void NpiSerialCallback( uint8 port, uint8 events );


static void PAStation_Do_Next_Connect();
static void PAStation_Connect_Monitor(gapDevRec_t * devRec);
static void PAStation_Start_Discovery_Service(uint8 taskID);


static void PAStation_Process_GATT_Message(gattMsgEvent_t *pMsg);
static void PAStation_Process_Discovery_Event(gattMsgEvent_t *pMsg);
static void PAStation_Process_Discovery_Char_Event( gattMsgEvent_t *pMsg );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  NULL,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  NULL,
  NULL
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_ON );
  

	NPI_InitTransport(NpiSerialCallback);
	//NPI_WriteTransport("SimpleBLETest_Init\r\n", 20);
	NPI_PrintString("SimpleBLETest_Init\r\n");
	//NPI_PrintValue("Print value in decimal format 1 = ", 168, NPI_FORMAT_DEC);
	//NPI_PrintString("\r\n");
	//NPI_PrintValue("Print value in hexadecimal format2 = 0x",0x88, NPI_FORMAT_HEX);
	//NPI_PrintString("\r\n");


	// Setup a delayed profile startup
	osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
	
	SimpleBLECentral_Start_Discovery_Device();

	
}


static void SimpleBLECentral_Start_Discovery_Device()
{
	osal_set_event(simpleBLETaskId,START_DISOCVERY_DEVICE_EVT);
}
/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    //GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );
    
    return ( events ^ START_DEVICE_EVT );
  }


	if(events & START_DISOCVERY_DEVICE_EVT)
	{
		NPI_PrintString("Start discovery device\n\r");
		GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
		                                   DEFAULT_DISCOVERY_ACTIVE_SCAN,
		                                   DEFAULT_DISCOVERY_WHITE_LIST );  
		return ( events ^ START_DISOCVERY_DEVICE_EVT );
	}

	if(events & START_DISCOVERY_EVT)
	{
		PAStation_Start_Discovery_Service(simpleBLETaskId);
		return ( events ^ START_DISCOVERY_EVT );
	}

  	if(events & CONNECT_NEXT_DEVICE_EVENT)
	{
		//osal_stop_timerEx(simpleBLETaskId,CONNECT_NEXT_DEVICE_EVENT);
		NPI_PrintString("Start Try to connect one device\n\r");
		if(simpleBLE_Conn_Index >= simpleBLEScanRes)
		{
			NPI_PrintString("restart discovery device\n\r");
			SimpleBLECentral_Start_Discovery_Device();
		}
		else
		{
			NPI_PrintString("connect device\n\r");
			//osal_start_timerEx(simpleBLETaskId,CONNECT_NEXT_DEVICE_EVENT,1000);
			PAStation_Connect_Monitor(&simpleBLEDevList[simpleBLE_Conn_Index]);
			simpleBLE_Conn_Index++;
		}
		return ( events ^ CONNECT_NEXT_DEVICE_EVENT );
	}
	
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}



/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
	PAStation_Process_GATT_Message((gattMsgEvent_t *) pMsg );

}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */

static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
	switch ( pEvent->gap.opcode )
	{
		case GAP_DEVICE_INIT_DONE_EVENT:  
		{
			//LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
			//LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
			//NPI_PrintString("GAP_DEVICE_INIT_DONE_EVENT");
			}
		break;

		case GAP_DEVICE_INFO_EVENT:
		{
			//NPI_PrintString("Some Device Found\n\r");
			// if filtering device discovery results based on service UUID
			if ( simpleBLEFindSvcUuid( PAMONITORPROFILE_SERV_UUID,
								 pEvent->deviceInfo.pEvtData,
								 pEvent->deviceInfo.dataLen ) )
			{
				simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
			}
		}
		break;

		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			//NPI_PrintString("Device Discovery finished\n\r");
			// discovery complete
			simpleBLE_Conn_Index = 0;
			NPI_PrintValue("Device Found Count:",simpleBLEScanRes,NPI_FORMAT_DEC);
			NPI_PrintString("\n\r");

			osal_start_timerEx(simpleBLETaskId,CONNECT_NEXT_DEVICE_EVENT,5);

		}
		break;
	
		case GAP_LINK_ESTABLISHED_EVENT:
		{
			if ( pEvent->gap.hdr.status == SUCCESS )
			{		   
				NPI_PrintString("Device Connect Success\n\r");
				PAStation_State = BLE_STATE_CONNECTE_SIMPLE;
				PAStationConnHandle = pEvent->linkCmpl.connectionHandle;
				PAStation_Start_Discovery_Service(simpleBLETaskId);
				//LSQ -- "Connected"
			}
			else
			{
				NPI_PrintString("Device Connect Failed\n\r");
				PAStation_State = BLE_STATE_IDLE;
				PAStationConnHandle = GAP_CONNHANDLE_INIT;
				PAStation_Do_Next_Connect();
				//LSQ -- "Connect Failed"
			}
		}
		break;

		case GAP_LINK_TERMINATED_EVENT:
		{	
			NPI_PrintString("Device DisConnected\n\r");
			PAStation_State = BLE_STATE_IDLE;
			PAStationConnHandle = GAP_CONNHANDLE_INIT;
			PAStation_Do_Next_Connect();
			//LSQ -- Disconnected"    "Reason:", pEvent->linkTerminate.reason,
		}
		break;

		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			//LSQ -- "Param Update"
		}
		break;

		default:
			break;
	}

	return ( TRUE );
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
*********************************************************************/

static void NpiSerialCallback( uint8 port, uint8 events )  
{  
    (void)port;//?? (void),?????????,???????????????  
  
    if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))   //?????  
    {  
        uint8 numBytes = 0;  
  
        numBytes = NPI_RxBufLen();           //????????????  
          
        if(numBytes == 0)  
        {  
            return;  
        }  
        else  
        {  
            //?????buffer  
            uint8 *buffer = osal_mem_alloc(numBytes);  
            if(buffer)  
            {  
                //???????????,??????     
                NPI_ReadTransport(buffer,numBytes);     
  
                //???????????-????   
                NPI_WriteTransport(buffer, numBytes);    
  
                //????????  
                osal_mem_free(buffer);  
            }  
        }  
    }  
}  



/*******************************************************************************************/
static void PAStation_Do_Next_Connect()
{
	osal_set_event(simpleBLETaskId,CONNECT_NEXT_DEVICE_EVENT);
}


static void PAStation_Connect_Monitor(gapDevRec_t * devRec)
{
	uint8 addrType;
    uint8 *peerAddr;
    
    // Connect or disconnect
    if ( PAStation_State == BLE_STATE_IDLE )
    {
        // connect to current device in scan result
        peerAddr = devRec->addr;
        addrType = devRec->addrType;
      
        PAStation_State = BLE_STATE_CONNECTING;
        NPI_PrintString("Try to connect\n\r");
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
    }
}

void PAStation_Start_Discovery_Service(uint8 taskID)
{

	VOID taskID;
	uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(PAMONITORPROFILE_SERV_UUID),
	                   					HI_UINT16(PAMONITORPROFILE_SERV_UUID) };
	NPI_PrintString("Discovering service ...\n\r");
	// Initialize cached handles
	PAStationSvcStartHdl = PAStationSvcEndHdl = 0;
	PAStationChar_Hdl[BLE_PAMONITOR_CHAR_1] = PAStationChar_Hdl[BLE_PAMONITOR_CHAR_2] = PAStationChar_Hdl[BLE_PAMONITOR_CHAR_3] = 0 ;
	PAStationChar_Hdl[BLE_PAMONITOR_CHAR_4] = PAStationChar_Hdl[BLE_PAMONITOR_CHAR_5] = PAStationChar_Hdl[BLE_PAMONITOR_CHAR_6] = 0 ;

	PAStation_State = BLE_STATE_DISC_SERVICE;

	GATT_DiscPrimaryServiceByUUID( PAStationConnHandle,
		                                 uuid,
		                                 ATT_BT_UUID_SIZE,
		                                 simpleBLETaskId);
}

void PAStation_Process_GATT_Message(gattMsgEvent_t *pMsg)
{

/*
  if ( PAStation_State!= BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      //LSQ -- Read Error
    }
    else
    {
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.pValue[0];

	  //LSQ  -- Read rsp  valueRead
    }
    
    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
	  //LSQ -- Write Error
    }
    else
    {
	  //LSQ  -- Write sent:
    }
    
    simpleBLEProcedureInProgress = FALSE;    

  }
  else*/

	switch(PAStation_State)
	{
		case BLE_STATE_DISC_SERVICE:
			PAStation_Process_Discovery_Event( pMsg );
			break;
		case BLE_STATE_DISC_CHARACTER:
			PAStation_Process_Discovery_Char_Event( pMsg );
			break;
		default:
			break;
	}
  
  GATT_bm_free( &pMsg->msg, pMsg->method );
  
}


void PAStation_Process_Discovery_Event(gattMsgEvent_t *pMsg)
{
	gattReadByTypeReq_t req;


	// Service found, store handles
	if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
			pMsg->msg.findByTypeValueRsp.numInfo > 0 )
	{
		NPI_PrintString("Device Service discoved \n\r");
		PAStationSvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
		PAStationSvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
	}

	// If procedure complete
	if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
			pMsg->hdr.status == bleProcedureComplete ) ||
			( pMsg->method == ATT_ERROR_RSP ) )
	{
		NPI_PrintString("Device Service discove complete \n\r");
		if ( PAStationSvcStartHdl != 0 )
		{
			// Discover characteristic
			PAStation_State = BLE_STATE_DISC_CHARACTER;
			/*
			req.startHandle = PAStationSvcStartHdl;
			req.endHandle = PAStationSvcEndHdl;
			req.type.len = ATT_BT_UUID_SIZE;
			req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
			req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

			GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
			*/

			GATT_DiscAllChars(PAStationConnHandle,PAStationSvcStartHdl,PAStationSvcEndHdl,simpleBLETaskId);
		}
	}
			

	
}

static void PAStation_Process_Discovery_Char_Event( gattMsgEvent_t *pMsg )
{
	// Characteristic found, store handle
	if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
			pMsg->msg.readByTypeRsp.numPairs > 0 )
	{
		/* simpleBLECharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
		          pMsg->msg.readByTypeRsp.pDataList[1]);

		//LSQ -- Simple Svc Found
		simpleBLEProcedureInProgress = FALSE;
		*/
		NPI_PrintString("Device Character discoved \n\r");
	}

	// If procedure complete
	if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
			pMsg->hdr.status == bleProcedureComplete ) ||
			( pMsg->method == ATT_ERROR_RSP ) )
	{
		NPI_PrintString("Device Character discove complete \n\r");
		PAStation_State = BLE_STATE_CONNECTED;
	}
	
}

