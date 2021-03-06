/******************************************************************************

 @file  simpleBLEPeripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
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
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_exgas_sensor.h"
#include "hal_dig.h"
#include "hal_beep.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
//#include "simpleGATTprofile.h"

#include "pamonitor.h"


#include "simplekeys.h"


#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "params.h"
#include "npi.h"

#include "crc.h"
/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160   //20ms

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     1

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         2

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


#define 	WORK_DISABLE					0
#define 	WORK_ENABLE						1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
static void NpiSerialCallback( uint8 port, uint8 events );

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing
#define BLE_DEVICE_NAME 0xA0,0x01,0x31,0x30,0x31   //0x20 means pamonitor and 0xA0 means gas sensor in fix position

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0A,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x50,   // 'P'
  0x41,   // 'A'
  0x4d,   // 'M'
  0x6f,   // 'o'
  0x6e,   // 'n'
  0x69,   // 'i'
  0x74,   // 't'
  0x6f,   // 'o'
  0x72,   // 'r'


  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( PAMONITORPROFILE_SERV_UUID ),
  HI_UINT16( PAMONITORPROFILE_SERV_UUID ),

  0x0D,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  0x0D,
  0x0D,
  BLE_DEVICE_NAME,
  GAS_TYPE_CH4 << 4,    //start of gas  ch4   (gas type | gas possible)
  0x00,
  0x00,
  0x00,      //end of gas
  0x30       //battery
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
//static void performPeriodicTask( void );
//static void simpleProfileChangeCB( uint8 paramID );
static void PAMonitorProfileChangeCB( uint8 paramID );
static void EXGASSensorValueMonitorCB( uint16 paramID );

static void PAMonitorWorkEnable(uint8 enable);
static void AlarmEnable(uint8 enable);


static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

	
// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

/*
// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
*/

// PAMonitor Profile Callbacks
static PAMonitorProfileCBs_t PAMonitorPeripheral_PAMonitorProfileCBs =
{
  PAMonitorProfileChangeCB    // Charactersitic value change callback
};



static EXGASSenorValueMonitorCBs_t EXGAS_Sensor_Value_MonitorCBSs=
{
	EXGASSensorValueMonitorCB    // call back function in monitor gas value
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */


#ifdef LSQ_ADD
static void SimpleBLE_Updata_Adverting_Data(uint8 gas_type, uint8 alarm, uint16 value)
{
	uint8 advertEnabled = FALSE; // Turn on Advertising
    // Disable connectable advertising.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                         &advertEnabled);
	advertData[16] = 0x00;
	advertData[16] = (gas_type << 4) | 0x08;  //gas type and data valid
	advertData[16] = advertData[16] | ((1 == alarm)? 0x04:0x00);
	advertData[17] = 0x00;       //data[17] reserve for future use
	osal_memcpy(&advertData[18],&value,2);
	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
   
    // Set to true for non-connectabel advertising.
    advertEnabled = TRUE;
    // Enable non-connectable advertising.
    GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                         &advertEnabled);
}

static void SimpleBLE_Updata_Adverting_Battery_Data(uint8 type, uint8 value)
{
	uint8 advertEnabled = FALSE; // Turn on Advertising
    // Disable connectable advertising.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                         &advertEnabled);
	
	advertData[20] = value;
	GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
   
    // Set to true for non-connectabel advertising.
    advertEnabled = TRUE;
    // Enable non-connectable advertising.
    GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                         &advertEnabled);
}


#endif

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{

	uint8 temp = 0x0A;
#ifdef LSQ_ADD
	HCI_EXT_ExtendRfRangeCmd();
	HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_4_DBM);
	HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
	
#endif
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
  	//device starts advertising upon initialization
  	uint8 initial_advertising_enable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    //uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    //GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );    //LSQ change to never stop advert  20170110

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  //GGS_AddService( GATT_ALL_SERVICES );            // GAP
  //GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
 // DevInfo_AddService();                           // Device Information Service
  //SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  #ifdef LSQ_ADD
  PAMonitorProfile_AddService(GATT_ALL_SERVICES);
  #endif

#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif
#if 0
  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }
#endif

	NPI_InitTransport(NpiSerialCallback);

#if defined EXGAS_SENSOR
	HalEXSensorInit();
	HalEXGasSensorRegisterCallback(&EXGAS_Sensor_Value_MonitorCBSs);
#endif
	// makes sure LEDs are off
	PAMonitorWorkEnable(WORK_ENABLE);

  // Register callback with SimpleGATTprofile
//  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );
	VOID PAMonitorProfile_RegisterAppCBs(&PAMonitorPeripheral_PAMonitorProfileCBs);


  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function
  uint8 temp = 0;
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }


  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    //performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

   if ( events & SBP_GAS_SENSOR_CALI_TMP_EVT )
  {
  	char res = 0x01;
    HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_ON );
	PAMonitorProfile_SetParameter(PAMONITORPROFILE_CHAR4, 1, &res);
    return (events ^ SBP_GAS_SENSOR_CALI_TMP_EVT);
  }
   

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {     
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, 
                                      ((keyChange_t *)pMsg)->keys );
      break;
 
    case GATT_MSG_EVENT:
      // Process GATT message
      simpleBLEPeripheral_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {
  	SK_Keys |= SK_KEY_RIGHT;
  }

}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    case GAPROLE_ADVERTISING:
      {

      }
      break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8 advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                           &advertEnabled);
        
        // Reset flag for next connection.
        first_conn_flag = 0;
      }
      break;
#endif //PLUS_BROADCASTER         
      
    case GAPROLE_CONNECTED:
      {        

          start_timer1();
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
            uint8 advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;
            
            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:        //"Connected Advertising"
      {

      }
      break;      
    case GAPROLE_WAITING:  //"Disconnected"
      {
#ifdef PLUS_BROADCASTER                
        uint8 advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                             &advertEnabled);
#endif //PLUS_BROADCASTER
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:    //"Timed Out"
      {

#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:      // "Error"
      {

      }
      break;

    default:      //""
      {

      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}



/*********************************************************************

*********************************************************************/

static void PAMonitorProfileChangeCB( uint8 paramID )
{
	uint8 newValue;
	
	switch( paramID )
	{
		case PAMONITORPROFILE_CHAR1:
			PAMonitorProfile_GetParameter( PAMONITORPROFILE_CHAR1, &newValue );
			break;
		
		case PAMONITORPROFILE_CHAR3:
			PAMonitorProfile_GetParameter( PAMONITORPROFILE_CHAR3, &newValue );
			if(0x01 == newValue)
			{
				HalLedSet( (HAL_LED_POWER ), HAL_LED_MODE_FLASH );
    			osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_GAS_SENSOR_CALI_TMP_EVT, SBP_PERIODIC_EVT_PERIOD );
			}
			else
			{
				HalLedSet( (HAL_LED_POWER ), HAL_LED_MODE_ON );
				osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_GAS_SENSOR_CALI_TMP_EVT);
			}
			break;
		case PAMONITORPROFILE_CHAR4:
					PAMonitorProfile_GetParameter( PAMONITORPROFILE_CHAR4, &newValue );
					break;

		case PAMONITORPROFILE_CHAR5:
			PAMonitorProfile_GetParameter( PAMONITORPROFILE_CHAR5, &newValue );
			break;

		default:
			// should not reach here!
			break;
	}

}



void PAMonitorWorkEnable(uint8 enable)
{
	uint8 initial_advertising_enable = FALSE;
	if(enable) //PAMonitor start work
	{
		initial_advertising_enable = TRUE;
	    // Set the GAP Role Parameters
	    //HalLedSet( (HAL_LED_POWER ), HAL_LED_INV_MODE_ON );
		//HalBeepSet( (HAL_BEEP_ALL), HAL_BEEP_MODE_OFF);
		//HalLedSet( (HAL_LED_ALARM), HAL_LED_INV_MODE_OFF );
	    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
		SimpleBLE_Updata_Adverting_Data(GAS_TYPE_CH4, 0, 0);
		
	}
	else
	{
		//HalLedSet( (HAL_LED_POWER ), HAL_LED_INV_MODE_OFF );
		//HalLedSet( (HAL_LED_ALARM), HAL_LED_INV_MODE_OFF );
        //HalBeepSet( (HAL_BEEP_ALL), HAL_BEEP_MODE_OFF);
		//HalDigEnterSleep();
		SimpleBLE_Updata_Adverting_Data(GAS_TYPE_CH4, 0, 0);
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
		
	}
	HalEXSensorEnable( initial_advertising_enable );
}



static void AlarmEnable(uint8 enable)
{
	if(enable) //PAMonitor start work
	{
		HalLedSet( (HAL_LED_ALARM), HAL_LED_INV_MODE_ON);
		HalBeepSet( (HAL_BEEP_ALL), HAL_BEEP_MODE_ON);
		HalDigShowAlarm(1);
	}
	else
	{
		HalLedSet( (HAL_LED_ALARM), HAL_LED_INV_MODE_OFF);
		HalBeepSet( (HAL_BEEP_ALL), HAL_BEEP_MODE_OFF);
		HalDigShowAlarm(0);
	}
}

static uint16 last_val = 0;
static uint16 no_up_count = 0;
#define MAX_NO_UP_COUNT 500
static void EXGASSensorValueMonitorCB( uint16 paramID )
{
	if(last_val == paramID)
	{
		if (no_up_count < MAX_NO_UP_COUNT)
			return;
	}
	
	last_val = paramID;
	if(paramID > 99)
	{
		paramID = 99;
	}
	//HalDigShow(paramID);
	if(paramID > 25)
	{
		 SimpleBLE_Updata_Adverting_Data(GAS_TYPE_CH4, 1, paramID);
		 //AlarmEnable(1);
	}
	else
	{
		 SimpleBLE_Updata_Adverting_Data(GAS_TYPE_CH4, 0, paramID);
		 //AlarmEnable(0);
	}	
	return;
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
                //NPI_WriteTransport(buffer, numBytes);    
                HalEXGasValueProcess( buffer, numBytes );
  
                //????????  
                osal_mem_free(buffer);  
            }  
        }  
    }  
}  


