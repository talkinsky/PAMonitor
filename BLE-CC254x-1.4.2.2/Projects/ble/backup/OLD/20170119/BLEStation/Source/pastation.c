#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "pamonitor.h"
#include "npi.h"
#include "pastation.h"

/********************************************************************************/
//////////define
#define PASTATION_NAME_LEN	7
static uint8 g_station_name[PASTATION_NAME_LEN]={0x53, 0x31, 0x31, 0x30, 0x30, 0x30, 0x31};   //S110001

static uint8 g_device_name[PAMONITORPROFILE_CHAR6_LEN];
static uint8 g_device_time[PAMONITORPROFILE_CHAR2_LEN];
static uint8 g_device_gas[PAMONITORPROFILE_CHAR4_LEN];

/*******************************************************************************/

static void PAStation_Print_Locate_Value();
static void PAStation_Print_Gas_Value();


/*******************************************************************************/



static uint8 print_buffer[50];
void PAStation_SetDevice_Name(uint8 *buf,uint8 len)
{
	osal_memcpy(g_device_name,buf,len);
}
void PAStation_SetDevice_Gas(uint8 *buf,uint8 len)
{
	osal_memcpy(g_device_gas,buf,len);
}
void PAStation_SetDevice_Time(uint8 *buf,uint8 len)
{
	osal_memcpy(g_device_time,buf,len);
}

void PAStation_Print_Device_Value()
{
	PAStation_Print_Locate_Value();
	//PAStation_Print_Gas_Value();
	//NPI_PrintString("\n\r");
}

static void PAStation_Print_Locate_Value()
{
	print_buffer[0] = 0x03;
	print_buffer[1] = 19;
	osal_memcpy(&print_buffer[2],g_station_name,PASTATION_NAME_LEN);
	osal_memcpy(&print_buffer[2 + PASTATION_NAME_LEN],g_device_name,PAMONITORPROFILE_CHAR6_LEN);
	osal_memcpy(&print_buffer[2 + PASTATION_NAME_LEN + PAMONITORPROFILE_CHAR6_LEN],g_device_time,PAMONITORPROFILE_CHAR2_LEN);
	print_buffer[2 + PASTATION_NAME_LEN + PAMONITORPROFILE_CHAR6_LEN + PAMONITORPROFILE_CHAR2_LEN] = 0x0C;
	NPI_PrintString_Length(print_buffer,2 + PASTATION_NAME_LEN + PAMONITORPROFILE_CHAR6_LEN + PAMONITORPROFILE_CHAR2_LEN + 1);
	//NPI_PrintString("\n\r");
}

static void PAStation_Print_Gas_Value()
{
	print_buffer[0] = 0x04;             																			//Package Type
	print_buffer[1] = 14;																							//Package Length
	osal_memcpy(&print_buffer[2],g_device_name,PAMONITORPROFILE_CHAR6_LEN);											//Terminal Name
	print_buffer[2 + PAMONITORPROFILE_CHAR6_LEN] = PAMONITORPROFILE_CHAR2_LEN;										//Value	Length
	print_buffer[2 + PAMONITORPROFILE_CHAR6_LEN +1 ] = PAMONITORPROFILE_CHAR2_LEN;									//Gas Type
	osal_memcpy(&print_buffer[2 + PAMONITORPROFILE_CHAR6_LEN + 2],g_device_gas,PAMONITORPROFILE_CHAR4_LEN);			//value
	print_buffer[2 + PAMONITORPROFILE_CHAR6_LEN + 2 + PAMONITORPROFILE_CHAR4_LEN ] = 0x0D;    //checksum				//Check Sum
	
	NPI_PrintString_Length(print_buffer,2 + PAMONITORPROFILE_CHAR6_LEN + 2 + PAMONITORPROFILE_CHAR4_LEN +1);
	//NPI_PrintString("\n\r");
}

