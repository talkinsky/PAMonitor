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

static uint8 g_device_name[PAMONITORPROFILE_CHAR6_LEN];
static uint8 g_device_time[PAMONITORPROFILE_CHAR2_LEN];
static uint8 g_device_gas[PAMONITORPROFILE_CHAR4_LEN];

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
	VOID memcpy(print_buffer,"Result_String:",14);
	VOID memcpy(&print_buffer[14],g_device_name,PAMONITORPROFILE_CHAR6_LEN);
	print_buffer[14+PAMONITORPROFILE_CHAR6_LEN] = '\0';
	NPI_PrintString(print_buffer);
	NPI_PrintString("\n\r");
}


