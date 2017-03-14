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

struct S_GAS_VALUE{
	uint8 possible;       //数据有效性，1表示数据有效，其它表示数据无效
    uint8 alarm;         //是否已经超过阈值报警
    uint8 value[3];       //气体数值
};


struct S_GAS_RESULT{
uint8 data_type;    //0xAA表示为气体浓度结果
uint8 terminial_id[13];   //终端的数据ID号
uint8 battery;
struct S_GAS_VALUE p_gas_value[6];           //气体数据，分别对应bit上的气体
uint8 checksum;              //异或校验核
};


struct S_GAS_RESULT g_gas_result;



/********************************************************************************/
//////////define
#define PASTATION_NAME_LEN	7
static uint8 g_station_name[PASTATION_NAME_LEN]={0x53, 0x31, 0x31, 0x30, 0x30, 0x30, 0x31};   //S110001

#define DEVICE_NAME_INFO_LENGH	13

/*******************************************************************************/

static void PAStation_Print_Gas_Value();


/*******************************************************************************/

void PAStation_SetDevice_Name(uint8 *buf,uint8 len)
{
	uint8 *g_device_name = buf;

	g_gas_result.terminial_id[0] = 'T';
	//device type
	g_gas_result.terminial_id[1] = (g_device_name[0] & 0x80)? '1':'0';
	g_gas_result.terminial_id[2] = (g_device_name[0] & 0x40)? '1':'0';
	g_gas_result.terminial_id[3] = (g_device_name[0] & 0x20)? '1':'0';

	//gas support
	g_gas_result.terminial_id[4] = (g_device_name[1] & 0x20)? '1':'0';
	g_gas_result.terminial_id[5] = (g_device_name[1] & 0x10)? '1':'0';
	g_gas_result.terminial_id[6] = (g_device_name[1] & 0x08)? '1':'0';
	g_gas_result.terminial_id[7] = (g_device_name[1] & 0x04)? '1':'0';
	g_gas_result.terminial_id[8] = (g_device_name[1] & 0x02)? '1':'0';
	g_gas_result.terminial_id[9] = (g_device_name[1] & 0x01)? '1':'0';

	//device number
	g_gas_result.terminial_id[10] = g_device_name[2];
	g_gas_result.terminial_id[11] = g_device_name[3];
	g_gas_result.terminial_id[12] = g_device_name[4];
	
}
void PAStation_SetDevice_Gas(uint8 *buf,uint8 len)
{
	//osal_memcpy(g_device_gas,buf,len);
	uint8 gas_type  = (buf[0] >> 4) & 0x0F ;
	g_gas_result.p_gas_value[gas_type].possible = (buf[0] & 0x0F)? 0x01:0x00;
	g_gas_result.p_gas_value[gas_type].alarm = 0;
	osal_memcpy((g_gas_result.p_gas_value[gas_type].value),&buf[1],len-2);
	g_gas_result.battery = buf[len - 1];
}

void PAStation_Print_Device_Value()
{
	PAStation_Print_Gas_Value();
	//NPI_PrintString("\n\r");
}


static void PAStation_Print_Gas_Value()
{
	uint8 i,checksum = 0x00;
	uint8 print_buffer[100];
	uint8 data_len = sizeof(struct S_GAS_RESULT);
	
	g_gas_result.data_type = 0xAA;
	osal_memcpy(print_buffer,(uint8 *)&g_gas_result,sizeof(struct S_GAS_RESULT));
	for(i = 1 ; i < (data_len -1); i++)
	{
		checksum = checksum ^ print_buffer[i];
	}
	print_buffer[data_len - 1] = checksum;
	
	NPI_PrintString_Length(print_buffer,data_len);
	//NPI_PrintString("\n\r");
}



