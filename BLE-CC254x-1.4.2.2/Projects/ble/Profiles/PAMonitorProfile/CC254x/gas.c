#include "gas.h"

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"

#include "gatt.h"

#include "hci.h"

#include "peripheral.h"

#include "gapbondmgr.h"


static uint8 gas_state =  GAS_STANDBY;

static uint8 g_cali[4] = {0x00 , 0x00 , 0x00 , 0x00};  //calibration data

static void gas_nv_data_init();
static uint8 get_nv_cali_data(uint8 * data);
static uint8 store_nv_cali_data(uint8 * data);

/*********************************************************/

/****************ADC Function******************************/
static void init_gas_ADC()
{

}

static void get_gas_ADC_value(uint8 * buf)
{

}

/******************************************************
*******************************************************/
void gas_monitor_init()
{
	gas_nv_data_init();
	init_gas_ADC();
}
uint8 gas_monitor_start()
{
	//here should enable gpio and start timer to wait about 10 Second
	gas_state = GAS_PREPARE;
}
uint8 gas_monitor_stop()
{
	//here should close all timer and close sensor power
	gas_state = GAS_STANDBY;
}
uint8 gas_monitor_do_cali()
{
	//if standby mode , do start first
	//if prepare mode , wait for monitor mode
	//if monitor mode , do monitor
}
	
uint8 gas_monitor_get_data(uint8 * buf)
{
	//get adc value
	//get_gas_ADC_value();
}

/******************************************************
*******************************************************/
static void gas_nv_data_init()
{
	get_nv_cali_data(g_cali);
}

static uint8 get_nv_cali_data(uint8* data)
{
	uint8 ret;
	ret = osal_snv_write(GAS_CALI_DATA_ID,4,data);
	if(SUCCESS != ret)
	{
		osal_memset(data,0x00,4);
	}
	return ret;
}
static uint8 store_nv_cali_data(uint8* data)
{
	uint8 ret = SUCCESS;
	ret = osal_snv_read(GAS_CALI_DATA_ID,4,data);
	return ret;
}

