#ifndef __PAMONITOR_GAS_DEF__
#define  __PAMONITOR_GAS_DEF__
#ifdef __cplusplus
extern "C"
{
#endif
#include "Hal_types.h"

/*****************
@ * state machine define
******************/
#define GAS_STANDBY 	0x00
#define GAS_PREPARE 	0x01
#define GAS_MONITOR 	0x02


/*********  NV data ID to store calibration**********/
#define GAS_CALI_DATA_ID 0xF0


void gas_monitor_init();
uint8 gas_monitor_start();
uint8 gas_monitor_stop();
uint8 gas_monitor_do_cali();

uint8 gas_monitor_get_data(uint8 * buf);



#ifdef __cplusplus
}
#endif

#endif
