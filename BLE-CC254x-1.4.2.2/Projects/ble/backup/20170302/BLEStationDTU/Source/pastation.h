

#ifndef __PASTATION_H__
#define __PASTATION_H__

#ifdef __cplusplus
extern "C"
{
#endif



/*********************************************************************
 * INCLUDES
 */



/*********************************************************************
*********************************************************************/

extern void PAStation_SetDevice_Name(uint8 *buf,uint8 len);
extern void PAStation_SetDevice_Gas(uint8 *buf,uint8 len);
extern void PAStation_SetDevice_Time(uint8 *buf,uint8 len);
extern void PAStation_Print_Device_Value();


#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */


