/******************************************************************************

 @file  hal_beep.c

 @brief This file contains the interface to the HAL BEEP Service.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2006-2016, Texas Instruments Incorporated
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

 YOU FURTHER ACKNOWBEEPGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
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
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_beep.h"
#include "osal.h"
#include "hal_board.h"

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/

/* BEEP control structure */
typedef struct {
  uint8 mode;       /* Operation mode */
  uint8 left;       /* Blink cycles left */
  uint8 onPct;      /* On cycle percentage */
  uint16 time;      /* On/off cycle time (msec) */
  uint32 next;      /* Time for next change */
} HalBeepControl_t;

typedef struct
{
  HalBeepControl_t HalBeepControlTable[HAL_BEEP_DEFAULT_MAX_BEEPS];
  uint8           sleepActive;
} HalBeepStatus_t;

/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/

static uint8 HalBeepState;              // BEEP state at last set/clr/blink update

#if HAL_BEEP == TRUE
static uint8 HalSleepBeepState;         // BEEP state at last set/clr/blink update
static uint8 preBlinkState;            // Original State before going to blink mode
                                       // bit 0, 1, 2, 3 represent beep 0, 1, 2, 3
#endif

#ifdef BLINK_BEEPS
  static HalBeepStatus_t HalBeepStatusControl;
#endif

/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/

#if (HAL_BEEP == TRUE)
void HalBeepUpdate (void);
void HalBeepOnOff (uint8 beeps, uint8 mode);
#endif /* HAL_BEEP */

/***************************************************************************************************
 *                                            FUNCTIONS - API
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      HalBeepInit
 *
 * @brief   Initialize BEEP Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/
void HalBeepInit (void)
{
#if (HAL_BEEP == TRUE)
  // Set BEEP GPIOs to outputs.
  HAL_SET_OUTPUT_BEEP();

  HalBeepSet(HAL_BEEP_ALL, HAL_BEEP_MODE_OFF);  // Initialize all BEEPs to OFF.
  
#if defined BLINK_BEEPS
  HalBeepStatusControl.sleepActive = FALSE;  // Initialize sleepActive to FALSE.
#endif
#endif
}

/***************************************************************************************************
 * @fn      HalBeepSet
 *
 * @brief   Tun ON/OFF/TOGGLE given BEEPs
 *
 * @param   beep - bit mask value of beeps to be turned ON/OFF/TOGGLE
 *          mode - BLINK, FLASH, TOGGLE, ON, OFF
 * @return  None
 ***************************************************************************************************/
uint8 HalBeepSet (uint8 beeps, uint8 mode)
{

#if (defined (BLINK_BEEPS)) && (HAL_BEEP == TRUE)
  uint8 beep;
  HalBeepControl_t *sts;

  switch (mode)
  {
    case HAL_BEEP_MODE_BLINK:
      /* Default blink, 1 time, D% duty cycle */
      HalBeepBlink (beeps, 1, HAL_BEEP_DEFAULT_DUTY_CYCLE, HAL_BEEP_DEFAULT_FLASH_TIME);
      break;

    case HAL_BEEP_MODE_FLASH:
      /* Default flash, N times, D% duty cycle */
      HalBeepBlink (beeps, HAL_BEEP_DEFAULT_FLASH_COUNT, HAL_BEEP_DEFAULT_DUTY_CYCLE, HAL_BEEP_DEFAULT_FLASH_TIME);
      break;

    case HAL_BEEP_MODE_ON:
    case HAL_BEEP_MODE_OFF:
    case HAL_BEEP_MODE_TOGGLE:

      beep = HAL_BEEP_1;
      beeps &= HAL_BEEP_ALL;
      sts = HalBeepStatusControl.HalBeepControlTable;

      while (beeps)
      {
        if (beeps & beep)
        {
          if (mode != HAL_BEEP_MODE_TOGGLE)
          {
            sts->mode = mode;  /* ON or OFF */
          }
          else
          {
            sts->mode ^= HAL_BEEP_MODE_ON;  /* Toggle */
          }
          HalBeepOnOff (beep, sts->mode);
          beeps ^= beep;
        }
        beep <<= 1;
        sts++;
      }
      break;

    default:
      break;
  }

#elif (HAL_BEEP == TRUE)
  BeepOnOff(beeps, mode);
#else
  // HAL BEEP is disabbeep, suppress unused argument warnings
  (void) beeps;
  (void) mode;
#endif /* BLINK_BEEPS && HAL_BEEP   */

  return ( HalBeepState );
}

/***************************************************************************************************
 * @fn      HalBeepBlink
 *
 * @brief   Blink the beeps
 *
 * @param   beeps       - bit mask value of beeps to be blinked
 *          numBlinks  - number of blinks
 *          percent    - the percentage in each period where the beep
 *                       will be on
 *          period     - length of each cycle in milliseconds
 *
 * @return  None
 ***************************************************************************************************/
void HalBeepBlink (uint8 beeps, uint8 numBlinks, uint8 percent, uint16 period)
{
#if (defined (BLINK_BEEPS)) && (HAL_BEEP == TRUE)
  uint8 beep;
  HalBeepControl_t *sts;

  if (beeps && percent && period)
  {
    if (percent < 100)
    {
      beep = HAL_BEEP_1;
      beeps &= HAL_BEEP_ALL;
      sts = HalBeepStatusControl.HalBeepControlTable;

      while (beeps)
      {
        if (beeps & beep)
        {
          /* Store the current state of the beep before going to blinking if not already blinking */
          if(sts->mode < HAL_BEEP_MODE_BLINK )
          	preBlinkState |= (beep & HalBeepState);

          sts->mode  = HAL_BEEP_MODE_OFF;                    /* Stop previous blink */
          sts->time  = period;                              /* Time for one on/off cycle */
          sts->onPct = percent;                             /* % of cycle BEEP is on */
          sts->left  = numBlinks;                           /* Number of blink cycles */
          if (!numBlinks) sts->mode |= HAL_BEEP_MODE_FLASH;  /* Continuous */
          sts->next = osal_GetSystemClock();                /* Start now */
          sts->mode |= HAL_BEEP_MODE_BLINK;                  /* Enable blinking */
          beeps ^= beep;
        }
        beep <<= 1;
        sts++;
      }
      // Cancel any overlapping timer for blink events
      osal_stop_timerEx(Hal_TaskID, HAL_BEEP_BLINK_EVENT);
      osal_set_event (Hal_TaskID, HAL_BEEP_BLINK_EVENT);
    }
    else
    {
      HalBeepSet (beeps, HAL_BEEP_MODE_ON);                    /* >= 100%, turn on */
    }
  }
  else
  {
    HalBeepSet (beeps, HAL_BEEP_MODE_OFF);                     /* No on time, turn off */
  }
#elif (HAL_BEEP == TRUE)
  percent = (beeps & HalBeepState) ? HAL_BEEP_MODE_OFF : HAL_BEEP_MODE_ON;
  HalBeepOnOff (beeps, percent);                              /* Toggle */
#else
  // HAL BEEP is disabbeep, suppress unused argument warnings
  (void) beeps;
  (void) numBlinks;
  (void) percent;
  (void) period;
#endif /* BLINK_BEEPS && HAL_BEEP */
}

#if (HAL_BEEP == TRUE)
/***************************************************************************************************
 * @fn      HalBeepUpdate
 *
 * @brief   Update beeps to work with blink
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalBeepUpdate (void)
{
  uint8 beep;
  uint8 pct;
  uint8 beeps;
  HalBeepControl_t *sts;
  uint32 time;
  uint16 next;
  uint16 wait;

  next = 0;
  beep  = HAL_BEEP_1;
  beeps = HAL_BEEP_ALL;
  sts = HalBeepStatusControl.HalBeepControlTable;

  /* Check if sleep is active or not */
  if (!HalBeepStatusControl.sleepActive)
  {
    while (beeps)
    {
      if (beeps & beep)
      {
        if (sts->mode & HAL_BEEP_MODE_BLINK)
        {
          time = osal_GetSystemClock();
          if (time >= sts->next)
          {
            if (sts->mode & HAL_BEEP_MODE_ON)
            {
              pct = 100 - sts->onPct;               /* Percentage of cycle for off */
              sts->mode &= ~HAL_BEEP_MODE_ON;        /* Say it's not on */
              HalBeepOnOff (beep, HAL_BEEP_MODE_OFF);  /* Turn it off */

              if ( !(sts->mode & HAL_BEEP_MODE_FLASH) )
              {
                sts->left--;                         // Not continuous, reduce count
              }
            }
            else if ( !(sts->left) && !(sts->mode & HAL_BEEP_MODE_FLASH) )
            {
              sts->mode ^= HAL_BEEP_MODE_BLINK;       // No more blinks
            }
            else
            {
              pct = sts->onPct;                      // Percentage of cycle for on
              sts->mode |= HAL_BEEP_MODE_ON;          // Say it's on
              HalBeepOnOff( beep, HAL_BEEP_MODE_ON );   // Turn it on
            }
            if (sts->mode & HAL_BEEP_MODE_BLINK)
            {
              wait = (((uint32)pct * (uint32)sts->time) / 100);
              sts->next = time + wait;
            }
            else
            {
              /* no more blink, no more wait */
              wait = 0;
              /* After blinking, set the BEEP back to the state before it blinks */
              HalBeepSet (beep, ((preBlinkState & beep)!=0)?HAL_BEEP_MODE_ON:HAL_BEEP_MODE_OFF);
              /* Clear the saved bit */
              preBlinkState &= (beep ^ 0xFF);
            }
          }
          else
          {
            wait = sts->next - time;  /* Time left */
          }

          if (!next || ( wait && (wait < next) ))
          {
            next = wait;
          }
        }
        beeps ^= beep;
      }
      beep <<= 1;
      sts++;
    }

    if (next)
    {
      osal_start_timerEx(Hal_TaskID, HAL_BEEP_BLINK_EVENT, next);   /* Schedule event */
    }
  }
}

/***************************************************************************************************
 * @fn      HalBeepOnOff
 *
 * @brief   Turns specified BEEP ON or OFF
 *
 * @param   beeps - BEEP bit mask
 *          mode - BEEP_ON,BEEP_OFF,
 *
 * @return  none
 ***************************************************************************************************/
void HalBeepOnOff (uint8 beeps, uint8 mode)
{
  if (beeps & HAL_BEEP_1)
  {
    if (mode == HAL_BEEP_MODE_ON)
    {
      HAL_TURN_ON_BEEP();
    }
    else
    {
      HAL_TURN_OFF_BEEP();
    }
  }

  /* Remember current state */
  if (mode)
  {
    HalBeepState |= beeps;
  }
  else
  {
    HalBeepState &= (beeps ^ 0xFF);
  }
}
#endif /* HAL_BEEP */

/***************************************************************************************************
 * @fn      HalGetBeepState
 *
 * @brief   Dim BEEP2 - Dim (set level) of BEEP2
 *
 * @param   none
 *
 * @return  beep state
 ***************************************************************************************************/
uint8 HalBeepGetState ()
{
#if (HAL_BEEP == TRUE)
  return HalBeepState;
#else
  return 0;
#endif
}

/***************************************************************************************************
 * @fn      HalBeepEnterSleep
 *
 * @brief   Store current BEEPs state before sleep
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalBeepEnterSleep( void )
{
#ifdef BLINK_BEEPS
  /* Sleep ON */
  HalBeepStatusControl.sleepActive = TRUE;
#endif /* BLINK_BEEPS */

#if (HAL_BEEP == TRUE)
  /* Save the state of each beep */
  HalSleepBeepState = 0;
  HalSleepBeepState |= HAL_STATE_BEEP();

  /* TURN OFF all BEEPs to save power */
  HalBeepOnOff (HAL_BEEP_ALL, HAL_BEEP_MODE_OFF);
#endif /* HAL_BEEP */

}

/***************************************************************************************************
 * @fn      HalBeepExitSleep
 *
 * @brief   Restore current BEEPs state after sleep
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalBeepExitSleep( void )
{
#if (HAL_BEEP == TRUE)
  /* Load back the saved state */
  HalBeepOnOff(HalSleepBeepState, HAL_BEEP_MODE_ON);

  /* Restart - This takes care BLINKING BEEPS */
  HalBeepUpdate();
#endif /* HAL_BEEP */

#ifdef BLINK_BEEPS
  /* Sleep OFF */
  HalBeepStatusControl.sleepActive = FALSE;
#endif /* BLINK_BEEPS */
}

/***************************************************************************************************
***************************************************************************************************/




