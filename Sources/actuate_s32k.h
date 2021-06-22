/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     actuate_s32k.h
*
* @date     March-28-2017
*
* @brief    Header file for actuator module
*
*******************************************************************************/
#ifndef _ACTUATE_S32K_H_
#define _ACTUATE_S32K_H_

#include "peripherals_config.h"
#include "mlib.h"

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/
typedef enum
{
	HW_INPUT_TRIG0,			/* FTM3 is triggered automatically by FTM0 init trigger through FTM3 signal input TRIG0 */
	HW_INPUT_TRIG1 			/* FTM3 is triggered manually by FTM3 SYNC bit through FTM3 signal input TRIG1 */
} ftm_hw_trigger_t;

/******************************************************************************
| Exported Variables
-----------------------------------------------------------------------------*/
extern const uint8_t ui8FTM3OutmaskVal[2][8];	// FTM3 channel output mask control
extern const uint16_t ui16FTM3SwOctrlVal[2][8];	// FTM3 channel software output control

/******************************************************************************
| Exported function prototypes
-----------------------------------------------------------------------------*/
extern tBool 	ACTUATE_EnableOutput(tBool ftmInputTrig);
extern tBool 	ACTUATE_DisableOutput(tBool ftmInputTrig);
extern tBool 	ACTUATE_SetDutycycle(tFloat dutyCycle, tBool ftmInputTrig);
extern tBool 	ACTUATE_SetPwmMask(uint8_t ui8OutMask, uint16_t ui16SwCtrl, tBool ftmInputTrig);

/******************************************************************************
| Inline functions
-----------------------------------------------------------------------------*/

#endif /* _ACTUATES_S32K_H_ */
