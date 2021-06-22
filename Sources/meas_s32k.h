/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     meas_s32k.h
*
* @date     March-28-2017
*
* @brief    Header file for measurement module
*
*******************************************************************************/
#ifndef _MEAS_S32K_H_
#define _MEAS_S32K_H_

#include "peripherals_config.h"
#include "BLDC_appconfig.h"
#include "mlib.h"

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Exported Variables
-----------------------------------------------------------------------------*/
extern const uint8_t bemfPhaseList[2][6];

/******************************************************************************
| Exported function prototypes
-----------------------------------------------------------------------------*/
extern tBool MEAS_GetBEMFVoltage(tFloat *getBEMFVoltage);
extern tBool MEAS_GetDCBVoltage(tFloat *getDCBVoltage);
extern tBool MEAS_GetDCBCurrent(tFloat *getDCBCurrent);
extern tBool MEAS_SetBEMFPhase(uint8_t bemfPhase);

/******************************************************************************
| Inline functions
-----------------------------------------------------------------------------*/

#endif /* _MEAS_S32K_H_ */
