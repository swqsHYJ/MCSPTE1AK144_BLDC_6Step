/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     actuate_s32k.c
*
* @date     March-28-2017
*
* @brief    Header file for actuator module
*
*******************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include "actuate_s32k.h"

/******************************************************************************
| External declarations
-----------------------------------------------------------------------------*/

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Global variable definitions   (scope: module-exported)
-----------------------------------------------------------------------------*/

  /* FTM3 channel output mask control */
   const uint8_t ui8FTM3OutmaskVal[2][8] =
  {
      /* Clockwise rotation direction */
      {
          0x34,   /* sector 0 */
          0x1C,   /* sector 1 */
          0x13,   /* sector 2 */
          0x31,   /* sector 3 */
          0x0D,   /* sector 4 */
          0x07,   /* sector 5 */
          0x05,   /* alignment vector */
          0x3F    /* PWM off */
      },
      /* Counterclockwise rotation direction */
      {
          0x31,   /* sector 3 */
          0x13,   /* sector 2 */
          0x1C,   /* sector 1 */
          0x34,   /* sector 0 */
          0x07,   /* sector 5 */
          0x0D,   /* sector 4 */
          0x05,   /* alignment vector */
          0x3F    /* PWM off */
      }
  };

  /* FTM3 channel software output control */
   const uint16_t ui16FTM3SwOctrlVal[2][8] =
  {
      /* Clockwise rotation direction */
      {
          0x0808, /* sector 0 */
          0x2020, /* sector 1 */
          0x2020, /* sector 2 */
          0x0202, /* sector 3 */
          0x0202, /* sector 4 */
          0x0808, /* sector 5 */
          0x0A0A, /* alignment vector */
          0x0000  /* PWM off */
      },
      /* Counterclockwise rotation direction */
      {
          0x0202, /* sector 3 */
          0x2020, /* sector 2 */
          0x2020, /* sector 1 */
          0x0808, /* sector 0 */
          0x0808, /* sector 5 */
          0x0202, /* sector 4 */
          0x0A0A, /* alignment vector */
          0x0000  /* PWM off */
      }
  };

/******************************************************************************
| Global variable definitions   (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function prototypes           (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function implementations      (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function implementations      (scope: module-exported)
-----------------------------------------------------------------------------*/

/**************************************************************************//*!
@brief Unmask PWM output and set 0% dytucyle

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_EnableOutput(tBool ftmInputTrig)
{
	uint16_t duty_cycle;

    // Enable PWM
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x0, false);

	// Apply 0% duty cycle
	duty_cycle = 0;
	
	// Update duty cycle
	ACTUATE_SetDutycycle(duty_cycle, ftmInputTrig);

	return 1;
}


/**************************************************************************//*!
@brief Mask PWM output and set 0% dytucyle

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_DisableOutput(tBool ftmInputTrig)
{
	uint16_t duty_cycle;

    /* Disable PWM */
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x3F, false);

	// Apply 0% duty cycle
	duty_cycle = 0;

	// Update duty cycle
	ACTUATE_SetDutycycle(duty_cycle, ftmInputTrig);

	return 1;
}

/**************************************************************************//*!
@brief Set PWM dytycyle, the dutycycle will by updated on next reload event

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_SetDutycycle(tFloat dutyCycle, tBool ftmInputTrig)
{
	tBool 				statePwm 	= true;
	uint16_t   			dutyTicks;
	const uint8_t 		channels[6] = {0, 1, 2, 3, 4, 5};

    /* Duty cycle in clock ticks format */
	dutyTicks = (uint16_t)MLIB_Mul(MLIB_Div(dutyCycle, 100.0F), HALF_PWM_MODULO);

	/* Set duty cycle for all PWM channels */
	uint16_t pwms[ 6] = {dutyTicks, 0, dutyTicks, 0, dutyTicks, 0};

    /* Clear FTM3SYNCBIT to prepare HW trigger for FTM3 */
    SIM->FTMOPT1 &= ~(SIM_FTMOPT1_FTM3SYNCBIT_MASK & (ftmInputTrig << SIM_FTMOPT1_FTM3SYNCBIT_SHIFT));

    /* Update PWM duty cycle */
	FTM_DRV_FastUpdatePwmChannels(INST_FLEXTIMER_PWM3, 6, channels, pwms, false);

    /* Set FTM3SYNCBIT to trigger and update FTM3 registers */
    SIM->FTMOPT1 |= (SIM_FTMOPT1_FTM3SYNCBIT_MASK & (ftmInputTrig << SIM_FTMOPT1_FTM3SYNCBIT_SHIFT));

	statePwm = false;

	return(statePwm);
}

/**************************************************************************//*!
@brief Set PWM Mask, Mask will by updated on next reload event

@param[in,out]

@return
******************************************************************************/
tBool ACTUATE_SetPwmMask(uint8_t ui8OutMask, uint16_t ui16SwCtrl, tBool ftmInputTrig)
{
	/* Clear FTM3SYNCBIT to prepare HW trigger for FTM3 */
    SIM->FTMOPT1 &= ~(SIM_FTMOPT1_FTM3SYNCBIT_MASK & (ftmInputTrig << SIM_FTMOPT1_FTM3SYNCBIT_SHIFT));

    // Apply Mask
    //FTM3->OUTMASK = ui8OutMask;
    FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, ui8OutMask, false);
    FTM3->SWOCTRL = ui16SwCtrl;

    /* Set FTM3SYNCBIT to trigger and update FTM3 registers */
    SIM->FTMOPT1 |= (SIM_FTMOPT1_FTM3SYNCBIT_MASK & (ftmInputTrig << SIM_FTMOPT1_FTM3SYNCBIT_SHIFT));

	return 1;
}

/* End of file */
