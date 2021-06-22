/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     meas_s32k.c
*
* @date     March-28-2017
*
* @brief    Header file for measurement module
*
*******************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include "meas_s32k.h"

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

  /* ADC0 input channel list sequence in 6-step control */
  const uint8_t bemfPhaseList[2][6] =
  {
	  /* Clockwise rotation direction */
      {
		  ADC0_BEMFC,	/* sector 0 */
		  ADC0_BEMFB,	/* sector 1 */
		  ADC0_BEMFA,	/* sector 2 */
		  ADC0_BEMFC,	/* sector 3 */
		  ADC0_BEMFB,	/* sector 4 */
		  ADC0_BEMFA	/* sector 5 */
      },
	  /* Counterclockwise rotation direction */
      {
	      ADC0_BEMFC,	/* sector 3 */
	      ADC0_BEMFA,	/* sector 2 */
	      ADC0_BEMFB,	/* sector 1 */
	      ADC0_BEMFC,	/* sector 0 */
	      ADC0_BEMFA,	/* sector 5 */
	      ADC0_BEMFB,	/* sector 4 */
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

/*******************************************************************************
*
* Function: 	tBool MEAS_GetBEMFVoltage(tFloat *BEMFVoltage)
*
* Description:  This function performs voltage measurement of the disconnected phase.
* 				Conversion complete interrupt is disabled.
*
* Param[in,out]: *BEMFVoltage - pointer to a variable - voltage
* 								of the disconnected phase
*
*
* @return     	# true  - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.
*
*******************************************************************************/
tBool MEAS_GetBEMFVoltage(tFloat *BEMFVoltage)
{
	uint16_t 			adcResult;

	ADC_DRV_GetChanResult(0, 0, &adcResult);
	*BEMFVoltage = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(adcResult & 0x00000FFF), (tFloat)0x00000FFF)), U_DCB_MAX);

	return 1;
}

/*******************************************************************************
*
* Function: 	tBool MEAS_GetDCBVoltage(tFloat *DCBVoltage)
*
* Description:  This function performs DC bus voltage measurement.
* 				Conversion complete interrupt is enabled.
*
* Param[in,out]: *DCBVoltage - pointer to a variable - DC bus voltage
*
*
* @return     	# true  - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.
*
*******************************************************************************/
tBool MEAS_GetDCBVoltage(tFloat *DCBVoltage)
{
	uint16_t 			adcResult;

	ADC_DRV_GetChanResult(1, 1, &adcResult);
	*DCBVoltage = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(adcResult & 0x00000FFF), (tFloat)0x00000FFF)), U_DCB_MAX);

	return 1;
}

/*******************************************************************************
*
* Function: 	tBool MEAS_GetDCBCurrent(tFloat *getDCBCurrent)
*
* Description:  This function performs DC bus current measurement.
* 				Conversion complete interrupt is disabled.
*
* Param[in,out]: *getDCBCurrent - pointer to a variable - DC bus current
*
*
* @return     	# true  - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.
*
*******************************************************************************/
tBool MEAS_GetDCBCurrent(tFloat *getDCBCurrent)
{
	uint16_t 			adcResult;

	ADC_DRV_GetChanResult(1, 0, &adcResult);

	*getDCBCurrent = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(adcResult & 0x00000FFF), (tFloat)0x00000FFF)), I_MAX);

	return 1;
}


/*******************************************************************************
*
* Function: 	void ADC_EnableTrigSeq(uint8_t *ui8Channel)
*
* Description:  This function selects ADC0 input channel to measure voltage
* 				of the disconnected phase based on the actual sector and
* 				rotation direction.
*
* Param[in]:    bemfPhase - ADC0 input channel
*
* * @return     # true  - when ADC channel configuration ended successfully
            	# false - when when ADC channel configuration is ongoing,
            			  or error occurred.
*
*******************************************************************************/
tBool MEAS_SetBEMFPhase(uint8_t bemfPhase)
{
	adc_chan_config_t 	adc0Ch0;

	adc0Ch0.channel = bemfPhase;
	adc0Ch0.interruptEnable = false;

	ADC_DRV_ConfigChan(0, 0, &adc0Ch0);

	return 1;
}

/* End of file */
