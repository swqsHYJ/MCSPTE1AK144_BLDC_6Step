/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K14x
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"
#include "pin_mux.h"
#include "clockMan1.h"
#include "pwrMan1.h"
#include "trgmux1.h"
#include "lpuart1.h"
#include "dmaController1.h"
#include "lpspiCom1.h"
#include "flexTimer_pwm3.h"
#include "flexTimer_ic1.h"
#include "pdb0.h"
#include "pdb1.h"
#include "adConv0.h"
#include "adConv1.h"

  volatile int exit_code = 0;

/* User includes (#include below this line is not maintained by Processor Expert) */

#include "peripherals_config.h"
#include "gd3000_init.h"
#include "freemaster.h"
#include "freemaster_tsa.h"
#include "BLDC_appconfig.h"
#include "actuate_s32k.h"
#include "meas_s32k.h"
#include "aml/common_aml.h"
#include "aml/gpio_aml.h"
#include "tpp/tpp.h"
#include "state_machine.h"
#include "motor_structure.h"
#include "hall_sensor.h"
#include "BLDC_appfreemaster_TSA.h"

  /*******************************************************************************
  * Local variables
  *******************************************************************************/
  gd3000Status_t    gd3000Status;	// GD3000 status variables
  tpp_drv_config_t  tppDrvConfig;	// GD3000 configuration structure

  /*------------------------------------
   * Application State and Control Variables
   * ----------------------------------*/
  uint8_t    		appState = APP_INIT;
  uint8_t    		rotationDir = ROTATION_DIR_CW;
  uint8_t 			appSwitchState = 0, faultSwitchClear;
  int16_t 			switchCounter[2], switchOffCounter;
  uint32_t   		ledCounter;
  tDriveStatus 		driveStatus;
  tFaultStatus 	 	faultStatus, faultStatusLatched;

  /*------------------------------------
   *  Measurement/Actuate Variables
   * ----------------------------------*/
  tADCresults  		ADCResults;
  tFloat   			duty_cycle;

  /*------------------------------------
   * BEMF Zero Cross Detection and SixStep Commutation control Variables
   * ----------------------------------*/
  tFloat 	bemfVoltage, bemfVoltageOld;
  tFloat 	timeBackEmf, timeOldBackEmf, timeZCToff;
  uint8_t  	NextCmtSector, ActualCmtSector;
  uint16_t 	timeZC, lastTimeZC;
  uint16_t 	ftm_mod_old;
  uint16_t 	actualPeriodZC;
  uint32_t 	period6ZC, period6Hall;
  uint16_t 	periodZC[6], periodHall[6];
  uint16_t 	advanceAngle;
  uint16_t 	NextCmtPeriod;
  uint16_t 	pdb_delay;
  uint16_t 	debugTmin, debugTmax, periodZcAvrg, stallCheckCounter;
  uint16_t 	debugTminLim, debugTmaxLim;
  uint16_t 	calibTimer, alignmentTimer, startCMTcounter, freewheelTimer;
  uint8_t 	ftm2Ch1XorSignal;
  tFloat 	delta;
  tSensorHall 	SensorHall;
  tBool 		sensorType;

  /*------------------------------------
   *  Speed and Current Control Loop Variables
   * ----------------------------------*/
  tFloat 	torqueErr;
  tFloat 	speedErr;
  tFloat 	requiredSpeed = N_MIN;
  tFloat 	requiredSpeedRamp;
  tFloat 	actualSpeed = 0.0F;
  tFloat 	speedPIOut, currentPIOut;
  tFloat 	u_dc_bus_filt, torque_filt;

  GFLIB_CONTROLLER_PIAW_P_T_FLT 	speedPIPrms, currentPIPrms;
  GFLIB_RAMP_T_FLT 				speedRampPrms;
  GDFLIB_FILTER_MA_T_FLT 		Udcb_filt, Idcb_filt, Idcb_calib;

  /*------------------------------------
   * MCAT - Referenced Variables
   * ----------------------------------*/

  /* MCAT Parameters tab */
  tFloat 	mcat_alignVoltage 			= ALIGN_VOLTAGE;
  uint16_t 	mcat_alignDuration 			= ALIGN_DURATION;

  /* MCAT Sensorless tab */
  tFloat 		mcat_NMin 				= N_MIN;
  uint16_t 		mcat_FreewheelTLong 	= FREEWHEEL_T_LONG;
  uint16_t 		mcat_FreewheelTShort	= FREEWHEEL_T_SHORT;
  uint8_t 		mcat_startCmtCnt 		= STARTUP_CMT_CNT;
  uint16_t 		mcat_startCmtPer 		= STARTUP_CMT_PER;
  tFloat 		mcat_startCmtAcceler 	= START_CMT_ACCELER;
  tFloat 		mcat_cmtTOff			= CMT_T_OFF;
  tFloat 		mcat_integThr 			= INTEG_TRH;

  /*------------------------------------
   * FreeMASTER constants
   * ----------------------------------*/
  tU32 fm_voltage;
  tU32 fm_current;
  tU32 fm_speed;

  uint16_t torqueFilter;
  uint8_t pdb0Counter;
  uint8_t pdb1Counter;

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
  /* Write your local variable definition here */

	// MCU peripherals initialization
 	McuClockConfig();
	McuCacheConfig();
	McuPowerConfig();
	McuIntConfig();
	McuTrigmuxConfig();
	McuPinsConfig();
	McuLpuartConfig();
	McuLpitConfig();
	McuAdcConfig();
	McuPdbConfig();
	McuFtmConfig();

    // FreeMASTER initialization
	FMSTR_Init();

    // MC34GD3000 initialization
    GD3000_Init();

    /* Set default current measurement bias offset
     * (actual offset will measured in the Calibration state function) */
	ADCResults.DCBIOffset = (I_MAX/2.0F);
	ADCResults.DCBVVoltage = 12.0F;

	// Application starts by FTM3 initialization trigger
	FTM3->EXTTRIG = FTM_EXTTRIG_INITTRIGEN_MASK;

    fm_voltage = FM_U_DCB_SCALE;
    fm_current = FM_I_SCALE;
    fm_speed = FM_N_SCALE;

	/* Initialize DC bus voltage moving average filter  */
    GDFLIB_FilterMAInit_FLT(&Udcb_filt);
    Udcb_filt.fltLambda = DCBV_FILTER_MA_LAMBDA;

    /* Initialize DC bus current moving average filter */
    GDFLIB_FilterMAInit_FLT(&Idcb_filt);
    Idcb_filt.fltLambda = TORQUE_LOOP_MAF;

    /* Initialize moving average filter for DC bus current offset calibration */
    GDFLIB_FilterMAInit_FLT(&Idcb_calib);
    Idcb_calib.fltLambda = CALIB_FILTER_MA_LAMBDA;

    /* Speed PI controller initialization */
    speedPIPrms.fltPropGain = SPEED_LOOP_KP_GAIN;
    speedPIPrms.fltIntegGain = SPEED_LOOP_KI_GAIN;
    speedPIPrms.fltUpperLimit = CTRL_LOOP_LIM_HIGH;
    speedPIPrms.fltLowerLimit = CTRL_LOOP_LIM_LOW;

    /* Current PI controller initialization */
    currentPIPrms.fltPropGain = TORQUE_LOOP_KP_GAIN;
    currentPIPrms.fltIntegGain = TORQUE_LOOP_KI_GAIN;
    currentPIPrms.fltUpperLimit = CTRL_LOOP_LIM_HIGH;;
    currentPIPrms.fltLowerLimit = CTRL_LOOP_LIM_LOW;

    /* SPeed ramp initialization */
    speedRampPrms.fltRampUp = SPEED_LOOP_RAMP_UP;
    speedRampPrms.fltRampDown = SPEED_LOOP_RAMP_DOWN;

    /* Hall initialization */
    SensorHall.Sector 	= 0;
    ftm2Ch1XorSignal  	= 0;

    /* StallCheck initialization */
    debugTmin 			= 0;
	debugTmax			= 0;
	periodZcAvrg		= 0;
	stallCheckCounter	= 0;

	/* MCAT Parameters tab */
	mcat_alignVoltage 		= ALIGN_VOLTAGE;
	mcat_alignDuration 		= ALIGN_DURATION;

	/* MCAT Sensorless tab */
	mcat_NMin 				= N_MIN;
	mcat_FreewheelTLong 	= FREEWHEEL_T_LONG;
	mcat_FreewheelTShort	= FREEWHEEL_T_SHORT;
	mcat_startCmtCnt 		= STARTUP_CMT_CNT;
	mcat_startCmtPer 		= STARTUP_CMT_PER;
	mcat_startCmtAcceler 	= START_CMT_ACCELER;
	mcat_cmtTOff			= CMT_T_OFF;
	mcat_integThr 			= INTEG_TRH;

	/* Selected sensor type */
	sensorType = HALL_SENSOR;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */
    for(;;)
    {
    	/* FreeMASTER */
    	FMSTR_Poll();

        /* Call BLDC application state machine function */
    	AppStateMachine[appState]();
    	AppStateLed[appState]();

        /* Check power stage faults */
        CheckFaults();

        // Rotor Stall detection
    	if(driveStatus.B.StallCheckReq == 1)
    	{
    		StallCheck();
    	}

    	// Read GD3000 Status register 0, if there is GD3000 interrupt
    	if(gd3000Status.B.gd3000IntFlag)
    	{
    		gd3000Status.B.gd3000IntFlag = false;
    		TPP_GetStatusRegister(&tppDrvConfig, tppSR0_deviceEvents,
    							  &(tppDrvConfig.deviceConfig.statusRegister[0U]));
    	}
    }

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/

/*******************************************************************************
*
* Function: 	TPP_InitializeOutputs(void)
*
* Description:  This function initialize output of the MC34GD3000 MOSFET Pre-driver
*
*******************************************************************************/
void TPP_InitializeOutputs(void)
{

}

/*******************************************************************************
* Interrupt Routines
*******************************************************************************/
/*******************************************************************************
*
* Function: 	PORTE_IRQHandler(void)
*
* Description:  PORTE Interrupt Service Routine to detect MC34GD3000 fault
*
*******************************************************************************/
void PORTE_IRQHandler(void)
{
	if (PINS_DRV_GetPinIntSel(PORTE, 10u) && gd3000Status.B.gd3000InitDone)
	{
		gd3000Status.B.gd3000IntFlag = true;
	}

	PINS_DRV_ClearPinIntFlagCmd(PORTE, 10u);

	gd3000Status.B.gd3000InitDone = true;
}

/*******************************************************************************
*
* Function: 	PDB0_IRQHandler(void)
*
* Description:  PDB0 Interrupt Service Routine
*
*******************************************************************************/
void PDB0_IRQHandler(void)
{
	pdb0Counter++;

	// Disable PDB0
	PDB_DRV_Disable(INST_PDB0);

	/* Clear PDB0 sequence errors */
	PDB_DRV_ClearAdcPreTriggerSeqErrFlags(INST_PDB0, 0, 0xFF);

	// Enable PDB0
	PDB_DRV_Enable(INST_PDB0);
}

/*******************************************************************************
*
* Function: 	PDB1_IRQHandler(void)
*
* Description:  PDB1 Interrupt Service Routine
*
*******************************************************************************/
void PDB1_IRQHandler(void)
{
	pdb1Counter++;

	// Disable PDB1
	PDB_DRV_Disable(INST_PDB1);

	// Clear PDB1 sequence errors
	PDB_DRV_ClearAdcPreTriggerSeqErrFlags(INST_PDB1, 0, 0xFF);

	// Enable PDB1
	PDB_DRV_Enable(INST_PDB1);
}

#if HALL_SENSOR
/*******************************************************************************
*
* Function: 	FTM2_Callback()
*
* Description:  Callback for Hall sensor signal processing. In sensorbased mode,
* 				SixStep Commutation Control is ensured by Hall sensor pattern
* 				and time between two consecutive commutations is measured by FlexTimer2.
*
*******************************************************************************/
void FTM2_Callback()
{
	// FTM2 XOR visualization in FreeMASTER
	ftm2Ch1XorSignal ^= 1;

	// Reset FTM2 counter every input capture event
	FTM2->CNT = 0;

	// Get commutation sector based on the Hall logic
	HALL_GetSector(&SensorHall);

	// Commutation period is measured by input capture mode of the FTM1_CH2 channel
	SensorHall.Period[SensorHall.Sector] = FTM_DRV_GetInputCaptureMeasurement(INST_FLEXTIMER_IC1, 1U);

	if (driveStatus.B.EnableCMT)
	{
		/* Prepare PWM settings for the next commutation sector */
		ACTUATE_SetPwmMask(ui8FTM3OutmaskVal[rotationDir][SensorHall.Sector],
						  ui16FTM3SwOctrlVal[rotationDir][SensorHall.Sector], HW_INPUT_TRIG0);
	}

	driveStatus.B.StallCheckReq = 1;
	driveStatus.B.HallEvent = 1;
}

#else
/*******************************************************************************
*
* Function: 	FTM0_Ovf_Reload_IRQHandler()
*
* Description:  Commutation interrupt service routine. In sensorless mode,
* 				SixStep Commutation Control is ensured by zero-cross detection algorithm
* 				and time between two consecutive commutations is measured by FlexTimer0.
*
*******************************************************************************/
void FTM0_Ovf_Reload_IRQHandler()
{
	ftm_mod_old = FTM_DRV_GetMod(FTM0);

    if(driveStatus.B.Sensorless == 1)
    {
        if(driveStatus.B.NewZC == 0)
        {
            /* In the middle between two commutations */
        	timeZC = actualPeriodZC >> 1;
        }

        // Update commutation period
    	FTM_DRV_CounterStop(INST_FLEXTIMER_MC0);
    	FTM_DRV_SetModuloCounterValue(INST_FLEXTIMER_MC0, (actualPeriodZC << 1), false);
    	FTM_DRV_CounterStart(INST_FLEXTIMER_MC0);

        timeZCToff = MLIB_Mul(((tFloat)actualPeriodZC), MLIB_Mul(mcat_cmtTOff, 0.01F));
        driveStatus.B.StallCheckReq = 1;
    }
    else
    {
    	// Update commutation period
    	FTM_DRV_CounterStop(INST_FLEXTIMER_MC0);
    	FTM_DRV_SetModuloCounterValue(INST_FLEXTIMER_MC0, NextCmtPeriod, false);
    	FTM_DRV_CounterStart(INST_FLEXTIMER_MC0);
    }

    ActualCmtSector = NextCmtSector;
    if(driveStatus.B.EnableCMT)
    {
    	// Measure back-EMF voltage of the disconnected stator phase
    	MEAS_SetBEMFPhase(bemfPhaseList[rotationDir][ActualCmtSector]);

        NextCmtSector++;
        if(NextCmtSector > 5)
        {
            NextCmtSector = 0;
        }

        /* Prepare PWM settings for the next commutation sector */
        ACTUATE_SetPwmMask(ui8FTM3OutmaskVal[rotationDir][NextCmtSector],
        				   ui16FTM3SwOctrlVal[rotationDir][NextCmtSector], HW_INPUT_TRIG0);
    }

    driveStatus.B.NewZC = 0;
    driveStatus.B.AdcSaved = 0;
    driveStatus.B.AfterCMT = 1;

    FTM_DRV_ClearStatusFlags(INST_FLEXTIMER_MC0, FTM_TIME_OVER_FLOW_FLAG);
}
#endif

/*******************************************************************************
*
* Function: 	ADC1_IRQHandler()
*
* Description:  ADC1 interrupt service routine
*
*******************************************************************************/
void ADC1_IRQHandler()
{
	// Voltage measurement of the disconnected phase
	MEAS_GetBEMFVoltage(&ADCResults.BEMFVoltage);
	// DC Bus current raw value measurement
	MEAS_GetDCBCurrent(&ADCResults.DCBIVoltageRaw);
	// DC Bus voltage measurement
	MEAS_GetDCBVoltage(&ADCResults.DCBVVoltage);
	// Hall counter measurement
	SensorHall.Ftm2HallCnt = FTM2->CNT;

	// Real DC Bus current = Raw value - DC bus current offset
	ADCResults.DCBIVoltage = MLIB_Sub(ADCResults.DCBIVoltageRaw, ADCResults.DCBIOffset);

	// bemfVoltage = Voltage of the disconnected phase - DC Bus voltage/2
	bemfVoltage = MLIB_Sub(ADCResults.BEMFVoltage, MLIB_Div(u_dc_bus_filt, 2.0F));

    u_dc_bus_filt = GDFLIB_FilterMA(ADCResults.DCBVVoltage, &Udcb_filt);

    if(duty_cycle > DC_THRESHOLD)
    {
        torque_filt = GDFLIB_FilterMA(ADCResults.DCBIVoltage, &Idcb_filt);
    }
    else
    {
        // Ignore DC bus current measurement at low duty cycles
        torque_filt = GDFLIB_FilterMA(0, &Idcb_filt);
    }

// ZC detection algorithm is ignored in Sensorbased mode
#if (!HALL_SENSOR)

	timeOldBackEmf = timeBackEmf;
	timeBackEmf = FTM_DRV_CounterRead(INST_FLEXTIMER_MC0);

		if(driveStatus.B.AfterCMT == 1)
		{
			if(timeBackEmf > timeZCToff)
			{
				driveStatus.B.AfterCMT = 0;
			}
		}

		if((driveStatus.B.AfterCMT == 0) && (driveStatus.B.NewZC == 0) && (driveStatus.B.Sensorless == 1))
		{

			/* If the BEMF voltage is falling, invert BEMF voltage value */
			if((ActualCmtSector & 0x01) == 0)
			{
				bemfVoltage = -bemfVoltage;
			}

			/* Rising BEMF zero-crossing detection */
			if(bemfVoltage >= 0)
			{
				/* Rising interpolation */
				delta = bemfVoltage - bemfVoltageOld;
				if((driveStatus.B.AdcSaved == 1) && (delta > bemfVoltage))
				{
					timeBackEmf -= MLIB_Mul(MLIB_Div(bemfVoltage, delta), MLIB_Sub(timeBackEmf, timeOldBackEmf));
				}
				else
				{
					timeBackEmf -= (MLIB_Div(MLIB_Sub(timeBackEmf, timeOldBackEmf), 2));
				}

				lastTimeZC = timeZC;
				timeZC = (uint16_t)timeBackEmf;

				// periodZC = (timeZC - lasTimeZC) + ftm_mod_old(no timer reset)
				periodZC[ActualCmtSector] = (ftm_mod_old - lastTimeZC) + timeZC;
				// Average of the previous and current ZC period
				actualPeriodZC = (actualPeriodZC + periodZC[ActualCmtSector]) >> 1;
				// advancedAngle(0.3815) = 0.5 * Advanced Angle(0.763)
				NextCmtPeriod = MLIB_Mul_F16(actualPeriodZC, advanceAngle);

				// Update commutation period -> FTM0_MOD = timeZC + nextCmtPeriod
				FTM_DRV_CounterStop(INST_FLEXTIMER_MC0);
				FTM_DRV_SetModuloCounterValue(INST_FLEXTIMER_MC0, timeZC + NextCmtPeriod, true);
				FTM_DRV_CounterStart(INST_FLEXTIMER_MC0);

				driveStatus.B.NewZC = 1;
			}

			bemfVoltageOld = bemfVoltage;   /* Save actual BEMF voltage (for ADC
											   samples interpolation) */
			driveStatus.B.AdcSaved = 1;
		}
#endif

	// Timer for Rotor alignment
	if(driveStatus.B.Alignment)
	{
		if(alignmentTimer > 0)
		{
			alignmentTimer--;
		}
			driveStatus.B.AdcSaved = 0;
	}

	// Calibration timer for DC bus current offset measurement
	if(driveStatus.B.Calib)
	{
	   	calibTimer--;
	}

    // Application variables record
    FMSTR_Recorder();

	//PTD->PCOR |= 1<<2;
}

/*******************************************************************************
*
* Function: 	LPIT0_Ch0_IRQHandler()
*
* Description:  LPIT channel 0 time-out interrupt service routine
* 				(speed and current control loop)
*
*******************************************************************************/
void LPIT0_Ch0_IRQHandler()
{
	uint8_t i;

	PTD->PSOR |= 1<<2;

    /* Speed control */
	#if HALL_SENSOR
    	period6ZC = SensorHall.Period[0];
    	for(i=1;i<6;i++)
    	{
    		period6ZC += SensorHall.Period[i];
    	}
	#else
    	period6ZC = periodZC[0];
    	for(i=1;i<6;i++)
		{
			period6ZC += periodZC[i];
		}
	#endif


    if(driveStatus.B.CloseLoop == 1)
	{
	    // Actual rotor speed is calculated based on ZC period or period measured by FTM2 Input Capture mode
	    actualSpeed = MLIB_Mul(MLIB_ConvertPU_FLTF32(MLIB_Div_F32(SPEED_SCALE_CONST, period6ZC)), N_MAX);

		torqueErr = MLIB_Sub(I_DCB_LIMIT, torque_filt);
	    currentPIOut = GFLIB_ControllerPIpAW(torqueErr, &currentPIPrms);

	    /* Speed control */

	    // Upper speed limit due to the limited DC bus voltage 12V
	    if(requiredSpeed >= N_NOM)
	    	requiredSpeed = N_NOM;

	    // Lower speed limit keeping reliable sensorless operation
	    if(requiredSpeed < mcat_NMin)
	    	requiredSpeed = mcat_NMin;

	    requiredSpeedRamp = GFLIB_Ramp(requiredSpeed, &speedRampPrms);
	    speedErr = MLIB_Sub(requiredSpeedRamp, actualSpeed);
	    speedPIOut = GFLIB_ControllerPIpAW(speedErr, &speedPIPrms);

	    if(currentPIOut >= speedPIOut)
	    {
	    	/* If max torque not achieved, use speed PI output */
	        currentPIPrms.fltIntegPartK_1 = speedPIOut;
	        currentPIPrms.fltInK_1 = 0;
	        /* PWM duty cycle update <- speed PI */
	        duty_cycle = speedPIOut;

	        driveStatus.B.CurrentLimiting = 0;
	    }
	    else
	    {
	    	/* Limit speed PI output by current PI if max. torque achieved */
	        speedPIPrms.fltIntegPartK_1 = currentPIOut;
	        speedPIPrms.fltInK_1 = 0;
	        /* PWM duty cycle update <- current PI */
        	duty_cycle = currentPIOut;

	        driveStatus.B.CurrentLimiting = 1;
	    }

        // Update PWM duty cycle
	    ACTUATE_SetDutycycle(duty_cycle, HW_INPUT_TRIG0);

	}

// Freewheeling is ignored in Sensorbased mode
#if (!HALL_SENSOR)

	if(driveStatus.B.Freewheeling)
		{
			if(freewheelTimer > 0)
			{
				freewheelTimer--;
			}
			else
			{
				driveStatus.B.Freewheeling = 0;
			}
		}
#endif

    /* pdb_delay calculated based on the actual duty_cycle
     * to measure DC bus voltage and Back EMF voltage
     * towards the end of the active PWM pulse
     */
    pdb_delay = (uint16_t)(MLIB_Mul(MLIB_Div(duty_cycle, 100.0F), PDB_DELAY_MAX));

    // Saturate, if pdb_delay is lower than PDB_DELAY_MIN
    if(pdb_delay < PDB_DELAY_MIN)
    	pdb_delay = PDB_DELAY_MIN;

    /* Update PDBs delays */
    PDB_DRV_SetAdcPreTriggerDelayValue(0, 0, 0, pdb_delay);
    PDB_DRV_SetAdcPreTriggerDelayValue(1, 0, 1, pdb_delay);
    PDB_DRV_LoadValuesCmd(0);
    PDB_DRV_LoadValuesCmd(1);

    CheckSwitchState();

    LPIT_DRV_ClearInterruptFlagTimerChannels(0, 0b1);

    PTD->PCOR |= 1<<2;
}

/******************************************************************************
*
* Function:		void AppInit(void)
*
* Description: 	BLDC application INIT state function
*
*******************************************************************************/
void AppInit(void)
{
    driveStatus.B.Alignment = 0;
    driveStatus.B.EnableCMT = 0;
    driveStatus.B.CloseLoop = 0;
    driveStatus.B.Calib = 0;
	driveStatus.B.Sensorless = 0;
	driveStatus.B.NewZC = 0;

    // Init parameters for DC bus current offset calibration
    calibTimer 				= CALIB_TIMER;
    ADCResults.DCBIOffset 	= (I_MAX/2.0F);
    Idcb_calib.fltAcc 		= (I_MAX/2.0F);

    // Init parameters for Speed control
    actualSpeed 			= 0.0F;
	advanceAngle 			= ADVANCE_ANGLE;

    // Disable all PWMs
    ACTUATE_DisableOutput(HW_INPUT_TRIG1);

    // Init parameters for SixStep Commutation control
	NextCmtSector 	= 0;	// Starting sector
	NextCmtPeriod 	= mcat_startCmtPer;
	startCMTcounter = mcat_startCmtCnt - 1;

    appState = APP_STOP;
}

/*******************************************************************************
*
* Function: 	void AppStop(void)
*
* Description: 	BLDC application STOP state function
*
*******************************************************************************/
void AppStop(void)
{

#if HALL_SENSOR
	driveStatus.B.StallCheckReq = 1;

	// Application can be turn on only if rotor stops
	if((appSwitchState == 1) && (driveStatus.B.HallEvent == 0))
	{
		// Enable actuator
		ACTUATE_EnableOutput(HW_INPUT_TRIG1);

		driveStatus.B.Calib = 1;

		appState = APP_CALIB;
	}

#else
	// Application can be turn on only if rotor stops
	if((appSwitchState == 1) && (driveStatus.B.Freewheeling == 0))
	{
		// Enable actuator
		ACTUATE_EnableOutput(HW_INPUT_TRIG1);

		driveStatus.B.Calib = 1;

		appState = APP_CALIB;
	}

#endif

}

/*******************************************************************************
*
* Function: 	void AppCalib(void)
*
* Description:  BLDC application CALIB state function
*
*******************************************************************************/
void AppCalib(void)
{
	// Measure DC bus current offset
	ADCResults.DCBIOffset = GDFLIB_FilterMA(ADCResults.DCBIVoltageRaw, &Idcb_calib);

    if(calibTimer == 0)
	{
		AppStopToAlignment();
	}
}

/*******************************************************************************
*
* Function: 	void AppStopToAlignment(void)
*
* Description:  BLDC application STOP to ALIGN state transition function
*
*******************************************************************************/
void AppStopToAlignment(void)
{
    driveStatus.B.Alignment = 1;
    driveStatus.B.EnableCMT = 0;
    driveStatus.B.CloseLoop = 0;
    driveStatus.B.Calib = 0;
	driveStatus.B.Sensorless = 0;
	driveStatus.B.NewZC = 0;

    alignmentTimer = mcat_alignDuration;
    duty_cycle = MLIB_Mul(MLIB_Div(mcat_alignVoltage, U_PH_NOM), 100.0F);

    // Update PWM duty cycle
    ACTUATE_SetDutycycle(duty_cycle, HW_INPUT_TRIG1);
    /* Apply PWM settings for motor alignment */
    ACTUATE_SetPwmMask(ui8FTM3OutmaskVal[0][6], ui16FTM3SwOctrlVal[0][6], HW_INPUT_TRIG1);

    appState = APP_ALIGNMENT;
}

/*******************************************************************************
*
* Function: 	void AppAlignment(void)
*
* Description:  BLDC application ALIGN state function
*
*******************************************************************************/
void AppAlignment(void)
{

	if(alignmentTimer == 0)
    	{
    		AppAlignmentToStart();
    	}
}

/*******************************************************************************
*
* Function: 	void AppAlignmentToStart(void)
*
* Description:  BLDC application ALIGN to START state transition function
*
*******************************************************************************/
void AppAlignmentToStart(void)
{
    driveStatus.B.Alignment = 0;
    driveStatus.B.EnableCMT = 1;
	driveStatus.B.AfterCMT  = 0;

	/* Prepare PWM settings for initial commutation sector */
	ACTUATE_SetPwmMask(ui8FTM3OutmaskVal[rotationDir][NextCmtSector],
					   ui16FTM3SwOctrlVal[rotationDir][NextCmtSector], HW_INPUT_TRIG0);

	// Open loop startup is ignored in Sensorbased mode
	#if (!HALL_SENSOR)
		// Stop FTM0 counter
		// Force commutation sector 0 PWM settings
		FTM_DRV_CounterStop(INST_FLEXTIMER_MC0);

		// Reset FTM0 counter
		FTM0->CNT = 0;
		// Apply STARTUP_CMT_PERIOD to MODULO
		FTM_DRV_SetModuloCounterValue(INST_FLEXTIMER_MC0, STARTUP_CMT_PER, false);
		// Start FTM0 counter
		FTM_DRV_CounterStart(INST_FLEXTIMER_MC0);

		NextCmtSector++;

		NextCmtPeriod = MLIB_Mul_F16(NextCmtPeriod, FRAC16(mcat_startCmtAcceler));

		/* Prepare PWM settings for the next commutation sector */
		ACTUATE_SetPwmMask(ui8FTM3OutmaskVal[rotationDir][NextCmtSector],
					   ui16FTM3SwOctrlVal[rotationDir][NextCmtSector], HW_INPUT_TRIG0);
	#endif

    appState = APP_START;
}

/*******************************************************************************
*
* Function: 	void AppStart(void)
*
* Description:  BLDC application START state function
*
*******************************************************************************/
void AppStart(void)
{
	#if HALL_SENSOR
		AppStartToRun();

	// Open loop startup is ignored in Sensorbased mode
	#else
		if(driveStatus.B.AfterCMT == 1)
		{
			timeZC = NextCmtPeriod >> 1;

			startCMTcounter--;
			if(startCMTcounter > 0)
			{
				driveStatus.B.AfterCMT = 0;

				NextCmtPeriod = MLIB_Mul_F16(NextCmtPeriod, FRAC16(mcat_startCmtAcceler));
			}
		}

		if(startCMTcounter == 0)
		{
			AppStartToRun();
		}
	#endif
}
/*******************************************************************************
*
* Function: 	void AppStartToRun(void)
*
* Description:  BLDC application START to RUN state transition function
*
*******************************************************************************/
void AppStartToRun(void)
{
	uint8_t i;

    /* Speed PI controller initialization */
    speedPIPrms.fltInK_1 = 0;
    speedPIPrms.fltIntegPartK_1 = duty_cycle;

    /* Current PI controller initialization */
    currentPIPrms.fltInK_1 = 0;
    currentPIPrms.fltIntegPartK_1 = speedPIPrms.fltIntegPartK_1;

    /* Speed ramp initialization */
    speedRampPrms.fltState = mcat_NMin;

    appState = APP_RUN;
    stallCheckCounter = 0;
    faultStatus.B.StallError = 0;

    // Hall/ZC period initialization before entering Close loop mode
	#if HALL_SENSOR
		for(i=0;i<6;i++)
		{
			SensorHall.Period[i] = NextCmtPeriod;
		}
	#else
		for(i=0;i<6;i++)
		{
			periodZC[i] = NextCmtPeriod;
		}
		actualPeriodZC = NextCmtPeriod;

		driveStatus.B.Sensorless = 1;
	#endif

    // Reset FTM2 counter
	FTM2->CNT = 0;
    // Clear FTM counter overflow flag
	FTM2->SC &= (~FTM_SC_TOF_MASK);

    driveStatus.B.CloseLoop = 1;
}

/*******************************************************************************
*
* Function: 	void AppRun(void)
*
* Description:  BLDC application RUN state function
*
*******************************************************************************/
void AppRun(void)
{
    if(appSwitchState == 0)
    {
		// Disable actuator
		ACTUATE_DisableOutput(HW_INPUT_TRIG1);

		freewheelTimer = mcat_FreewheelTLong;
		mcat_FreewheelTShort = 0;
		mcat_integThr = 0;
		driveStatus.B.Freewheeling = 1;

		appState = APP_INIT;
    }
}

/*******************************************************************************
*
* Function: 	void AppFault(void)
*
* Description: 	BLDC application FAULT state function
*
*******************************************************************************/
void AppFault(void)
{
    if(faultSwitchClear == 1)
    {
        driveStatus.B.Fault = 0;
        faultStatus.R = 0;
        faultStatusLatched.R = 0;

        // Clear GD3000 Errors and interrupts
        tppDrvConfig.deviceConfig.statusRegister[0U] = 0U;
        TPP_ClearInterrupts(&tppDrvConfig, TPP_CLINT0_MASK, TPP_CLINT1_MASK);

        faultSwitchClear = 0;
        appState = APP_INIT;
    }
}

/*******************************************************************************
*
* Function: 	void CheckFaults(void)
*
* Description:  BLDC application fault detection function.
*
*******************************************************************************/
void CheckFaults(void)
{
	/* DC bus current overcurrent */
	if(ADCResults.DCBIVoltage > I_DCB_OVERCURRENT)
    {
        driveStatus.B.Alignment = 0;
        driveStatus.B.EnableCMT = 0;
        driveStatus.B.CloseLoop = 0;
        driveStatus.B.Sensorless = 0;
        driveStatus.B.NewZC = 0;

        faultStatus.B.OverDCBusCurrent = 1;

    	// Disable actuator
    	ACTUATE_DisableOutput(HW_INPUT_TRIG1);
    }
    else
    {
        faultStatus.B.OverDCBusCurrent = 0;
    }


    /* DC bus voltage overvoltage */
    if(ADCResults.DCBVVoltage > U_DCB_OVERVOLTAGE)
    {
        faultStatus.B.OverDCBusVoltage = 1;

        driveStatus.B.Alignment = 0;
        driveStatus.B.EnableCMT = 0;
        driveStatus.B.CloseLoop = 0;
        driveStatus.B.Sensorless = 0;
        driveStatus.B.NewZC = 0;

    	// Disable actuator
    	ACTUATE_DisableOutput(HW_INPUT_TRIG1);
    }
    else
    {
        faultStatus.B.OverDCBusVoltage = 0;
    }

    /* DC bus voltage undervoltage */
	if(ADCResults.DCBVVoltage < U_DCB_UNDERVOLTAGE)
	{
		faultStatus.B.UnderDCBusVoltage = 1;

		driveStatus.B.Alignment = 0;
		driveStatus.B.EnableCMT = 0;
		driveStatus.B.CloseLoop = 0;
		driveStatus.B.Sensorless = 0;
		driveStatus.B.NewZC = 0;

    	// Disable actuator
    	ACTUATE_DisableOutput(HW_INPUT_TRIG1);
	}
	else
	{
		faultStatus.B.UnderDCBusVoltage = 0;
	}


	/* MC34GD3000 MOSFET Pre-driver error */
    if (tppDrvConfig.deviceConfig.statusRegister[0U])
    {
    	faultStatus.B.PreDriverError = 1;

		driveStatus.B.Alignment = 0;
		driveStatus.B.EnableCMT = 0;
		driveStatus.B.CloseLoop = 0;
		driveStatus.B.Sensorless = 0;
		driveStatus.B.NewZC = 0;

    	// Disable actuator
    	ACTUATE_DisableOutput(HW_INPUT_TRIG1);
    }
    else
    {
    	faultStatus.B.PreDriverError = 0;
    }

    /* Stall error */
    if(faultStatus.B.StallError)
    {
        driveStatus.B.Alignment = 0;
        driveStatus.B.EnableCMT = 0;
        driveStatus.B.CloseLoop = 0;
        driveStatus.B.Sensorless = 0;
        driveStatus.B.NewZC = 0;

    	// Disable actuator
    	ACTUATE_DisableOutput(HW_INPUT_TRIG1);
    }

    faultStatusLatched.R |= faultStatus.R;

    if(faultStatusLatched.R != 0)
    {
        driveStatus.B.Fault = 1;
        appSwitchState = 0;
        appState = APP_FAULT;
    }
    else
    {
        faultSwitchClear = 0;
    }
}

/*******************************************************************************
*
* Function: 	void CheckSwitchState(void)
*
* Description:  User switch state detection function
*
*******************************************************************************/
void CheckSwitchState(void)
{
    if(switchOffCounter == 0)
    {
        /* Speed up or start the motor */
        if(((PINS_DRV_ReadPins(PTC) >> 12) & 1))
        {
            switchCounter[0]++;

            if(switchCounter[0] > SW_PRESS_DEBOUNCE)
            {
                if(appSwitchState == 0)
                {
                	rotationDir = ROTATION_DIR_CW;
                    appSwitchState = 1;
                    switchOffCounter = SW_PRESS_OFF;
                }
                else
                {
                    requiredSpeed += SPEED_INC;
                }

                switchCounter[0] = 0;
            }
        }

        /* Speed down or start the motor */
        if(((PINS_DRV_ReadPins(PTC) >> 13) & 1))
        {
            switchCounter[1]++;

            if(switchCounter[1] > SW_PRESS_DEBOUNCE)
            {
            	if(appSwitchState == 0)
            	{
            		rotationDir = ROTATION_DIR_CCW;
            	    appSwitchState = 1;
   	                switchOffCounter = SW_PRESS_OFF;
            	}
            	else
                {
            	    requiredSpeed -= SPEED_DEC;
                }

            	switchCounter[1] = 0;
            }
        }

        /* Clear faults or stop the motor */
        if(((PINS_DRV_ReadPins(PTC) >> 13) & 1) && ((PINS_DRV_ReadPins(PTC) >> 12) & 1))
        {
            if(appState == APP_FAULT)
            {
                faultSwitchClear = 1;
            }

            appSwitchState = 0;
            switchOffCounter = SW_PRESS_OFF;
        }
    }
    else
    {
        switchOffCounter--;
    }

}

/*******************************************************************************
*
* Function: 	void StallCheck(void)
*
* Description:  Stall check function
*
*******************************************************************************/
void StallCheck(void)
{
	// In Sensorbased mode, StallCheck monitors Hall events
	#if HALL_SENSOR
		if(FTM2->SC & FTM_SC_TOF_MASK)
		{
			driveStatus.B.HallEvent = 0;
			// Rotor Stall detection in Sensorbased mode based on Hall input
			if(driveStatus.B.CloseLoop)
			{
				// Disable FTM2 and Clear timer overflow flag
				FTM2->SC &= (~FTM_SC_CLKS(1) | (~FTM_SC_TOF_MASK));
				// Reset FTM2 counter
				FTM2->CNT = 0;
				// Rotor Stall Error
				faultStatus.B.StallError = 1;
			}
		}

	// In Sensorless mode, StallCheck evaluates ZC period value
	#else

		uint8_t i;
		uint16_t max = 0, min = 65535;

		driveStatus.B.StallCheckReq = 0;

		for(i=0; i<6; i++)
		{
			if(periodZC[i] > max)
			{
				max = periodZC[i];
			}

			if(periodZC[i] < min)
			{
				min = periodZC[i];
			}
		}

		/* Save min and max commutation periods for tuning purposes */
		debugTmin = min;
		debugTmax = max;

		periodZcAvrg = MLIB_Mul_F32(period6ZC,FRAC32(0.1666));

		/* Save min and max commutation periods limits for tuning purposes */
		debugTmaxLim = periodZcAvrg << 1;
		debugTminLim = periodZcAvrg >> 1;

		if ((max > (periodZcAvrg << 1)) || (min < (periodZcAvrg >> 1)))
		{
			if (stallCheckCounter < STALLCHECK_MAX_ERRORS)
			{
				stallCheckCounter++;
			}
		}
		else
		{
			if (min < STALLCHECK_MIN_CMT_PERIOD)
			{
				if (stallCheckCounter < STALLCHECK_MAX_ERRORS)
				{
					stallCheckCounter++;
				}
			}
			else
			{
				if (stallCheckCounter > 0)
				{
					stallCheckCounter--;
				}
			}
		}

		if (stallCheckCounter >= STALLCHECK_MAX_ERRORS)
		{
			faultStatus.B.StallError = 1;
		}

#endif

}
/***************************************************************************//*!
*
* Function:		void RGBLedOFF()
*
* Description:	This function turns RGB LED off
*
******************************************************************************/
void RGBLedOFF()
{
	PINS_DRV_SetPins(PTD, 1<<0);		// RGB Blue  Led OFF
	PINS_DRV_SetPins(PTD, 1<<15);		// RGB Red 	 Led OFF
	PINS_DRV_SetPins(PTD, 1<<16);		// RGB Green Led OFF
}

/***************************************************************************//*!
*
* Function:		void RGBLedBlueON()
*
* Description:	This function turns RGB Blue LED on
*
******************************************************************************/
void RGBLedBlueON()
{
	PINS_DRV_ClearPins(PTD, 1<<0);		// RGB Blue  Led ON
	PINS_DRV_SetPins(PTD, 1<<15);		// RGB Red 	 Led OFF
	PINS_DRV_SetPins(PTD, 1<<16);		// RGB Green Led OFF
}

/***************************************************************************//*!
*
* Function:		void RGBLedRedON()
*
* Description:	This function turns RGB Red LED on
*
******************************************************************************/
void RGBLedRedON()
{
	PINS_DRV_SetPins(PTD, 1<<0);		// RGB Blue  Led OFF
	PINS_DRV_ClearPins(PTD, 1<<15);		// RGB Red 	 Led ON
	PINS_DRV_SetPins(PTD, 1<<16);		// RGB Green Led OFF
}

/***************************************************************************//*!
*
* Function:		void RGBLedGreenON()
*
* Description:	This function turns RGB Green LED on
*
******************************************************************************/
void RGBLedGreenON()
{
	PINS_DRV_ClearPins(PTD, 1<<16);		// RGB Green Led ON
	PINS_DRV_SetPins(PTD, 1<<0);		// RGB Blue  Led OFF
	PINS_DRV_SetPins(PTD, 1<<15);		// RGB Red 	 Led OFF
}

/***************************************************************************//*!
*
* Function:		void RGBLedGreenFlash()
*
* Description:	This function flashes RGB Green LED
*
******************************************************************************/
void RGBLedGreenFlash()
{
	ledCounter += 1;

	/* RGB Green Led FLASHING */
	if(ledCounter > LED_FLASH_FREQ)
	{
		PINS_DRV_TogglePins(PTD, 1<<16);
		ledCounter = 0;
	}
	PINS_DRV_SetPins(PTD, 1<<0);		// RGB Blue  Led OFF
	PINS_DRV_SetPins(PTD, 1<<15);		// RGB Red 	 Led OFF
}

