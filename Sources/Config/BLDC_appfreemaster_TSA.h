/*******************************************************************************
*
* Copyright 2016-2018 NXP
*
****************************************************************************//*!
*
* @file     BLDC_appfreemaster_TSA.h
*
* @date     Sept-04-2018
*
* @brief    Header file for BLDC Application
*
*******************************************************************************/

#ifndef CONFIG_BLDC_APPFREEMASTER_TSA_H_
#define CONFIG_BLDC_APPFREEMASTER_TSA_H_

/******************************************************************************
* Includes
******************************************************************************/

#include "freemaster.h"

/******************************************************************************
| Defines and macros
-----------------------------------------------------------------------------*/

extern 		gd3000Status_t    				gd3000Status;
extern 		tpp_drv_config_t  				tppDrvConfig;
extern 		tDriveStatus 					driveStatus;
extern 		tFaultStatus 					faultStatus;
extern 		tFaultStatus 					faultStatusLatched;
extern 		tADCresults  					ADCResults;
extern 		GFLIB_CONTROLLER_PIAW_P_T_FLT 	speedPIPrms;
extern 		GFLIB_CONTROLLER_PIAW_P_T_FLT 	currentPIPrms;
extern 		GFLIB_RAMP_T_FLT 				speedRampPrms;
extern 		GDFLIB_FILTER_MA_T_FLT 			Udcb_filt;
extern 		GDFLIB_FILTER_MA_T_FLT 			Idcb_filt;
extern 		GDFLIB_FILTER_MA_T_FLT 			Idcb_calib;
extern 		tSensorHall						SensorHall;

extern 		tFloat 							speedErr;
extern		tFloat 							requiredSpeed;
extern		tFloat 							requiredSpeedRamp;
extern	 	tFloat 							speedPIOut;
extern		uint8_t  						ActualCmtSector;
extern		uint16_t 						actualPeriodZC;
extern		uint16_t    					periodZC[6];
extern		uint16_t						periodHall[6];
extern		tFloat 							actualSpeed;
extern		uint16_t 						advanceAngle;
extern		uint16_t 						alignmentTimer;
extern		uint8_t    						appState;
extern		tFloat 							bemfVoltage;
extern		tFloat 							bemfVoltage;
extern		uint16_t 						debugTmin, debugTmax, periodZcAvrg, stallCheckCounter;
extern		uint16_t 						debugTminLim, debugTmaxLim, startCMTcounter;
extern		uint16_t 						ftm_mod_old;
extern		uint16_t 						lastTimeZC;
extern		uint16_t 						NextCmtPeriod;
extern		uint16_t 						pdb_delay;
extern		uint8_t  						NextCmtSector;
extern		uint32_t 						period6ZC, period6Hall;
extern		uint8_t    						rotationDir;
extern		tFloat 							timeBackEmf, timeZCToff;
extern		uint16_t 						timeZC, calibTimer;
extern 		tFloat   						duty_cycle;
extern		tFloat							torque_filt;
extern		tFloat							u_dc_bus_filt;
extern		uint8_t    						hall, mod;
extern 		uint8_t 						appSwitchState, faultSwitchClear;
extern		tU32 							fm_voltage;
extern		tU32 							fm_current;
extern		tU32 							fm_speed;
extern 		tFloat 							mcat_alignVoltage;
extern 		uint16_t 						mcat_alignDuration;
extern 		tFloat 							mcat_integThr;
extern 		tFloat 							mcat_integThr;
extern 		uint16_t 						mcat_FreewheelTLong;
extern 		uint16_t 						mcat_FreewheelTShort;
extern 		tFloat 							mcat_cmtTOff;
extern 		tFloat 							mcat_NMin;
extern 		uint16_t 						mcat_FreewheelTLong;
extern 		uint16_t 						mcat_FreewheelTShort;
extern 		uint8_t 						mcat_startCmtCnt;
extern 		uint16_t 						mcat_startCmtPer;
extern 		tFloat 							mcat_startCmtAcceler;
extern 		uint8_t 						pdb0Counter;
extern 		uint8_t 						pdb1Counter;
extern		uint8_t 						ftm2Ch1XorSignal;
extern		tBool 							sensorType;

/*	*************** begin TSA table - S32K144_BLDC   ************* */
FMSTR_TSA_TABLE_BEGIN(S32K_BLDC)

/*  */
	FMSTR_TSA_RW_VAR(gd3000Status,        			FMSTR_TSA_USERTYPE(gd3000Status_t))
	FMSTR_TSA_RW_VAR(tppDrvConfig,        			FMSTR_TSA_USERTYPE(tpp_drv_config_t))
	FMSTR_TSA_RW_VAR(driveStatus,        			FMSTR_TSA_USERTYPE(tDriveStatus))
	FMSTR_TSA_RW_VAR(faultStatus,        			FMSTR_TSA_USERTYPE(tFaultStatus))
	FMSTR_TSA_RW_VAR(faultStatusLatched,        	FMSTR_TSA_USERTYPE(tFaultStatus))
	FMSTR_TSA_RW_VAR(ADCResults,        			FMSTR_TSA_USERTYPE(tADCresults))
	FMSTR_TSA_RW_VAR(speedPIPrms,        			FMSTR_TSA_USERTYPE(GFLIB_CONTROLLER_PIAW_P_T_FLT))
	FMSTR_TSA_RW_VAR(currentPIPrms,        			FMSTR_TSA_USERTYPE(GFLIB_CONTROLLER_PIAW_P_T_FLT))
	FMSTR_TSA_RW_VAR(speedRampPrms,        			FMSTR_TSA_USERTYPE(GFLIB_RAMP_T_FLT))
	FMSTR_TSA_RW_VAR(Udcb_filt,        				FMSTR_TSA_USERTYPE(GDFLIB_FILTER_MA_T_FLT))
	FMSTR_TSA_RW_VAR(Idcb_filt,        				FMSTR_TSA_USERTYPE(GDFLIB_FILTER_MA_T_FLT))
	FMSTR_TSA_RW_VAR(Idcb_calib,        			FMSTR_TSA_USERTYPE(GDFLIB_FILTER_MA_T_FLT))
	FMSTR_TSA_RW_VAR(SensorHall,        			FMSTR_TSA_USERTYPE(tSensorHall))


/*  ***************				VARIABLES 			    ******************* */
	FMSTR_TSA_RW_VAR(speedErr,        			FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(requiredSpeed,        		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(requiredSpeedRamp,   		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(speedPIOut,        		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(ActualCmtSector,        	FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(actualPeriodZC,        	FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(periodZC,        			FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(periodHall,        		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(actualSpeed,   			FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(advanceAngle,        		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(alignmentTimer,        	FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(appState,        			FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(appSwitchState,   			FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(pdb_delay,   				FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(faultSwitchClear,        	FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(bemfVoltage,        		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(bemfVoltage,        		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(debugTmin,   				FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(debugTmax,        			FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(periodZcAvrg,        		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(stallCheckCounter,        	FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(debugTminLim,   			FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(debugTmaxLim,        		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(startCMTcounter,        	FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(duty_cycle,        		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(fm_voltage,        		FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(fm_current,        		FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(fm_speed,        			FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(ftm_mod_old,   			FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(lastTimeZC,        		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(NextCmtPeriod,        		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(NextCmtSector,        		FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(period6ZC,        			FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(timeBackEmf,        		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(timeZCToff,        		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(timeZC,   					FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(rotationDir,        		FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(calibTimer,   				FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(mcat_alignVoltage,   		FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(mcat_alignDuration,   		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(mcat_NMin,   				FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(mcat_FreewheelTLong,   	FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(mcat_FreewheelTShort,  	FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(mcat_startCmtCnt,   		FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(mcat_startCmtPer,   		FMSTR_TSA_UINT16)
	FMSTR_TSA_RW_VAR(mcat_startCmtAcceler,   	FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(mcat_cmtTOff,   			FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(mcat_integThr,   			FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(torque_filt,   			FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(u_dc_bus_filt,   			FMSTR_TSA_FLOAT)
	FMSTR_TSA_RW_VAR(period6Hall,      			FMSTR_TSA_UINT32)
	FMSTR_TSA_RW_VAR(pdb0Counter,        		FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(pdb1Counter,        		FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(ftm2Ch1XorSignal,        	FMSTR_TSA_UINT8)
	FMSTR_TSA_RW_VAR(sensorType,        		FMSTR_TSA_UINT8)


/*	*************** 			STRUCTURES              ******************* */
	FMSTR_TSA_STRUCT(tDriveStatus)
		FMSTR_TSA_MEMBER(tDriveStatus, 		R, 					FMSTR_TSA_UINT16)

	FMSTR_TSA_STRUCT(tFaultStatus)
		FMSTR_TSA_MEMBER(tFaultStatus, 		R, 					FMSTR_TSA_UINT8)

	FMSTR_TSA_STRUCT(tFaultStatus)
		FMSTR_TSA_MEMBER(tFaultStatus, 		B, 					FMSTR_TSA_UINT8)

	FMSTR_TSA_STRUCT(tADCresults)
		FMSTR_TSA_MEMBER(tADCresults, 		BEMFVoltage, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(tADCresults, 		DCBVVoltage, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(tADCresults, 		DCBIVoltage, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(tADCresults, 		DCBIVoltageRaw, 	FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(tADCresults, 		DCBIOffset, 		FMSTR_TSA_FLOAT)

	FMSTR_TSA_STRUCT(gd3000Status_t)
		FMSTR_TSA_MEMBER(gd3000Status_t, 		R, 				FMSTR_TSA_UINT16)

	FMSTR_TSA_STRUCT(tpp_drv_config_t)
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		en1PinInstance, 		FMSTR_TSA_USERTYPE(aml_instance_t))
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		en1PinIndex, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		en2PinInstance, 		FMSTR_TSA_USERTYPE(aml_instance_t))
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		en2PinIndex, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		rstPinInstance, 		FMSTR_TSA_USERTYPE(aml_instance_t))
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		rstPinIndex, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		spiInstance, 			FMSTR_TSA_USERTYPE(aml_instance_t))
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		spiTppConfig, 			FMSTR_TSA_USERTYPE(spi_tpp_config_t))
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		csPinInstance, 			FMSTR_TSA_USERTYPE(aml_instance_t))
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		csPinIndex, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_drv_config_t, 		deviceConfig, 			FMSTR_TSA_USERTYPE(tpp_device_data_t))

	FMSTR_TSA_STRUCT(tpp_device_data_t)
		FMSTR_TSA_MEMBER(tpp_device_data_t, opMode, 			FMSTR_TSA_USERTYPE(tpp_device_mode_t))
		FMSTR_TSA_MEMBER(tpp_device_data_t, statusRegister, 	FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_device_data_t, intMask0, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_device_data_t, intMask1, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_device_data_t, modeMask, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tpp_device_data_t, deadtime, 			FMSTR_TSA_UINT16)

	FMSTR_TSA_STRUCT(GFLIB_CONTROLLER_PIAW_P_T_FLT)
		FMSTR_TSA_MEMBER(GFLIB_CONTROLLER_PIAW_P_T_FLT, fltPropGain, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_CONTROLLER_PIAW_P_T_FLT, fltIntegGain, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_CONTROLLER_PIAW_P_T_FLT, fltLowerLimit, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_CONTROLLER_PIAW_P_T_FLT, fltUpperLimit, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_CONTROLLER_PIAW_P_T_FLT, fltIntegPartK_1,	FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_CONTROLLER_PIAW_P_T_FLT, fltInK_1, 			FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_CONTROLLER_PIAW_P_T_FLT, u16LimitFlag, 		FMSTR_TSA_UINT16)

	FMSTR_TSA_STRUCT(GFLIB_RAMP_T_FLT)
		FMSTR_TSA_MEMBER(GFLIB_RAMP_T_FLT, 		fltState, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_RAMP_T_FLT, 		fltRampUp, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GFLIB_RAMP_T_FLT, 		fltRampDown, 	FMSTR_TSA_FLOAT)

	FMSTR_TSA_STRUCT(GDFLIB_FILTER_MA_T_FLT)
		FMSTR_TSA_MEMBER(GDFLIB_FILTER_MA_T_FLT, fltAcc, 		FMSTR_TSA_FLOAT)
		FMSTR_TSA_MEMBER(GDFLIB_FILTER_MA_T_FLT, fltLambda,		FMSTR_TSA_FLOAT)

	FMSTR_TSA_STRUCT(tSensorHall)
		FMSTR_TSA_MEMBER(tSensorHall, 			InA, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tSensorHall, 			InB, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tSensorHall, 			InC, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tSensorHall, 			InABC, 			FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tSensorHall, 			Sector, 		FMSTR_TSA_UINT8)
		FMSTR_TSA_MEMBER(tSensorHall, 			Period,			FMSTR_TSA_UINT16)
		FMSTR_TSA_MEMBER(tSensorHall, 			Ftm2HallCnt,	FMSTR_TSA_UINT16)

FMSTR_TSA_TABLE_END()


/* TSA table list */
FMSTR_TSA_TABLE_LIST_BEGIN()
	FMSTR_TSA_TABLE(S32K_BLDC)
FMSTR_TSA_TABLE_LIST_END()


#endif /* CONFIG_BLDC_APPFREEMASTER_TSA_H_ */
