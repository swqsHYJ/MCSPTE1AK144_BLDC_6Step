/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     motor_structure.h
*
* @date     March-28-2017
*
* @brief    Header file for FOC Drive
*
*******************************************************************************/
#ifndef MOTOR_STRUCTURE_H_
#define MOTOR_STRUCTURE_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "gdflib.h"
#include "gflib.h"
#include "gmclib.h"
#include "mlib.h"
#include "freemaster.h"
#include "SWLIBS_Config.h"

  /*****************************************************************************
  * Define Hall based or Sensorless based BLDC SixStep Control
  *
  * HALL_SENSOR  0 		Sensorless operation, motor position/speed obtained by the back-EMF voltage
  * 					zero-cross detection method
  * HALL_SENSOR  1 		Sensorbased operation, motor position/speed is obtained by the Hall sensor
  *
  ******************************************************************************/
#define HALL_SENSOR 					0

/******************************************************************************
| Defines and macros
-----------------------------------------------------------------------------*/
#define ROTATION_DIR_CW         		0
#define ROTATION_DIR_CCW        		1

/* ADVANCE_ANGLE' = 0.5 * ADVANCED_ANGLE  */
#define ADVANCE_ANGLE   				FRAC16(0.3815)

/* Duty cycle limit for DC bus current measurement */
#define DC_THRESHOLD    				10.0F

/* DC Bus Voltage MA filter defined by Lambda */
#define DCBV_FILTER_MA_LAMBDA    		0.25F
/* DC Bus Current Offset MA filter defined by Lambda */
#define CALIB_FILTER_MA_LAMBDA 			0.001F
/* Wait 0.5s to settle DC bus current offset
 * CALIB_TIMER = PWM freq/2Hz = 20kHz/2Hz */
#define CALIB_TIMER						10000

/* Speed increase step [RPM] */
#define SPEED_INC   					500.0F
/* Speed decrease step [RPM] */
#define SPEED_DEC   					500.0F
/* Maximal speed [RPM] */
#define SPEED_MAX   					2000.0F//2000.0F

/* Maximum number of stall check errors */
#define STALLCHECK_MAX_ERRORS			6
/* Minimal stall commutation period */
/* 20KRPM => 125us => 156.25 @625kHz */
#define STALLCHECK_MIN_CMT_PERIOD		156

/* User switch debounce timeout */
#define SW_PRESS_DEBOUNCE   			75
/* User switch input blocking delay */
#define SW_PRESS_OFF        			250
/* User LED flashing period */
#define LED_FLASH_FREQ      			80000

/******************************************************************************
| Typedefs and structures
-----------------------------------------------------------------------------*/
typedef union {
    uint16_t R;
    struct {
        uint16_t Alignment:1;
        uint16_t Sensorless:1;
        uint16_t StallCheckReq:1;
        uint16_t EnableCMT:1;
        uint16_t AfterCMT:1;
        uint16_t CloseLoop:1;
        uint16_t NewZC:1;
        uint16_t AdcSaved:1;
        uint16_t CurrentLimiting:1;
        uint16_t Fault:1;
        uint16_t Freewheeling:1;
        uint16_t Calib:1;
        uint16_t HallEvent:1;
        uint16_t Reserved:3;
    }B;
}tDriveStatus;

typedef union {
    uint8_t R;
    struct {
        uint8_t OverDCBusCurrent:1;
        uint8_t OverDCBusVoltage:1;
        uint8_t UnderDCBusVoltage:1;
        uint8_t PreDriverError:1;
        uint8_t StallError:1;
        uint8_t Reserved:3;
    }B;
}tFaultStatus;

typedef struct {
	tFloat BEMFVoltage;
	tFloat DCBVVoltage;
	tFloat DCBIVoltage;
	tFloat DCBIVoltageRaw;
	tFloat DCBIOffset;
}tADCresults;

typedef struct {
	uint8_t 	InA;
	uint8_t 	InB;
	uint8_t 	InC;
	uint8_t 	InABC;
	uint8_t 	Sector;
	uint16_t 	Period[6];
    uint16_t 	Ftm2HallCnt;

}tSensorHall;

typedef union
{
	tU16 R;
    struct
    {
        tU16 gd3000IntFlag			: 1;   /*  */
        tU16 gd3000ClearErr         : 1;   /*  */
        tU16 gd3000Error            : 1;   /*  */
        tU16 gd3000InitDone			: 1;
        tU16 Reserved				: 12;   /* RESERVED */
    }B;
}gd3000Status_t;





#endif /* MOTOR_STRUCTURE_H_ */
