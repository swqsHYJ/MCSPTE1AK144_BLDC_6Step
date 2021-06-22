/**********************************************************************/
// File Name: {FM_project_loc}/Sources/Config/BLDC_appconfig.h 
//
// Date:  26. November, 2018
//
// Automatically generated file for static configuration of the BLDC application
/**********************************************************************/

#ifndef __BLDC_CONFIG_SETUP_H
#define __BLDC_CONFIG_SETUP_H


//Motor Parameters                      
//----------------------------------------------------------------------
//Pole-pair number                      = 2 [-]
//Back-EMF constant                     = 0.01567 [V.sec/rad]
//Phase current nominal                 (2.3F)
//Phase voltage nominal                 (12.0F)
//----------------------------------------------------------------------

//Application scales                    
#define I_MAX                           (25.0F)
#define U_DCB_MAX                       (45.0F)
#define N_MAX                           (22500.0F)//(2500.0F)
#define I_DCB_OVERCURRENT               (10.0F)
#define U_DCB_UNDERVOLTAGE              (9.0F)
#define U_DCB_OVERVOLTAGE               (18.0F)
#define I_DCB_LIMIT                     (3.0F)
#define U_DCB_TRIP                      (25.0F)
#define N_NOM                           (20000.0F)//(2000.0F)
#define I_PH_NOM                        (5.3F)
#define U_PH_NOM                        (12.0F)
//Mechanical Alignment                  
#define ALIGN_VOLTAGE                   (1.5F)
#define ALIGN_DURATION                  (20000)

//BLDC Control Loop                     
//----------------------------------------------------------------------
//Loop sample time                      = 0.001 [sec]
//----------------------------------------------------------------------
//Control loop limits                   
#define CTRL_LOOP_LIM_HIGH              (100.0F)//(90.0F)
#define CTRL_LOOP_LIM_LOW               (10.0F)//(10.0F)

#define SPEED_LOOP_KP_GAIN              (0.001000000000F)//(0.001000000000F)
#define SPEED_LOOP_KI_GAIN              (0.000100000000F)

//Speed ramp increments                 
#define SPEED_LOOP_RAMP_UP              (500.0F)//(10.0F)
#define SPEED_LOOP_RAMP_DOWN            (500.0F)//(10.0F)

//Torque Controller - Parallel type     
#define TORQUE_LOOP_KP_GAIN             (0.1F)
#define TORQUE_LOOP_KI_GAIN             (0.03F)
#define TORQUE_LOOP_MAF                 (0.03125F)

//Sensorless Control Module             
//----------------------------------------------------------------------
//Timer Frequency                       = 625000 [Hz]
//----------------------------------------------------------------------
#define N_MIN                           (1650.0F)//(400.0F)
#define N_START_TRH                     (1000.0F)//(400.0F)
#define STARTUP_CMT_CNT                 (4)
#define STARTUP_CMT_PER                 (12500)
#define CMT_T_OFF                       (20.0F)
#define FREEWHEEL_T_LONG                (100)
#define FREEWHEEL_T_SHORT               (50)
#define SPEED_SCALE_CONST               (1000)//(7500)
#define CMT_PER_MIN                     (1250)
#define START_CMT_ACCELER               (0.854987973338F)
#define INTEG_TRH                       (0)

//FreeMASTER Scale Variables            
//----------------------------------------------------------------------
//Note: Scaled at input = 1000          
//----------------------------------------------------------------------
#define FM_I_SCALE                      (25000)
#define FM_U_DCB_SCALE                  (45000)
#define FM_N_SCALE                      (22000000)//(2500000)

#endif
/**********************************************************************/
/**********************************************************************/
