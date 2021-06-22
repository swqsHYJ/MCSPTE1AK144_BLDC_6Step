/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : pwrMan1.h
**     Project     : MCSPTE1AK144_BLDC_6Step
**     Processor   : S32K144_100
**     Component   : power_manager
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2021-04-29, 14:22, # CodeGen: 0
**     Contents    :
**         POWER_SYS_Init                  - status_t POWER_SYS_Init(power_manager_user_config_t *(*)...
**         POWER_SYS_Deinit                - status_t POWER_SYS_Deinit(void);
**         POWER_SYS_SetMode               - status_t POWER_SYS_SetMode(uint8_t powerModeIndex,power_manager_policy_t...
**         POWER_SYS_GetLastMode           - status_t POWER_SYS_GetLastMode(uint8_t* powerModeIndexPtr);
**         POWER_SYS_GetLastModeConfig     - status_t POWER_SYS_GetLastModeConfig(power_manager_user_config_t**...
**         POWER_SYS_GetCurrentMode        - power_manager_modes_t POWER_SYS_GetCurrentMode(void);
**         POWER_SYS_GetErrorCallbackIndex - uint8_t POWER_SYS_GetErrorCallbackIndex(void);
**         POWER_SYS_GetErrorCallback      - power_manager_callback_user_config_t* POWER_SYS_GetErrorCallback(void);
**         POWER_SYS_GetDefaultConfig      - void POWER_SYS_GetDefaultConfig(power_manager_user_config_t * const config);
**         POWER_SYS_GetResetSrcStatusCmd  - bool POWER_SYS_GetResetSrcStatusCmd(const RCM_Type * const baseAddr , const...
**
**     Copyright 1997 - 2015 Freescale Semiconductor, Inc. 
**     Copyright 2016-2017 NXP 
**     All Rights Reserved.
**     
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file pwrMan1.h
** @version 01.00
*/         
/*!
**  @addtogroup pwrMan1_module pwrMan1 module documentation
**  @{
*/         
#ifndef pwrMan1_H
#define pwrMan1_H
/* MODULE pwrMan1. */
/* Include inherited beans */
#include "clockMan1.h"
#include "Cpu.h"
#include "power_manager.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.11, When an array with external linkage
 * is declared, its size should be explicitly specified.
 * The number of configurations/callbacks can be zero.
 * On the other side C language forbids declaring array of size zero.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * Application or driver example may not use all symbols that are
 * generated by configurations generator.
 */


/*! @brief User configuration structure 0 */
extern power_manager_user_config_t pwrMan1_InitConfig0;
/*! @brief Count of user configuration structures */
#define POWER_MANAGER_CONFIG_CNT 1U
/*! @brief Array of pointers to User configuration structures */
extern power_manager_user_config_t * powerConfigsArr[];
/*! @brief Count of user Callbacks */
#define POWER_MANAGER_CALLBACK_CNT 0U

/*! @brief Array of pointers to User defined static Callbacks configuration structures */
extern power_manager_callback_user_config_t * powerStaticCallbacksConfigsArr[];


#endif
/* ifndef pwrMan1_H */
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

