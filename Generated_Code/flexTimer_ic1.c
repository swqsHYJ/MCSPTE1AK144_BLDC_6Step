/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : flexTimer_ic1.c
**     Project     : MCSPTE1AK144_BLDC_6Step
**     Processor   : S32K144_100
**     Component   : ftm_ic
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2021-04-29, 14:22, # CodeGen: 0
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
** @file flexTimer_ic1.c
** @version 01.00
*/         
/*!
**  @addtogroup flexTimer_ic1_module flexTimer_ic1 module documentation
**  @{
*/         

/* Module flexTimer_ic1.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver header as external; the header is not included 
 * by this file.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variable will be used in other source files in application code.
 */

#include "flexTimer_ic1.h"



/* Callbacks declaration */
extern void FTM2_Callback(ic_event_t event, void *userData);
/* Channels configuration structure for flexTimer_ic1 input capture */
 ftm_input_ch_param_t flexTimer_ic1_InputCaptureChannelConfig[1] =
{
    {
     1U, /* Channel id */
     FTM_EDGE_DETECT, /* Input capture operation mode */
     FTM_BOTH_EDGES, /* Edge alignment mode */
     FTM_NO_MEASUREMENT, /* Signal measurement operation type */
     0U, /* Filter value */
     false, /* Filter state (enabled/disabled) */
     true, /* Continuous measurement state */
     NULL, /* Vector of callbacks  parameters for channels events */
     FTM2_Callback /* Vector of callbacks for channels events */
    }
};

/* Input capture configuration for flexTimer_ic1 */
 ftm_input_param_t flexTimer_ic1_InputCaptureConfig =
{
     1U, /* Number of channels */
     65535U, /* Max count value */
     flexTimer_ic1_InputCaptureChannelConfig /* Channels configuration */
};

/* Global configuration of flexTimer_ic1 */
ftm_user_config_t  flexTimer_ic1_InitConfig =
{
    {
        true,   /* Software trigger state */
        false,  /* Hardware trigger 1 state */
        false,  /* Hardware trigger 2 state */
        false,  /* Hardware trigger 3 state */
        false, /* Max loading point state */
        false, /* Min loading point state */
        FTM_SYSTEM_CLOCK, /* Update mode for INVCTRL register */
        FTM_SYSTEM_CLOCK, /* Update mode for SWOCTRL register */
        FTM_SYSTEM_CLOCK, /* Update mode for OUTMASK register */
        FTM_SYSTEM_CLOCK, /* Update mode for CNTIN register */
        false, /* Automatic clear of the trigger*/
        FTM_UPDATE_NOW, /* Synchronization point */
    },
     FTM_MODE_INPUT_CAPTURE, /* Mode of operation for FTM */
     FTM_CLOCK_DIVID_BY_128, /* FTM clock prescaler */
     FTM_CLOCK_SOURCE_SYSTEMCLK,   /* FTM clock source */
     FTM_BDM_MODE_00, /* FTM debug mode */
     false, /* Interrupt state */
     true /* Initialization trigger */
};

/* END flexTimer_ic1. */

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

