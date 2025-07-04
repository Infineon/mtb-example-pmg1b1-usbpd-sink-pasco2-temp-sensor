/******************************************************************************
* File Name:   debug.c
*
* Description: This file implements uart based debugging functionality.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cybsp.h"
#include "cycfg_peripherals.h"
#include "debug.h"
#include "config.h"
#include "cy_scb_uart.h"

#if DEBUG_UART_ENABLE

cy_stc_scb_uart_context_t UART_context;

cy_stc_syspm_callback_params_t callback_params =
{
    .base = UART_HW,
    .context = &UART_context
};

cy_stc_syspm_callback_t uart_deep_sleep_cb =
{
    Cy_SCB_UART_DeepSleepCallback,
    CY_SYSPM_DEEPSLEEP,
    0,
    &callback_params,
    NULL,
    NULL
};

void debug_init(void)
{
#if SW_UART_USED
    /* Configure and enable the UART peripheral */
    SW_Tx_UART_Start();

#else
    Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    Cy_SysPm_RegisterCallback(&uart_deep_sleep_cb);
    Cy_SCB_UART_Enable(UART_HW);
#endif
}

void debug_print(const char* string)
{
#if SW_UART_USED
    /* Send a string over serial terminal */
    SW_Tx_UART_PutString(string);
#else
    /* Send a string over serial terminal */
    Cy_SCB_UART_PutString(UART_HW, string);
#endif
}

#endif /* DEBUG_UART_ENABLE */
