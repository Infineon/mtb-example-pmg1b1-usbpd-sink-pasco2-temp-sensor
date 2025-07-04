/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PMG1B1 MCU: USBPD Sink PAS CO2
*              and Temperature Sensor Example for ModusToolbox.
*
* Related Document: See README.md
*
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#if (!CY_PD_SINK_ONLY)
#include "psource.h"
#endif  /* (!CY_PD_SINK_ONLY) */
#include "swap.h"
#include "vdm.h"
#include "cy_usbpd_buck_boost.h"
#include "mtbcfg_ezpd.h"
#include "i2c_master.h"

#if (TEMPERATURE_SENSOR_COUNT != 0u)
#include "thermistor.h"
#endif /* (TEMPERATURE_SENSOR_COUNT != 0u) */

#if DEBUG_UART_ENABLE
#include "debug.h"
static char temp[80];
#endif

#if ENABLE_PASCO2_I2C_INTERFACE

/* PAS CO2 sensor macros */
#define WAIT_SENSOR_RDY_MS                          (2000U)     /* Wait time in milliseconds for the sensor to be ready */
#define DEFAULT_PRESSURE_REF_HPA                    (0x3F7)     /* Default atmospheric pressure to compensate for (hPa) */
#define PASCO2_INIT_RETRY_COUNT                     (3u)
#define TIMER_PERIOD_MSEC                           (10000U)    /* Sensor Measurement time period in milliseconds (Minimum value is 10000U) */
#define TIMER_OFFSET_MSEC                           (499U)      /* This offset is added to the timer period to avoid conflict between record
                                                                 * and read of PAS CO2 sensor measurements. The XENSIV PAS CO2 sensor records
                                                                 * CO2 data every 10 seconds */
volatile bool pasco2_sensor_read_flag;                          /* Read flag for PAS CO2 sensor */
volatile bool pasco2_sensor_read_flag = false;
volatile bool pasco2_sensor_init_flag = false;                  /* Init flag for PAS CO2 sensor */
volatile bool is_pasco2_initialized = false;                    /* Flag for PAS CO2 sensor initialization status*/
uint8_t init_retry_counter = PASCO2_INIT_RETRY_COUNT;           /* Retry counter for PAS CO2 sensor initialization*/
volatile uint32_t pasco2_current_timer_val;                     /* Variable to store current timer value*/
static xensiv_pasco2_t xensiv_pasco2;

#endif /* ENABLE_PASCO2_I2C_INTERFACE */

cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;

cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

/* Do not change max_current value to avoid 20CSA saturation. */
cy_stc_auto_cfg_settings_t usbpd_port0_auto_config =
{
    .max_current = 500,                /**<  Maximum current that can be sourced in steps of 10 mA units. */
};

/* BB VOUT OCP configuration. */
cy_stc_fault_vbat_ocp_cfg_t bat_ocp_config =
{
    .mode = 3,                       /**< Internal OCP with software debounce. */
    .threshold = 30,                 /**< OCP threshold (in percents over nominal current). */
    .debounce = 20,                  /**< software debounce time (in milliseconds). */
};

cy_stc_pdutils_sw_timer_t *Cy_PdUtils_SwTimer_GetContext(void)
{
    return &gl_TimerCtx;
}

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

cy_stc_pdstack_context_t *gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
    &gl_PdStackPort0Ctx,
};

bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT)
const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT) */

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t *ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    batt_pd_event_handler(ctx, evt, data);

    /* Nothing to do at present. */
    (void)ctx;
    (void)evt;
    (void)data;
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT)
static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler(&(gl_TimerCtx));
}
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT) */

static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_PD_ILO)
    if(0u != (gl_UsbPdPort0Ctx.base->intr1_masked & PDSS_INTR1_MASKED_LF_CNTR_MASKED))
    {
        gl_UsbPdPort0Ctx.base->intr1 = PDSS_INTR1_LF_CNTR_MATCH;

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
        /* Load the timer match register. */
        Cy_LF_SetMatch(&gl_UsbPdPort0Ctx, (Cy_LF_GetCount() + gl_TimerCtx.multiplier));
#endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0) */

        Cy_PdUtils_SwTimer_InterruptHandler(&gl_TimerCtx);
    }
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_PD_ILO) */
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

cy_stc_pd_dpm_config_t *get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    app_event_handler,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
   vbus_get_value,
#if CY_PD_USB4_SUPPORT_ENABLE
        NULL,
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
};
#if (TEMPERATURE_SENSOR_COUNT != 0u)
/* Array to map the thermal voltage read by thermistor to corresponding temperature */
static const uint16_t gl_thermistor_volt_map[VOLT_TO_TEMP_MAP_TABLE_COUNT] =
{
    /* This voltage-temperature mapping is based on NTCG164KF104FTDS NTC thermistor
     * If a different thermistor is to be used, the table has to be updated.
     */
    4848, /* Corresponds to 20 degree C */
    4806, /* Corresponds to 25 degree C */
    4756, /* Corresponds to 30 degree C */
    4696, /* Corresponds to 35 degree C */
    4623, /* Corresponds to 40 degree C */
    4538, /* Corresponds to 45 degree C */
    4438, /* Corresponds to 50 degree C */
    4324, /* Corresponds to 55 degree C */
    4194, /* Corresponds to 60 degree C */
    4049, /* Corresponds to 65 degree C */
    3889, /* Corresponds to 70 degree C */
    3715, /* Corresponds to 75 degree C */
    3529, /* Corresponds to 80 degree C */
    3334, /* Corresponds to 85 degree C */
    3131, /* Corresponds to 90 degree C */
    2925, /* Corresponds to 95 degree C */
    2718, /* Corresponds to 100 degree C */
    2513, /* Corresponds to 105 degree C */
    2312, /* Corresponds to 110 degree C */
    2118  /* Corresponds to 115 degree C */
};
#endif /* (TEMPERATURE_SENSOR_COUNT != 0u) */

#if ENABLE_PASCO2_I2C_INTERFACE
/* This function handles the timer interrupt that set the flag for the sensor to be read.*/
void timer_interrupt_handler(void)
{
    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC );

    if (pasco2_current_timer_val == TIMER_PERIOD_MSEC)
    {
        pasco2_sensor_read_flag = true;
    }
    else if (pasco2_current_timer_val == WAIT_SENSOR_RDY_MS)
    {
        pasco2_sensor_init_flag = true;
    }
}

/* This function handles the initialization of the Timer modules used to measure the temperature data periodically */
void init_sensor_timer_module (uint32_t timer_period)
{
    /* Start the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not. It is not used
     * here for simplicity.
     */
    cy_rslt_t result;

    result = Cy_TCPWM_Counter_Init(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, &CYBSP_TIMER_config);
    if (result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Set the timer period in milliseconds. To count N cycles, period should be
     * set to N-1.
     */
    Cy_TCPWM_Counter_SetPeriod(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, timer_period+TIMER_OFFSET_MSEC);

    /* Check if the desired interrupt is enabled prior to triggering */
    if (0UL != (CY_TCPWM_INT_ON_TC & Cy_TCPWM_GetInterruptMask(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
    {
       Cy_TCPWM_SetInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC);
    }

     /* Clear any pending interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_CC_OR_TC);

    /* Enable the Timer Module */
    Cy_TCPWM_Counter_Enable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);

    /* Trigger a software start on the counter instance. This is required when
     * no other hardware input signal is connected to the component to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(CYBSP_TIMER_HW, CYBSP_TIMER_MASK);

}

/* Below function disables the Timer module */
void deinit_sensor_timer_module(void)
{
    Cy_TCPWM_Counter_Disable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);
}
#endif /* ENABLE_PASCO2_I2C_INTERFACE */

#if APP_FW_LED_ENABLE
/* LED blink rate in milliseconds */
static uint16_t gl_LedBlinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
#endif

#if APP_FW_LED_ENABLE
void led_timer_cb(
    cy_timer_id_t id,      /**< Timer ID for which callback is being generated. */
    void *callbackContext) /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SOURCE)
        {
            /* Turn ON the User LED. */
            Cy_GPIO_Clr(USER_STATUS_LED_PORT, USER_STATUS_LED_PIN);
        }
        else
        {
            /* Toggle the User LED and re-start timer to schedule the next toggle event. */
            Cy_GPIO_Inv(USER_STATUS_LED_PORT, USER_STATUS_LED_PIN);
        }
    }
    else
    {
        /* Turn OFF the User LED. */
        Cy_GPIO_Set(USER_STATUS_LED_PORT, USER_STATUS_LED_PIN);
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, gl_LedBlinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/*
 * Solution level callback functions for the application layer
 */
const app_sln_handler_t sln_cbk =
{
    sln_pd_event_handler,
    app_get_callback_ptr,
    get_pdstack_context,
    mux_ctrl_init,
    NULL
};

int main(void)
{
    /* Variables to store CO2 level data and debug messages*/
    uint16_t ppm;
    char_t co2_string[50];
    char_t fail_string[60];

    cy_rslt_t result;
    cy_en_sysint_status_t intr_result;
    cy_stc_pdutils_timer_config_t timerConfig;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT)
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);
#endif /* (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_WDT) */

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
#if (CY_PDUTILS_TIMER_TYPE == CY_PDUTILS_TIMER_TYPE_PD_ILO)
    timerConfig.hw_timer_ctx = &gl_UsbPdPort0Ctx;
#else
    timerConfig.hw_timer_ctx = NULL;
#endif

    /* Enable global interrupts */
    __enable_irq();

#if DEBUG_UART_ENABLE
    debug_init();

    /* Sequence to clear screen */
    DEBUG_PRINT("\x1b[2J\x1b[;H");

    /* Print "CO2 Sensor" */
    DEBUG_PRINT("******************************\n");
    DEBUG_PRINT("\n\rPAS CO2 and Temperature Sensor\n");
    DEBUG_PRINT("\n\r****************************** \r\n");
#endif
#if ENABLE_PASCO2_I2C_INTERFACE
    /* Enable TCPWM interrupt */
    cy_stc_sysint_t intrCfg =
    {
       /*.intrSrc =*/ CYBSP_TIMER_IRQ, /* Interrupt source is Timer interrupt */
       /*.intrPriority =*/ 3UL   /* Interrupt priority is 3 */
    };
    intr_result = Cy_SysInt_Init(&intrCfg, timer_interrupt_handler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    /* Enable Timer Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);
    /* Initialize I2C master SCB */
    result = InitI2CMaster();
#if DEBUG_UART_ENABLE
    if (result != I2C_SUCCESS)
    {
        DEBUG_PRINT("\n\r I2C Master Init Failed \n" );
    }
#endif
#endif /* ENABLE_PASCO2_I2C_INTERFACE */

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

#if (TEMPERATURE_SENSOR_COUNT != 0u)
    register_thermistor_mapping_table(&gl_PdStackPort0Ctx, (const uint16_t *)gl_thermistor_volt_map);
#endif
    /* Register callbacks for solution level functions */
    register_soln_function_handler(&gl_PdStackPort0Ctx, (app_sln_handler_t *)&sln_cbk);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

    /* Init Auto Config param. Only max_current field is used by PDL to set CSA TRIMS and CC_GAIN.  */
    mtb_usbpd_port0_config.autoConfig = &usbpd_port0_auto_config;
    /* Initialize the USBPD driver */
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    gl_UsbPdPort0Ctx.vbusCsaRsense = gl_UsbPdPort0Ctx.usbpdConfig->buckBoostConfig->current_sense_res;
    gl_UsbPdPort0Ctx.peak_current_sense_resistor = gl_UsbPdPort0Ctx.usbpdConfig->buckBoostConfig->peak_current_sense_resistor;
    /* Set pointer to Battery OCP context */
#if (BAT_HW_OCP_ENABLE != 0u)
    gl_UsbPdPort0Ctx.usbpdConfig->vbatOcpConfig = &bat_ocp_config;
#endif /* (BAT_HW_OCP_ENABLE != 0u) */
    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                        &gl_UsbPdPort0Ctx,
                        &mtb_usbpd_port0_pdstack_config,
                        app_get_callback_ptr(&gl_PdStackPort0Ctx),
                        &pdstack_port0_dpm_params,
                        &gl_TimerCtx);

    /* Initialize Buck boost controller */
    Cy_USBPD_BB_Init(&gl_UsbPdPort0Ctx);

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&gl_PdStackPort0Ctx);

    /* Register solution space fault handler callbacks */
    fault_handler_register_cbks(&gl_PdStackPort0Ctx);

    /* Perform application level initialization. */
    app_init(&gl_PdStackPort0Ctx);

    sln_pd_event_handler(&gl_PdStackPort0Ctx, APP_EVT_POWER_CYCLE, NULL);

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);

#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED_TIMER_ID,
                             gl_LedBlinkRate, led_timer_cb);
#endif /* APP_FW_LED_ENABLE */

    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     * Since this application does not have any other function, the PMG1 device can be placed in "deep sleep"
     * mode for power saving whenever the PD stack and drivers are idle.
     */


    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
        
        /* Perform any application level tasks. */
        app_task(&gl_PdStackPort0Ctx);

#if ENABLE_PASCO2_I2C_INTERFACE
        if (is_12V_available == true)
        {
            if (is_pasco2_initialized == false)
            {
                /* Check the initialization counter for the PAS CO2 Sensor */
                if (init_retry_counter > 0)
                {
                    /* Initializing the timer module to start timer for WAIT_SENSOR_RDY_MS
                     * to wait for the sensor to be ready*/
                    if (0UL == (CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING &
                                    Cy_TCPWM_Counter_GetStatus(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
                    {
                        pasco2_current_timer_val = WAIT_SENSOR_RDY_MS;
                        init_sensor_timer_module(pasco2_current_timer_val);
                        pasco2_sensor_init_flag = false;
                    }
                    if (pasco2_sensor_init_flag == true)
                    {
                        /* Initialize the PAS CO2 Sensor */
                        result = xensiv_pasco2_my_init_i2c(&xensiv_pasco2);
                        pasco2_sensor_init_flag = false;

                        /* Check if the sensor initialization was successful */
                        if (result == CY_RSLT_SUCCESS)
                        {
                            is_pasco2_initialized = true;

                            /* Deinitializing the timer module to stop timer after successful initialization of sensor*/
                            deinit_sensor_timer_module();

                    #if DEBUG_UART_ENABLE
                            /* Print Debug messages over UART. */
                            DEBUG_PRINT("\n\r PAS CO2 device initialization successful \n" );
                    #endif

                            /* Initializing the timer module to start timer for TIMER_PERIOD_MSEC for periodic
                            * CO2 and temperature data measurements*/
                            if (0UL == (CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING &
                                    Cy_TCPWM_Counter_GetStatus(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
                            {
                                pasco2_current_timer_val = TIMER_PERIOD_MSEC;
                                init_sensor_timer_module(pasco2_current_timer_val);
                                pasco2_sensor_read_flag = false;
                            }
                        }
                        else
                        {
                            is_pasco2_initialized = false;
                            init_retry_counter--;
                            if (init_retry_counter == 0)
                            {
                            #if DEBUG_UART_ENABLE
                                /* Print Debug messages over UART. */
                                sprintf(fail_string, "\n\r PAS CO2 device initialization failed after %d attempts \n", PASCO2_INIT_RETRY_COUNT);
                                debug_print(fail_string);
                            #endif
                            }
                        }
                    }
                }
            }


            if ((pasco2_sensor_read_flag == true) && (is_pasco2_initialized == true))
            {
                /* Read the CO2 sensor data */
                result = xensiv_pasco2_my_read(&xensiv_pasco2, DEFAULT_PRESSURE_REF_HPA, &ppm);

                if (result == CY_RSLT_SUCCESS)
                {
                    /* Conversion from int to char for UART transmit */
                    sprintf(co2_string,"\n\r CO2: %d ppm", ppm);

                    /* Send a string over serial terminal */
#if DEBUG_UART_ENABLE
                    debug_print(co2_string);
                    /* Read and print the temperature data */
                    measure_onboard_ntcp0_sensor_data(get_battery_charging_context((&gl_PdStackPort0Ctx)->port));
                    measure_onboard_ntcp1_sensor_data(get_battery_charging_context((&gl_PdStackPort0Ctx)->port));
#endif
                }
                /* Set the read flag to false to wait till timer expires again */
                pasco2_sensor_read_flag = false;
            }
        }
#endif /* ENABLE_PASCO2_I2C_INTERFACE */

        /* Perform tasks associated with instrumentation. */
        instrumentation_task();

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
      if (is_12V_available == false)
      {
          if(gl_PdStackPort0Ctx.dpmConfig.connect == false)
            system_sleep(&gl_PdStackPort0Ctx,NULL);
      }
#endif /* SYS_DEEPSLEEP_ENABLE */

#if (CY_PDUTILS_TIMER_TYPE != CY_PDUTILS_TIMER_TYPE_WDT)
        /* Clears the WatchDog to prevent device reset */
        Cy_WDT_ClearWatchdog();
#endif /* (CY_PDUTILS_TIMER_TYPE != CY_PDUTILS_TIMER_TYPE_WDT) */
    }
}

/* [] END OF FILE */
