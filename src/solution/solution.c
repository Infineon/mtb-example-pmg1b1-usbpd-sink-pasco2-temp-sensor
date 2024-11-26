/**
 * @file solution.c
 *
 * @brief @{Solution source file solution layer port.@}
 *
 *******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

#include "config.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_idac_ctrl.h"
#include "cy_usbpd_phy.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_buck_boost.h"

#include "psink.h"
#include "app.h"


#if DEBUG_UART_ENABLE
#include "debug.h"
static char temp[80];
#endif

/* Filter value for Battery UVOV comparators */
#define BAT_UVOV_FILTER                         (0x0Au)

extern cy_stc_pdstack_context_t * get_pdstack_context(uint8_t portIdx);
extern bool app_psrc_vbus_ocp_cbk(void * callbackCtx, bool compOut);
#if ENABLE_PASCO2_I2C_INTERFACE
bool is_12V_available = false;
#endif /* ENABLE_PASCO2_I2C_INTERFACE */

batt_chg_state_t gl_sln_batt_chg_state = BATT_CHG_IDLE;

batt_chg_alt_state_t gl_sln_batt_chg_alt_state = BATT_CHG_ALT_LOOK4BATTERY;

static void soln_batt_chge_cmd(cy_stc_pdstack_context_t *ptrPdStackContext, batt_chge_src_cmd_t src_cmd);

static void sln_ibtr_cb(void * callbackCtx, bool value);

void solution_init (cy_stc_pdstack_context_t *ptrPdStackContext)
{
    cy_stc_battery_charging_context_t * ptrBatteryContext = get_battery_charging_context(ptrPdStackContext->port);
    ptrBatteryContext->ptrPdStack = ptrPdStackContext;

    reset_battery_status(ptrBatteryContext);
}

#if BAT_HW_OVP_ENABLE
static bool soln_vbat_ovp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    soln_batt_chgr_hw_disable(ptrPdStackContext);

    DEBUG_PRINT("HW_BAT_OVP_DET\n");
    batt_stat->batt_ovp_fault_active = true;
    return true;
}
#endif /* BAT_HW_OVP_ENABLE */

#if BAT_HW_UVP_ENABLE
static bool soln_vbat_uvp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    soln_batt_chgr_hw_disable(ptrPdStackContext);

    DEBUG_PRINT("HW_BAT_UVP_DET\n");
    batt_stat->batt_uvp_fault_active = true;
    return true;
}
#endif /* BAT_HW_UVP_ENABLE */

#if PDL_VOUTBB_RCP_ENABLE
static bool soln_voutbb_rcp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    soln_batt_chgr_hw_disable(ptrPdStackContext);

    batt_stat->batt_rcp_fault_active = true;


    Cy_USBPD_Fault_Voutbb_RcpDisable(ptrUsbPdContext);
    DEBUG_PRINT("HW_VOUTBB_RCP_INTR\n");
    return true;
}
#endif /* PDL_VOUTBB_RCP_ENABLE */

#if BAT_HW_OCP_ENABLE
static bool soln_vbat_ocp_cbk(void *callbackContext, bool state)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *) callbackContext;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *) ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    soln_batt_chgr_hw_disable(ptrPdStackContext);

    batt_stat->batt_ocp_fault_active = true;
    Cy_USBPD_Fault_Vbat_OcpDisable(ptrUsbPdContext, CCG_SRC_FET);
    DEBUG_PRINT("HW_BAT_OCP_INTR\n");
    return true;
}
#endif /* BAT_HW_OCP_ENABLE */

static void sln_vbtr_cb(void * callbackCtx, bool value)
{
    (void)value;

    /* VBTR transition is completed. */
    if(gl_sln_batt_chg_state == BATT_CHG_BB_SET_VBAT_WAIT)
    {
        gl_sln_batt_chg_state = BATT_CHG_BB_SET_IBAT;
    }
}

void sln_set_volt(cy_stc_usbpd_context_t *context, uint16_t voltmv)
{
    int16_t new_dac, cur_dac;
    uint32_t state;

    Cy_USBPD_Hal_Enable_CV(context);

#if VBUS_CTRL_TRIM_ADJUST_ENABLE
    new_dac = Cy_USBPD_Vbus_GetTrimIdac(context, voltmv);
#else
    new_dac = (volt_mv - CY_PD_VSAFE_5V) / 20u;
#endif
    state  = Cy_SysLib_EnterCriticalSection();

    cur_dac = Cy_USBPD_Hal_Get_Fb_Dac(context);

    /* Configure VBTR operation */
    Cy_USBPD_VBTR_Config(context, cur_dac, new_dac, sln_vbtr_cb);

    /* Start VBTR operation */
    Cy_USBPD_VBTR_Start(context);

    Cy_SysLib_ExitCriticalSection(state);
}

/* Update the Buck boost output voltage and current based on the USB-C Input power and current battery voltage */
bool is_input_power_sufficient (cy_stc_pdstack_context_t* ptrPdStackContext)
{
    uint32_t calc_input_power;

    calc_input_power = (ptrPdStackContext->dpmStat.contract.minVolt * ptrPdStackContext->dpmStat.contract.curPwr) / 100;

    if(calc_input_power >= MIN_USBC_VBUS_TOTAL_POWER)
    {
        DEBUG_PRINT("\n\r Input Power sufficient \n");
        return true;
    }
    else
    {
        DEBUG_PRINT("\n\r Input Power NOT sufficient \n");
        return false;
    }

}

/* Battery Charger Solution PD event handler */
void batt_pd_event_handler(cy_stc_pdstack_context_t* ptrPdStackContext, cy_en_pdstack_app_evt_t evt, const void *data)
{
    const cy_stc_pdstack_pd_contract_info_t* contract_status;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
            DEBUG_PRINT("\n\r TypeC Start \n");
            break;
        case APP_EVT_CONNECT:
#if VREG_INRUSH_DET_ENABLE
            Cy_USBPD_Fault_VregInrushDetEn(ptrPdStackContext->ptrUsbPdContext);
#endif /* VREG_INRUSH_DET_ENABLE */
#if VREG_BROWN_OUT_DET_ENABLE
            if(ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK)
            {
                sol_brown_out_control(ptrPdStackContext, true);
            }
#endif /* VREG_BROWN_OUT_DET_ENABLE*/
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\n\r Connect ");
#endif
            /* Run charger state machine with SOLN_BATT_MONITOR_TASK_TIMER_PERIOD. */
            (void)Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID,
                    SOLN_BATT_MONITOR_TASK_TIMER_PERIOD);

            {
                gl_sln_batt_chg_state = BATT_CHG_INIT;
            }
            break;
        
        case APP_EVT_PE_DISABLED:
            /* Intentional fall-through. */
        case APP_EVT_HARD_RESET_SENT:                       /* Intentional fall through */
        case APP_EVT_HARD_RESET_RCVD:                       /* Intentional fall through */
        case APP_EVT_VBUS_PORT_DISABLE:                     /* Intentional fall through */
        case APP_EVT_DISCONNECT:                            /* Intentional fall through */
        case APP_EVT_TYPE_C_ERROR_RECOVERY:                 /* Intentional fall through */
        case APP_EVT_VBUS_UVP_FAULT:
        case APP_EVT_VBUS_OVP_FAULT:
#if BATTERY_CHARGING_ENABLE
        case APP_EVT_BC_DETECTION_COMPLETED:
#endif

#if VREG_INRUSH_DET_ENABLE
            /* Disable Vreg Inrush protection. */
            Cy_USBPD_Fault_VregInrushDetDis(ptrPdStackContext->ptrUsbPdContext);
#endif /* VREG_INRUSH_DET_ENABLE */
#if VREG_BROWN_OUT_DET_ENABLE
            /* Disable Brown-out detector. */
            sol_brown_out_control(ptrPdStackContext, false);
#endif /* VREG_BROWN_OUT_DET_ENABLE*/
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT_VAR(" Charger event is: %#x \n", (evt));
#endif
            Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID);
            Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID);
            /* battery charging is disabled */
                /* Disable only if charging is active */
                if(gl_sln_batt_chg_state != BATT_CHG_INIT)
                {
                    soln_batt_chgr_hw_disable(ptrPdStackContext);
                }
 
            gl_sln_batt_chg_state = BATT_CHG_INIT;

            /* Clear pending BAT OVP and OCP flags on TypeC disconnect. */
            if(evt == APP_EVT_DISCONNECT)
            {
               clear_flags_on_usbc_disconnect(ptrBatteryChargingContext);
            }
            break;
            
        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE: 
            contract_status = (cy_stc_pdstack_pd_contract_info_t*)data;

            if ((contract_status->status == CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED))
            {
                DEBUG_PRINT_VAR("\n\r Contract volt:: %i mV", ptrPdStackContext->dpmStat.contract.minVolt);
                DEBUG_PRINT_VAR("\n\r Contract Curr:: %i mA\n", (ptrPdStackContext->dpmStat.contract.curPwr *10));
            }
            break;
        default:
            break;
    }
}

static void soln_batt_chge_cmd(cy_stc_pdstack_context_t *ptrPdStackContext, batt_chge_src_cmd_t src_cmd)
{

    switch(src_cmd)
    {
        case BATT_CHGE_BB_EN:
            if(Cy_USBPD_BB_IsEnabled(ptrPdStackContext->ptrUsbPdContext) == false)
            {
                Cy_USBPD_BB_Enable(ptrPdStackContext->ptrUsbPdContext);
#if DEBUG_PWR_INFO_ENABLE
                DEBUG_PRINT( "\n\r BB En\r\n");
#endif
            }
            break;

        case BATT_CHGE_BB_DIS:
            /* Buck Boost is disabled */
#if PDL_VOUTBB_RCP_ENABLE
            Cy_USBPD_Fault_Voutbb_RcpDisable(ptrPdStackContext->ptrUsbPdContext);
#endif /* PDL_VOUTBB_RCP_ENABLE */
            Cy_USBPD_BB_Disable(ptrPdStackContext->ptrUsbPdContext);
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\n\r BB Dis\r\n");
#endif
            break;
        case BATT_CHGE_BB_OUT_EN:
#if BAT_HW_UVP_ENABLE
            Cy_USBPD_Fault_Vbat_UvpEnable(ptrPdStackContext->ptrUsbPdContext, PRIMARY_VBATT_UVP_THRESHOLD, BAT_UVOV_FILTER, soln_vbat_uvp_cbk, CCG_SRC_FET);
#endif /* BAT_HW_UVP_ENABLE */
#if BAT_HW_OVP_ENABLE
            Cy_USBPD_Fault_Vbat_OvpEnable(ptrPdStackContext->ptrUsbPdContext, PRIMARY_VBATT_OVP_THRESHOLD, BAT_UVOV_FILTER, soln_vbat_ovp_cbk, CCG_SRC_FET);
#endif /* BAT_HW_OVP_ENABLE */

            Cy_GPIO_Write(B1_VOUT_DC_EN_H_PORT, B1_VOUT_DC_EN_H_PIN, 1u);
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\n\r BAT Switch ON ..\r\n");
#endif
            break;

        case BATT_CHGE_BB_OUT_DIS:
            /* Note: CCG_SRC_FET is unused in procedure */
#if BAT_HW_UVP_ENABLE
            Cy_USBPD_Fault_Vbat_UvpDisable(ptrPdStackContext->ptrUsbPdContext, CCG_SRC_FET);
#endif /* BAT_HW_UVP_ENABLE */
#if BAT_HW_OVP_ENABLE
            Cy_USBPD_Fault_Vbat_OvpDisable(ptrPdStackContext->ptrUsbPdContext, CCG_SRC_FET);
#endif /* BAT_HW_OVP_ENABLE */
#if BAT_HW_OCP_ENABLE
            Cy_USBPD_Fault_Vbat_OcpDisable(ptrPdStackContext->ptrUsbPdContext, CCG_SRC_FET);
#endif /* BAT_HW_OCP_ENABLE */
            Cy_GPIO_Write(B1_VOUT_DC_EN_H_PORT, B1_VOUT_DC_EN_H_PIN, 0u);
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\n\r BAT Switch OFF ..\r\n");
#endif
            break;

        case BATT_CHGE_BB_IN_EN:
            /* Note: non-functional for SINK_ONLY mode */
            Cy_USBPD_Vbus_GdrvPfetOn(ptrPdStackContext->ptrUsbPdContext, false);
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\n\r InSwON\r\n");
#endif
            break;
            
        case BATT_CHGE_BB_IN_DIS:
            Cy_USBPD_Vbus_GdrvPfetOff(ptrPdStackContext->ptrUsbPdContext, false);
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\n\r InSwOFF\r\n");
#endif
            break;

        default:
            break;
    }
}

void soln_batt_chgr_hw_disable(cy_stc_pdstack_context_t* ptrPdStackContext)
{

    /* Disable Battery Charging */
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_DIS);
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_OUT_DIS);
    soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_IN_DIS);
#if ENABLE_PASCO2_I2C_INTERFACE
    is_12V_available = false;
#endif /* ENABLE_PASCO2_I2C_INTERFACE */
    Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID);
}

void soln_task(cy_stc_pdstack_context_t* ptrPdStackContext)
{
    uint16_t calc_batt_ip_curr;
    uint32_t calc_output_power;
    uint16_t bb_vout;
#if ENABLE_BATT_CURR_MONITORING
    uint32_t batt_cur;
#endif
    uint32_t cur_max_limit;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    if(Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID) == false)
    {
        Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, SOLN_BATT_MONITOR_TASK_TIMER_ID, SOLN_BATT_MONITOR_TASK_TIMER_PERIOD);
    }
    else
    {
        return;
    }

#if ENABLE_NTCP0_TEMP_MONITORING && !ENABLE_PASCO2_I2C_INTERFACE
    measure_onboard_ntcp0_sensor_data(ptrBatteryChargingContext);
#endif
#if ENABLE_NTCP1_TEMP_MONITORING && !ENABLE_PASCO2_I2C_INTERFACE
    measure_onboard_ntcp1_sensor_data(ptrBatteryChargingContext);
#endif

#if DEBUG_UART_ENABLE && DEBUG_PWR_INFO_ENABLE
    /* Report charging FSM state and fault flags */
    uint8_t fault_st = 0;
    if(batt_stat->batt_uvp_fault_active == true)
        fault_st |= 1;
    if(batt_stat->batt_ovp_fault_active == true)
        fault_st |= 2;
    if(batt_stat->batt_ocp_fault_active == true)
        fault_st |= 4;
    if(batt_stat->ntcp0_otp_fault_active == true)
        fault_st |= 0x10;
    if(batt_stat->ntcp1_otp_fault_active == true)
    {
        fault_st |= 0x20;
    }
    if(batt_stat->batt_rcp_fault_active == true)
    {
        fault_st |= 0x40;
    }
    sprintf(temp, "\n\r CHGR FSM STATE %i ALT %i FLTT %x\n",gl_sln_batt_chg_state,gl_sln_batt_chg_alt_state,fault_st);
    debug_print(temp);

#endif
    if((batt_stat->batt_ovp_fault_active == true) ||
            (batt_stat->batt_uvp_fault_active == true) ||
            (batt_stat->ntcp0_otp_fault_active == true) ||
            (batt_stat->ntcp1_otp_fault_active == true) ||
            (batt_stat->batt_rcp_fault_active == true) ||
            (batt_stat->batt_ocp_fault_active == true)
            )
    {
#if DEBUG_PWR_INFO_ENABLE
        DEBUG_PRINT("\r\n >>>Battery is at fault ");
#endif
        /* If Charging continues then disable it. */
        if((gl_sln_batt_chg_state >= BATT_CHG_BB_SET_VBAT) && (gl_sln_batt_chg_state <= BATT_CHG_CHARGE_LOOP))
        {
            soln_batt_chgr_hw_disable(ptrPdStackContext);
            gl_sln_batt_chg_state = BATT_CHG_INIT;
            return;
        }
        /* SNK role failures processing */
        else if(gl_sln_batt_chg_state == BATT_CHG_TYPEC_ATTACHED_SNK)
        {
            gl_sln_batt_chg_state = BATT_CHG_INIT;
            return;
        }
        else if(gl_sln_batt_chg_state == BATT_CHG_INIT)
        {
            /* Note: infinite UVP retries are allowed */
            if( (batt_stat->batt_ocp_fault_active == true) ||
                (batt_stat->batt_rcp_fault_active == true) ||
                (batt_stat->batt_ovp_fault_active == true) ||
                (batt_stat->ntcp0_otp_fault_active == true) ||
                (batt_stat->ntcp1_otp_fault_active == true)
                )
            {
                /* wait here while temp/VbatOV will be back to normal */
                return;
            }
        }
        else
        {
            /* Do nothing */
        }
    }

    switch(gl_sln_batt_chg_state)
    {
    
    /* Initial FSM state. */
    case BATT_CHG_IDLE:  
        gl_sln_batt_chg_state = BATT_CHG_INIT;
        break;

    /* 
     * Check battery presence and wait here for PD contract.
     * Also loop here after Battery OCP/OVP fault. Wait on battery removal
     */
    case BATT_CHG_INIT:
        /* check on Type-C attach */
        if(ptrPdStackContext->dpmConfig.attach == true)
        {
            if(ptrPdStackContext->dpmConfig.contractExist  == true)
            {
                gl_sln_batt_chg_state = BATT_CHG_TYPEC_ATTACHED_SNK;
            #if SIMULATE_ERROR_SNK
                Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, DBG_BATT_TIMER_ID, DBG_BATT_TIMER_PERIOD);
            #endif /* SIMULATE_ERROR_SNK */
            }
            else
            {
                /* Stay here while SNK TypeC-only and BC detection is not finished */
            }
        }
        break;

    case BATT_CHG_TYPEC_ATTACHED_SNK:
        /* Wait on PD contract with sufficient power. */

            if (ptrPdStackContext->dpmConfig.contractExist == false)
            {
                DEBUG_PRINT("\r\n No PD Contract ");
                break;
            }

        /* Check if Battery is present and PD power is sufficient */
        if(is_input_power_sufficient (ptrPdStackContext))
        {
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\r\n pwr_sufficient ");
#endif /* DEBUG_PWR_INFO_ENABLE */
        #if DEBUG_UART_ENABLE && DEBUG_PWR_INFO_ENABLE
            sprintf(temp, "\n\r Contract volt:: %i mV\n", ptrPdStackContext->dpmStat.contract.minVolt);
            debug_print(temp);
            sprintf(temp, "\n\r Contract Curr:: %i mA\n", (ptrPdStackContext->dpmStat.contract.curPwr *10));
            debug_print(temp);
        #endif /* DEBUG_UART_ENABLE && DEBUG_PWR_INFO_ENABLE */

            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_IN_EN);
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_EN);
            gl_sln_batt_chg_state = BATT_CHG_BB_SET_VBAT;
        }
        break;
    
    case BATT_CHG_BB_SET_VBAT:
        /* Set BB output voltage using VBTR block. */
#if DEBUG_PWR_INFO_ENABLE
        DEBUG_PRINT("\r\n CHG_BB_SET_VBAT ");
#endif /* DEBUG_PWR_INFO_ENABLE */
        if(Cy_USBPD_BB_IsReady(ptrPdStackContext->ptrUsbPdContext) == true)
        {
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\r\n BB_IsReady ");
#endif /* DEBUG_PWR_INFO_ENABLE */
        #ifdef BB_RESTART_ON_TIMEOUT_ENABLED
            Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID);
        #endif /* BB_RESTART_ON_TIMEOUT_ENABLED */

            bb_vout = ptrPdStackContext->ptrUsbPdContext->usbpdConfig->buckBoostConfig->bb_output_volt;
            DEBUG_PRINT_VAR("\n\r BB Out Volt:: %d mV \n", bb_vout);
            sln_set_volt(ptrPdStackContext->ptrUsbPdContext,bb_vout);
            gl_sln_batt_chg_state = BATT_CHG_BB_SET_VBAT_WAIT;
        }
        
        #ifdef BB_RESTART_ON_TIMEOUT_ENABLED
        else if(Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID) == false)
        {
            /* Try to do BB_enable every 1 sec */
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_IN_EN);
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_EN);
            batt_stat->bb_out_configure = true;
            Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID, BATT_TIMER_PERIOD);
        }
        #endif /* BB_RESTART_ON_TIMEOUT_ENABLED */
        break;
        /* Wait here on VBTR_DONE interrupt. */

    case BATT_CHG_BB_SET_VBAT_WAIT:
        break;
        /* Set BB output current using IBTR block. */

    case BATT_CHG_BB_SET_IBAT:
        bb_vout = ptrPdStackContext->ptrUsbPdContext->usbpdConfig->buckBoostConfig->bb_output_volt;

        calc_output_power = (uint32_t)((ptrPdStackContext->dpmStat.contract.minVolt * ptrPdStackContext->dpmStat.contract.curPwr)
                                        * INPUT_OUTPUT_EFFICIENCY_REDUCE_PERCENTAGE) / 100 ;

        calc_batt_ip_curr = calc_output_power / bb_vout;
#if DEBUG_PWR_INFO_ENABLE
        DEBUG_PRINT_VAR("\r BB Out Curr:: %d mA\n", (calc_batt_ip_curr * 10));
#endif /* DEBUG_PWR_INFO_ENABLE */

        /* Calculate max current limit for the actual Rsense */
        cur_max_limit = ((VBAT_INPUT_CURR_MAX_SETTING * CSA_IDEAL_RSENSE) / ptrPdStackContext->ptrUsbPdContext->vbusCsaRsense);

        if(calc_batt_ip_curr > cur_max_limit)
        {
            calc_batt_ip_curr = cur_max_limit;
        }

        if(calc_batt_ip_curr < MIN_IBAT_CHARGING_CURR)
        {
            calc_batt_ip_curr = MIN_IBAT_CHARGING_CURR;
        }

        DEBUG_PRINT_VAR("\r BB Out Curr:: %d mA\n", (calc_batt_ip_curr * 10));

        ptrPdStackContext->ptrUsbPdContext->ibtrCbk = sln_ibtr_cb;
        Cy_USBPD_CF_Enable(ptrPdStackContext->ptrUsbPdContext, calc_batt_ip_curr);
        gl_sln_batt_chg_state = BATT_CHG_ENA_BAT_FET;
        Cy_PdUtils_SwTimer_StartWocb(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID, BATT_TIMER_PERIOD);
        break;
        /* Enable Battery FET after BATT_TIMER_PERIOD expiration. */

    case BATT_CHG_ENA_BAT_FET:
        if(Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, BATT_TIMER_ID) == false)
        {
#if DEBUG_PWR_INFO_ENABLE
            DEBUG_PRINT("\r\n Battery is not at fault and ready to charge ");
#endif /* DEBUG_PWR_INFO_ENABLE */
            soln_batt_chge_cmd(ptrPdStackContext, BATT_CHGE_BB_OUT_EN);
#if PDL_VOUTBB_RCP_ENABLE
            gl_sln_batt_chg_state = BATT_CHG_ENA_RCP_DELAY;
#else
            gl_sln_batt_chg_state = BATT_CHG_CHARGE_LOOP;
#endif
#if ENABLE_PASCO2_I2C_INTERFACE
            is_12V_available = true;
#endif /* ENABLE_PASCO2_I2C_INTERFACE */
#if VBUS_SNK_UVP_ENABLE
                /* In SNK mode BB block discharges disconnected VBUS too fast causing
                 * false UVP triggering before true TypeC Detach detection.
                 * So we need to disable SNK-mode VBUS UVP .
                 * Its functionality anyway is duplicated by Brownout detector and Sink Detach detection procedure in TypeC machine
                 * UVP will be still enabled in SRC mode.
                 */

                if(ptrPdStackContext->dpmConfig.connect)
                {
                    app_status_t* app_stat = app_get_status(ptrPdStackContext->port);
                    app_uvp_enable(ptrPdStackContext, app_stat->psnk_volt, false, app_psnk_vbus_uvp_cbk);
                    DEBUG_PRINT("UVP configured\n");
                }
#endif /* VBUS_UVP_ENABLE */
            }
            break;
#if PDL_VOUTBB_RCP_ENABLE
    /* Delay RCP Enable for 2 cycles of Charger FSM (400mS) to avoif false triggering. */
    case BATT_CHG_ENA_RCP_DELAY:
        gl_sln_batt_chg_state = BATT_CHG_ENA_RCP_DELAY1;
            break;
        case BATT_CHG_ENA_RCP_DELAY1:
            Cy_USBPD_Fault_Voutbb_RcpEnable(ptrPdStackContext->ptrUsbPdContext, soln_voutbb_rcp_cbk);
                gl_sln_batt_chg_state = BATT_CHG_CHARGE_LOOP;
                break;
#endif /* PDL_VOUTBB_RCP_ENABLE */
        case BATT_CHG_CHARGE_LOOP:

#if ENABLE_BATT_CURR_MONITORING
            /* Monitor current in SNK/charging mode */
            batt_cur = Cy_USBPD_Hal_MeasureCur(ptrPdStackContext->ptrUsbPdContext);
            DEBUG_PRINT_VAR("\n Measured IBAT: %u mA", (unsigned int)(batt_cur * 10));
#endif


            break;
        case BATT_CHG_END_OF_CHARGE:
            /* Loop here when battery is full. */
            break;

        default:
            break;
    }
}

#if VREG_BROWN_OUT_DET_ENABLE
/* Function using to control Brown Out buck-boost comparator and VDDD detector.
 * Shall be enable when VDDD is already 5V and disabled when VDDD=3V.
 */
void sol_brown_out_control(cy_stc_pdstack_context_t * context, bool enable)
{
    PPDSS_REGS_T pd = context->ptrUsbPdContext->base;
    if(enable)
    {
        pd->bbctrl_func_ctrl3 &= ~PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VDDD_BOD;
        pd->bb_40vreg_ctrl |= PDSS_BB_40VREG_CTRL_BB_40VREG_EN_VDDD_DET_COMP;
        Cy_USBPD_Fault_BrownOutDetEn(context->ptrUsbPdContext);
        Cy_SysLib_DelayUs(10);

        Cy_USBPD_Adc_SelectVref(context->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID, CY_USBPD_ADC_VREF_VDDD);
        uint16_t vddd = Cy_USBPD_Adc_Calibrate(context->ptrUsbPdContext, APP_GPIO_POLL_ADC_ID);
        if(vddd < 3500)
        {
            /* In this place VBUS is already 5V, but VDDD is less 3.5V.
             * This means something is loading VDDD and this event was missed
             * during BrownOut detection.
             */
            pd->intr17_set |= PDSS_INTR17_SET_PDBB_VREG_VDDD_BOD;
        }
    }
    else
    {
        Cy_USBPD_Fault_BrownOutDetDis(context->ptrUsbPdContext);
        pd->bb_40vreg_ctrl &= ~PDSS_BB_40VREG_CTRL_BB_40VREG_EN_VDDD_DET_COMP;
        pd->bbctrl_func_ctrl3 |= PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VDDD_BOD;
    }
}
#endif /* VREG_BROWN_OUT_DET_ENABLE*/

static void sln_ibtr_cb(void * callbackCtx, bool value)
{
    (void)value;
#if BAT_HW_OCP_ENABLE
    cy_stc_usbpd_context_t *ptrUsbPdContext = (cy_stc_usbpd_context_t *)callbackCtx;
    cy_stc_pdstack_context_t * ptrPdStackContext = (cy_stc_pdstack_context_t *)ptrUsbPdContext->pdStackContext;
    cy_stc_battery_charging_context_t* ptrBatteryChargingContext = get_battery_charging_context(ptrPdStackContext->port);
    cy_stc_battery_status_t* batt_stat = &(ptrBatteryChargingContext->batteryStatus);

    /* Set OCP level to 1A for low output current, because of high current monitor error when I < 1A */
    if(batt_stat->cur_bb_iout <= CY_USBPD_I_1A)
    {
        Cy_USBPD_Fault_Vbat_OcpEnable(ptrUsbPdContext, CY_USBPD_I_1A, soln_vbat_ocp_cbk);
    }
    else
    {
        Cy_USBPD_Fault_Vbat_OcpEnable(ptrUsbPdContext, batt_stat->cur_bb_iout, soln_vbat_ocp_cbk);
    }
#else
    (void)callbackCtx;
#endif /* BAT_HW_OCP_ENABLE */
}

/* [] END OF FILE */
