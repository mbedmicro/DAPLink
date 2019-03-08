/*******************************************************************************
* @file    target_reset.c
* @brief   Target reset for the PSoC6 target
*
********************************************************************************
* Copyright (2019) Cypress Semiconductor Corporation
* or a subsidiary of Cypress Semiconductor Corporation.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may
* not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*****************************************************************************/

#include "swd_host.h"
#include "swd.h"
#include "cyfitter.h"
#include "PSOC6xxx.h"
#include "target_config.h"
#include "target_family.h"
#include "IO_Config.h"
#include "DAP_config.h"

#define MAX_AP                  (2u)
#define SYS_AP                  (0u << 24u)
#define CM0_AP                  (1u << 24u)
#define CM4_AP                  (2u << 24u)
#define CPU_ID                  (0xE000ED00u)
#define CPUSS_CM4_PWR_CTL       (0x40210080u)
#define READY_VAL_SEQ           (0x12344321u)
#define ACQUIRE_WAIT_MS         (15000u)
#define ASQUIRE_CHK_INTERVAL_MS (10u)
#define CSW_ACCESS_MASK         (0x23000052u)
#define CM4_PWR_MASK            (0x00000003u)
#define CM4_PWR_ON_VALUE        (0x05fa0003u)
#define CM4_PWR_OFF_VALUE       (0x05fa0000u)
#define TEST_MODE_REGISTER      (0x40260100u)
#define TEST_MODE_VALUE         (0x80000000u)

volatile bool isTargetAcquired;

__inline void SetDAPDefaultIdleCycle(void)
{
    // per IHI0031C transfer recommendation
    DAP_Data.transfer.idle_cycles = 8U;
}
static void SetCustomDAPDefaults(void)
{
    // PSoC6 defaults
    DAP_Data.fast_clock  = 1U;    
    DAP_Data.transfer.idle_cycles = 6U;
}

static void target_before_init_debug(void)
{
    // any target specific sequences needed before attaching
    //	to the DAP across JTAG or SWD
    if (!isTargetAcquired)
    {
        if (kit_has_sflash_restriction())
        {
            // secure flow - implies that the target will be acquired in any case
            isTargetAcquired = true;
            bool ActiveAPFound = false;
            bool ReadyValFound = false;
            uint32_t readyValAddr = get_readyval_addr();
            
            SWDAcquirePSoC6BLE(NOTESTBIT_ACQUIRE, 1);
            for (uint32_t i = 0; !ReadyValFound && (i < ACQUIRE_WAIT_MS); i+=ASQUIRE_CHK_INTERVAL_MS)
            {
                // find active AP loop
                if (!ActiveAPFound)
                {
                    for (uint32_t ap_index = 1; ap_index <= MAX_AP; ap_index++)
                    {
                        g_target_family_psoc6.apsel = ap_index << 24u;
                        if (swd_write_dp(DP_SELECT, g_target_family_psoc6.apsel) &&
                            swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ) &&
                            swd_write_word(TEST_MODE_REGISTER, TEST_MODE_VALUE))
                        {
                            ActiveAPFound = true;
                            break;
                        }
                        else
                        {
                            JTAG2SWD();
                            swd_clear_errors();
                        }
                    }
                }
                
                if (ActiveAPFound && !ReadyValFound)
                {
                    uint32_t ready_val = 0;
                    if (swd_read_word(readyValAddr, &ready_val))
                    {    
                        if (ready_val == READY_VAL_SEQ)
                        {
                            // Target acquired by the normal way
                            ReadyValFound = true;
                            break;
                        }
                    }
                    else
                    {
                        swd_clear_errors();
                    }
                }
                
                CyDelay(ASQUIRE_CHK_INTERVAL_MS);
            }
            
            if (!ActiveAPFound)
            {
                // out of band AP selection if timeout reached
                uint32_t cpu_id = 0;
                g_target_family_psoc6.apsel = CM0_AP;
                // last check availability of CM0
                JTAG2SWD();
                swd_clear_errors();
                if (!swd_read_word(CPU_ID, &cpu_id) || cpu_id == 0)
                {
                    // CM0 AP still unaviable, fallback to CM4
                    g_target_family_psoc6.apsel = CM4_AP;
                    swd_clear_errors();
                }
            }
            else
            {
                if (g_target_family_psoc6.apsel == CM4_AP)
                {
                    // last check availability of CM0 if CM4 is accesible
                    uint32_t cpu_id = 0;
                    g_target_family_psoc6.apsel = CM0_AP;
                    if (!swd_read_word(CPU_ID, &cpu_id) || cpu_id == 0)
                    {
                        // CM0 AP still unaviable, fallback to CM4
                        g_target_family_psoc6.apsel = CM4_AP;
                        swd_clear_errors();
                    }
                }
            }
        }
        else
        {
            // normal flow
            uint32_t status = SWDAcquirePSoC6BLE(RESET_ACQUIRE, NUMBER_OF_ATTEMPTS);
            if (status == ACQUIRE_PASS)
            {
                isTargetAcquired = true;
                // find valid AP with core
                uint32_t cpu_id = 0;
                for (uint32_t ap_index = 0; cpu_id == 0 && (ap_index <= MAX_AP); ap_index++)
                {
                    g_target_family_psoc6.apsel = ap_index << 24u;
                    if (!swd_read_word(CPU_ID, &cpu_id))
                    {
                        swd_clear_errors();
                        cpu_id = 0;
                    }
                }
            }
        }
        

        // wake up CM4 if going to work via CM4_AP
        // power off CM4 if going to work via CM0_AP
        if ((g_target_family_psoc6.apsel != SYS_AP) && swd_write_ap(AP_CSW, CSW_ACCESS_MASK))
        {
            if (g_target_family_psoc6.apsel == CM0_AP)
            {
                // power off CM4
                swd_write_word(CPUSS_CM4_PWR_CTL, CM4_PWR_OFF_VALUE);
            }
            else
            {
                if (g_target_family_psoc6.apsel == CM4_AP)
                {
                    uint32_t pwr_ctl;
                    if (swd_read_word(CPUSS_CM4_PWR_CTL, &pwr_ctl)
                            && ((pwr_ctl & CM4_PWR_MASK) != CM4_PWR_MASK))
                    {
                        // wake up CM4
                        swd_write_word(CPUSS_CM4_PWR_CTL, CM4_PWR_ON_VALUE);
                    }
                }
            }
        }
    }
    // speed up swd
    SetCustomDAPDefaults();
    return;
}

static uint8_t target_set_state(target_state_t state)
{
    // invoke reset by sw (VECT_REQ or SYS_REQ) or hw (hardware IO toggle)
    //return swd_set_target_state_sw(state);
    //or

    if (state == RESET_PROGRAM)
    {
        target_before_init_debug();
        if (!isTargetAcquired)
        {
            return 0;
        }
        uint32_t status = swd_set_target_state_sw(HALT);
        return status;
    }

    return swd_set_target_state_hw(state);
}

target_family_descriptor_t g_target_family_psoc6 = {
    .family_id = kCypress_psoc6_FamilyID,
    .target_set_state = target_set_state,
    .prerun_target_config = prerun_target_config,
    .target_before_init_debug = target_before_init_debug,
};

const target_family_descriptor_t *g_target_family = &g_target_family_psoc6;
