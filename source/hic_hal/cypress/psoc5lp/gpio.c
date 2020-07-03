/*******************************************************************************

* @file    gpio.c
* @brief   Implementation of GPIO api for the PSoC5LP
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
#include "gpio.h"
#include "compiler.h"
#include "IO_Config.h"
#include "daplink.h"
#include "cypins.h"
#include "LED_Amber.h"
#include "LED_Green.h"
#include "LED_Red.h"
#include "ModeButton.h"
#include "Pin_HWVersion.h"

// This GPIO configuration is only valid for the PSoC5LP
COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_PSOC5LP);

#define APP_SWITCH_TIMEOUT_SINGLE_BTN      (2000u/30u)
#define APP_SWITCH_TIMEOUT                 (100u/30u)

uint32_t app_switch_counter = 0;
uint32_t app_switch_counter_last = 0;

/******************************************************************************
*  HW_CheckAppSwitch
***************************************************************************//**
* Check on-board buttons and perform switch to KitProg application if user
* requested that action by pressing buttons.
*
******************************************************************************/
void HW_CheckAppSwitch(void)
{
    /* Check that button is pressed */
    if (ModeButton_Read() == 0)
    {
        app_switch_counter++;
    }
    else
    {
        /* button released */
        app_switch_counter_last = app_switch_counter;
        app_switch_counter = 0;
    }

    /* Check that button is pressed for required duration and release */
    if (app_switch_counter_last > APP_SWITCH_TIMEOUT)
    {
        SetKitProgActiveApp(KP3_MODE_BULK);
    }
}

/*******************************************************************************
* Function Name: interrogate_kit_hw_id()
********************************************************************************
* Summary:
* Calculates the hardware version of the KitProg board.
*
* Parameters:
*  void
*
* Return:
*  HW ID value
*
*******************************************************************************/
uint8_t interrogate_kit_hw_id(void)
{
   uint8_t kitProgHwId;
	 kitProgHwId = (~Pin_HWVersion_Read()) & Pin_HWVersion_MASK;

   /* These pins no longer used, set them to HiZ analogue to save power */
   Pin_HWVersion_SetDriveMode(Pin_HWVersion_DM_ALG_HIZ);

	 return kitProgHwId;

}

uint8_t gpio_get_reset_btn_fwrd()
{
    HW_CheckAppSwitch();
    return 0;
}

uint8_t gpio_get_reset_btn_no_fwrd(void)
{
    return 0;
}

void gpio_init(void)
{
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    if (kit_has_three_led())
    {
        LED_Red_Write(state);
    }
}

void gpio_set_hid_led(gpio_led_state_t state)
{
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    if (kit_has_three_led())
    {
        LED_Green_Write(state);
    }
}

void gpio_set_board_power(bool powerEnabled)
{
}
