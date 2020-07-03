/*******************************************************************************
* @file    target.c
* @brief   Target information for the target MCU
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

#include "target_config.h"
#include "error.h"
#include "PSOC6xxx.h"
#include "psoc5lp.h"
#include "stdbool.h"

// The files xxx_flash_blob.c must only be included in target.c
#include "p6a_2m_flash_blob.c"
#include "p6a_512k_flash_blob.c"
#include "p6_ble_flash_blob.c"
#include "p6_s_int_flash_blob.c"
#include "p6_s25fl64l_flash_blob.c"
#include "p6_s25fl128s_flash_blob.c"
#include "p6_s25f512s_flash_blob.c"
#include "p6a_2m_s_int_flash_blob.c"
#include "p6a_512k_s_int_flash_blob.c"
#include "p6a_s25f512s_flash_blob.c"

#define IMAGE_MAGIC                 (0x96f3b83du)

static const sector_info_t p6_sectors_info[] =
{
    { PSOC6_MAIN_FLASH_START_ADDR, 512u },    // Internal flash
    { PSOC6_XIP_START_ADDR, KB(256u) },    // SMIF
};

static const sector_info_t AUGUST_CYW43012_sectors_info[] =
{
    { PSOC6_MAIN_FLASH_START_ADDR, 512u },    // Internal flash
    { PSOC6_XIP_START_ADDR, KB(4u) },    // SMIF
};


// default target information
target_cfg_t target_device = {
    .sectors_info   = p6_sectors_info,
    .sector_info_length = (sizeof(p6_sectors_info))/(sizeof(sector_info_t)),
    .flash_regions    = {
        { .start = PSOC6_SROM_START_ADDR,       .end = PSOC6_SROM_START_ADDR + KB(128u) },           // SROM
        { .start = PSOC6_MAIN_FLASH_START_ADDR, .end = PSOC6_MAIN_FLASH_START_ADDR + MB(2u), 
            .flash_algo = (program_target_t *) &CY8C6xxA_flash_prog, .flags = kRegionIsDefault },    // Main Flash
        { .start = PSOC6_WFLASH_START_ADDR,     .end = PSOC6_WFLASH_START_ADDR + KB(32u), 
            .flash_algo = (program_target_t *) &CY8C6xxA_WFLASH_flash_prog },                        // WFLASH (Work)
        { .start = PSOC6_SFLASH_START_ADDR,     .end = PSOC6_SFLASH_START_ADDR + KB(32u), 
            .flash_algo = (program_target_t *) &CY8C6xxA_SFLASH_flash_prog },                        // SFLASH
        { .start = PSOC6_XIP_START_ADDR,        .end = PSOC6_XIP_START_ADDR + MB(64u), 
            .flash_algo = (program_target_t *) &CY8C6xxA_SMIF_S25FL512S_flash_prog }                 // SMIF
    },
    .ram_regions      = {
        { .start = PSOC6_SRAM_START_ADDR, .end = PSOC6_SRAM_START_ADDR + KB(288u) }
    }
};

static uint8_t validate_secured_bin_nvic(const uint8_t *buf)
{
    uint32_t nvic_val = 0u;
    memcpy(&nvic_val, buf, sizeof(nvic_val));
    return ((nvic_val == IMAGE_MAGIC) ? 1u : 0u);
}

void init_flash_algo(uint32_t si_id)
{
    // Initialize main flash,WFlash, SFlash and SMIF programming algorithms
    // and set proper flsash geometry based on family type/silicon ID

    if (kit_has_sflash_restriction())
    {
        // PSoC64 & PSoC64 2M based kits
        // set secure binary file validation procedure
        g_target_family_psoc6.validate_bin_nvic = validate_secured_bin_nvic;
        switch (get_kit_uid()) 
        {
        case CY8CPROTO_064_SB:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + MB(1u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C64xx_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxx_WFLASH_flash_prog;
            target_device.flash_regions[3].start = PSOC6_XIP_START_ADDR;
            target_device.flash_regions[3].end = PSOC6_XIP_START_ADDR + MB(16u);
            target_device.flash_regions[3].flash_algo = (program_target_t *) &CY8C6xxx_SMIF_S25FL128S_flash_prog;
            target_device.flash_regions[4].start = 0u;
            target_device.flash_regions[4].end = 0u;
            target_device.flash_regions[4].flash_algo = NULL; 
            break;
        
        case AUGUST_CYW43012:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + MB(1u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C64xx_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxx_WFLASH_flash_prog;
            target_device.flash_regions[3].start = PSOC6_XIP_START_ADDR;
            target_device.flash_regions[3].end = PSOC6_XIP_START_ADDR + MB(8u);
            target_device.flash_regions[3].flash_algo = (program_target_t *) &CY8C6xxx_SMIF_S25FL064L_flash_prog;
            target_device.flash_regions[4].start = 0u;
            target_device.flash_regions[4].end = 0u;
            target_device.flash_regions[4].flash_algo = NULL; 
            target_device.sectors_info = AUGUST_CYW43012_sectors_info;      /* sector size 4K for SMIF */
            break;

        case CY8CKIT_064S2_4343W:
        case CY8CKIT_064B0S2_4343W:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + MB(2u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C64xA_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxA_WFLASH_flash_prog;
            target_device.flash_regions[3].start = PSOC6_XIP_START_ADDR;
            target_device.flash_regions[3].end = PSOC6_XIP_START_ADDR + MB(64u);
            target_device.flash_regions[3].flash_algo = (program_target_t *) &CY8C6xxA_SMIF_S25FL512S_flash_prog;
            target_device.flash_regions[4].start = 0u;
            target_device.flash_regions[4].end = 0u;
            target_device.flash_regions[4].flash_algo = NULL; 
            break;
        
        case CY8CPROTO_064B0S3:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + KB(512u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C64x5_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxA_WFLASH_flash_prog;
            target_device.flash_regions[3].start = PSOC6_XIP_START_ADDR;
            target_device.flash_regions[3].end = PSOC6_XIP_START_ADDR + MB(64u);
            target_device.flash_regions[3].flash_algo = (program_target_t *) &CY8C6xxA_SMIF_S25FL512S_flash_prog;
            target_device.flash_regions[4].start = 0u;
            target_device.flash_regions[4].end = 0u;
            target_device.flash_regions[4].flash_algo = NULL; 
            break;

        case CY8CPROTO_064B0S1_BLE:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + MB(1u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C64xx_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxx_WFLASH_flash_prog;
            target_device.flash_regions[3].start = 0u;
            target_device.flash_regions[3].end = 0u;
            target_device.flash_regions[3].flash_algo = NULL;
            target_device.flash_regions[4].start = 0u;
            target_device.flash_regions[4].end = 0u;
            target_device.flash_regions[4].flash_algo = NULL; 
            break;
            
        default:
            // unknown board
            target_device.flash_regions[0].start = 0u;
            target_device.flash_regions[0].end = 0u;
            target_device.flash_regions[0].flash_algo = NULL; 
            target_device.flash_regions[1].start = 0u;
            target_device.flash_regions[1].end = 0u;
            target_device.flash_regions[1].flash_algo = NULL; 
            target_device.flash_regions[2].start = 0u;
            target_device.flash_regions[2].end = 0u;
            target_device.flash_regions[2].flash_algo = NULL; 
            target_device.flash_regions[3].start = 0u;
            target_device.flash_regions[3].end = 0u;
            target_device.flash_regions[3].flash_algo = NULL; 
            target_device.flash_regions[4].start = 0u;
            target_device.flash_regions[4].end = 0u;
            target_device.flash_regions[4].flash_algo = NULL; 
        
            break;
        }
    }
    else
    {
        // set generic binary file validation procedure
        g_target_family_psoc6.validate_bin_nvic = NULL;
        switch (si_id & PSOC6_FAMILY_ID_LO_MSK)
        {
            // PSoC6-BLE family
        case PSOC6A_BLE2_FAMILY_ID_LO:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + MB(1u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C6xx7_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxx_WFLASH_flash_prog;
            target_device.flash_regions[3].flash_algo = (program_target_t *) &CY8C6xxx_SFLASH_flash_prog;
            target_device.flash_regions[4].flash_algo = (program_target_t *) &CY8C6xxx_SMIF_S25FL512S_flash_prog;
            break;

            // PSoC6A-2M family
        case PSOC6A_2M_FAMILY_ID_LO:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + MB(2u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C6xxA_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxA_WFLASH_flash_prog;
            target_device.flash_regions[3].flash_algo = (program_target_t *) &CY8C6xxA_SFLASH_flash_prog;
            target_device.flash_regions[4].flash_algo = (program_target_t *) &CY8C6xxA_SMIF_S25FL512S_flash_prog;
            break;

            // PSoC6A-512K family
        case PSOC6A_512K_FAMILY_ID_LO:
            target_device.flash_regions[1].end =   PSOC6_MAIN_FLASH_START_ADDR + KB(512u);
            target_device.flash_regions[1].flash_algo = (program_target_t *) &CY8C6xx5_flash_prog;
            target_device.flash_regions[2].flash_algo = (program_target_t *) &CY8C6xxA_WFLASH_flash_prog;
            target_device.flash_regions[3].flash_algo = (program_target_t *) &CY8C6xxA_SFLASH_flash_prog;
            target_device.flash_regions[4].flash_algo = (program_target_t *) &CY8C6xxA_SMIF_S25FL512S_flash_prog;
            break;

            // unknown family
        default:
            target_device.flash_regions[0].start = 0u;
            target_device.flash_regions[0].end = 0u;
            target_device.flash_regions[0].flash_algo = NULL; 
            target_device.flash_regions[1].start = 0u;
            target_device.flash_regions[1].end = 0u;
            target_device.flash_regions[1].flash_algo = NULL; 
            target_device.flash_regions[2].start = 0u;
            target_device.flash_regions[2].end = 0u;
            target_device.flash_regions[2].flash_algo = NULL; 
            target_device.flash_regions[3].start = 0u;
            target_device.flash_regions[3].end = 0u;
            target_device.flash_regions[3].flash_algo = NULL; 
            target_device.flash_regions[4].start = 0u;
            target_device.flash_regions[4].end = 0u;
            target_device.flash_regions[4].flash_algo = NULL; 
            break;
        }
    }
}
