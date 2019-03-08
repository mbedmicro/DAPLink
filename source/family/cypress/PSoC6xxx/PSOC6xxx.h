/*******************************************************************************
* @file PSOC6xxx.h
*
* @brief
*  This file provides constants and function declaration for PSoC6xxx family
*  device acquiring, silicon id reading.
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

#if !defined(PSOC6XXX_H)
#define PSOC6XXX_H

#include "CyLib.h"
#include "target_family.h"

// Acquire timeout in Timer_CSTick ticks (8Hz rate, 8 ticks == 1000ms)
#define PSOC6_TVII_AQUIRE_TIMEOUT_TICKS          (16u)

// Data shift
#define SHIFT_24                                 (24u)
#define SHIFT_16                                 (16u)
#define SHIFT_8                                  (8u)

// CY8C6xx6, CY8C6XX7 devices
#define MEM_BASE_IPC_1M                          (0x40230000u)
#define IPC_STRUCT0_1M                            MEM_BASE_IPC_1M
#define IPC_STRUCT1_1M                           (IPC_STRUCT0_1M + IPC_STRUCT_SIZE)
#define IPC_STRUCT2_1M                           (IPC_STRUCT1_1M + IPC_STRUCT_SIZE)
#define IPC_INTR_STRUCT_1M                       (0x40231000u)
#define IPC_STRUCT_LOCK_STATUS_OFFSET_1M         (0x10u)
#define IPC_STRUCT_DATA_1M                       (IPC_STRUCT2_1M + IPC_STRUCT_DATA_OFFSET)
#define IPC_STRUCT2_NOTIFY_DATA_1M               (IPC_STRUCT2_1M + IPC_STRUCT_NOTIFY_OFFSET)
#define IPC_INTR_STRUCT_DATA_1M                  (IPC_INTR_STRUCT_1M + IPC_INTR_STRUCT_INTR_IPC_MASK_OFFSET)

// CY8C6xx8, CY8C6XXA devices
#define MEM_BASE_IPC_2M                          (0x40220000u)
#define IPC_STRUCT0_2M                            MEM_BASE_IPC_2M
#define IPC_STRUCT1_2M                           (IPC_STRUCT0_2M + IPC_STRUCT_SIZE)
#define IPC_STRUCT2_2M                           (IPC_STRUCT1_2M + IPC_STRUCT_SIZE)
#define IPC_INTR_STRUCT_2M                       (0x40221000u)
#define IPC_STRUCT_LOCK_STATUS_OFFSET_2M         (0x1Cu)
#define IPC_STRUCT_DATA_2M                       (IPC_STRUCT2_2M + IPC_STRUCT_DATA_OFFSET)
#define IPC_STRUCT2_NOTIFY_DATA_2M               (IPC_STRUCT2_2M + IPC_STRUCT_NOTIFY_OFFSET)
#define IPC_INTR_STRUCT_DATA_2M                  (IPC_INTR_STRUCT_2M + IPC_INTR_STRUCT_INTR_IPC_MASK_OFFSET)

// IPC strustures definitions fro PSoC6 SROM API usage
#define MXS40_SROMAPI_SILID_CODE                 (0x00000001u)
#define PSOC6_SROM_SILID_CODE_LO                 (0x0000FF00u  & (0u << SHIFT_8))
#define PSOC6_SROM_SILID_CODE_HI                 (0x0000FF00u  & (1u << SHIFT_8))
#define IPC_2                                    (2u)
#define IPC_STRUCT_SIZE                          (0x20u)
#define IPC_STRUCT_ACQUIRE_TIMEOUT               (5000u)
#define MXS40_SROMAPI_DATA_LOCATION_MSK          (0x00000001u)
#define SRAM_SCRATCH_ADDR                        (0x08000400u)
#define IPC_INTR_STRUCT_INTR_IPC_MASK_OFFSET     (0x08u)
#define IPC_INTR_STRUCT_DATA_VALUE               (1u << (SHIFT_16 + 2u))
#define IPC_STRUCT_DATA_OFFSET                   (0x0Cu)
#define IPC_STRUCT_NOTIFY_OFFSET                 (0x08u)
#define IPC_STRUCT_DATA_TIMEOUT                  (9000u)
#define IPC_STRUCT_ACQUIRE_OFFSET                (0x00u)
#define IPC_STRUCT_ACQUIRE_SUCCESS_MSK           (0x80000000u)
#define IPC_STRUCT_LOCK_STATUS_ACQUIRED_MSK      (0x80000000u)
#define SROM_STATUS_SUCCESS_MASK                 (0xF0000000u)
#define MXS40_SROMAPI_STAT_SUCCESS               (0xA0000000u)

// Number of PSoC6 acquiring attempts
#define NUMBER_OF_ATTEMPTS                       (0x10u)

// Number of PSoC6 silicon ID reading attemts
#define GET_SIID_ATTEMTS                         (2u)

// Masks for Silicond ID parts for PSoC6
#define PSOC6_FAMILY_ID_HI_MSK                   (0x0000FF00u)
#define PSOC6_FAMILY_ID_LO_MSK                   (0x000000FFu)
#define PSOC6_REV_ID_MAJ_MSK                     (0x00F00000u)
#define PSOC6_REV_ID_MIN_MSK                     (0x000F0000u)
#define PSOC6_SILICON_ID_LO_MSK                  (0x000000FFu)
#define PSOC6_SILICON_ID_HI_MSK                  (0x0000FF00u)
#define PSOC6_SILICON_ID_MASK                    (0xFFFF00FFu)

// Family ID for PSoC6xxx targets
#define PSOC6A_BLE2_FAMILY_ID                    (0x100u)
#define PSOC6A_2M_FAMILY_ID                      (0x102u)
#define PSoC6A_512K_FAMILY_ID                    (0x105u)
#define MXS28PLAYER_FAMILY_ID                    (0x10Au)

#define PSOC6_FAMILY_ID_HI                       (0x01u)

#define PSOC6A_BLE2_FAMILY_ID_LO                 (PSOC6A_BLE2_FAMILY_ID & PSOC6_FAMILY_ID_LO_MSK)
#define PSOC6A_2M_FAMILY_ID_LO                   (PSOC6A_2M_FAMILY_ID   & PSOC6_FAMILY_ID_LO_MSK)
#define PSOC6A_512K_FAMILY_ID_LO                 (PSoC6A_512K_FAMILY_ID & PSOC6_FAMILY_ID_LO_MSK)
#define MXS28PLAYER_FAMILY_ID_LO                 (MXS28PLAYER_FAMILY_ID & PSOC6_FAMILY_ID_LO_MSK)

// Acquire modes
#define RESET_ACQUIRE       (0x00u)
#define POWER_ACQUIRE       (0x01u)
#define NOTESTBIT_ACQUIRE   (0x02u)

// Flash related difines
#define PSOC6_SROM_START_ADDR                    (0x00000000u)
#define PSOC6_MAIN_FLASH_START_ADDR              (0x10000000u)
#define PSOC6_WFLASH_START_ADDR                  (0x14000000u)
#define PSOC6_SFLASH_START_ADDR                  (0x16000000u)
#define PSOC6_XIP_START_ADDR                     (0x18000000u)

#define PSOC6_SRAM_START_ADDR                    (0x08000000u)

// Kits unique ID enums
typedef enum names
{
    CY8CKIT_062_BLE       = 0x0005u,
    CY8CKIT_062_WIFI_BT   = 0x0006u,
    CY8CPROTO_063_BLE     = 0x000Bu,
    CY8CPROTO_062_4343W   = 0x0010u,
    CY8CPROTO_064_SB      = 0x0012u,
    CYW943012P6EVB_01     = 0x0015u,
    CY8CKIT_062_4343W     = 0x0016u,
    CY8CPROTO_062S2_43012 = 0x0017u,
    CY8CKIT_062S2_43012   = 0x0018u,
    CY8CPROTO_062S3_4343W = 0x0019u,
    AUGUST_CYW43012       = 0x001Au,
    CYW9P62S1_43438EVB_01 = 0x001Bu,
    CYW9P62S1_43012EVB_01 = 0x001Cu,
    CY8CKIT_064S2_4343W   = 0x001Du,
    CY8CPROTO_064B0S3     = 0x001Eu,
    CY8CPROTO_064B0S1_BLE = 0x001Fu,
    CY8CKIT_064B0S2_4343W = 0x0020u,
    CYFEATHER_RP01        = 0x0022u,
    UNSPECIFIED
} CyKits;

extern target_family_descriptor_t g_target_family_psoc6;
extern uint16_t familyID;

uint32_t get_silcon_id(void);
void  init_flash_algo(uint32_t si_id);
uint32_t get_fb_version(void);
uint32_t SWDAcquirePSoC6BLE(uint8_t acquireMode, uint8_t attempts);
void prerun_target_config(void);
uint32_t get_readyval_addr(void);
uint16_t get_kit_uid(void);

#endif /* PSOC6XXX_H */
