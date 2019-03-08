/*******************************************************************************
* @file    cy8ckit.c
* @brief   Board ID definitions for all Cypress povided kits.
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

#include "target_family.h"
#include "target_board.h"

#include "CyLib.h"
#include "Pin_VoltageEn.h"
#include "EEPROM_ModeStorage.h"
#include "PSOC6xxx.h"
#include "psoc5lp.h"
#include "Pin_UART_Rx.h"

// Minimum flashboot version with sflash restriction
#define MIN_FB_SR_VER               (0x24000000u)

// Ready value addr. For 1M family is 0x4023004Cu, for 2M family is 0x4022004Cu
#define READY_ADDR_1M           (IPC_STRUCT2_1M + IPC_STRUCT_DATA_OFFSET)
#define READY_ADDR_2M           (IPC_STRUCT2_2M + IPC_STRUCT_DATA_OFFSET)

// Maximum count of supported  HW ids
#define MAX_SUPPORTED_HWID          (13u)

// Silicon ids by mpn without revision bits (mask is 0xFFFF00FF)

#define CY8C6245LQI_S3D72   (0xE7010005u)
#define CY8C6247BZI_D54     (0xE2060000u)
#define CY8C6247FDI_D32     (0xE2330000u)
#define CY8C6247FDI_D52     (0xE2340000u)
#define CY8C624ABZI_S2D44   (0xE4530002u)
#define CY8C624ABZI_S2D44A0 (0xE4020002u)
#define CY8C624AFNI_D43     (0xE4140002u)
#define CY8C6347BZI_BLD53   (0xE2070000u)
#define CYB06445LQI_S3D42   (0xE70D0005u)
#define CYB06447BZI_BLD53   (0xE2610000u)
#define CYB06447BZI_D54     (0xE2620000u)
#define CYB0644ABZI_S2D44   (0xE4700002u)
#define CYBLE_416045_02     (0xE2F00000u)

// Unique ID record header
#define UID_PSOC5_HEADER            (0x6970122Eu)

#define P5LP_EEPROM_ROW_SIZE        (16u)
#define CRC8_2S_COMP_BASE           (0x0100u)

// Unique ID record struct
typedef struct __attribute__((packed)) unique_id_struct
{
    uint32_t signature;     // Unique id header (PSoC5 silicon ID) - 4 bytes
    char mbed_board_id[4];  // mbed ID - 4 bytes
    uint16_t uid;           // Unique kit ID - 2 bytes
    uint64_t feature_list;  // Kit feature list - 8 bytes
    char name[32];          // Full kit name - 32 bytes
    uint8_t hw_id;          // Kit HW id - 1 byte
    uint32_t target_siid;   // Target silicon id - 4 bytes
    uint16_t ver;           // Record version(1 byte for major part, 
                            // 1 byte for minor part) - 2 bytes
    uint8_t reserved[6];    // Reserved for future use - 6 bytes
    uint8_t checksum;       // Record checksum - 1 byte
} unique_id_struct_t;

// Board properties struct definition
typedef struct
{
    bool kitHasThreeLeds;  /**< Kit has three lEDs, assuming in false case kit has single LED */
    bool possibleDAPLink;  /**< HW of the board allows to DAPLink work */
} board_properties_t;

const board_properties_t board_config[MAX_SUPPORTED_HWID + 1u] =
{
    [0]  = { .kitHasThreeLeds = false, .possibleDAPLink = false }, /* HWID = 0 */
    [1]  = { .kitHasThreeLeds = true, . possibleDAPLink = false }, /* HWID = 1 */
    [2]  = { .kitHasThreeLeds = true, . possibleDAPLink = true },  /* HWID = 2 */
    [3]  = { .kitHasThreeLeds = false, .possibleDAPLink = true },  /* HWID = 3 */
    [4]  = { .kitHasThreeLeds = false, .possibleDAPLink = false }, /* HWID = 4 */
    [5]  = { .kitHasThreeLeds = true, . possibleDAPLink = false }, /* HWID = 5 */
    [6]  = { .kitHasThreeLeds = false, .possibleDAPLink = false }, /* HWID = 6 */
    [7]  = { .kitHasThreeLeds = true, . possibleDAPLink = false }, /* HWID = 7 */
    [8]  = { .kitHasThreeLeds = false, .possibleDAPLink = false }, /* HWID = 8 */
    [9]  = { .kitHasThreeLeds = false, .possibleDAPLink = false }, /* HWID = 9 */
    [10] = { .kitHasThreeLeds = true, . possibleDAPLink = false }, /* HWID = 10(0x0A) */
    [11] = { .kitHasThreeLeds = false, .possibleDAPLink = true },  /* HWID = 11(0x0B) */
    [12] = { .kitHasThreeLeds = false, .possibleDAPLink = false }, /* HWID = 12(0x0C) */
    [13] = { .kitHasThreeLeds = false, .possibleDAPLink = true }   /* HWID = 13(0x0D) */
};

static bool sflash_restriction = false; 

void prerun_board_config(void);

// Default Daplink board_info
const board_info_t g_board_info =
{
    .info_version = kBoardInfoVersion,
    .board_id = "19FF",
    .family_id = kCypress_psoc6_FamilyID,
    .target_cfg = &target_device,
    .prerun_board_config = prerun_board_config,
    .flags = kEnablePageErase
};

static char board_id[5] = "";                   /*!< 4-char board ID plus null terminator */

const char *const board_ids[] = {
    [CY8CKIT_062_BLE      ] = "1902",
    [CY8CKIT_062_WIFI_BT  ] = "1900",
    [CY8CPROTO_063_BLE    ] = "1904",
    [CY8CPROTO_062_4343W  ] = "1901",
    [CYW943012P6EVB_01    ] = "1906",
    [CY8CKIT_062_4343W    ] = "1905",
    [CY8CPROTO_062S2_43012] = "1909",
    [CY8CPROTO_064_SB     ] = "1907",
    [CY8CKIT_062S2_43012  ] = "190B",
    [AUGUST_CYW43012      ] = "190D",
    [CYW9P62S1_43012EVB_01] = "1903",
    [CYW9P62S1_43438EVB_01] = "1908",
    [CY8CPROTO_062S3_4343W] = "190E",
    [CY8CKIT_064S2_4343W  ] = "190A",
    [CY8CPROTO_064B0S3    ] = "190C",
    [CY8CPROTO_064B0S1_BLE] = "190F",
    [CY8CKIT_064B0S2_4343W] = "1910",
    [CYFEATHER_RP01       ] = "1912",
    [UNSPECIFIED          ] = "19FF"
};

// Kit detection record struct
typedef struct kit_id_struct
{
    uint8_t hw_id;          // Kit HW id - 1 byte
    uint32_t target_siid;   // Target silicon id - 4 bytes
    uint16_t uid;           // Unique kit ID - 2 bytes
} kit_id_struct_t;

// List of kits that can be detected
// Be patient: combination of hw_id and target_siid sall be unique
const kit_id_struct_t kit_ids[] =
{
    { .hw_id = 0x02u, .target_siid = CY8C6247BZI_D54,     .uid = CY8CKIT_062_WIFI_BT },
    { .hw_id = 0x02u, .target_siid = CY8C624ABZI_S2D44,   .uid = CY8CKIT_062_4343W },
    { .hw_id = 0x02u, .target_siid = CY8C624ABZI_S2D44A0, .uid = CY8CKIT_062_4343W },
    { .hw_id = 0x02u, .target_siid = CY8C6347BZI_BLD53,   .uid = CY8CKIT_062_BLE },
    { .hw_id = 0x02u, .target_siid = CYB0644ABZI_S2D44,   .uid = CY8CKIT_064S2_4343W },
    { .hw_id = 0x03u, .target_siid = CYB06447BZI_BLD53,   .uid = CY8CPROTO_064B0S1_BLE },
    { .hw_id = 0x03u, .target_siid = CYB06447BZI_D54,     .uid = CY8CPROTO_064_SB },
    { .hw_id = 0x03u, .target_siid = CYBLE_416045_02,     .uid = CY8CPROTO_063_BLE },
    { .hw_id = 0x0Bu, .target_siid = CYB06445LQI_S3D42,   .uid = CY8CPROTO_064B0S3 }, 
    { .hw_id = 0x0Bu, .target_siid = CY8C6245LQI_S3D72,   .uid = CY8CPROTO_062S3_4343W },
    { .hw_id = 0x0Bu, .target_siid = CY8C624ABZI_S2D44,   .uid = CY8CPROTO_062_4343W },
    { .hw_id = 0x0Bu, .target_siid = CY8C624ABZI_S2D44A0, .uid = CY8CPROTO_062_4343W },
    { .hw_id = 0x0Bu, .target_siid = CY8C624AFNI_D43,     .uid = CYFEATHER_RP01 },
    { .hw_id = 0x0Du, .target_siid = CY8C6247BZI_D54,     .uid = CYW9P62S1_43438EVB_01 },
    { .hw_id = 0x0Du, .target_siid = CY8C6247FDI_D32,     .uid = CYW9P62S1_43012EVB_01 },
    { .hw_id = 0x0Du, .target_siid = CY8C6247FDI_D52,     .uid = CYW9P62S1_43012EVB_01 },
    { .hw_id = 0x0Du, .target_siid = CY8C624ABZI_S2D44,   .uid = CY8CKIT_062S2_43012 },
    { .hw_id = 0x0Du, .target_siid = CY8C624ABZI_S2D44A0, .uid = CY8CKIT_062S2_43012 },
    { .hw_id = 0x0Du, .target_siid = CYB0644ABZI_S2D44,   .uid = CY8CKIT_064B0S2_4343W }
};
    
kit_id_struct_t id = 
{
    .uid = UNSPECIFIED
};

// Read uiniqe id record and calculate crc
static uint8_t get_uniq_id(unique_id_struct_t *uid_record)
{
    reg8* modeAddress = (reg8 *) CYDEV_EE_BASE;
    uint8_t eeprom_array[sizeof(unique_id_struct_t)];
    uint8_t crc = 0;

    EEPROM_ModeStorage_Start();

    for (uint8_t i = 0; i < sizeof(unique_id_struct_t); i++)
    {
        eeprom_array[i] = modeAddress[P5LP_EEPROM_ROW_SIZE + i];

        if (i < (sizeof(unique_id_struct_t) - 1u))
        {
            crc += eeprom_array[i];
        }
    }

    EEPROM_ModeStorage_Stop();
    memcpy(uid_record, eeprom_array, sizeof(unique_id_struct_t));

    return (CRC8_2S_COMP_BASE - crc);
}

// Define board id based on HW id and silicon ID
static void define_board_id(void)
{
    uint8_t crc_tmp = 0u;
    bool possibleDAPLink = false;
    unique_id_struct_t uid_record;

    // Read unique id record
    crc_tmp = get_uniq_id(&uid_record);

    if ((UID_PSOC5_HEADER == uid_record.signature) && (crc_tmp == uid_record.checksum))
    {
        // Read HW ID, Silicon ID and mbed board ID from unique id record
        id.uid = uid_record.uid;
        id.target_siid = uid_record.target_siid;
        memcpy(board_id, uid_record.mbed_board_id, sizeof(uid_record.mbed_board_id));
        board_id[4] = 0; // string terminator
        familyID = (PSOC6_FAMILY_ID_HI << SHIFT_8) | (id.target_siid & PSOC6_FAMILY_ID_LO_MSK);
        
        switch (uid_record.uid)
        {
            // For PSoC 64 based kits
        case CY8CPROTO_064_SB:
        case AUGUST_CYW43012:
        case CY8CPROTO_064B0S3:
        case CY8CKIT_064S2_4343W:
        case CY8CPROTO_064B0S1_BLE:
        case CY8CKIT_064B0S2_4343W:
            sflash_restriction = true;
            break;
        
        default:
            break;
        }
    }
    
    // Read the real HW ID of kit anyway
    id.hw_id = interrogate_kit_hw_id();
    
    // Validate HW ID
    if (id.uid == UNSPECIFIED)
    {
        if ((id.hw_id <= MAX_SUPPORTED_HWID) && (board_config[id.hw_id].possibleDAPLink))
        {
            possibleDAPLink = true;
            
            // Powering up target for further recognition
            Pin_VoltageEn_Write(1u);

            // set resistive pull down drive mode for UART RX pin
            Pin_UART_Rx_Write(0u);
            Pin_UART_Rx_SetDriveMode(PIN_DM_RES_DWN);

            // Wait for target initialization
            CyDelay(100u);
        }
    } 
    else
    {
        // Cy_kit is currently known per uniq id, so it's time to turn on the target
        Pin_VoltageEn_Write(1u);

        // set resistive pull down drive mode for UART RX pin
        Pin_UART_Rx_Write(0u);
        Pin_UART_Rx_SetDriveMode(PIN_DM_RES_DWN);

        // Wait for target initialization
        CyDelay(100u);
    }
        
    
    // Analyze HW ID and Silicon ID in case
    // when unique id record is empty or corrupted
    if (possibleDAPLink)
    {
        // Read silicon IDe
        id.target_siid = get_silcon_id();

        // Read flashboot version
        uint32_t fb_ver = get_fb_version();
        if (fb_ver >= MIN_FB_SR_VER)
        {
            sflash_restriction = true;
        }
        else
        {
            sflash_restriction = false;
        }

        // Go through the list of known pairs HW_ID/Silicon_ID until the first match
        for( uint32_t i = 0; i < (sizeof(kit_ids)/sizeof(kit_id_struct_t)); i++)
        {
            if ( (id.hw_id == kit_ids[i].hw_id) && (id.target_siid == kit_ids[i].target_siid) )
            {
                id.uid = kit_ids[i].uid;
                break;
            }
        }
    }
    
    if (id.uid == UNSPECIFIED)
    {
        // Switch back to KP3 if the kit isn't supported by DAPLink
        SetKitProgActiveApp(KP3_MODE_BULK);
    } 
    else
    {
        // Use board_id as rt_board_id if it isn't empty or value from board_ids array otherwise
        if ( board_id[0] == 0 )
        {
            g_board_info.target_cfg->rt_board_id = board_ids[id.uid];
        }
        else
        {
            g_board_info.target_cfg->rt_board_id = board_id;
        }
    }
}

// Called in main_task() to init before USB and files are configured
void prerun_board_config(void)
{
    // Define board ID
    define_board_id();

    //Reset target
    target_set_state(RESET_RUN);
}

// Called in main_task() to init before USB and files are configured
void prerun_target_config(void)
{
    // Initialize main flash,WFlash, SFlash and SMIF programming algorithms
    // and set proper flash geometry based on family type/silicon ID
    init_flash_algo(id.target_siid);
}

// Get .kitHasThreeLeds board property
bool kit_has_three_led(void)
{
    return board_config[id.hw_id].kitHasThreeLeds;
}

// Get SFLASH restriction to write
bool kit_has_sflash_restriction(void)
{
    return sflash_restriction;
}

// Calculate ready value address
uint32_t get_readyval_addr(void)
{
    uint32_t readyval_addr;
    switch (id.target_siid & PSOC6_FAMILY_ID_LO_MSK)
    {
            // PSoC6-BLE family
        case PSOC6A_BLE2_FAMILY_ID_LO:
            readyval_addr = READY_ADDR_1M;
            break;

            // PSoC6A-2M family
        case PSOC6A_2M_FAMILY_ID_LO:
            readyval_addr = READY_ADDR_2M;
            break;

            // PSoC6A-512K family
        case PSOC6A_512K_FAMILY_ID_LO:
            readyval_addr = READY_ADDR_2M;
            break;

            // PSoC6-BLE family by default
        default:
            readyval_addr = READY_ADDR_1M;
            break;
    }
    return readyval_addr;
}

// Get Kit HW id
uint8_t get_kit_hw_id(void)
{
    return id.hw_id;
}

// Get Kit unique product ID
uint16_t get_kit_uid(void)
{
    return id.uid;
}
