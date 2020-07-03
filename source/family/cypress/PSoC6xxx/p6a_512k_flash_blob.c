/*******************************************************************************
* @file    p6a_512k_flash_blob.c
* @brief   Flash algorithm for the PSoC6A 512K target MCU
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

#include "flash_blob.h"

// Main Flash algo

static const uint32_t CY8C6xx5_flash_prog_blob[] = {
    0xE00ABE00u, 0x062D780Du, 0x24084068u, 0xD3000040u, 0x1E644058u, 0x1C49D1FAu, 0x2A001E52u, 0x4770D1F2u,
    0x47702000u, 0x47702000u, 0xf000b510u, 0xbd10f9fdu, 0xf000b510u, 0xbd10f9fdu, 0x4611b510u, 0xf9fcf000u,
    0xb510bd10u, 0xfa1cf000u, 0xb510bd10u, 0xfa24f000u, 0x0000bd10u, 0x47706001u, 0x60086800u, 0x4a114770u,
    0x43502100u, 0x1c49e000u, 0xd8fc4288u, 0xb5304770u, 0x480d4602u, 0xe00f4c0du, 0x021b7813u, 0x23004058u,
    0x04051c52u, 0x0040d502u, 0xe0004060u, 0x1c5b0040u, 0xb280b2dbu, 0xd3f42b08u, 0x1e49460bu, 0x2b00b289u,
    0xbd30d1eau, 0x00000d05u, 0x0000ffffu, 0x00001021u, 0x4669b508u, 0xf7ff482bu, 0x9800ffcfu, 0x0f000500u,
    0x2000d001u, 0x2001bd08u, 0xb530bd08u, 0xb08b4c26u, 0x7820444cu, 0xd13c2800u, 0xffeaf7ffu, 0x40682501u,
    0xaa097220u, 0xa807a908u, 0x91019202u, 0xab069000u, 0xa904aa05u, 0xf000a803u, 0x2800f8d5u, 0x4668d129u,
    0x70637b03u, 0x70a27c02u, 0x70e17d01u, 0x71207e00u, 0x7f004668u, 0xa8087160u, 0x71a07800u, 0x7900a808u,
    0x20ff71e0u, 0x2b017260u, 0x2a00d112u, 0x2a02d004u, 0x2a05d006u, 0xe00bd008u, 0xd10929e2u, 0xe0062000u,
    0xd10529e4u, 0xe0022002u, 0xd10129e7u, 0x72602005u, 0x48057025u, 0x4448b00bu, 0x4803bd30u, 0x78004448u,
    0x00004770u, 0x40200000u, 0xfffffff4u, 0x460cb5f7u, 0x014049a3u, 0x18452600u, 0xb0822701u, 0x4669351cu,
    0xf7ff4628u, 0x9800ff69u, 0x46200fc1u, 0x40214308u, 0x40784388u, 0x9904d107u, 0xd804428eu, 0xf7ff2001u,
    0x1c76ff5eu, 0x4078e7ebu, 0xbdf0b005u, 0x460eb5f8u, 0x24004993u, 0x18450140u, 0x46284669u, 0xff4cf7ffu,
    0x0fc09800u, 0x42b4d106u, 0x2001d804u, 0xff47f7ffu, 0xe7f11c64u, 0x40482101u, 0xb5f8bdf8u, 0x460e4615u,
    0x24004607u, 0x46384629u, 0xff36f7ffu, 0x0f006828u, 0xd007280au, 0x42b42000u, 0x2001d805u, 0xff2ff7ffu,
    0xe7ef1c64u, 0x21012001u, 0xbdf84048u, 0xb081b5f3u, 0xf7ff4606u, 0x2800ffa2u, 0xf7ffd003u, 0x7a00ff56u,
    0xf7ffe003u, 0x2101ff45u, 0x28004048u, 0x2801d003u, 0x2001d004u, 0x2500bdfeu, 0xe0024c71u, 0x25014c70u,
    0x217d3420u, 0x462800c9u, 0xffb0f7ffu, 0xd1f12800u, 0x17c707f0u, 0xd0011c7fu, 0x447e4e6au, 0x300c4620u,
    0x90004631u, 0xfef6f7ffu, 0x30104628u, 0x40812101u, 0xf7ff4865u, 0x4620feefu, 0x30082101u, 0xfeeaf7ffu,
    0x21004a62u, 0xf7ff4628u, 0x2800ff71u, 0x21ffd1d2u, 0x9a0231f5u, 0xd0012f00u, 0xe0004630u, 0xf7ff9800u,
    0xbdfeff9cu, 0x460eb5feu, 0x461c4607u, 0xa9014615u, 0xf7ff2001u, 0x2800ffabu, 0x9801d11eu, 0x70380a00u,
    0x70309801u, 0x99089801u, 0x70080c00u, 0x466920ffu, 0xf7ff3002u, 0x2800ff9bu, 0x9900d10eu, 0x70290a09u,
    0x70219900u, 0x03099900u, 0x99090f0au, 0x9900700au, 0x0f0a0209u, 0x700a990au, 0xb538bdfeu, 0x48414c44u,
    0x44784621u, 0xf7ff38a8u, 0x4669fea5u, 0xf7ff4620u, 0xbd38ff7du, 0x4c3bb5f8u, 0x447c4d3eu, 0x3cc04606u,
    0x46204629u, 0xfe96f7ffu, 0x1d204631u, 0xfe92f7ffu, 0x46284669u, 0xff6af7ffu, 0xb5f8bdf8u, 0x4d364c31u,
    0x4606447cu, 0x46293ce6u, 0xf7ff4620u, 0x4631fe83u, 0xf7ff1d20u, 0x4669fe7fu, 0xf7ff4628u, 0xbdf8ff57u,
    0x4c2eb5f8u, 0x447c4606u, 0x46204d2bu, 0xf7ff4629u, 0x4631fe71u, 0xf7ff1d20u, 0x4669fe6du, 0xf7ff4628u,
    0xbdf8ff45u, 0x4c25b5f8u, 0x4606460fu, 0x4d24447cu, 0x46293c26u, 0xf7ff4620u, 0x21fffe5du, 0x1d203107u,
    0xfe58f7ffu, 0x46314620u, 0xf7ff3008u, 0x4620fe53u, 0x300c4639u, 0xfe4ef7ffu, 0x46284669u, 0xff26f7ffu,
    0xb5f8bdf8u, 0x460f4c15u, 0x447c4606u, 0x3c644d15u, 0x46204629u, 0xfe3ef7ffu, 0x310721ffu, 0xf7ff1d20u,
    0x4620fe39u, 0x30084631u, 0xfe34f7ffu, 0x46394620u, 0xf7ff300cu, 0x4669fe2fu, 0xf7ff4628u, 0xbdf8ff07u,
    0x40220000u, 0x000002aau, 0x40221008u, 0x00003a98u, 0x0a000100u, 0x1c000100u, 0x14000100u, 0x0000019eu,
    0x06000100u, 0x05000100u, 0xf7ffb510u, 0xbd10ff66u, 0xf7ffb510u, 0xbd10ff6fu, 0xf7ffb510u, 0xbd10ffa3u,
    0x4605b570u, 0x2000460cu, 0x4628e008u, 0xff62f7ffu, 0xd1052800u, 0x1e6435ffu, 0x350235ffu, 0xd1f42c00u,
    0xb510bd70u, 0x48134604u, 0x02492101u, 0xf0004478u, 0x4910f841u, 0x39084479u, 0xf7ff4620u, 0xbd10ffa2u,
    0x2300b530u, 0x5cd5e004u, 0x42a55cc4u, 0x1c5bd102u, 0xd3f8428bu, 0xbd3018c0u, 0x4604b530u, 0x46032000u,
    0x5ce5e005u, 0xd0014295u, 0xbd302001u, 0x428b1c5bu, 0xbd30d3f7u, 0x000000acu, 0xc004e001u, 0x29041f09u,
    0x078bd2fbu, 0x8002d501u, 0x07c91c80u, 0x7002d000u, 0x29004770u, 0x07c3d00bu, 0x7002d002u, 0x1e491c40u,
    0xd3042902u, 0xd5020783u, 0x1c808002u, 0xe7e31e89u, 0xe7ee2200u, 0xe7df2200u, 0xffffff00u, 0xffffffffu,
    0x0000ffffu, 0x00000000u
};

// Start address of flash
// static const uint32_t flash_start = 0x10000000;
// Size of flash
// static const uint32_t flash_size = 0x00080000;

static const program_target_t CY8C6xx5_flash_prog =
    // Main Flash
    {
        .init =         PSOC6_SRAM_START_ADDR + 0x00000021u, // Init
        .uninit =       PSOC6_SRAM_START_ADDR + 0x00000025u, // UnInit
        .erase_chip =   PSOC6_SRAM_START_ADDR + 0x00000029u, // EraseChip
        .erase_sector = PSOC6_SRAM_START_ADDR + 0x00000031u, // EraseSector
        .program_page = PSOC6_SRAM_START_ADDR + 0x00000039u, // ProgramPage
        .verify =       PSOC6_SRAM_START_ADDR + 0x00000043u, // Verify
        // BKPT : start of blob + 1
        // RSB  : blob start + header + rw data offset
        // RSP  : stack pointer
        .sys_call_s =
            {
                PSOC6_SRAM_START_ADDR + 0x00000001u,
                PSOC6_SRAM_START_ADDR + 0x00000504u,
                PSOC6_SRAM_START_ADDR + 0x00001000u
            },
        .program_buffer = PSOC6_SRAM_START_ADDR + 0x00001000u,      // mem buffer location
        .algo_start = PSOC6_SRAM_START_ADDR,                       // location to write prog_blob in target RAM
        .algo_size = sizeof(CY8C6xx5_flash_prog_blob),  // prog_blob size
        .algo_blob = CY8C6xx5_flash_prog_blob,          // address of prog_blob
        .program_buffer_size = 512u,              // ram_to_flash_bytes_to_be_written
        .algo_flags = (kAlgoVerifyReturnsAddress | kAlgoSingleInitType)
    };

