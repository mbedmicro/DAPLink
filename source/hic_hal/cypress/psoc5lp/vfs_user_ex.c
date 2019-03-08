/**
 * @file    vfs_user.c
 * @brief   Implementation of vfs_user.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * Copyright 2019, Cypress Semiconductor Corporation 
 * or a subsidiary of Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
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
 */

#include "string.h"
#include "vfs_manager.h"
#include "util.h"
#include "psoc5lp.h"
#include "Bootloadable.h"

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

//! @brief Constants for Cypress specific magic action or config files.
typedef enum _cy_magic_file {
    kBootloaderModeActionFile,  //!< Switch to Cypress bootloader.
    kKP3ModeActionFile,         //!< Switch to KitProg3.
} cy_magic_file_t;

//! @brief Mapping from filename string to magic file enum.
typedef struct _cy_magic_file_info {
    const char *name;      //!< Name of the magic file, must be in 8.3 format.
    cy_magic_file_t which; //!< Enum for the file.
} cy_magic_file_info_t;

//! @brief Table of magic files and their names.
static const cy_magic_file_info_t s_cy_magic_file_info[] = {
        { "START_CBACT", kBootloaderModeActionFile  },
        { "START_KPACT", kKP3ModeActionFile         },
    };

//! @brief Hook for magic files.
//!
//! This hook is intended to simplify checking for magic files. In addition to allowing support for
//! new magic files, you can also change the behaviour of or disable standard magic files.
//!
//! @param filename Name of the file that was created.
//! @param[out] do_remount Whether the caller should remount the MSD volume. Only applies if true
//!     is returned. The default is true, so if the hook does not modify this parameter and returns
//!     true, a remount is performed.
//! @retval true The hook handled the specified file. A remount will be performed if requested,
//!     but otherwise no other standard behaviour is applied.
//! @retval false The hook did not handle the file; continue with canonical behaviour.
bool vfs_user_magic_file_hook(const vfs_filename_t filename, bool *do_remount)
{
    int32_t which_magic_file = -1;
    // Compare the new file's name to our table of magic filenames.
    for (int32_t i = 0; i < ARRAY_SIZE(s_cy_magic_file_info); ++i) 
    {
        if (!memcmp(filename, s_cy_magic_file_info[i].name, sizeof(vfs_filename_t)))
        {
            which_magic_file = i;
        }
    }
    
    // Check if we matched a magic filename and handle it.
    if (which_magic_file != -1) 
    {
        switch (which_magic_file) 
        {
        case kBootloaderModeActionFile:
            // Switch to Cypress bootloader.
            Bootloadable_Load();
            break;
        case kKP3ModeActionFile:
            // Switch to KitProg3.
            SetKitProgActiveApp(KP3_MODE_BULK);
            break;
        default:
            util_assert(false);
            break;
        }
        
        return true;
    }
    else
    {
        return false;
    }
}
