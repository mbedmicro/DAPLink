/*******************************************************************************
* @file    PSOC6xxx.c
* @brief   PSOC6xxx family core API for acquiring  and target detecting
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

#include "PSOC6xxx.h"
#include "DAP_config.h"
#include "CyLib.h"
#include "Timer_CSTick.h"
#include "swd_host.h"
#include "target_family.h"
#include "psoc5lp.h"

uint16_t familyID = 0;      /* valid 12bit values mask for PSoC6 is 0x10X */

#define FB_VERSION_ADDR     (0x16002004u)
#define FB_VERSION_UNKNOWN  (0xFFFFFFFFu)

#define SYS_AP              (0u << 24u)
#define TEST_MODE_REGISTER  (0x40260100u)
#define TEST_MODE_VALUE     (0x80000000u)

#define BASE_ADDR_MASK      (0xFFFFF000u)
#define BASE_FORMAT_MASK    (0x00000003u)
#define PIDR0_ADDR          (0x00000FE0u)
#define PIDR4_ADDR          (0x00000FD0u)
#define PIDR_0_3_VALID_MASK (0x000FF000u)
#define PIDR_0_3_VALID_VAL  (0x000B4000u)
#define FAMILY_ID_MASK      (0x00000FFFu)


/******************************************************************************
*  DAP_AP_GetFamilyID
***************************************************************************//**
* Obtain Family ID of PSoC 6 from rom table on the first call and save its value in 
* the global variable familyID.
* Next time only the stored value will be returned
*
* @return 12 bit Family ID
*
******************************************************************************/
static uint16_t DAP_AP_GetFamilyID()
{
    if (familyID == 0)
    {
        uint32_t romBaseAddr;
        bool familyIDValid = true;
        if (swd_read_ap(AP_ROM, &romBaseAddr))
        {
            /* check format of BASE register */
            if ((romBaseAddr & BASE_FORMAT_MASK) == BASE_FORMAT_MASK)
            {
                romBaseAddr &= BASE_ADDR_MASK;
            }
            else
            {
                familyIDValid = false;
            }
            
            uint32_t pidr_0_3 = 0;
            uint32_t pidr_4_7 = 0;

            /* read PIDR0-PIDR3 */
            for (uint32_t i = 0; familyIDValid && (i < 4u); i++)
            {
                uint32_t pidr_val;
                uint32_t pidr_addr = romBaseAddr + PIDR0_ADDR + (i * 4u);
                if(swd_read_word(pidr_addr,&pidr_val))
                {
                    pidr_0_3 |= (pidr_val & 0xFFu) << (i * SHIFT_8);
                }
                else
                {
                    familyIDValid = false;
                }
            }

            /* read PIDR4-PIDR7 */
            for (uint32_t i = 0; familyIDValid && (i < 4u); i++)
            {
                uint32_t pidr_val;
                uint32_t pidr_addr = romBaseAddr + PIDR4_ADDR + (i * 4u);
                if(swd_read_word(pidr_addr,&pidr_val))
                {
                    pidr_4_7 = (pidr_val & 0xFFu) << (i * SHIFT_8);
                }
                else
                {
                    familyIDValid = false;
                }
            }

            /* validate PIDR values and extract family ID */
            if ( familyIDValid && ( (pidr_0_3 & PIDR_0_3_VALID_MASK) == PIDR_0_3_VALID_VAL ) && ( pidr_4_7 == 0u ) )
            {
                familyID = pidr_0_3 & FAMILY_ID_MASK;
            }
            else
            {
                familyIDValid = false;
            }
        }
        
        if (!familyIDValid)
        {
            swd_clear_errors();
        }            
    }
    else
    {
        /* Family ID is already known, no need to get it again */
    }
    return familyID;
}    

/******************************************************************************
*  SWDAcquirePSoC6BLE
***************************************************************************//**
* Acquires PSoC 6 BLE.
*
* @param[in]  acquireMode   Reset or Power cycle acquire.
* @param[in]  attempts      Number of retries before declaring acquire failed.
*
* @return Flag whether the PSoC 6 BLE is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
uint32_t SWDAcquirePSoC6BLE(uint8_t acquireMode, uint8_t attempts)
{
    uint8_t count;
    bool ret = false;
    uint8_t enableInterrupts;

    PORT_SWD_SETUP();
    
    enableInterrupts = CyEnterCriticalSection();

    /* Acquire algorithm for PSoC6 */
    for (count = 0u; count < attempts; count++)
    {
        bool swdAcquired = false;
        uint16_t tmrVal = 0u;
        uint16_t tmrDiff = 0u;

//      just only reset mode implemented
//
//        if (acquireMode == 0u)
//        {
//            (void)RESET_TARGET();
//        }
//        else
//        {
//            /* PowerCycle mode acquire. */
//            if (KitIsMiniProg())
//            {
//                Pin_VoltageEn_Write(0u);
//                CyDelayUs(100u);
//                Pin_VoltageEn_Write(1u);
//                CyDelayUs(100u);
//            }
//        }
            (void)RESET_TARGET();

        /* #1 Try to acquire SWD port */
        tmrVal = Timer_CSTick_ReadCounter();

        do
        {
            /* Send JTAG to SWD sequence with LineReset before and after; Read SWD ID code */
            swdAcquired = JTAG2SWD();

            /* Timer_CSTick is running at 8Hz rate */
            /* It is used as a time source here instead of loop counter */
            /* This ensures we will get `real` timeout value */
            tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
        }
        while ((tmrDiff < PSOC6_TVII_AQUIRE_TIMEOUT_TICKS) && (!swdAcquired));

        if(!swdAcquired)
        {
            /* NACK, try to acquire again */
            continue;
        }

        /* #2 Power up debug port */
        if (!swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) 
        {
            /* NACK, try to acquire again */
            continue;
        }
        
        if ( acquireMode & NOTESTBIT_ACQUIRE ) 
        {
            ret = true;
            break;
        }            

        /* #3 Select SYS AP*/
        g_target_family_psoc6.apsel = SYS_AP;
        
        /* Obtain Family ID */
        DAP_AP_GetFamilyID();

        /* #4 Set Test Mode*/
        if (!swd_write_word(TEST_MODE_REGISTER, TEST_MODE_VALUE))
        {
          /* NACK, try to acquire again */
          continue;
        }

        ret = true;
        break;
    }

    CyExitCriticalSection(enableInterrupts);

    return (ret ? ACQUIRE_PASS : ACQUIRE_FAIL);
}

/******************************************************************************
* Function Name: PSoC6_Ipc_PollLockStatus()
*******************************************************************************
* Summary:
*  Lock defined IPC_STRUCT status
*
* Parameters:
*  ipcID        : IPC_STRUCT ID number 0-  IPC_STRUCT_0, 1- IPC_STRUCT_1,
*                               2- IPC_STRUCT_2
*  lockExpected : expected status value
*  timeout      : IPC_STRUCTX acquring timeout
*  dev_family_lo: device family ID 0 - CY8C6XX6,CY8C6XX7, 1 - CY8C6XX8,CY8C6XXA
*
* Return:
*  nonzero - IPC_STRUCT_X status was sucsesfully locked
*  zero    - IPC_STRUCT_X status was not locked
*
******************************************************************************/
static uint8_t PSoC6_Ipc_PollLockStatus(uint8_t ipcId, uint8_t lockExpected, uint32_t timeout, uint8_t dev_family_lo)
{
    uint32_t timeElapsed = 0u;
    uint32_t readData = 0;
    uint32_t ipcAddr;
    uint8_t status;
    uint8_t isExpectedStatus;

    /* Calculate IPC_STRUCT lock status address based on device family  */
    if (dev_family_lo == PSOC6A_BLE2_FAMILY_ID_LO)
    {
        ipcAddr = IPC_STRUCT0_1M + IPC_STRUCT_SIZE * ipcId + IPC_STRUCT_LOCK_STATUS_OFFSET_1M;
    }
    else
    {
        ipcAddr = IPC_STRUCT0_2M + IPC_STRUCT_SIZE * ipcId + IPC_STRUCT_LOCK_STATUS_OFFSET_2M;
    }

    do
    {
        /* Check Lock Status */
        status = swd_read_word(ipcAddr, &readData);
        if (!status)
        {
            break;
        }
        isExpectedStatus = (lockExpected && (readData & IPC_STRUCT_LOCK_STATUS_ACQUIRED_MSK) ) || (!lockExpected && !(readData & IPC_STRUCT_LOCK_STATUS_ACQUIRED_MSK));
        timeElapsed ++;
    }
    while((isExpectedStatus == 0) && (timeElapsed < timeout));

    return (status);
}

/******************************************************************************
* Function Name: PSoC6_Ipc_Acquire()
*******************************************************************************
* Summary:
*  Acquire defined IPC structure before executing SROM API
*
* Parameters:
*  ipcID : IPC_STRUCT ID number 0-  IPC_STRUCT_0, 1- IPC_STRUCT_1,
*                               2- IPC_STRUCT_2
*  timeout      : IPC_STRUCTX acquring timeout
*  dev_family_lo: device family ID 0 - CY8C6XX6,CY8C6XX7, 1 - CY8C6XX8,CY8C6XXA
*
* Return:
*  nonzero - IPC_STRUCT_X was sucsesfully acquired
*  zero    - IPC_STRUCT_X acquiring was not performed
*
******************************************************************************/
static uint8_t PSoC6_Ipc_Acquire(uint8_t ipcId, uint32_t timeout, uint8_t dev_family_lo)
{
    uint32_t timeElapsed = 0u;
    uint32_t readData = 0;
    uint32_t ipcAddr;
    uint8_t status;

    /* Calculate IPC_STRUCT address based on device family */
    if (dev_family_lo == PSOC6A_BLE2_FAMILY_ID_LO)
    {
        ipcAddr = IPC_STRUCT0_1M + IPC_STRUCT_SIZE * ipcId + IPC_STRUCT_ACQUIRE_OFFSET;
    }
    else
    {
        ipcAddr = IPC_STRUCT0_2M + IPC_STRUCT_SIZE * ipcId + IPC_STRUCT_ACQUIRE_OFFSET;
    }

    do
    {
        /* Acquire the lock in DAP IPC struct (IPC_STRUCT.ACQUIRE).*/
        status = swd_write_word(ipcAddr, IPC_STRUCT_ACQUIRE_SUCCESS_MSK);
        if (!status)
        {
            timeElapsed = timeout;
        }
        else
        {
            /* Read written value back. */
            status = swd_read_word(ipcAddr, &readData);

            if (!status)
            {
                timeElapsed = timeout;
            }
        }

        timeElapsed++;
    }
    while(((readData & IPC_STRUCT_ACQUIRE_SUCCESS_MSK) == 0) && (timeElapsed < timeout));

    if (status)
    {
        /* Check PSoC6_Ipc_PollLockStatus */
        status = PSoC6_Ipc_PollLockStatus(ipcId ,1u ,timeout - timeElapsed, dev_family_lo);
    }

    return (status);
}

/******************************************************************************
* Function Name: PSoC6_PollSromApiStatus
*******************************************************************************
* Summary:
*  Polls the IPC_STRUCT  status/data  till previous API call is completed  or
*  a timeout  condition occurred, whichever is  earlier.
*
* Parameters:
*  addr_32: SROM API status polling address
*  timeout: Polling timeout
*  dataOut: Pinter to API executing data
*
* Return:
*  nonzero - SROM executed the task successfully
*  zero    - SROM task is not executed successfully and a timeout error occured.
*            The failure code is stored in the statusCode global variable.
*
* Note:
*  This function is called after non volatile memory operations like Read,
*  Write of Flash, to check if SROM task has been executed which is indicated
*  by SWD_ACK.
*
******************************************************************************/
static uint8_t PSoC6_PollSromApiStatus(uint32_t addr_32, uint32_t timeout, uint32_t *dataOut)
{
    uint32_t timeElapsed = 0u;
    uint8_t status;

    do
    {
        /* Poll data */
        status =  swd_read_word(addr_32, dataOut);

        if (!status)
        {
            break;
        }

        timeElapsed++;
    }
    while ((( *dataOut & SROM_STATUS_SUCCESS_MASK) != MXS40_SROMAPI_STAT_SUCCESS)\
           && (timeElapsed < timeout));

    return (status);
}

/******************************************************************************
* Function Name: PSoC6_CallSromApi()
*******************************************************************************
* Summary:
*  Call PSoC6 SROM API. Depending on parameters_32 value s API's arguments can
*  be located  in IPC data structure  or in SRAM memory. Besides API's arguments
*  params_32 include API opcode. After API compeletion resulting data is pulled
*  out form IPC data struct or from SRAM
*
* Parameters:
*  params_32 : 4-byte value that combines API opcode and API arguments location
*  data_32   : pionter for 4-byte API return data
*  dev_family_lo   : device family ID 0 - CY8C6XX6,CY8C6XX7, 1 - CY8C6XX8,CY8C6XXA
*
* Return:
*  nonzero - SROM executed the task successfully
*  zero    - SROM task is not executed successfully and a timeout error occured.
*            The failure code is stored in the statusCode global variable.
*
* Note:
*  This function is called to perform volatile memory operations like Read,
*  Write of Flash.
*
******************************************************************************/
static uint8_t PSoC6_CallSromApi(uint32_t params_32, uint32_t *data_32, uint8_t dev_family_lo)
{
    uint8_t status;
    uint8_t apiStatus = 0;

    //  Acquire IPC_STRUCT[2] for DAP
    status = PSoC6_Ipc_Acquire(IPC_2, IPC_STRUCT_ACQUIRE_TIMEOUT, dev_family_lo);

    if (status)
    {
        // Make System Call
        // Write to IPC_STRUCT2.DATA - Sys call ID and Parameters OR address in SRAM, where they are located
        if ((params_32 & MXS40_SROMAPI_DATA_LOCATION_MSK) == 0)
        {
            if (dev_family_lo == PSOC6A_BLE2_FAMILY_ID_LO)
            {
                status = swd_write_word(IPC_STRUCT_DATA_1M, SRAM_SCRATCH_ADDR);
            }
            else
            {
                status = swd_write_word(IPC_STRUCT_DATA_2M, SRAM_SCRATCH_ADDR);
            }
        }
        else
        {
            if (dev_family_lo == PSOC6A_BLE2_FAMILY_ID_LO)
            {
                status = swd_write_word(IPC_STRUCT_DATA_1M, params_32);
            }
            else
            {
                status = swd_write_word(IPC_STRUCT_DATA_2M, params_32);
            }
        }

        if (status)
        {
            // Enable notification interrupt of IPC_INTR_STRUCT0(CM0+) for IPC_STRUCT2
            if (dev_family_lo == PSOC6A_BLE2_FAMILY_ID_LO)
            {
                status = swd_write_word(IPC_INTR_STRUCT_DATA_1M, IPC_INTR_STRUCT_DATA_VALUE);
            }
            else
            {
                status = swd_write_word(IPC_INTR_STRUCT_DATA_2M, IPC_INTR_STRUCT_DATA_VALUE);
            }

            if (status)
            {
                // Notify to IPC_INTR_STRUCT0. IPC_STRUCT2.MASK <- Notify
                if (dev_family_lo == PSOC6A_BLE2_FAMILY_ID_LO)
                {
                    status = swd_write_word(IPC_STRUCT2_NOTIFY_DATA_1M, 0x00000001u);
                }
                else
                {
                    status = swd_write_word(IPC_STRUCT2_NOTIFY_DATA_2M, 0x00000001u);
                }

                if (status)
                {
                    // Poll lock status
                    status = PSoC6_Ipc_PollLockStatus(2u, 0u, IPC_STRUCT_ACQUIRE_TIMEOUT, dev_family_lo);

                    if (status)
                    {
                        // Poll Data byte
                        if ((params_32 & MXS40_SROMAPI_DATA_LOCATION_MSK) == 0)
                        {
                            status = PSoC6_PollSromApiStatus(SRAM_SCRATCH_ADDR, IPC_STRUCT_DATA_TIMEOUT, data_32);
                        }
                        else
                        {
                            if (dev_family_lo == PSOC6A_BLE2_FAMILY_ID_LO)
                            {
                                status = PSoC6_PollSromApiStatus(IPC_STRUCT2_1M + IPC_STRUCT_DATA_OFFSET,
                                        IPC_STRUCT_DATA_TIMEOUT, data_32);
                            }
                            else
                            {
                                status = PSoC6_PollSromApiStatus(IPC_STRUCT2_2M + IPC_STRUCT_DATA_OFFSET,
                                        IPC_STRUCT_DATA_TIMEOUT, data_32);
                            }
                        }

                        if (status)
                        {
                            apiStatus = 0x01;
                        }
                    }
                }
            }
        }
    }

    return apiStatus;
}

/******************************************************************************
* Function Name: GetSiliconId
*******************************************************************************
*
* Summary:
*  This function obtains Silicon Device ID using SROM commands.
*
* Parameters:
*  None
*
* Return:
*  Device ID - Returns Silicon Device ID read from chip.
*  or
*  FAILURE - Returns Failure (0x00000000), if any of the intermediate step
*  returns a fail message.
*
* Note:
*
******************************************************************************/
uint32_t get_silcon_id(void)
{
    // read psoc6 silicon id
    uint32_t target_siid = 0;

    // prepare swd for operations
    swd_init();

    // Acquire PSoC6 device before  reading silicon ID
    if (SWDAcquirePSoC6BLE(RESET_ACQUIRE, NUMBER_OF_ATTEMPTS) == ACQUIRE_PASS)
    {
        uint8_t enableInterrupts = CyEnterCriticalSection();
        uint8_t dev_family_lo = familyID & PSOC6_FAMILY_ID_LO_MSK;

        // get family id and revision id
        uint32_t parameter1 = MXS40_SROMAPI_SILID_CODE + PSOC6_SROM_SILID_CODE_LO;
        uint32_t siliconIdData1 = 0;
        uint32_t siliconIdData2 = 0;

        if (PSoC6_CallSromApi(parameter1, &siliconIdData1, dev_family_lo))
        {
            /* get Silicon ID and protection state */
            parameter1 = MXS40_SROMAPI_SILID_CODE + PSOC6_SROM_SILID_CODE_HI;

            if (PSoC6_CallSromApi(parameter1, &siliconIdData2, dev_family_lo))
            {
                target_siid = (siliconIdData1 & PSOC6_FAMILY_ID_LO_MSK)
                    | ((siliconIdData1 & PSOC6_REV_ID_MAJ_MSK) >> SHIFT_8)
                    | ((siliconIdData1 & PSOC6_REV_ID_MIN_MSK) >> SHIFT_8)
                    | ((siliconIdData2 & PSOC6_SILICON_ID_LO_MSK) << SHIFT_16)
                    | ((siliconIdData2 & PSOC6_SILICON_ID_HI_MSK) << SHIFT_16);
            }
        }
        if (target_siid == 0)
        {
            /* clear swd error if failed to get silicon id */
            swd_clear_errors();
        }

        CyExitCriticalSection(enableInterrupts);
        
    }

    return (target_siid & PSOC6_SILICON_ID_MASK);
}

/******************************************************************************
* Function Name: get_fb_version
*******************************************************************************
*
* Summary:
*  Read version of flash boot.
** 
* Return:
*  version word or 0xFFFFFFFF on error
*
*******************************************************************************/
uint32_t get_fb_version(void)
{
    uint32_t version;
    if (!swd_read_word(FB_VERSION_ADDR, &version))
    {
        swd_clear_errors();
        version = FB_VERSION_UNKNOWN;
    }

    return version;
}
