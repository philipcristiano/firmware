/* 
 * File:   platform_system_flags.h
 * Author: mat
 *
 * Created on 12 November 2014, 06:24
 */

#ifndef PLATFORM_SYSTEM_FLAGS_H
#define	PLATFORM_SYSTEM_FLAGS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>    
    
typedef struct platform_system_flags {    
        uint16_t header[2];
    uint16_t CORE_FW_Version_SysFlag;
    uint16_t NVMEM_SPARK_Reset_SysFlag;
    uint16_t FLASH_OTA_Update_SysFlag;
    uint16_t OTA_FLASHED_Status_SysFlag;
    uint16_t Factory_Reset_SysFlag;
    uint16_t IWDG_Enable_SysFlag;
    uint8_t dfu_on_no_firmware;     // flag to enable DFU mode when no firmware is available.
    uint8_t unused;    
    uint16_t reserved[7];
} platform_system_flags_t;


#ifdef	__cplusplus
}
#endif

#endif	/* PLATFORM_SYSTEM_FLAGS_H */

