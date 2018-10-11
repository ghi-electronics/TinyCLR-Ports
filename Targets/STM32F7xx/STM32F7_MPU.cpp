#include "STM32F7.h"
#include <stdio.h>

/* @Specifies the control mode of the MPU during hard fault*/
#define  MPU_HFNMI_PRIVDEF_NONE      ((uint32_t)0x00000000)
#define  MPU_HARDFAULT_NMI           ((uint32_t)0x00000002)
#define  MPU_PRIVILEGED_DEFAULT      ((uint32_t)0x00000004)
#define  MPU_HFNMI_PRIVDEF           ((uint32_t)0x00000006)

/* @defgroup CORTEX_MPU_Region_Enable CORTEX MPU Region Enable*/
#define  MPU_REGION_ENABLE     ((uint8_t)0x01)
#define  MPU_REGION_DISABLE    ((uint8_t)0x00)

/* @defgroup CORTEX_MPU_Instruction_Access CORTEX MPU Instruction Access*/
#define  MPU_INSTRUCTION_ACCESS_ENABLE      ((uint8_t)0x00)
#define  MPU_INSTRUCTION_ACCESS_DISABLE     ((uint8_t)0x01)

/* @defgroup CORTEX_MPU_Access_Shareable CORTEX MPU Instruction Access Shareable*/
#define  MPU_ACCESS_SHAREABLE        ((uint8_t)0x01)
#define  MPU_ACCESS_NOT_SHAREABLE    ((uint8_t)0x00)

/* @defgroup CORTEX_MPU_Access_Cacheable CORTEX MPU Instruction Access Cacheable*/
#define  MPU_ACCESS_CACHEABLE         ((uint8_t)0x01)
#define  MPU_ACCESS_NOT_CACHEABLE     ((uint8_t)0x00)

/* @defgroup CORTEX_MPU_Access_Bufferable CORTEX MPU Instruction Access Bufferable */
#define  MPU_ACCESS_BUFFERABLE         ((uint8_t)0x01)
#define  MPU_ACCESS_NOT_BUFFERABLE     ((uint8_t)0x00)

/* @defgroup CORTEX_MPU_TEX_Levels MPU TEX Levels */
#define  MPU_TEX_LEVEL0    ((uint8_t)0x00)
#define  MPU_TEX_LEVEL1    ((uint8_t)0x01)
#define  MPU_TEX_LEVEL2    ((uint8_t)0x02)

/* @defgroup CORTEX_MPU_Region_Permission_Attributes CORTEX MPU Region Permission Attributes */
#define  MPU_REGION_NO_ACCESS      ((uint8_t)0x00)
#define  MPU_REGION_PRIV_RW        ((uint8_t)0x01)
#define  MPU_REGION_PRIV_RW_URO    ((uint8_t)0x02)
#define  MPU_REGION_FULL_ACCESS    ((uint8_t)0x03)
#define  MPU_REGION_PRIV_RO        ((uint8_t)0x05)
#define  MPU_REGION_PRIV_RO_URO    ((uint8_t)0x06)

void STM32F7_Mpu_Enable(uint32_t mpuControl) {
    /* Enable the MPU */
    MPU->CTRL = mpuControl | MPU_CTRL_ENABLE_Msk;

    /* Enable fault exceptions */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
}

void STM32F7_Mpu_Disable() {
    /* Disable fault exceptions */
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;

    /* Disable the MPU */
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
}

void STM32F7_Mpu_Configuration(uint32_t address, STM32F7_Mpu_RegionSize mpuRegionSize, STM32F7_Mpu_RegionNumber mpuRegionNumber, bool doCache) {    
    auto size = (uint32_t)mpuRegionSize;
    auto regionNumber = (uint32_t)mpuRegionNumber;

    STM32F7_Mpu_Disable();
    
    MPU->RNR = regionNumber;
    MPU->RBAR = address;

    uint32_t rasrReg = (MPU_REGION_ENABLE << MPU_RASR_ENABLE_Pos) | (MPU_REGION_FULL_ACCESS << MPU_RASR_AP_Pos) | (MPU_ACCESS_NOT_SHAREABLE << MPU_RASR_S_Pos) | (MPU_TEX_LEVEL0 << MPU_RASR_TEX_Pos) | (MPU_INSTRUCTION_ACCESS_ENABLE << MPU_RASR_XN_Pos) | (size << MPU_RASR_SIZE_Pos);

    if (doCache) {
        rasrReg |= (MPU_ACCESS_BUFFERABLE << MPU_RASR_B_Pos) | (MPU_ACCESS_CACHEABLE << MPU_RASR_C_Pos);
    }
    else {
        rasrReg |= (MPU_ACCESS_NOT_BUFFERABLE << MPU_RASR_B_Pos) | (MPU_ACCESS_NOT_CACHEABLE << MPU_RASR_C_Pos);
    }

    MPU->RASR = rasrReg;

    STM32F7_Mpu_Enable(MPU_HFNMI_PRIVDEF);
}

void STM32F7_Mpu_Reset() {
    STM32F7_Mpu_Disable();

    for (auto i = static_cast<uint8_t>(STM32F7_Mpu_RegionNumber::Region0); i <=  static_cast<uint8_t>(STM32F7_Mpu_RegionNumber::Region7); i++) {
        MPU->RNR = i;
        MPU->RBAR = 0;
        MPU->RASR = 0;
    }

    STM32F7_Mpu_Enable(MPU_HFNMI_PRIVDEF);
}