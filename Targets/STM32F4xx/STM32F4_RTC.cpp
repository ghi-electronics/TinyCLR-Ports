// Copyright GHI Electronics, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "STM32F4.h"

#define PWR_OFFSET               (PWR_BASE - PERIPH_BASE)

/* --- CR Register ---*/

/* Alias word address of DBP bit */
#define CR_OFFSET                (PWR_OFFSET + 0x00)
#define DBP_BitNumber            0x08
#define CR_DBP_BB                (PERIPH_BB_BASE + (CR_OFFSET * 32) + (DBP_BitNumber * 4))

/* ------------ RCC registers bit address in the alias region ----------- */
#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)

/* Alias word address of RTCEN bit */
#define BDCR_OFFSET               (RCC_OFFSET + 0x70)
#define RTCEN_BitNumber           0x0F
#define BDCR_RTCEN_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (RTCEN_BitNumber * 4))

/* BDCR register base address */
#define BDCR_ADDRESS              (PERIPH_BASE + BDCR_OFFSET)

#define RCC_LSE_OFF                      ((uint8_t)0x00)
#define RCC_LSE_ON                       ((uint8_t)0x01)

#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x3B)
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_BORRST                  ((uint8_t)0x79)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)

#define RCC_FLAG_MASK                 ((uint8_t)0x1F)

#define RCC_RTCCLKSource_LSE             ((uint32_t)0x00000100)

/** @defgroup RTC_Hour_Formats
  * @{
  */
#define RTC_HourFormat_24              ((uint32_t)0x00000000)
#define RTC_HourFormat_12              ((uint32_t)0x00000040)

  /* Masks Definition */
#define RTC_TR_RESERVED_MASK    ((uint32_t)0x007F7F7F)
#define RTC_DR_RESERVED_MASK    ((uint32_t)0x00FFFF3F)
#define RTC_INIT_MASK           ((uint32_t)0xFFFFFFFF)
#define RTC_RSF_MASK            ((uint32_t)0xFFFFFF5F)

#define RTC_BKP_DR0                       ((uint32_t)0x00000000)

#define RTC_BKP_VALUE   0x32F2

static TinyCLR_Rtc_Provider rtcProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* STM32F4_Rtc_GetApi() {
    rtcProvider.Parent = &timeApi;
    rtcProvider.Index = 0;
    rtcProvider.Acquire = &STM32F4_Rtc_Acquire;
    rtcProvider.Release = &STM32F4_Rtc_Release;
    rtcProvider.GetNow = &STM32F4_Rtc_GetNow;
    rtcProvider.SetNow = &STM32F4_Rtc_SetNow;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.RtcProvider";
    timeApi.Type = TinyCLR_Api_Type::RtcProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &rtcProvider;

    return &timeApi;
}
uint8_t STM32F4_Rtc_ByteToBcd2(uint8_t value) {
    uint8_t bcdhigh = 0;

    while (value >= 10) {
        bcdhigh++;
        value -= 10;
    }
    return  ((uint8_t)(bcdhigh << 4) | value);
}

uint8_t STM32F4_Rtc_Bcd2ToByte(uint8_t value) {
    return ((((uint8_t)(value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10) + (value & (uint8_t)0x0F));
}

TinyCLR_Result STM32F4_Rtc_EnterInitializeMode() {
    int timeout = 0xFFFFFF;
    /* Check if the Initialization mode is set */
    if ((RTC->ISR & RTC_ISR_INITF) == 0) {
        /* Set the Initialization mode */
        RTC->ISR |= RTC_INIT_MASK;
        /* Wait till RTC is in INIT state and if Time out is reached exit */
        while ((timeout-- > 0) && ((RTC->ISR & RTC_ISR_INITF) == 0));
    }
    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;

}

TinyCLR_Result STM32F4_Rtc_ExitInitializeMode() {
    RTC->ISR &= (uint32_t)~RTC_ISR_INIT;
}

void STM32F4_Rtc_EnableWriteProtection() {
    RTC->WPR = 0xFF;
}

void STM32F4_Rtc_DisableWriteProtection() {
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
}

TinyCLR_Result STM32F4_Rtc_WaitForSynchro() {

    int32_t timeout = 0xFFFFFF;
    /* Disable the write protection for RTC registers */
    // RTC->WPR = 0xCA;
    // RTC->WPR = 0x53;
    STM32F4_Rtc_DisableWriteProtection();

    /* Clear RSF flag */
    RTC->ISR &= (uint32_t)RTC_RSF_MASK;

    /* Wait the registers to be synchronised */
    while (((RTC->ISR & RTC_ISR_RSF) == 0) && (timeout-- > 0));

    /* Enable the write protection for RTC registers */
    //RTC->WPR = 0xFF;
    STM32F4_Rtc_EnableWriteProtection();

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;

}

TinyCLR_Result STM32F4_Rtc_Configuration() {
    /* Enable the PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Allow access to RTC */
    *(reinterpret_cast<uint32_t *>(CR_DBP_BB)) = 1;

    /* Enable the LSE OSC */
    /* Reset LSEON bit */
    *(reinterpret_cast<uint32_t *>(BDCR_ADDRESS)) = RCC_LSE_OFF;

    /* Reset LSEBYP bit */
    *(reinterpret_cast<uint32_t *>(BDCR_ADDRESS)) = RCC_LSE_OFF;

    /* Set LSEON bit */
    *(reinterpret_cast<uint32_t *>(BDCR_ADDRESS)) = RCC_LSE_ON;

    /* Wait till LSE is ready */
    uint32_t flag_mask = RCC_FLAG_LSERDY & RCC_FLAG_MASK;
    uint32_t status_reg = RCC->BDCR;

    int timeout = 0xFFFFFF;

    while ((status_reg & ((uint32_t)1 << flag_mask)) == (uint32_t)RESET && timeout-- > 0) {
        status_reg = RCC->BDCR;
    }

    if (timeout == 0) {
        return TinyCLR_Result::InvalidOperation;
    }

    /* Select the RTC Clock Source */
    RCC->BDCR |= (RCC_RTCCLKSource_LSE & 0x00000FFF);

    /* Enable the RTC Clock */
    *(reinterpret_cast<uint32_t *>(BDCR_RTCEN_BB)) = 1;

    /* Wait for RTC APB registers synchronisation */
    return STM32F4_Rtc_WaitForSynchro();
}

TinyCLR_Result STM32F4_Rtc_Initialize() {

    /* Disable the write protection for RTC registers */
    //RTC->WPR = 0xCA;
    //RTC->WPR = 0x53;
    STM32F4_Rtc_DisableWriteProtection();

    if (STM32F4_Rtc_EnterInitializeMode() != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    /* Clear RTC CR FMT Bit */
    RTC->CR &= ((uint32_t)~(RTC_CR_FMT));
    /* Set RTC_CR register */
    RTC->CR |= ((uint32_t)(RTC_HourFormat_24));

    /* Configure the RTC PRER */
    RTC->PRER = (uint32_t)(0xFF);
    RTC->PRER |= (uint32_t)(0x7F << 16);

    /* Exit Initialization mode */
    STM32F4_Rtc_ExitInitializeMode();

    /* Enable the write protection for RTC registers */
    //RTC->WPR = 0xFF;
    STM32F4_Rtc_EnableWriteProtection();

    return TinyCLR_Result::Success;
}

uint32_t STM32F4_Rtc_ReadBackupRegister() {
    /* Read the specified register */
    return (*(reinterpret_cast<uint32_t *>(RTC_BASE + 0x50 + (RTC_BKP_DR0 * 4))));

}

void STM32F4_Rtc_WriteBackupRegister(uint32_t value) {
    /* Write the specified register */
    *(reinterpret_cast<uint32_t *>(RTC_BASE + 0x50 + (RTC_BKP_DR0 * 4))) = value;

}

TinyCLR_Result STM32F4_Rtc_Acquire(const TinyCLR_Rtc_Provider* self) {
    if (STM32F4_Rtc_ReadBackupRegister() == RTC_BKP_VALUE) {

    }

    if (STM32F4_Rtc_Configuration() != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    if (STM32F4_Rtc_Initialize() != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Rtc_Release(const TinyCLR_Rtc_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Rtc_GetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime& value) {
    uint32_t  time;
    uint32_t  date;

    // value.Year = 2010;
    // value.Month = 2;
    // value.DayOfYear = 33;
    // value.DayOfMonth = 2;
    // value.DayOfWeek = 0;
    // value.Hour = 13;
    // value.Minute = 14;
    // value.Second = 15;
    // value.Millisecond = 516;

    /* Get the RTC_TR register */
    time = (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK);

    uint8_t hour = (uint8_t)((time & (RTC_TR_HT | RTC_TR_HU)) >> 16);
    uint8_t minute = (uint8_t)((time & (RTC_TR_MNT | RTC_TR_MNU)) >> 8);
    uint8_t second = (uint8_t)(time & (RTC_TR_ST | RTC_TR_SU));

    value.Hour = (uint8_t)STM32F4_Rtc_Bcd2ToByte(hour);
    value.Minute = (uint8_t)STM32F4_Rtc_Bcd2ToByte(minute);
    value.Second = (uint8_t)STM32F4_Rtc_Bcd2ToByte(second);
    value.Millisecond = 0;

    /* Get the RTC_TR register */
    date = (uint32_t)(RTC->DR & RTC_DR_RESERVED_MASK);

    uint8_t year = (uint8_t)((date & (RTC_DR_YT | RTC_DR_YU)) >> 16);
    uint8_t month = (uint8_t)((date & (RTC_DR_MT | RTC_DR_MU)) >> 8);
    uint8_t day_of_month = (uint8_t)(date & (RTC_DR_DT | RTC_DR_DU));
    uint8_t day_of_week = (uint8_t)((date & (RTC_DR_WDU)) >> 13);

    value.Year = (uint8_t)STM32F4_Rtc_Bcd2ToByte(year) + 1980;
    value.Month = (uint8_t)STM32F4_Rtc_Bcd2ToByte(month);
    value.DayOfMonth = (uint8_t)STM32F4_Rtc_Bcd2ToByte(day_of_month);
    value.DayOfWeek = (uint8_t)STM32F4_Rtc_Bcd2ToByte(day_of_week);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Rtc_SetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime value) {
    uint32_t  time;
    uint32_t  date;

    time = (uint32_t)(((uint32_t)STM32F4_Rtc_ByteToBcd2(value.Hour) << 16) | \
        ((uint32_t)STM32F4_Rtc_ByteToBcd2(value.Minute) << 8) | \
        ((uint32_t)STM32F4_Rtc_ByteToBcd2(value.Second)));

    date = (((uint32_t)STM32F4_Rtc_ByteToBcd2(value.Year) << 16) | \
        ((uint32_t)STM32F4_Rtc_ByteToBcd2(value.Month) << 8) | \
        ((uint32_t)STM32F4_Rtc_ByteToBcd2(value.DayOfMonth)) | \
        ((uint32_t)value.DayOfWeek << 13));

    /* Disable the write protection for RTC registers */
    STM32F4_Rtc_DisableWriteProtection();

    /* Set Initialization mode */
    if (STM32F4_Rtc_EnterInitializeMode() != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    /* Set the RTC_TR register */
    RTC->TR = (uint32_t)(time & RTC_TR_RESERVED_MASK);

    /* Set the RTC_DR register */
    RTC->DR = (uint32_t)(date & RTC_DR_RESERVED_MASK);

    STM32F4_Rtc_ExitInitializeMode();

    STM32F4_Rtc_WaitForSynchro();

    /* Enable the write protection for RTC registers */
    STM32F4_Rtc_EnableWriteProtection();

    STM32F4_Rtc_WriteBackupRegister(RTC_BKP_VALUE);

    return TinyCLR_Result::Success;
}
