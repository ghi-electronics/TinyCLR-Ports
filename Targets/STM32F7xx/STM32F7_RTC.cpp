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

#include "STM32F7.h"

#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region                                */

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

#define RTC_TIMEOUT 0xFFFFFF

#define TOTAL_RTC_CONTROLLERS 1

static TinyCLR_Rtc_Controller rtcControllers[TOTAL_RTC_CONTROLLERS];
static TinyCLR_Api_Info rtcApi[TOTAL_RTC_CONTROLLERS];

const char* rtcApiNames[TOTAL_RTC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.STM32F7.RtcController\\0"
};

void STM32F7_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_RTC_CONTROLLERS; i++) {
        rtcControllers[i].ApiInfo = &rtcApi[i];
        rtcControllers[i].Acquire = &STM32F7_Rtc_Acquire;
        rtcControllers[i].Release = &STM32F7_Rtc_Release;
        rtcControllers[i].IsValid = &STM32F7_Rtc_IsValid;
        rtcControllers[i].GetTime = &STM32F7_Rtc_GetTime;
        rtcControllers[i].SetTime = &STM32F7_Rtc_SetTime;

        rtcApi[i].Author = "GHI Electronics, LLC";
        rtcApi[i].Name = rtcApiNames[i];
        rtcApi[i].Type = TinyCLR_Api_Type::RtcController;
        rtcApi[i].Version = 0;
        rtcApi[i].Implementation = &rtcControllers[i];
        rtcApi[i].State = nullptr;

        apiManager->Add(apiManager, &rtcApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::RtcController, rtcApi[0].Name);
}

uint8_t STM32F7_Rtc_ByteToBcd2(uint8_t value) {
    uint8_t bcdhigh = 0;

    while (value >= 10) {
        bcdhigh++;
        value -= 10;
    }
    return  ((uint8_t)(bcdhigh << 4) | value);
}

uint8_t STM32F7_Rtc_Bcd2ToByte(uint8_t value) {
    return ((((uint8_t)(value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10) + (value & (uint8_t)0x0F));
}

TinyCLR_Result STM32F7_Rtc_SetInitializeMode(bool set) {
    int timeout = RTC_TIMEOUT;
    if (set) {
        /* Check if the Initialization mode is set */
        if ((RTC->ISR & RTC_ISR_INITF) == 0) {
            /* Set the Initialization mode */
            RTC->ISR |= RTC_INIT_MASK;
            /* Wait till RTC is in INIT state and if Time out is reached exit */
            while ((timeout-- > 0) && ((RTC->ISR & RTC_ISR_INITF) == 0));
        }
    }
    else {
        RTC->ISR &= ~RTC_ISR_INIT;
    }

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

void STM32F7_Rtc_SetWriteProtection(bool set) {
    if (set)
        RTC->WPR = 0xFF;
    else {
        RTC->WPR = 0xCA;
        RTC->WPR = 0x53;
    }
}

TinyCLR_Result STM32F7_Rtc_WaitForSynchro() {
    int32_t timeout = RTC_TIMEOUT;

    /* Disable the write protection for RTC registers */
    STM32F7_Rtc_SetWriteProtection(false);

    /* Clear RSF flag */
    RTC->ISR &= RTC_RSF_MASK;

    /* Wait the registers to be synchronised */
    while (((RTC->ISR & RTC_ISR_RSF) == 0) && (timeout-- > 0));

    /* Enable the write protection for RTC registers */
    STM32F7_Rtc_SetWriteProtection(true);

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result STM32F7_Rtc_Configuration() {
    /* Enable the PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Allow access to RTC */
    PWR->CR1 |= PWR_CR1_DBP;

    /* Clear LSI */
    RCC->CSR &= ~(RCC_CSR_LSION);

    /* Enable LSE */
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);

    /* Wait till LSE is ready */
    uint32_t flag_mask = RCC_FLAG_LSERDY & RCC_FLAG_MASK;
    uint32_t status_reg = RCC->BDCR;

    int timeout = RTC_TIMEOUT;

    while ((status_reg & (1 << flag_mask)) == RESET && timeout-- > 0) {
        status_reg = RCC->BDCR;
    }

    if (timeout == 0) {
        return TinyCLR_Result::InvalidOperation;
    }

    /* Select the RTC Clock Source */
    RCC->BDCR |= (RCC_RTCCLKSource_LSE & 0x00000FFF);

    MODIFY_REG(RCC->BDCR, RCC_BDCR_LSEDRV, (uint32_t)(RCC_BDCR_LSEDRV));

    /* Enable the RTC Clock */
    RCC->BDCR |= (RCC_BDCR_RTCEN);

    /* Wait for RTC APB registers synchronisation */
    return STM32F7_Rtc_WaitForSynchro();
}

TinyCLR_Result STM32F7_Rtc_Initialize() {
    /* Disable the write protection for RTC registers */
    STM32F7_Rtc_SetWriteProtection(false);

    if (STM32F7_Rtc_SetInitializeMode(true) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    /* Clear RTC CR FMT Bit */
    RTC->CR &= ~(RTC_CR_FMT);
    /* Set RTC_CR register */
    RTC->CR |= RTC_HourFormat_24;

    /* Configure the RTC PRER */
    RTC->PRER = 0xFF;
    RTC->PRER |= 0x7F << 16;

    /* Exit Initialization mode */
    STM32F7_Rtc_SetInitializeMode(false);

    /* Enable the write protection for RTC registers */
    STM32F7_Rtc_SetWriteProtection(true);

    return TinyCLR_Result::Success;
}

void STM32F7_Rtc_WriteBackupRegister() {
    *(reinterpret_cast<uint32_t *>(RTC_BASE + 0x50)) = (uint32_t)0x32F2;
}

uint32_t STM32F7_Rtc_ReadBackupRegister() {
    return *(reinterpret_cast<uint32_t *>(RTC_BASE + 0x50));
}

TinyCLR_Result STM32F7_Rtc_Acquire(const TinyCLR_Rtc_Controller* self) {
    if (STM32F7_Rtc_Configuration() != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    if (STM32F7_Rtc_Initialize() != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Rtc_Release(const TinyCLR_Rtc_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Rtc_IsValid(const TinyCLR_Rtc_Controller* self, bool& value) {
    TinyCLR_Rtc_DateTime rtcNow;

    value = (STM32F7_Rtc_GetTime(self, rtcNow) == TinyCLR_Result::Success);

    if (rtcNow.Second >= 60 || rtcNow.Minute >= 60 || rtcNow.Hour >= 24 || rtcNow.DayOfMonth >= 32 || rtcNow.Month >= 13 || rtcNow.DayOfWeek >= 8)
        value = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value) {
    uint32_t  time;
    uint32_t  date;

    if (STM32F7_Rtc_ReadBackupRegister() != 0x32F2) {
        return TinyCLR_Result::InvalidOperation;
    }

    /* Get the RTC_TR register */
    time = RTC->TR & RTC_TR_RESERVED_MASK;

    uint8_t hour = static_cast<uint8_t>((time & (RTC_TR_HT | RTC_TR_HU)) >> 16);
    uint8_t minute = static_cast<uint8_t>((time & (RTC_TR_MNT | RTC_TR_MNU)) >> 8);
    uint8_t second = static_cast<uint8_t>(time & (RTC_TR_ST | RTC_TR_SU));

    value.Hour = STM32F7_Rtc_Bcd2ToByte(hour);
    value.Minute = STM32F7_Rtc_Bcd2ToByte(minute);
    value.Second = STM32F7_Rtc_Bcd2ToByte(second);
    value.Millisecond = 0;

    /* Get the RTC_TR register */
    date = RTC->DR & RTC_DR_RESERVED_MASK;

    uint8_t year = static_cast<uint8_t>((date & (RTC_DR_YT | RTC_DR_YU)) >> 16);
    uint8_t month = static_cast<uint8_t>((date & (RTC_DR_MT | RTC_DR_MU)) >> 8);
    uint8_t day_of_month = static_cast<uint8_t>(date & (RTC_DR_DT | RTC_DR_DU));
    uint8_t day_of_week = static_cast<uint8_t>((date & (RTC_DR_WDU)) >> 13);

    value.Year = STM32F7_Rtc_Bcd2ToByte(year) + 1980;
    value.Month = STM32F7_Rtc_Bcd2ToByte(month);
    value.DayOfMonth = STM32F7_Rtc_Bcd2ToByte(day_of_month);
    value.DayOfWeek = STM32F7_Rtc_Bcd2ToByte(day_of_week);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value) {
    uint32_t  time;
    uint32_t  date;

    time = (STM32F7_Rtc_ByteToBcd2(value.Hour) << 16) | (STM32F7_Rtc_ByteToBcd2(value.Minute) << 8) | (STM32F7_Rtc_ByteToBcd2(value.Second));
    date = (STM32F7_Rtc_ByteToBcd2(value.Year - 1980) << 16) | (STM32F7_Rtc_ByteToBcd2(value.Month) << 8) | (STM32F7_Rtc_ByteToBcd2(value.DayOfMonth)) | (value.DayOfWeek << 13);

    /* Disable the write protection for RTC registers */
    STM32F7_Rtc_SetWriteProtection(false);

    /* Set Initialization mode */
    if (STM32F7_Rtc_SetInitializeMode(true) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    /* Set the RTC_TR register */
    RTC->TR = time & RTC_TR_RESERVED_MASK;

    /* Set the RTC_DR register */
    RTC->DR = date & RTC_DR_RESERVED_MASK;

    STM32F7_Rtc_SetInitializeMode(false);

    STM32F7_Rtc_WaitForSynchro();

    /* Enable the write protection for RTC registers */
    STM32F7_Rtc_SetWriteProtection(true);

    STM32F7_Rtc_WriteBackupRegister();

    if (STM32F7_Rtc_ReadBackupRegister() != 0x32F2) { // Ensure data is written
        return TinyCLR_Result::InvalidOperation;
    }

    return TinyCLR_Result::Success;
}
