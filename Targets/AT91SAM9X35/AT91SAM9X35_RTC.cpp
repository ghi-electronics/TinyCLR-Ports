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


#include "AT91.h"


#define AT91_RTC_TIMR_AM    (0x0)
#define AT91_RTC_TIMR_PM    (0x1)

#define AT91C_PMC_CSS   (0x3 << 0)

// Real Time Clock Register
#define AT91C_BASE_RTC  0xFFFFFEB0
#define RTC_CR          (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x00)))
#define RTC_TIMR        (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x08)))
#define RTC_CALR        (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x0C)))
#define RTC_TIMALR      (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x10)))
#define RTC_CALALR      (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x14)))
#define RTC_SR          (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x18)))
#define RTC_SCCR        (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x1C)))
#define RTC_VER         (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_RTC + 0x2C)))

// Real TimeOut
#define RTC_TIMEOUT     (0x00FFFFFF)

//Slow Clock Configuration Register
#define SCKCR_SCKCR     (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_SCKCR + 0x0)))

//Power Manager Control Register
#define PMC_MCKR        (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_PMC + 0x0030)))
#define PMC_SR          (*(reinterpret_cast<volatile uint32_t*>(AT91C_BASE_PMC + 0x0068)))

#define TOTAL_RTC_CONTROLLERS 1

static TinyCLR_Rtc_Controller rtcControllers[TOTAL_RTC_CONTROLLERS];
static TinyCLR_Api_Info rtcApi[TOTAL_RTC_CONTROLLERS];

const char* rtcApiNames[TOTAL_RTC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.AT91.RtcController\\0"
};

void AT91_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_RTC_CONTROLLERS; i++) {
        rtcControllers[i].ApiInfo = &rtcApi[i];
        rtcControllers[i].Acquire = &AT91_Rtc_Acquire;
        rtcControllers[i].Release = &AT91_Rtc_Release;
        rtcControllers[i].IsValid = &AT91_Rtc_IsValid;
        rtcControllers[i].GetTime = &AT91_Rtc_GetTime;
        rtcControllers[i].SetTime = &AT91_Rtc_SetTime;

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

void AT91_Rtc_BinaryCodedDecimalExtract(uint32_t valueToConvert, uint32_t &tens, uint32_t &ones) {
    tens = valueToConvert / 10;
    ones = valueToConvert % 10;
}

uint32_t AT91_Rtc_BinaryCodedDecimalCombine(uint32_t tens, uint32_t ones) {
    uint32_t CombinedBinaryCodedDecimal = 0;

    CombinedBinaryCodedDecimal = tens * 10;
    CombinedBinaryCodedDecimal += ones;

    return CombinedBinaryCodedDecimal;
}

TinyCLR_Result AT91_Rtc_Acquire(const TinyCLR_Rtc_Controller* self) {
    if ((PMC_MCKR & AT91C_PMC_CSS) != 0) {
        volatile uint32_t dumpReg = SCKCR_SCKCR;

        //Enable the 32,768 Hz oscillator by setting the bit OSC32EN to 1.
        SCKCR_SCKCR |= (1 << 1);

        //Wait 32,768 Hz Startup Time for clock stabilization (software loop).
        AT91_Time_Delay(nullptr, 100000);

        //Switch from internal RC to 32,768 Hz oscillator by setting the bit OSCSEL to 1.
        SCKCR_SCKCR |= (1 << 3);

        //Wait 5 slow clock cycles for internal resynchronization.
        AT91_Time_Delay(nullptr, 100000);

        //Disable the RC oscillator by setting the bit RCEN to 0.
        SCKCR_SCKCR &= ~(1 << 0);

        dumpReg = SCKCR_SCKCR;
        dumpReg = PMC_SR;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Rtc_Release(const TinyCLR_Rtc_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Rtc_IsValid(const TinyCLR_Rtc_Controller* self, bool& value) {
    TinyCLR_Rtc_DateTime rtcNow;

    value = (AT91_Rtc_GetTime(self, rtcNow) == TinyCLR_Result::Success);

    if (rtcNow.Second >= 60 || rtcNow.Minute >= 60 || rtcNow.Hour >= 24 || rtcNow.DayOfMonth >= 32 || rtcNow.Month >= 13 || rtcNow.DayOfWeek >= 8)
        value = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value) {
    uint32_t calenderRegister = RTC_CALR;
    uint32_t timeRegister = RTC_TIMR;
    uint32_t fullYear = 0;
    uint32_t hundredYear = 0;

    if (RTC_VER > 0) { // Valid Entry Register, detect any incorrect value this register will be not 0.
        return TinyCLR_Result::InvalidOperation;
    }
    else {
        if ((calenderRegister & 0x7F) == 0x19)
            fullYear = 1900;
        else if ((calenderRegister & 0x7F) == 0x20)
            fullYear = 2000;

        hundredYear = AT91_Rtc_BinaryCodedDecimalCombine((((calenderRegister & (0xFF << 8)) >> 8) >> 4), (((calenderRegister & (0xFF << 8)) >> 8) & 0xF));
        value.Year = (uint32_t)(fullYear + hundredYear);
        value.Month = (uint32_t)AT91_Rtc_BinaryCodedDecimalCombine((((calenderRegister & (0x1F << 16)) >> 16) >> 4), (((calenderRegister & (0x1F << 16)) >> 16) & 0xF));
        value.DayOfMonth = (uint32_t)AT91_Rtc_BinaryCodedDecimalCombine((((calenderRegister & (0x3F << 24)) >> 24) >> 4), (((calenderRegister & (0x3F << 24)) >> 24) & 0xF));
        value.DayOfWeek = (uint32_t)AT91_Rtc_BinaryCodedDecimalCombine((((calenderRegister & (0x07 << 21)) >> 21) >> 4), (((calenderRegister & (0x07 << 21)) >> 21) & 0xF));
        value.Hour = AT91_Rtc_BinaryCodedDecimalCombine((((timeRegister & (0x3F << 16)) >> 16) >> 4), (((timeRegister & (0x3F << 16)) >> 16) & 0xF));

        if (((timeRegister & 0x400000) >> 22) == AT91_RTC_TIMR_PM)
            value.Hour = value.Hour + 12;
        else
            value.Hour = value.Hour;

        value.Minute = (uint32_t)AT91_Rtc_BinaryCodedDecimalCombine((((timeRegister & (0x7F << 8)) >> 8) >> 4), (((timeRegister & (0x7F << 8)) >> 8) & 0xF));
        value.Second = (uint32_t)AT91_Rtc_BinaryCodedDecimalCombine(((timeRegister & 0x7F) >> 4), ((timeRegister & 0x7F) & 0xF));
        value.Millisecond = 1;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value) {
    uint32_t calenderRegister = 0;
    uint32_t timeRegister = 0;
    uint32_t lowerHundredYears = 0;
    uint32_t tens = 0;
    uint32_t ones = 0;
    uint32_t timeout = 0;

    if (RTC_VER > 0) { // Valid Entry Register, detect any incorrect value this register will be not 0.
        return TinyCLR_Result::InvalidOperation;
    }

    if ((value.Year < 1900) || (value.Year > 2099) ||
        (value.Month < 1) || (value.Month > 12) ||
        (value.DayOfMonth < 1) || (value.DayOfMonth > 31)) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    RTC_CR = 0x2;

    while ((RTC_SR & 0x1) == 0);

    while ((RTC_SR & 0x4) == 0);

    if (value.Year < 2000) {
        calenderRegister |= ((0x1 << 4) | 0x9);
        lowerHundredYears = value.Year - 1900;
    }
    else {
        calenderRegister |= ((0x2 << 4) | 0x0);
        lowerHundredYears = value.Year - 2000;
    }

    // Add value.Year
    AT91_Rtc_BinaryCodedDecimalExtract(lowerHundredYears, tens, ones);
    calenderRegister |= (uint32_t)((tens << 12) | (ones << 8));
    // Add value.Month
    AT91_Rtc_BinaryCodedDecimalExtract(value.Month, tens, ones);
    calenderRegister |= (uint32_t)((tens << 20) | (ones << 16));
    // Add dayOfWeek
    calenderRegister |= (uint32_t)(((value.DayOfWeek + 1) << 21));
    // Add value.DayOfMonth
    AT91_Rtc_BinaryCodedDecimalExtract(value.DayOfMonth, tens, ones);
    calenderRegister |= (uint32_t)((tens << 28) | (ones << 24));

    AT91_Time_Delay(nullptr, 500000);
    // Write Calender to register
    RTC_CALR = calenderRegister;
    timeRegister = 0;

    RTC_CR |= (1 << 0);

    timeout = 0;
    while ((timeRegister & 0x1) == 0) {
        timeRegister = RTC_SR;

        if (timeout++ > RTC_TIMEOUT)
            return TinyCLR_Result::InvalidOperation;
    }

    timeout = 0;
    while (timeRegister != 0) {
        timeRegister = RTC_TIMR;
        RTC_TIMR = 0;

        if (timeout++ > RTC_TIMEOUT)
            return TinyCLR_Result::InvalidOperation;
    }

    // Add hour
    AT91_Rtc_BinaryCodedDecimalExtract(value.Hour, tens, ones);
    timeRegister = (uint32_t)((AT91_RTC_TIMR_AM << 22) | (tens << 20) | (ones << 16));
    RTC_TIMR = timeRegister;

    // Add value.Minute
    AT91_Rtc_BinaryCodedDecimalExtract(value.Minute, tens, ones);
    timeRegister = (uint32_t)((tens << 12) | (ones << 8));
    RTC_TIMR |= timeRegister;

    // Add value.Second
    AT91_Rtc_BinaryCodedDecimalExtract(value.Second, tens, ones);
    timeRegister = (uint32_t)((tens << 4) | ones);
    RTC_TIMR |= timeRegister;

    // Clear Status Register
    RTC_SCCR |= 1 << 2;
    RTC_CR &= ~((1 << 0) | (1 << 1));

    return TinyCLR_Result::Success;
}
