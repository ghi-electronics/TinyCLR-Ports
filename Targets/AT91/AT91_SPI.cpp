// Copyright Microsoft Corporation
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

#define SSP0_BASE 0xE0068000

#define SSP0CR0 (*(volatile unsigned long *)0xE0068000)
#define SSP0CR0_OFFSET 0x0
#define SSP0CR0_SCR_MASK 0xFF00
#define SSP0CR0_SCR_BIT 8
#define SSP0CR0_CPHA_MASK 0x80
#define SSP0CR0_CPHA 0x80
#define SSP0CR0_CPHA_BIT 7
#define SSP0CR0_CPOL_MASK 0x40
#define SSP0CR0_CPOL 0x40
#define SSP0CR0_CPOL_BIT 6
#define SSP0CR0_FRF_MASK 0x30
#define SSP0CR0_FRF_BIT 4
#define SSP0CR0_DSS_MASK 0xF
#define SSP0CR0_DSS_BIT 0

#define SSP0CR1 (*(volatile unsigned long *)0xE0068004)
#define SSP0CR1_OFFSET 0x4
#define SSP0CR1_SOD_MASK 0x8
#define SSP0CR1_SOD 0x8
#define SSP0CR1_SOD_BIT 3
#define SSP0CR1_MS_MASK 0x4
#define SSP0CR1_MS 0x4
#define SSP0CR1_MS_BIT 2
#define SSP0CR1_SSE_MASK 0x2
#define SSP0CR1_SSE 0x2
#define SSP0CR1_SSE_BIT 1
#define SSP0CR1_LBE_MASK 0x1
#define SSP0CR1_LBE 0x1
#define SSP0CR1_LBE_BIT 0

#define SSP0DR (*(volatile unsigned long *)0xE0068008)
#define SSP0DR_OFFSET 0x8

#define SSP0SR (*(volatile unsigned long *)0xE006800C)
#define SSP0SR_OFFSET 0xC
#define SSP0SR_BSY_MASK 0x10
#define SSP0SR_BSY 0x10
#define SSP0SR_BSY_BIT 4
#define SSP0SR_RFF_MASK 0x8
#define SSP0SR_RFF 0x8
#define SSP0SR_RFF_BIT 3
#define SSP0SR_RNE_MASK 0x4
#define SSP0SR_RNE 0x4
#define SSP0SR_RNE_BIT 2
#define SSP0SR_TNF_MASK 0x2
#define SSP0SR_TNF 0x2
#define SSP0SR_TNF_BIT 1
#define SSP0SR_TFE_MASK 0x1
#define SSP0SR_TFE 0x1
#define SSP0SR_TFE_BIT 0

#define SSP0CPSR (*(volatile unsigned long *)0xE0068010)
#define SSP0CPSR_OFFSET 0x10
#define SSP0CPSR_CPSDVSR_MASK 0xFF
#define SSP0CPSR_CPSDVSR_BIT 0

#define SSP0IMSC (*(volatile unsigned long *)0xE0068014)
#define SSP0IMSC_OFFSET 0x14
#define SSP0IMSC_TXIM_MASK 0x8
#define SSP0IMSC_TXIM 0x8
#define SSP0IMSC_TXIM_BIT 3
#define SSP0IMSC_RXIM_MASK 0x4
#define SSP0IMSC_RXIM 0x4
#define SSP0IMSC_RXIM_BIT 2
#define SSP0IMSC_RTIM_MASK 0x2
#define SSP0IMSC_RTIM 0x2
#define SSP0IMSC_RTIM_BIT 1
#define SSP0IMSC_RORIM_MASK 0x1
#define SSP0IMSC_RORIM 0x1
#define SSP0IMSC_RORIM_BIT 0

#define SSP0RIS (*(volatile unsigned long *)0xE0068018)
#define SSP0RIS_OFFSET 0x18
#define SSP0RIS_TXRIS_MASK 0x8
#define SSP0RIS_TXRIS 0x8
#define SSP0RIS_TXRIS_BIT 3
#define SSP0RIS_RXRIS_MASK 0x4
#define SSP0RIS_RXRIS 0x4
#define SSP0RIS_RXRIS_BIT 2
#define SSP0RIS_RTRIS_MASK 0x2
#define SSP0RIS_RTRIS 0x2
#define SSP0RIS_RTRIS_BIT 1
#define SSP0RIS_RORRIS_MASK 0x1
#define SSP0RIS_RORRIS 0x1
#define SSP0RIS_RORRIS_BIT 0

#define SSP0MIS (*(volatile unsigned long *)0xE006801C)
#define SSP0MIS_OFFSET 0x1C
#define SSP0MIS_TXMIS_MASK 0x8
#define SSP0MIS_TXMIS 0x8
#define SSP0MIS_TXMIS_BIT 3
#define SSP0MIS_RXMIS_MASK 0x4
#define SSP0MIS_RXMIS 0x4
#define SSP0MIS_RXMIS_BIT 2
#define SSP0MIS_RTMIS_MASK 0x2
#define SSP0MIS_RTMIS 0x2
#define SSP0MIS_RTMIS_BIT 1
#define SSP0MIS_RORMIS_MASK 0x1
#define SSP0MIS_RORMIS 0x1
#define SSP0MIS_RORMIS_BIT 0

#define SSP0ICR (*(volatile unsigned long *)0xE0068020)
#define SSP0ICR_OFFSET 0x20
#define SSP0ICR_RTIC_MASK 0x2
#define SSP0ICR_RTIC 0x2
#define SSP0ICR_RTIC_BIT 1
#define SSP0ICR_RORIC_MASK 0x1
#define SSP0ICR_RORIC 0x1
#define SSP0ICR_RORIC_BIT 0

#define SSP0DMACR (*(volatile unsigned long *)0xE0068024)
#define SSP0DMACR_OFFSET 0x24
#define SSP0DMACR_RXDMAE_MASK 0x1
#define SSP0DMACR_RXDMAE 0x1
#define SSP0DMACR_RXDMAE_BIT 0
#define SSP0DMACR_TXDMAE_MASK 0x2
#define SSP0DMACR_TXDMAE 0x2
#define SSP0DMACR_TXDMAE_BIT 1

#define SSP1_BASE 0xE0030000

#define SSP1CR0 (*(volatile unsigned long *)0xE0030000)
#define SSP1CR0_OFFSET 0x0
#define SSP1CR0_SCR_MASK 0xFF00
#define SSP1CR0_SCR_BIT 8
#define SSP1CR0_CPHA_MASK 0x80
#define SSP1CR0_CPHA 0x80
#define SSP1CR0_CPHA_BIT 7
#define SSP1CR0_CPOL_MASK 0x40
#define SSP1CR0_CPOL 0x40
#define SSP1CR0_CPOL_BIT 6
#define SSP1CR0_FRF_MASK 0x30
#define SSP1CR0_FRF_BIT 4
#define SSP1CR0_DSS_MASK 0xF
#define SSP1CR0_DSS_BIT 0

#define SSP1CR1 (*(volatile unsigned long *)0xE0030004)
#define SSP1CR1_OFFSET 0x4
#define SSP1CR1_SOD_MASK 0x8
#define SSP1CR1_SOD 0x8
#define SSP1CR1_SOD_BIT 3
#define SSP1CR1_MS_MASK 0x4
#define SSP1CR1_MS 0x4
#define SSP1CR1_MS_BIT 2
#define SSP1CR1_SSE_MASK 0x2
#define SSP1CR1_SSE 0x2
#define SSP1CR1_SSE_BIT 1
#define SSP1CR1_LBE_MASK 0x1
#define SSP1CR1_LBE 0x1
#define SSP1CR1_LBE_BIT 0

#define SSP1DR (*(volatile unsigned long *)0xE0030008)
#define SSP1DR_OFFSET 0x8

#define SSP1SR (*(volatile unsigned long *)0xE003000C)
#define SSP1SR_OFFSET 0xC
#define SSP1SR_BSY_MASK 0x10
#define SSP1SR_BSY 0x10
#define SSP1SR_BSY_BIT 4
#define SSP1SR_RFF_MASK 0x8
#define SSP1SR_RFF 0x8
#define SSP1SR_RFF_BIT 3
#define SSP1SR_RNE_MASK 0x4
#define SSP1SR_RNE 0x4
#define SSP1SR_RNE_BIT 2
#define SSP1SR_TNF_MASK 0x2
#define SSP1SR_TNF 0x2
#define SSP1SR_TNF_BIT 1
#define SSP1SR_TFE_MASK 0x1
#define SSP1SR_TFE 0x1
#define SSP1SR_TFE_BIT 0

#define SSP1CPSR (*(volatile unsigned long *)0xE0030010)
#define SSP1CPSR_OFFSET 0x10
#define SSP1CPSR_CPSDVSR_MASK 0xFF
#define SSP1CPSR_CPSDVSR_BIT 0

#define SSP1IMSC (*(volatile unsigned long *)0xE0030014)
#define SSP1IMSC_OFFSET 0x14
#define SSP1IMSC_TXIM_MASK 0x8
#define SSP1IMSC_TXIM 0x8
#define SSP1IMSC_TXIM_BIT 3
#define SSP1IMSC_RXIM_MASK 0x4
#define SSP1IMSC_RXIM 0x4
#define SSP1IMSC_RXIM_BIT 2
#define SSP1IMSC_RTIM_MASK 0x2
#define SSP1IMSC_RTIM 0x2
#define SSP1IMSC_RTIM_BIT 1
#define SSP1IMSC_RORIM_MASK 0x1
#define SSP1IMSC_RORIM 0x1
#define SSP1IMSC_RORIM_BIT 0

#define SSP1RIS (*(volatile unsigned long *)0xE0030018)
#define SSP1RIS_OFFSET 0x18
#define SSP1RIS_TXRIS_MASK 0x8
#define SSP1RIS_TXRIS 0x8
#define SSP1RIS_TXRIS_BIT 3
#define SSP1RIS_RXRIS_MASK 0x4
#define SSP1RIS_RXRIS 0x4
#define SSP1RIS_RXRIS_BIT 2
#define SSP1RIS_RTRIS_MASK 0x2
#define SSP1RIS_RTRIS 0x2
#define SSP1RIS_RTRIS_BIT 1
#define SSP1RIS_RORRIS_MASK 0x1
#define SSP1RIS_RORRIS 0x1
#define SSP1RIS_RORRIS_BIT 0

#define SSP1MIS (*(volatile unsigned long *)0xE003001C)
#define SSP1MIS_OFFSET 0x1C
#define SSP1MIS_TXMIS_MASK 0x8
#define SSP1MIS_TXMIS 0x8
#define SSP1MIS_TXMIS_BIT 3
#define SSP1MIS_RXMIS_MASK 0x4
#define SSP1MIS_RXMIS 0x4
#define SSP1MIS_RXMIS_BIT 2
#define SSP1MIS_RTMIS_MASK 0x2
#define SSP1MIS_RTMIS 0x2
#define SSP1MIS_RTMIS_BIT 1
#define SSP1MIS_RORMIS_MASK 0x1
#define SSP1MIS_RORMIS 0x1
#define SSP1MIS_RORMIS_BIT 0

#define SSP1ICR (*(volatile unsigned long *)0xE0030020)
#define SSP1ICR_OFFSET 0x20
#define SSP1ICR_RTIC_MASK 0x2
#define SSP1ICR_RTIC 0x2
#define SSP1ICR_RTIC_BIT 1
#define SSP1ICR_RORIC_MASK 0x1
#define SSP1ICR_RORIC 0x1
#define SSP1ICR_RORIC_BIT 0

#define SSP1DMACR (*(volatile unsigned long *)0xE0030024)
#define SSP1DMACR_OFFSET 0x24
#define SSP1DMACR_RXDMAE_MASK 0x1
#define SSP1DMACR_RXDMAE 0x1
#define SSP1DMACR_RXDMAE_BIT 0
#define SSP1DMACR_TXDMAE_MASK 0x2
#define SSP1DMACR_TXDMAE 0x2
#define SSP1DMACR_TXDMAE_BIT 1

#define DATA_BIT_LENGTH_16  16
#define DATA_BIT_LENGTH_8   8

static const uint32_t g_AT91_Spi_Miso_Pins[] = AT91_SPI_MISO_PINS;
static const uint32_t g_AT91_Spi_Mosi_Pins[] = AT91_SPI_MOSI_PINS;
static const uint32_t g_AT91_Spi_Sclk_Pins[] = AT91_SPI_CLK_PINS;

static const AT91_Gpio_PinFunction g_AT91_Spi_Miso_AltMode[] = AT91_SPI_MISO_ALT_MODE;
static const AT91_Gpio_PinFunction g_AT91_Spi_Mosi_AltMode[] = AT91_SPI_MOSI_ALT_MODE;
static const AT91_Gpio_PinFunction g_AT91_Spi_Sclk_AltMode[] = AT91_SPI_CLK_ALT_MODE;

struct SpiController {
    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t writeLength;
    size_t readLength;
    size_t readOffset;

    int32_t ChipSelectLine;
    int32_t ClockFrequency;
    int32_t DataBitLength;

    TinyCLR_Spi_Mode Mode;
};

static SpiController g_SpiController[TOTAL_SPI_CONTROLLERS];

static uint8_t spiProviderDefs[TOTAL_SPI_CONTROLLERS * sizeof(TinyCLR_Spi_Provider)];
static TinyCLR_Spi_Provider* spiProviders[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi;

const TinyCLR_Api_Info* AT91_Spi_GetApi() {
    for (int i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        spiProviders[i] = (TinyCLR_Spi_Provider*)(spiProviderDefs + (i * sizeof(TinyCLR_Spi_Provider)));
        spiProviders[i]->Parent = &spiApi;
        spiProviders[i]->Index = i;
        spiProviders[i]->Acquire = &AT91_Spi_Acquire;
        spiProviders[i]->Release = &AT91_Spi_Release;
        spiProviders[i]->SetActiveSettings = &AT91_Spi_SetActiveSettings;
        spiProviders[i]->Read = &AT91_Spi_Read;
        spiProviders[i]->Write = &AT91_Spi_Write;
        spiProviders[i]->TransferFullDuplex = &AT91_Spi_TransferFullDuplex;
        spiProviders[i]->TransferSequential = &AT91_Spi_TransferSequential;
        spiProviders[i]->GetChipSelectLineCount = &AT91_Spi_GetChipSelectLineCount;
        spiProviders[i]->GetMinClockFrequency = &AT91_Spi_GetMinClockFrequency;
        spiProviders[i]->GetMaxClockFrequency = &AT91_Spi_GetMaxClockFrequency;
        spiProviders[i]->GetSupportedDataBitLengths = &AT91_Spi_GetSupportedDataBitLengths;
    }

    spiApi.Author = "GHI Electronics, LLC";
    spiApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.SpiProvider";
    spiApi.Type = TinyCLR_Api_Type::SpiProvider;
    spiApi.Version = 0;
    spiApi.Count = TOTAL_SPI_CONTROLLERS;
    spiApi.Implementation = spiProviders;
    return &spiApi;
}

bool AT91_Spi_Transaction_Start(int32_t controller) {

    uint32_t clkPin, misoPin, mosiPin;
    AT91_Gpio_PinFunction clkMode, misoMode, mosiMode;

    AT91XX_SPI & SPI = AT91XX::SPI(controller);

    clkPin = g_AT91_Spi_Sclk_Pins[controller];
    misoPin = g_AT91_Spi_Miso_Pins[controller];
    mosiPin = g_AT91_Spi_Mosi_Pins[controller];

    clkMode = g_AT91_Spi_Sclk_AltMode[controller];
    misoMode = g_AT91_Spi_Miso_AltMode[controller];
    mosiMode = g_AT91_Spi_Mosi_AltMode[controller];

    int SCR, CPSDVSR;
    uint32_t clockKhz = g_SpiController[controller].ClockFrequency / 1000;
    uint32_t divider = (100 * AT91XX_SPI::c_SPI_Clk_KHz / clockKhz); // 100 is only to avoid floating points
    divider /= 2; // because CPSDVSR is even numbeer 2 to 254, so we are calculating using X = 2*CPSDVSR (x:1 to 127);
    divider += 50;
    divider /= 100;

    SCR = divider / (127 + 1);// assuming X = Max value = 127
    CPSDVSR = divider / (SCR + 1); // This is X
    CPSDVSR *= 2;

    if (SCR > 255) {

        SCR = 255;

    }

    if (CPSDVSR <= 2) {

        CPSDVSR = 2;

    }

    if (CPSDVSR >= 255) {

        CPSDVSR = 254;

    }

    SPI.SSPxCPSR = CPSDVSR; // An even number between 2 and 254

    AT91_Gpio_ConfigurePin(clkPin, AT91_Gpio_Direction::Input, clkMode, AT91_Gpio_PinMode::Inactive);
    AT91_Gpio_ConfigurePin(misoPin, AT91_Gpio_Direction::Input, misoMode, AT91_Gpio_PinMode::Inactive);
    AT91_Gpio_ConfigurePin(mosiPin, AT91_Gpio_Direction::Input, mosiMode, AT91_Gpio_PinMode::Inactive);


    SPI.SSPxCR1 = 0x02;//master

    // set how many bits
    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {

        SPI.SSPxCR0 = 0x0F;

    }
    /*
    else if(AT91XX_SPI_Driver::is9Bit[index]) { // 9 bit

        SPI.SSPxCR0 = 0x08;
    }
    */
    else { // 8 bit

        SPI.SSPxCR0 = 0x07;

    }

    SPI.SSPxCR0 &= ~((1 << 4) | (1 << 5));// SPI mode
    SPI.SSPxCR0 &= ~(1 << 7);
    SPI.SSPxCR0 &= ~(1 << 6);

    switch (g_SpiController[controller].Mode) {

        case TinyCLR_Spi_Mode::Mode0: // CPOL = 0, CPHA = 0.

            break;

        case TinyCLR_Spi_Mode::Mode1: // CPOL = 0, CPHA = 1.
            SPI.SSPxCR0 |= (1 << 7);
            break;

        case TinyCLR_Spi_Mode::Mode2: //  CPOL = 1, CPHA = 0.
            SPI.SSPxCR0 |= (1 << 6);
            break;

        case TinyCLR_Spi_Mode::Mode3: // CPOL = 1, CPHA = 1
            SPI.SSPxCR0 |= (1 << 6) | (1 << 7);
            break;
    }

    SPI.SSPxCR0 &= ~(0xFF << 8);
    SPI.SSPxCR0 |= (SCR << 8);

    // CS setup
    AT91_Gpio_EnableOutputPin(g_SpiController[controller].ChipSelectLine, false);

    return true;
}

bool AT91_Spi_Transaction_Stop(int32_t controller) {
    AT91_Gpio_Write(nullptr, g_SpiController[controller].ChipSelectLine, TinyCLR_Gpio_PinValue::High);

    TinyCLR_Gpio_PinDriveMode res = TinyCLR_Gpio_PinDriveMode::InputPullDown;

    if (g_SpiController[controller].Mode == TinyCLR_Spi_Mode::Mode3) {
        res = TinyCLR_Gpio_PinDriveMode::InputPullUp;
    }

    int32_t clkPin = g_AT91_Spi_Sclk_Pins[controller];
    int32_t misoPin = g_AT91_Spi_Miso_Pins[controller];
    int32_t mosiPin = g_AT91_Spi_Mosi_Pins[controller];


    AT91_Gpio_EnableInputPin(clkPin, res);
    AT91_Gpio_EnableInputPin(misoPin, TinyCLR_Gpio_PinDriveMode::InputPullDown);
    AT91_Gpio_EnableInputPin(mosiPin, TinyCLR_Gpio_PinDriveMode::InputPullDown);

    return true;
}


bool AT91_Spi_Transaction_nWrite8_nRead8(int32_t controller) {

    uint8_t Data8;
    uint8_t* Write8 = g_SpiController[controller].writeBuffer;
    int32_t WriteCount = g_SpiController[controller].writeLength;
    uint8_t* Read8 = g_SpiController[controller].readBuffer;
    int32_t ReadCount = g_SpiController[controller].readLength;
    int32_t ReadStartOffset = g_SpiController[controller].readOffset;
    int32_t ReadTotal = 0;

    AT91XX_SPI & SPI = AT91XX::SPI(controller);

    if (ReadCount) {
        ReadTotal = ReadCount + ReadStartOffset; // we need to read as many bytes as the buffer is long, plus the offset at which we start
    }

    // nothing to read, just write to make it faster
    if (ReadCount == 0) {
        while (WriteCount--) {
            SPI.SSPxDR = *Write8++;
            while ((SPI.SSPxSR & 0x02) == 0);
        }

        // complete and discard anything else
        while (SPI.SSPxSR & 0x01 == 0); // more bytes to transmit
        while (SPI.SSPxSR & 0x10);//BSY

        // get all bytes
        while (SPI.SSPxSR & 0x4) {
            Data8 = SPI.SSPxDR;
            while (SPI.SSPxSR & 0x10);//BSY
        }

        return true;
    }

    int32_t loopCnt = ReadTotal;

    // take the max of Read+offset or WrCnt
    if (loopCnt < WriteCount) {
        loopCnt = WriteCount;
    }
    // we will use WriteCount to move in the Write8 array
    // so we do no want to go past the end when we will check for
    // WriteCount to be bigger than zero
    WriteCount -= 1;

    // Start transmission
    while (loopCnt--) {
        // Write Transmit Data
        SPI.SSPxDR = Write8[0];

        // repeat last write word for all subsequent reads
        if (WriteCount) {
            WriteCount--;
            Write8++;
        }

        // wait while the Transmission is in progress
        // No error checking as there is no mechanism to report errors
        if (controller == 0) { // SPI 0\
            while (!(SSP0SR & 0x04));//SPIF
            while ((SSP0SR & 0x10));//BSY

        }
        else {
            while (!(SSP1SR & 0x04));//SPIF
            while ((SSP1SR & 0x10));//BSY
        }
        // Read recieved data
        if (controller == 0) {
            Data8 = ((uint8_t)SSP0DR);

        }
        else {
            Data8 = ((uint8_t)SSP1DR);
        }

        // only save data once we have reached ReadCount-1 portion of words
        ReadTotal--;

        if ((ReadTotal >= 0) && (ReadTotal < ReadCount)) {
            Read8[0] = Data8;
            Read8++;
        }
    }

    return true;
}

bool AT91_Spi_Transaction_nWrite16_nRead16(int32_t controller) {

    return true;
}

TinyCLR_Result AT91_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (AT91_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return AT91_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result AT91_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = readBuffer;
    g_SpiController[controller].readLength = readLength;
    g_SpiController[controller].writeBuffer = (uint8_t*)writeBuffer;
    g_SpiController[controller].writeLength = writeLength;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = buffer;
    g_SpiController[controller].readLength = length;
    g_SpiController[controller].writeBuffer = nullptr;
    g_SpiController[controller].writeLength = 0;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = nullptr;
    g_SpiController[controller].readLength = 0;
    g_SpiController[controller].writeBuffer = (uint8_t*)buffer;
    g_SpiController[controller].writeLength = length;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    int32_t controller = (self->Index);

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].ChipSelectLine = chipSelectLine;
    g_SpiController[controller].ClockFrequency = clockFrequency;
    g_SpiController[controller].DataBitLength = dataBitLength;
    g_SpiController[controller].Mode = mode;
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Acquire(const TinyCLR_Spi_Provider* self) {
    int32_t controller = (self->Index);

    int32_t clkPin = g_AT91_Spi_Sclk_Pins[controller];
    int32_t misoPin = g_AT91_Spi_Miso_Pins[controller];
    int32_t mosiPin = g_AT91_Spi_Mosi_Pins[controller];

    // Check each pin single time make sure once fail not effect to other pins
    if (!AT91_Gpio_OpenPin(clkPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(misoPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(mosiPin))
        return TinyCLR_Result::SharingViolation;

    switch (controller) {
        case 0:
            AT91XX::SYSCON().PCONP |= PCONP_PCSSP0;
            break;

        case 1:
            AT91XX::SYSCON().PCONP |= PCONP_PCSSP1;
            break;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Release(const TinyCLR_Spi_Provider* self) {
    int32_t controller = (self->Index);

    int32_t clkPin = g_AT91_Spi_Sclk_Pins[controller];
    int32_t misoPin = g_AT91_Spi_Miso_Pins[controller];
    int32_t mosiPin = g_AT91_Spi_Mosi_Pins[controller];

    // Check each pin single time make sure once fail not effect to other pins
    AT91_Gpio_ClosePin(clkPin);
    AT91_Gpio_ClosePin(misoPin);
    AT91_Gpio_ClosePin(mosiPin);


    switch (controller) {
        case 0:
            AT91XX::SYSCON().PCONP &= ~PCONP_PCSSP0;
            break;

        case 1:
            AT91XX::SYSCON().PCONP &= ~PCONP_PCSSP1;
            break;

    }

    return TinyCLR_Result::Success;
}

int32_t AT91_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self) {
    // Theoretically this could read the Clock and PLL configurations
    // to determine an actual realistic minimum, however there doesn't
    // seem to be a lot of value in that since the CPU_SPI_Xaction_Start
    // has to determine the applicability of the selected speed at the
    // time a transaction is started anyway.
    return 1;
}

int32_t AT91_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self) {
    // Theoretically this could read the Clock and PLL configurations
    // to determine an actual realistic maximum, however there doesn't
    // seem to be a lot of value in that since the CPU_SPI_Xaction_Start
    // has to determine the applicability of the selected speed at the
    // time a transaction is started anyway.
    // Max supported (e.g. not overclocked) AHB speed / 2
    return 48000000;
}

int32_t AT91_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self) {
    // This could maintain a map of the actual pins
    // that are available for a particular port.
    // (Not all pins can be mapped to all ports.)
    // The value of doing that, however, is marginal
    // since the count of possible chip selects doesn't
    // really help in determining which chip select to
    // use so just report the total count of all GPIO
    // pins as possible so that the selected Chip select
    // line coresponds to a GPIO pin number directly
    // without needing any additional translation/mapping.
    return TOTAL_GPIO_PINS;
}

static const int32_t dataBitsCount = 2;
static int32_t dataBits[dataBitsCount] = { 8, 16 };

TinyCLR_Result AT91_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        memcpy(dataBitLengths, dataBits, (dataBitsCount < dataBitLengthsCount ? dataBitsCount : dataBitLengthsCount) * sizeof(int32_t));

    dataBitLengthsCount = dataBitsCount;

    return TinyCLR_Result::Success;
}

void AT91_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        AT91_Spi_Release(spiProviders[i]);
    }
}