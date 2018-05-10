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

#include <string.h>
#include <LPC17.h>

#define DATA_BIT_LENGTH_16  16
#define DATA_BIT_LENGTH_8   8

#define SSP0_BASE 0x40088000//0xE0030000

#define SSP0CR0 (*(volatile unsigned long *)0x40088000)//0xE0030000)
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

#define SSP0CR1 (*(volatile unsigned long *)0x40088004)//0xE0030004)
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

#define SSP0DR (*(volatile unsigned long *)0x40088008)//0xE0030008)
#define SSP0DR_OFFSET 0x8

#define SSP0SR (*(volatile unsigned long *)0x4008800C)//0xE003000C)
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

#define SSP0CPSR (*(volatile unsigned long *)0x40088010)//0xE0030010)
#define SSP0CPSR_OFFSET 0x10
#define SSP0CPSR_CPSDVSR_MASK 0xFF
#define SSP0CPSR_CPSDVSR_BIT 0

#define SSP0IMSC (*(volatile unsigned long *)0x40088014)//0xE0030014)
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

#define SSP0RIS (*(volatile unsigned long *)0x40088018)//0xE0030018)
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

#define SSP0MIS (*(volatile unsigned long *)0x4008801C)//0xE003001C)
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

#define SSP0ICR (*(volatile unsigned long *)0x40088020)//0xE0030020)
#define SSP0ICR_OFFSET 0x20
#define SSP0ICR_RTIC_MASK 0x2
#define SSP0ICR_RTIC 0x2
#define SSP0ICR_RTIC_BIT 1
#define SSP0ICR_RORIC_MASK 0x1
#define SSP0ICR_RORIC 0x1
#define SSP0ICR_RORIC_BIT 0

#define SSP0DMACR (*(volatile unsigned long *)0x40088024)//0xE0030024)
#define SSP0DMACR_OFFSET 0x24
#define SSP0DMACR_RXDMAE_MASK 0x1
#define SSP0DMACR_RXDMAE 0x1
#define SSP0DMACR_RXDMAE_BIT 0
#define SSP0DMACR_TXDMAE_MASK 0x2
#define SSP0DMACR_TXDMAE 0x2
#define SSP0DMACR_TXDMAE_BIT 1

// SPI 1
#define SSP1_BASE 0x40030000//0xE0030000

#define SSP1CR0 (*(volatile unsigned long *)0x40030000)//0xE0030000)
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

#define SSP1CR1 (*(volatile unsigned long *)0x40030004)//0xE0030004)
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

#define SSP1DR (*(volatile unsigned long *)0x40030008)//0xE0030008)
#define SSP1DR_OFFSET 0x8

#define SSP1SR (*(volatile unsigned long *)0x4003000C)//0xE003000C)
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

#define SSP1CPSR (*(volatile unsigned long *)0x40030010)//0xE0030010)
#define SSP1CPSR_OFFSET 0x10
#define SSP1CPSR_CPSDVSR_MASK 0xFF
#define SSP1CPSR_CPSDVSR_BIT 0

#define SSP1IMSC (*(volatile unsigned long *)0x40030014)//0xE0030014)
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

#define SSP1RIS (*(volatile unsigned long *)0x40030018)//0xE0030018)
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

#define SSP1MIS (*(volatile unsigned long *)0x4003001C)//0xE003001C)
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

#define SSP1ICR (*(volatile unsigned long *)0x40030020)//0xE0030020)
#define SSP1ICR_OFFSET 0x20
#define SSP1ICR_RTIC_MASK 0x2
#define SSP1ICR_RTIC 0x2
#define SSP1ICR_RTIC_BIT 1
#define SSP1ICR_RORIC_MASK 0x1
#define SSP1ICR_RORIC 0x1
#define SSP1ICR_RORIC_BIT 0

#define SSP1DMACR (*(volatile unsigned long *)0x40030024)//0xE0030024)
#define SSP1DMACR_OFFSET 0x24
#define SSP1DMACR_RXDMAE_MASK 0x1
#define SSP1DMACR_RXDMAE 0x1
#define SSP1DMACR_RXDMAE_BIT 0
#define SSP1DMACR_TXDMAE_MASK 0x2
#define SSP1DMACR_TXDMAE 0x2
#define SSP1DMACR_TXDMAE_BIT 1


// SPI2
#define SSP2_BASE 0x400AC000//0x400AC000

#define SSP2CR0 (*(volatile unsigned long *)0x400AC000)//0x400AC000)
#define SSP2CR0_OFFSET 0x0
#define SSP2CR0_SCR_MASK 0xFF00
#define SSP2CR0_SCR_BIT 8
#define SSP2CR0_CPHA_MASK 0x80
#define SSP2CR0_CPHA 0x80
#define SSP2CR0_CPHA_BIT 7
#define SSP2CR0_CPOL_MASK 0x40
#define SSP2CR0_CPOL 0x40
#define SSP2CR0_CPOL_BIT 6
#define SSP2CR0_FRF_MASK 0x30
#define SSP2CR0_FRF_BIT 4
#define SSP2CR0_DSS_MASK 0xF
#define SSP2CR0_DSS_BIT 0

#define SSP2CR1 (*(volatile unsigned long *)0x400AC004)//0x400AC004)
#define SSP2CR1_OFFSET 0x4
#define SSP2CR1_SOD_MASK 0x8
#define SSP2CR1_SOD 0x8
#define SSP2CR1_SOD_BIT 3
#define SSP2CR1_MS_MASK 0x4
#define SSP2CR1_MS 0x4
#define SSP2CR1_MS_BIT 2
#define SSP2CR1_SSE_MASK 0x2
#define SSP2CR1_SSE 0x2
#define SSP2CR1_SSE_BIT 1
#define SSP2CR1_LBE_MASK 0x1
#define SSP2CR1_LBE 0x1
#define SSP2CR1_LBE_BIT 0

#define SSP2DR (*(volatile unsigned long *)0x400AC008)//0x400AC008)
#define SSP2DR_OFFSET 0x8

#define SSP2SR (*(volatile unsigned long *)0x400AC00C)//0x400AC00C)
#define SSP2SR_OFFSET 0xC
#define SSP2SR_BSY_MASK 0x10
#define SSP2SR_BSY 0x10
#define SSP2SR_BSY_BIT 4
#define SSP2SR_RFF_MASK 0x8
#define SSP2SR_RFF 0x8
#define SSP2SR_RFF_BIT 3
#define SSP2SR_RNE_MASK 0x4
#define SSP2SR_RNE 0x4
#define SSP2SR_RNE_BIT 2
#define SSP2SR_TNF_MASK 0x2
#define SSP2SR_TNF 0x2
#define SSP2SR_TNF_BIT 1
#define SSP2SR_TFE_MASK 0x1
#define SSP2SR_TFE 0x1
#define SSP2SR_TFE_BIT 0

#define SSP2CPSR (*(volatile unsigned long *)0x400AC010)//0x400AC010)
#define SSP2CPSR_OFFSET 0x10
#define SSP2CPSR_CPSDVSR_MASK 0xFF
#define SSP2CPSR_CPSDVSR_BIT 0

#define SSP2IMSC (*(volatile unsigned long *)0x400AC014)//0x400AC014)
#define SSP2IMSC_OFFSET 0x14
#define SSP2IMSC_TXIM_MASK 0x8
#define SSP2IMSC_TXIM 0x8
#define SSP2IMSC_TXIM_BIT 3
#define SSP2IMSC_RXIM_MASK 0x4
#define SSP2IMSC_RXIM 0x4
#define SSP2IMSC_RXIM_BIT 2
#define SSP2IMSC_RTIM_MASK 0x2
#define SSP2IMSC_RTIM 0x2
#define SSP2IMSC_RTIM_BIT 1
#define SSP2IMSC_RORIM_MASK 0x1
#define SSP2IMSC_RORIM 0x1
#define SSP2IMSC_RORIM_BIT 0

#define SSP2RIS (*(volatile unsigned long *)0x400AC018)//0x400AC018)
#define SSP2RIS_OFFSET 0x18
#define SSP2RIS_TXRIS_MASK 0x8
#define SSP2RIS_TXRIS 0x8
#define SSP2RIS_TXRIS_BIT 3
#define SSP2RIS_RXRIS_MASK 0x4
#define SSP2RIS_RXRIS 0x4
#define SSP2RIS_RXRIS_BIT 2
#define SSP2RIS_RTRIS_MASK 0x2
#define SSP2RIS_RTRIS 0x2
#define SSP2RIS_RTRIS_BIT 1
#define SSP2RIS_RORRIS_MASK 0x1
#define SSP2RIS_RORRIS 0x1
#define SSP2RIS_RORRIS_BIT 0

#define SSP2MIS (*(volatile unsigned long *)0x400AC01C)//0x400AC01C)
#define SSP2MIS_OFFSET 0x1C
#define SSP2MIS_TXMIS_MASK 0x8
#define SSP2MIS_TXMIS 0x8
#define SSP2MIS_TXMIS_BIT 3
#define SSP2MIS_RXMIS_MASK 0x4
#define SSP2MIS_RXMIS 0x4
#define SSP2MIS_RXMIS_BIT 2
#define SSP2MIS_RTMIS_MASK 0x2
#define SSP2MIS_RTMIS 0x2
#define SSP2MIS_RTMIS_BIT 1
#define SSP2MIS_RORMIS_MASK 0x1
#define SSP2MIS_RORMIS 0x1
#define SSP2MIS_RORMIS_BIT 0

#define SSP2ICR (*(volatile unsigned long *)0x400AC020)//0x400AC020)
#define SSP2ICR_OFFSET 0x20
#define SSP2ICR_RTIC_MASK 0x2
#define SSP2ICR_RTIC 0x2
#define SSP2ICR_RTIC_BIT 1
#define SSP2ICR_RORIC_MASK 0x1
#define SSP2ICR_RORIC 0x1
#define SSP2ICR_RORIC_BIT 0

#define SSP2DMACR (*(volatile unsigned long *)0x400AC024)//0x400AC024)
#define SSP2DMACR_OFFSET 0x24
#define SSP2DMACR_RXDMAE_MASK 0x1
#define SSP2DMACR_RXDMAE 0x1
#define SSP2DMACR_RXDMAE_BIT 0
#define SSP2DMACR_TXDMAE_MASK 0x2
#define SSP2DMACR_TXDMAE 0x2
#define SSP2DMACR_TXDMAE_BIT 1

struct LPC17xx_SPI {
    static const uint32_t c_SPI0_Base = 0x40088000; //0xE0068000;
    static const uint32_t c_SPI1_Base = 0x40030000; // 0xE0030000;
    static const uint32_t c_SPI2_Base = 0x400AC000; // 0x400AC000; // TQD

    static const uint32_t c_SPI_Clk_KHz = (LPC17_SYSTEM_CLOCK_HZ / 2 / 1000);

    static const uint32_t c_SPI0 = 0;
    static const uint32_t c_SPI1 = 1;
    static const uint32_t c_SPI2 = 2;

    volatile uint32_t SSPxCR0;
    volatile uint32_t SSPxCR1;
    volatile uint32_t SSPxDR;
    volatile uint32_t SSPxSR;
    volatile uint32_t SSPxCPSR;

    static const uint32_t CONTROLREG_BitEnable = 0x00000004;
    static const uint32_t CONTROLREG_MODE_Master = 0x00000020;
    static const uint32_t CONTROLREG_PHA_1 = 0x00000008;
    static const uint32_t CONTROLREG_POL_1 = 0x00000010;
};

static const LPC17_Gpio_Pin g_lpc17_spi_miso_pins[] = LPC17_SPI_MISO_PINS;
static const LPC17_Gpio_Pin g_lpc17_spi_mosi_pins[] = LPC17_SPI_MOSI_PINS;
static const LPC17_Gpio_Pin g_lpc17_spi_sclk_pins[] = LPC17_SPI_SCLK_PINS;

struct SpiController {
    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t writeLength;
    size_t readLength;
    size_t readOffset;

    int32_t chipSelectLine;
    int32_t clockFrequency;
    int32_t dataBitLength;

    bool isOpened;

    TinyCLR_Spi_Mode spiMode;
};

static SpiController g_SpiController[TOTAL_SPI_CONTROLLERS];

static uint8_t spiProviderDefs[TOTAL_SPI_CONTROLLERS * sizeof(TinyCLR_Spi_Provider)];
static TinyCLR_Spi_Provider* spiProviders[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi;

void LPC17_Spi_Reset();

const TinyCLR_Api_Info* LPC17_Spi_GetApi() {
    for (int i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        spiProviders[i] = (TinyCLR_Spi_Provider*)(spiProviderDefs + (i * sizeof(TinyCLR_Spi_Provider)));
        spiProviders[i]->Parent = &spiApi;
        spiProviders[i]->Index = i;
        spiProviders[i]->Acquire = &LPC17_Spi_Acquire;
        spiProviders[i]->Release = &LPC17_Spi_Release;
        spiProviders[i]->SetActiveSettings = &LPC17_Spi_SetActiveSettings;
        spiProviders[i]->Read = &LPC17_Spi_Read;
        spiProviders[i]->Write = &LPC17_Spi_Write;
        spiProviders[i]->TransferFullDuplex = &LPC17_Spi_TransferFullDuplex;
        spiProviders[i]->TransferSequential = &LPC17_Spi_TransferSequential;
        spiProviders[i]->GetChipSelectLineCount = &LPC17_Spi_GetChipSelectLineCount;
        spiProviders[i]->GetMinClockFrequency = &LPC17_Spi_GetMinClockFrequency;
        spiProviders[i]->GetMaxClockFrequency = &LPC17_Spi_GetMaxClockFrequency;
        spiProviders[i]->GetSupportedDataBitLengths = &LPC17_Spi_GetSupportedDataBitLengths;
    }

    spiApi.Author = "GHI Electronics, LLC";
    spiApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.SpiProvider";
    spiApi.Type = TinyCLR_Api_Type::SpiProvider;
    spiApi.Version = 0;
    spiApi.Count = TOTAL_SPI_CONTROLLERS;
    spiApi.Implementation = spiProviders;

    return &spiApi;
}

bool LPC17_Spi_Transaction_Start(int32_t controller) {
    LPC17_Gpio_Write(nullptr, g_SpiController[controller].chipSelectLine, TinyCLR_Gpio_PinValue::Low);
    return true;
}

bool LPC17_Spi_Transaction_Stop(int32_t controller) {
    LPC17_Gpio_Write(nullptr, g_SpiController[controller].chipSelectLine, TinyCLR_Gpio_PinValue::High);
    return true;
}


bool LPC17_Spi_Transaction_nWrite8_nRead8(int32_t controller) {
    LPC17xx_SPI & SPI = *(LPC17xx_SPI*)(size_t)((controller == 0) ? (LPC17xx_SPI::c_SPI0_Base) : ((controller == 1) ? (LPC17xx_SPI::c_SPI1_Base) : (LPC17xx_SPI::c_SPI2_Base)));

    uint8_t Data8;
    uint8_t* Write8 = g_SpiController[controller].writeBuffer;
    int32_t WriteCount = g_SpiController[controller].writeLength;
    uint8_t* Read8 = g_SpiController[controller].readBuffer;
    int32_t ReadCount = g_SpiController[controller].readLength;
    int32_t ReadStartOffset = g_SpiController[controller].readOffset;
    int32_t ReadTotal = 0;

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
        else if (controller == 1) {
            while (!(SSP1SR & 0x04));//SPIF
            while ((SSP1SR & 0x10));//BSY
        }
        else if (controller == 2) {
            while (!(SSP2SR & 0x04));//SPIF
            while ((SSP2SR & 0x10));//BSY
        }

        // Read recieved data
        if (controller == 0) {
            Data8 = ((uint8_t)SSP0DR);

        }
        else if (controller == 1) {
            Data8 = ((uint8_t)SSP1DR);
        }
        else if (controller == 2) {
            Data8 = ((uint8_t)SSP2DR);
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

bool LPC17_Spi_Transaction_nWrite16_nRead16(int32_t controller) {

    return true;
}

TinyCLR_Result LPC17_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (LPC17_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return LPC17_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result LPC17_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!LPC17_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = readBuffer;
    g_SpiController[controller].readLength = readLength;
    g_SpiController[controller].writeBuffer = (uint8_t*)writeBuffer;
    g_SpiController[controller].writeLength = writeLength;

    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {
        if (!LPC17_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!LPC17_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!LPC17_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!LPC17_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = buffer;
    g_SpiController[controller].readLength = length;
    g_SpiController[controller].writeBuffer = nullptr;
    g_SpiController[controller].writeLength = 0;

    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {
        if (!LPC17_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!LPC17_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!LPC17_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!LPC17_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = nullptr;
    g_SpiController[controller].readLength = 0;
    g_SpiController[controller].writeBuffer = (uint8_t*)buffer;
    g_SpiController[controller].writeLength = length;

    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {
        if (!LPC17_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!LPC17_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!LPC17_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    int32_t controller = (self->Index);

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (g_SpiController[controller].chipSelectLine == chipSelectLine
        && g_SpiController[controller].dataBitLength == dataBitLength
        && g_SpiController[controller].spiMode == mode
        && g_SpiController[controller].clockFrequency == clockFrequency) {
        return TinyCLR_Result::Success;
    }

    g_SpiController[controller].chipSelectLine = chipSelectLine;
    g_SpiController[controller].clockFrequency = clockFrequency;
    g_SpiController[controller].dataBitLength = dataBitLength;
    g_SpiController[controller].spiMode = mode;

    LPC17xx_SPI & SPI = *(LPC17xx_SPI *)(size_t)((controller == 0) ? (LPC17xx_SPI::c_SPI0_Base) : ((controller == 1) ? (LPC17xx_SPI::c_SPI1_Base) : (LPC17xx_SPI::c_SPI2_Base)));

    int SCR, CPSDVSR;
    uint32_t clockKhz = g_SpiController[controller].clockFrequency / 1000;
    uint32_t divider = (100 * LPC17xx_SPI::c_SPI_Clk_KHz / clockKhz); // 100 is only to avoid floating points
    divider /= 2; // because CPSDVSR is even numbeer 2 to 254, so we are calculating using X = 2*CPSDVSR (x:1 to 127);
    divider += 50;
    divider /= 100;

    SCR = divider / (127 + 1);// assuming X = Max value = 127
    CPSDVSR = divider / (SCR + 1); // This is X
    CPSDVSR *= 2;

    if (SCR > 255)
        SCR = 255;

    if (CPSDVSR <= 2)
        CPSDVSR = 2;

    if (CPSDVSR >= 255)
        CPSDVSR = 254;

    // Ensure that out frequency is smaller than input value
    uint32_t freq_out = (LPC17xx_SPI::c_SPI_Clk_KHz * 1000) / (CPSDVSR * (SCR + 1));

    while ((g_SpiController[controller].clockFrequency > 0) && (freq_out > g_SpiController[controller].clockFrequency)) {
        CPSDVSR++;
        if (CPSDVSR >= 254) {

            SCR++;

            if (SCR > 255) {
                SCR = 255;
                break;
            }
            else {
                CPSDVSR = 2;
            }
        }
        freq_out = (LPC17xx_SPI::c_SPI_Clk_KHz * 1000) / (CPSDVSR * (SCR + 1));
    }

    SPI.SSPxCPSR = CPSDVSR; // An even number between 2 and 254

    SPI.SSPxCR1 = 0x02;//master

    // set how many bits
    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {

        SPI.SSPxCR0 = 0x0F;

    }
    else { // 8 bit

        SPI.SSPxCR0 = 0x07;
    }

    SPI.SSPxCR0 &= ~((1 << 4) | (1 << 5));// SPI mode
    SPI.SSPxCR0 &= ~(1 << 7);
    SPI.SSPxCR0 &= ~(1 << 6);

    switch (g_SpiController[controller].spiMode) {

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

    if (g_SpiController[controller].chipSelectLine != PIN_NONE) {
        if (LPC17_Gpio_OpenPin(g_SpiController[controller].chipSelectLine)) {
            LPC17_Gpio_EnableOutputPin(g_SpiController[controller].chipSelectLine, true);
        }
        else {
            return TinyCLR_Result::SharingViolation;
        }

    }
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Spi_Acquire(const TinyCLR_Spi_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t controller = (self->Index);

    uint32_t clkPin, misoPin, mosiPin;

    LPC17_Gpio_PinFunction clkMode, misoMode, mosiMode;

    clkPin = g_lpc17_spi_sclk_pins[controller].number;
    misoPin = g_lpc17_spi_miso_pins[controller].number;
    mosiPin = g_lpc17_spi_mosi_pins[controller].number;

    clkMode = g_lpc17_spi_sclk_pins[controller].pinFunction;
    misoMode = g_lpc17_spi_miso_pins[controller].pinFunction;
    mosiMode = g_lpc17_spi_mosi_pins[controller].pinFunction;

    // Check each pin single time make sure once fail not effect to other pins
    if (!LPC17_Gpio_OpenPin(clkPin))
        return TinyCLR_Result::SharingViolation;

    if (!LPC17_Gpio_OpenPin(misoPin))
        return TinyCLR_Result::SharingViolation;

    if (!LPC17_Gpio_OpenPin(mosiPin))
        return TinyCLR_Result::SharingViolation;

    switch (controller) {
    case 0:
        LPC_SC->PCONP |= PCONP_PCSSP0;
        break;

    case 1:
        LPC_SC->PCONP |= PCONP_PCSSP1;
        break;

    case 2:
        LPC_SC->PCONP |= PCONP_PCSSP2;
        break;
    }

    LPC17_Gpio_ConfigurePin(clkPin, LPC17_Gpio_Direction::Input, clkMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_ConfigurePin(misoPin, LPC17_Gpio_Direction::Input, misoMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_ConfigurePin(mosiPin, LPC17_Gpio_Direction::Input, mosiMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    g_SpiController[controller].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Spi_Release(const TinyCLR_Spi_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t controller = (self->Index);

    switch (controller) {
    case 0:
        LPC_SC->PCONP &= ~PCONP_PCSSP0;
        break;

    case 1:
        LPC_SC->PCONP &= ~PCONP_PCSSP1;
        break;

    case 2:
        LPC_SC->PCONP &= ~PCONP_PCSSP2;
        break;
    }

    g_SpiController[controller].clockFrequency = 0;
    g_SpiController[controller].dataBitLength = 0;

    if (g_SpiController[controller].isOpened) {
        int32_t clkPin = g_lpc17_spi_sclk_pins[controller].number;
        int32_t misoPin = g_lpc17_spi_miso_pins[controller].number;
        int32_t mosiPin = g_lpc17_spi_mosi_pins[controller].number;

        LPC17_Gpio_ClosePin(clkPin);
        LPC17_Gpio_ClosePin(misoPin);
        LPC17_Gpio_ClosePin(mosiPin);

        if (g_SpiController[controller].chipSelectLine != PIN_NONE) {
            // Release the pin, set pin un-reserved
            LPC17_Gpio_ClosePin(g_SpiController[controller].chipSelectLine);

            // Keep chip select is inactive by internal pull up
            LPC17_Gpio_ConfigurePin(g_SpiController[controller].chipSelectLine, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::PullUp, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

            g_SpiController[controller].chipSelectLine = PIN_NONE;
        }
    }

    g_SpiController[controller].isOpened = false;

    return TinyCLR_Result::Success;
}

int32_t LPC17_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self) {
    // CPSDVSR min 2, max 254
    // SCR min 0, max 127
    return (LPC17_AHB_CLOCK_HZ / 2) / (254 * (127 + 1));

}

int32_t LPC17_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self) {
    // CPSDVSR min 2, max 254
    // SCR min 0, max 127
    return (LPC17_AHB_CLOCK_HZ / 2) / (2 * (0 + 1));
}

int32_t LPC17_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self) {
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
    return LPC17_Gpio_GetPinCount(nullptr);
}

static const int32_t dataBitsCount = 2;
static int32_t dataBits[dataBitsCount] = { 8, 16 };

TinyCLR_Result LPC17_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        memcpy(dataBitLengths, dataBits, (dataBitsCount < dataBitLengthsCount ? dataBitsCount : dataBitLengthsCount) * sizeof(int32_t));

    dataBitLengthsCount = dataBitsCount;

    return TinyCLR_Result::Success;
}

void LPC17_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        LPC17_Spi_Release(spiProviders[i]);

        g_SpiController[i].isOpened = false;
    }
}