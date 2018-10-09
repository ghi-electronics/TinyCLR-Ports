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
#include "../../Drivers/USBClient/USBClient.h"

// -------- UDPHS_IEN : (UDPHS Offset: 0x10) UDPHS Interrupt Enable Register --------
#define AT91C_UDPHS_DET_SUSPD (0x1 <<  1) // (UDPHS) Suspend Interrupt Enable/Clear/Status
#define AT91C_UDPHS_MICRO_SOF (0x1 <<  2) // (UDPHS) Micro-SOF Interrupt Enable/Clear/Status
#define AT91C_UDPHS_IEN_SOF   (0x1 <<  3) // (UDPHS) SOF Interrupt Enable/Clear/Status
#define AT91C_UDPHS_ENDRESET  (0x1 <<  4) // (UDPHS) End Of Reset Interrupt Enable/Clear/Status
#define AT91C_UDPHS_WAKE_UP   (0x1 <<  5) // (UDPHS) Wake Up CPU Interrupt Enable/Clear/Status
#define AT91C_UDPHS_ENDOFRSM  (0x1 <<  6) // (UDPHS) End Of Resume Interrupt Enable/Clear/Status
#define AT91C_UDPHS_UPSTR_RES (0x1 <<  7) // (UDPHS) Uppipe Resume Interrupt Enable/Clear/Status
#define AT91C_UDPHS_EPT_INT_0 (0x1 <<  8) // (UDPHS) Endpoint 0 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_1 (0x1 <<  9) // (UDPHS) Endpoint 1 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_2 (0x1 << 10) // (UDPHS) Endpoint 2 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_3 (0x1 << 11) // (UDPHS) Endpoint 3 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_4 (0x1 << 12) // (UDPHS) Endpoint 4 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_5 (0x1 << 13) // (UDPHS) Endpoint 5 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_6 (0x1 << 14) // (UDPHS) Endpoint 6 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_7 (0x1 << 15) // (UDPHS) Endpoint 7 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_8 (0x1 << 16) // (UDPHS) Endpoint 8 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_9 (0x1 << 17) // (UDPHS) Endpoint 9 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_10 (0x1 << 18) // (UDPHS) Endpoint 10 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_11 (0x1 << 19) // (UDPHS) Endpoint 11 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_12 (0x1 << 20) // (UDPHS) Endpoint 12 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_13 (0x1 << 21) // (UDPHS) Endpoint 13 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_14 (0x1 << 22) // (UDPHS) Endpoint 14 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_15 (0x1 << 23) // (UDPHS) Endpoint 15 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_1 (0x1 << 25) // (UDPHS) DMA Channel 1 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_2 (0x1 << 26) // (UDPHS) DMA Channel 2 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_3 (0x1 << 27) // (UDPHS) DMA Channel 3 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_4 (0x1 << 28) // (UDPHS) DMA Channel 4 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_5 (0x1 << 29) // (UDPHS) DMA Channel 5 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_6 (0x1 << 30) // (UDPHS) DMA Channel 6 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_7 (0x1 << 31) // (UDPHS) DMA Channel 7 Interrupt Enable/Status

// -------- UDPHS_EPTCFG : (UDPHS_EPT Offset: 0x0) UDPHS Endpoint Config Register --------
#define AT91C_UDPHS_EPT_SIZE  (0x7 <<  0) // (UDPHS_EPT) Endpoint Size
#define 	AT91C_UDPHS_EPT_SIZE_8                    (0x0) // (UDPHS_EPT)    8 bytes
#define 	AT91C_UDPHS_EPT_SIZE_16                   (0x1) // (UDPHS_EPT)   16 bytes
#define 	AT91C_UDPHS_EPT_SIZE_32                   (0x2) // (UDPHS_EPT)   32 bytes
#define 	AT91C_UDPHS_EPT_SIZE_64                   (0x3) // (UDPHS_EPT)   64 bytes
#define 	AT91C_UDPHS_EPT_SIZE_128                  (0x4) // (UDPHS_EPT)  128 bytes
#define 	AT91C_UDPHS_EPT_SIZE_256                  (0x5) // (UDPHS_EPT)  256 bytes
#define 	AT91C_UDPHS_EPT_SIZE_512                  (0x6) // (UDPHS_EPT)  512 bytes
#define 	AT91C_UDPHS_EPT_SIZE_1024                 (0x7) // (UDPHS_EPT) 1024 bytes
#define AT91C_UDPHS_EPT_DIR   (0x1 <<  3) // (UDPHS_EPT) Endpoint Direction 0:OUT, 1:IN
#define 	AT91C_UDPHS_EPT_DIR_OUT                  (0x0 <<  3) // (UDPHS_EPT) Direction OUT
#define 	AT91C_UDPHS_EPT_DIR_IN                   (0x1 <<  3) // (UDPHS_EPT) Direction IN
#define AT91C_UDPHS_EPT_TYPE  (0x3 <<  4) // (UDPHS_EPT) Endpoint Type
#define 	AT91C_UDPHS_EPT_TYPE_CTL_EPT              (0x0 <<  4) // (UDPHS_EPT) Control endpoint
#define 	AT91C_UDPHS_EPT_TYPE_ISO_EPT              (0x1 <<  4) // (UDPHS_EPT) Isochronous endpoint
#define 	AT91C_UDPHS_EPT_TYPE_BUL_EPT              (0x2 <<  4) // (UDPHS_EPT) Bulk endpoint
#define 	AT91C_UDPHS_EPT_TYPE_INT_EPT              (0x3 <<  4) // (UDPHS_EPT) Interrupt endpoint
#define AT91C_UDPHS_BK_NUMBER (0x3 <<  6) // (UDPHS_EPT) Number of Banks
#define 	AT91C_UDPHS_BK_NUMBER_0                    (0x0 <<  6) // (UDPHS_EPT) Zero Bank, the EndPoint is not mapped in memory
#define 	AT91C_UDPHS_BK_NUMBER_1                    (0x1 <<  6) // (UDPHS_EPT) One Bank (Bank0)
#define 	AT91C_UDPHS_BK_NUMBER_2                    (0x2 <<  6) // (UDPHS_EPT) Double bank (Ping-Pong : Bank0 / Bank1)
#define 	AT91C_UDPHS_BK_NUMBER_3                    (0x3 <<  6) // (UDPHS_EPT) Triple Bank (Bank0 / Bank1 / Bank2)
#define AT91C_UDPHS_NB_TRANS  (0x3 <<  8) // (UDPHS_EPT) Number Of Transaction per Micro-Frame (High-Bandwidth iso only)
#define AT91C_UDPHS_EPT_MAPD  ((uint32_t)0x1 << 31) // (UDPHS_EPT) Endpoint Mapped (read only

// -------- UDPHS_EPTCTLENB : (UDPHS_EPT Offset: 0x4) UDPHS Endpoint Control Enable Register --------
#define AT91C_UDPHS_EPT_ENABL (0x1 <<  0) // (UDPHS_EPT) Endpoint Enable
#define AT91C_UDPHS_AUTO_VALID (0x1 <<  1) // (UDPHS_EPT) Packet Auto-Valid Enable/Disable
#define AT91C_UDPHS_INTDIS_DMA (0x1 <<  3) // (UDPHS_EPT) Endpoint Interrupts DMA Request Enable/Disable
#define AT91C_UDPHS_NYET_DIS  (0x1 <<  4) // (UDPHS_EPT) NYET Enable/Disable
#define AT91C_UDPHS_DATAX_RX  (0x1 <<  6) // (UDPHS_EPT) DATAx Interrupt Enable/Disable
#define AT91C_UDPHS_MDATA_RX  (0x1 <<  7) // (UDPHS_EPT) MDATA Interrupt Enabled/Disable
#define AT91C_UDPHS_ERR_OVFLW (0x1 <<  8) // (UDPHS_EPT) OverFlow Error Interrupt Enable/Disable/Status
#define AT91C_UDPHS_RX_BK_RDY (0x1 <<  9) // (UDPHS_EPT) Received OUT data
#define AT91C_UDPHS_TX_COMPLT (0x1 << 10) // (UDPHS_EPT) Transmitted IN data Complete Interrupt Enable/Disable or Transmitted IN data Complete (clear)
#define AT91C_UDPHS_ERR_TRANS (0x1 << 11) // (UDPHS_EPT) Transaction Error Interrupt Enable/Disable
#define AT91C_UDPHS_TX_PK_RDY (0x1 << 11) // (UDPHS_EPT) TX Packet Ready Interrupt Enable/Disable
#define AT91C_UDPHS_RX_SETUP  (0x1 << 12) // (UDPHS_EPT) Received SETUP Interrupt Enable/Disable
#define AT91C_UDPHS_ERR_FL_ISO (0x1 << 12) // (UDPHS_EPT) Error Flow Clear/Interrupt Enable/Disable
#define AT91C_UDPHS_STALL_SNT (0x1 << 13) // (UDPHS_EPT) Stall Sent Clear
#define AT91C_UDPHS_ERR_CRISO (0x1 << 13) // (UDPHS_EPT) CRC error / Error NB Trans / Interrupt Enable/Disable
#define AT91C_UDPHS_NAK_IN    (0x1 << 14) // (UDPHS_EPT) NAKIN ERROR FLUSH / Clear / Interrupt Enable/Disable
#define AT91C_UDPHS_NAK_OUT   (0x1 << 15) // (UDPHS_EPT) NAKOUT / Clear / Interrupt Enable/Disable
#define AT91C_UDPHS_BUSY_BANK (0x1 << 18) // (UDPHS_EPT) Busy Bank Interrupt Enable/Disable
#define AT91C_UDPHS_SHRT_PCKT (0x1 << 31) // (UDPHS_EPT) Short Packet / Interrupt Enable/Disable
// -------- UDPHS_CTRL : (UDPHS Offset: 0x0) UDPHS Control Register --------
#define AT91C_UDPHS_DEV_ADDR  (0x7F <<  0) // (UDPHS) UDPHS address
#define AT91C_UDPHS_FADDR_EN  (0x1 <<  7) // (UDPHS) Function address Enable
#define AT91C_UDPHS_EN_UDPHS  (0x1 <<  8) // (UDPHS) UDPHS Enable
#define AT91C_UDPHS_DETACH    (0x1 <<  9) // (UDPHS) Detach Command
#define AT91C_UDPHS_REWAKEUP  (0x1 << 10) // (UDPHS) Send Remote Wake Up
#define AT91C_UDPHS_PULLD_DIS (0x1 << 11) // (UDPHS) PullDown Disable

// -------- UDPHS_IPFEATURES : (UDPHS Offset: 0xf8) UDPHS Features Register --------
#define AT91C_UDPHS_EPT_NBR_MAX (0xF <<  0) // (UDPHS) Max Number of Endpoints
#define AT91C_UDPHS_DMA_CHANNEL_NBR (0x7 <<  4) // (UDPHS) Number of DMA Channels
#define AT91C_UDPHS_DMA_B_SIZ (0x1 <<  7) // (UDPHS) DMA Buffer Size
#define AT91C_UDPHS_DMA_FIFO_WORD_DEPTH (0xF <<  8) // (UDPHS) DMA FIFO Depth in words
#define AT91C_UDPHS_FIFO_MAX_SIZE (0x7 << 12) // (UDPHS) DPRAM size
#define AT91C_UDPHS_BW_DPRAM  (0x1 << 15) // (UDPHS) DPRAM byte write capability
#define AT91C_UDPHS_DATAB16_8 (0x1 << 16) // (UDPHS) UTMI DataBus16_8
#define AT91C_UDPHS_ISO_EPT_1 (0x1 << 17) // (UDPHS) Endpoint 1 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_2 (0x1 << 18) // (UDPHS) Endpoint 2 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_3 (0x1 << 19) // (UDPHS) Endpoint 3 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_4 (0x1 << 20) // (UDPHS) Endpoint 4 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_5 (0x1 << 21) // (UDPHS) Endpoint 5 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_6 (0x1 << 22) // (UDPHS) Endpoint 6 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_7 (0x1 << 23) // (UDPHS) Endpoint 7 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_8 (0x1 << 24) // (UDPHS) Endpoint 8 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_9 (0x1 << 25) // (UDPHS) Endpoint 9 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_10 (0x1 << 26) // (UDPHS) Endpoint 10 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_11 (0x1 << 27) // (UDPHS) Endpoint 11 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_12 (0x1 << 28) // (UDPHS) Endpoint 12 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_13 (0x1 << 29) // (UDPHS) Endpoint 13 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_14 (0x1 << 30) // (UDPHS) Endpoint 14 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_15 (0x1 << 31) // (UDPHS) Endpoint 15 High Bandwidth Isochronous Capability

#define AT91C_UDPHS_BYTE_COUNT (0x7FF << 20) // (UDPHS_EPT) UDPHS Byte Count

/* -------- UDPHS_EPTSETSTA : (UDPHS Offset: N/A) UDPHS Endpoint Set Status Register -------- */
#define UDPHS_EPTSETSTA_FRCESTALL (0x1u << 5) /**< \brief (UDPHS_EPTSETSTA) Stall Handshake Request Set */
#define UDPHS_EPTSETSTA_KILL_BANK (0x1u << 9) /**< \brief (UDPHS_EPTSETSTA) KILL Bank Set (for IN Endpoint) */
#define UDPHS_EPTSETSTA_TX_PK_RDY (0x1u << 11) /**< \brief (UDPHS_EPTSETSTA) TX Packet Ready Set */

/* -------- UDPHS_CTRL : (UDPHS Offset: 0x00) UDPHS Control Register -------- */
#define UDPHS_CTRL_DEV_ADDR_Pos 0
#define UDPHS_CTRL_DEV_ADDR_Msk (0x7fu << UDPHS_CTRL_DEV_ADDR_Pos) /**< \brief (UDPHS_CTRL) UDPHS address */
#define UDPHS_CTRL_DEV_ADDR(value) ((UDPHS_CTRL_DEV_ADDR_Msk & ((value) << UDPHS_CTRL_DEV_ADDR_Pos)))
#define UDPHS_CTRL_FADDR_EN (0x1u << 7) /**< \brief (UDPHS_CTRL) Function address Enable */
#define UDPHS_CTRL_EN_UDPHS (0x1u << 8) /**< \brief (UDPHS_CTRL) UDPHS Enable */
#define UDPHS_CTRL_DETACH (0x1u << 9) /**< \brief (UDPHS_CTRL) Detach Command */
#define UDPHS_CTRL_REWAKEUP (0x1u << 10) /**< \brief (UDPHS_CTRL) Send Remote Wake Up */
#define UDPHS_CTRL_PULLD_DIS (0x1u << 11) /**< \brief (UDPHS_CTRL) Pull-Down Disable */

#define AT91C_CKGR_UPLLEN_ENABLED  (0x1 << 16) // (PMC) The UTMI PLL is enabled
#define AT91C_CKGR_BIASEN_ENABLED  (0x1 << 24) // (PMC) The UTMI BIAS is enabled
#define SHIFT_INTERUPT             8

#define USB_CTRL_WMAXPACKETSIZE0_EP_WRITE                   64
#define USB_BULK_WMAXPACKETSIZE_EP_WRITE                    64
#define USB_BULK_WMAXPACKETSIZE_EP_READ                     64

#define USB_MAX_DATA_PACKET_SIZE    64

struct AT91_UDPHS_EPT {
    volatile uint32_t	 UDPHS_EPTCFG; 	// UDPHS Endpoint Config Register
    volatile uint32_t	 UDPHS_EPTCTLENB; 	// UDPHS Endpoint Control Enable Register
    volatile uint32_t	 UDPHS_EPTCTLDIS; 	// UDPHS Endpoint Control Disable Register
    volatile uint32_t	 UDPHS_EPTCTL; 	// UDPHS Endpoint Control Register
    volatile uint32_t	 Reserved0[1]; 	//
    volatile uint32_t	 UDPHS_EPTSETSTA; 	// UDPHS Endpoint Set Status Register
    volatile uint32_t	 UDPHS_EPTCLRSTA; 	// UDPHS Endpoint Clear Status Register
    volatile uint32_t	 UDPHS_EPTSTA; 	// UDPHS Endpoint Status Register
};

struct AT91_UDPHS_DMA {
    volatile uint32_t	 UDPHS_DMANXTDSC; 	// UDPHS DMA Channel Next Descriptor address
    volatile uint32_t	 UDPHS_DMAADDRESS; 	// UDPHS DMA Channel address Register
    volatile uint32_t	 UDPHS_DMACONTROL; 	// UDPHS DMA Channel Control Register
    volatile uint32_t	 UDPHS_DMASTATUS; 	// UDPHS DMA Channel Status Register
};

struct AT91_UDPHS {
    volatile uint32_t	 UDPHS_CTRL; 	// UDPHS Control Register					0x0000
    volatile uint32_t	 UDPHS_FNUM; 	// UDPHS Frame Number Register				0x0004
    volatile uint32_t	 Reserved0[2]; 	//											0x0008, 0x000C
    volatile uint32_t	 UDPHS_IEN; 	// UDPHS Interrupt Enable Register			0x0010
    volatile uint32_t	 UDPHS_INTSTA; 	// UDPHS Interrupt Status Register			0x0014
    volatile uint32_t	 UDPHS_CLRINT; 	// UDPHS Clear Interrupt Register			0x0018
    volatile uint32_t	 UDPHS_EPTRST; 	// UDPHS Endpoints Reset Register			0x001C
    volatile uint32_t	 Reserved1[44]; 	//										0x0020, 0x0024, 0x0028, 0x002C, 0x0030, 0x0034, 0x0038, 0x003C, 0x0040, 0x0044, 0x0048, 0x004C, 0x0050, 0x0054, 0x0058, 0x005C,
                                        //										0x0060, 0x0064, 0x0068, 0x006C, 0x0070, 0x0074, 0x0078, 0x007C, 0x0080, 0x0084, 0x0088, 0x008C, 0x0090, 0x0094, 0x0098, 0x009C,
                                        //										0x00A0, 0x00A4, 0x00A8, 0x00AC, 0x00B0, 0x00B4, 0x00B8, 0x00BC, 0x00C0, 0x00C4, 0x00C8, 0x00CC
    volatile uint32_t	 UDPHS_TSTSOFCNT; 	// UDPHS Test SOF Counter Register		0x00D0
    volatile uint32_t	 UDPHS_TSTCNTA; 	// UDPHS Test A Counter Register		0x00D4
    volatile uint32_t	 UDPHS_TSTCNTB; 	// UDPHS Test B Counter Register		0x00D8
    volatile uint32_t	 UDPHS_TSTMODREG; 	// UDPHS Test Mode Register				0x00DC
    volatile uint32_t	 UDPHS_TST; 	// UDPHS Test Register						0x00E0
    volatile uint32_t	 Reserved2[2]; 	//											0x00E4, 0x00E8
    volatile uint32_t	 UDPHS_RIPPADDRSIZE; 	// UDPHS PADDRSIZE Register			0x00EC
    volatile uint32_t	 UDPHS_RIPNAME1; 	// UDPHS Name1 Register					0x00F0
    volatile uint32_t	 UDPHS_RIPNAME2; 	// UDPHS Name2 Register					0x00F4
    volatile uint32_t	 UDPHS_IPFEATURES; 	// UDPHS Features Register				0x00F8
    volatile uint32_t	 UDPHS_IPVERSION; 	// UDPHS Version Register				0x00FC
    struct AT91_UDPHS_EPT	 UDPHS_EPT[16]; 	// UDPHS Endpoint struct		0x0100
    struct AT91_UDPHS_DMA	 UDPHS_DMA[8]; 	// UDPHS DMA channel struct (not use [0])
};

struct AT91_UDPHS_EPTFIFO {
    volatile uint32_t	 UDPHS_READEPT0[16384]; 	// FIFO Endpoint data Register 0
    volatile uint32_t	 UDPHS_READEPT1[16384]; 	// FIFO Endpoint data Register 1
    volatile uint32_t	 UDPHS_READEPT2[16384]; 	// FIFO Endpoint data Register 2
    volatile uint32_t	 UDPHS_READEPT3[16384]; 	// FIFO Endpoint data Register 3
    volatile uint32_t	 UDPHS_READEPT4[16384]; 	// FIFO Endpoint data Register 4
    volatile uint32_t	 UDPHS_READEPT5[16384]; 	// FIFO Endpoint data Register 5
    volatile uint32_t	 UDPHS_READEPT6[16384]; 	// FIFO Endpoint data Register 6
    volatile uint32_t	 UDPHS_READEPT7[16384]; 	// FIFO Endpoint data Register 7
    volatile uint32_t	 UDPHS_READEPT8[16384]; 	// FIFO Endpoint data Register 8
    volatile uint32_t	 UDPHS_READEPT9[16384]; 	// FIFO Endpoint data Register 9
    volatile uint32_t	 UDPHS_READEPTA[16384]; 	// FIFO Endpoint data Register 10
    volatile uint32_t	 UDPHS_READEPTB[16384]; 	// FIFO Endpoint data Register 11
    volatile uint32_t	 UDPHS_READEPTC[16384]; 	// FIFO Endpoint data Register 12
    volatile uint32_t	 UDPHS_READEPTD[16384]; 	// FIFO Endpoint data Register 13
    volatile uint32_t	 UDPHS_READEPTE[16384]; 	// FIFO Endpoint data Register 14
    volatile uint32_t	 UDPHS_READEPTF[16384]; 	// FIFO Endpoint data Register 15
};

struct AT91_UsbDeviceDriver {
    UsClientState *usClientState;

    uint8_t			previousDeviceState;

    bool			firstDescriptorPacket;
    bool			txRunning[AT91_USB_ENDPOINT_COUNT];
    bool			txNeedZLPS[AT91_USB_ENDPOINT_COUNT];
};

static AT91_UsbDeviceDriver usbDeviceDrivers[AT91_TOTAL_USB_CONTROLLERS];

struct AT91_UDP_ENDPOINT_ATTRIBUTE {
    uint16_t		Dir_Type;
    uint16_t		Payload;
    bool		DualBank;
    uint32_t		dFlag;
};

void AT91_UsbDevice_StallEndPoint(uint32_t ep);

uint32_t AT91_UsbDevice_WriteEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t cnt);
uint32_t AT91_UsbDevice_ReadEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t len);

void AT91_UsbDevice_ConfigEndPoint();
void AT91_UsbDevice_EndpointIsr(UsClientState* usClientState, uint32_t endpoint);
void AT91_UsbDevice_ClearTxQueue(UsClientState* usClientState, int32_t endpoint);

__inline void AT91_UsbDevice_PmcEnableUsbClock(void) {
    (*(volatile uint32_t *)0xFFFFFC10) = 1 << AT91C_ID_UDP;
    (*(volatile uint32_t *)0xFFFFFC1C) |= AT91C_CKGR_UPLLEN_ENABLED;
    (*(volatile uint32_t *)(AT91C_BASE_UDP + 0xE0)) |= 3; // Full Speed
}

__inline void AT91_UsbDevice_PmcDisableUsbClock(void) {
    (*(volatile uint32_t *)0xFFFFFC14) = 1 << AT91C_ID_UDP;
    (*(volatile uint32_t *)0xFFFFFC1C) &= ~AT91C_CKGR_UPLLEN_ENABLED;
}

__inline void AT91_UsbDevice_PmcDisableUTMIBIAS(void) {
    (*(volatile uint32_t *)0xFFFFFC1C) &= ~AT91C_CKGR_BIASEN_ENABLED;
}

AT91_UDP_ENDPOINT_ATTRIBUTE AT91_UsbDevice_EndpointAttr[] =
{
    {AT91C_UDPHS_EPT_TYPE_CTL_EPT | AT91C_UDPHS_EPT_DIR_OUT , USB_CTRL_WMAXPACKETSIZE0_EP_WRITE, false, AT91C_UDPHS_BK_NUMBER_1},
    {0                                                      , USB_BULK_WMAXPACKETSIZE_EP_WRITE , true , AT91C_UDPHS_BK_NUMBER_2},
    {0                                                      , USB_BULK_WMAXPACKETSIZE_EP_READ  , true , AT91C_UDPHS_BK_NUMBER_2},
};

void AT91_UsbDevice_ResetEvent(UsClientState *usClientState) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    // MCK + UDPCK are already enabled
    // Pull-Up is already connected
    // Transceiver must be enabled
    // Endpoint 0 must be enabled

    // program Endpoint Status/control
    int32_t endpointCount = sizeof(AT91_UsbDevice_EndpointAttr) / sizeof(AT91_UDP_ENDPOINT_ATTRIBUTE);

    for (int32_t i = 0; i < endpointCount; i++) {
        uint32_t dwEptcfg = 0;
        // Reset endpoint fifos
        pUdp->UDPHS_EPTRST = 1 << i;
        // Enable endpoint interrupt
        pUdp->UDPHS_IEN |= (1 << SHIFT_INTERUPT << i);
        // Set endpoint configration
        dwEptcfg = AT91_UsbDevice_EndpointAttr[i].Dir_Type | AT91_UsbDevice_EndpointAttr[i].dFlag;
        switch (AT91_UsbDevice_EndpointAttr[i].Payload) {
        case 8:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_8;
            break;
        case 16:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_16;
            break;
        case 32:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_32;
            break;
        case 64:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_64;
            break;
        case 128:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_128;
            break;
        case 256:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_256;
            break;
        case 512:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_512;
            break;
        case 1024:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_1024;
            break;
        default:
            break;
        }

        pUdp->UDPHS_EPT[i].UDPHS_EPTCFG = dwEptcfg;

        pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL | AT91C_UDPHS_RX_BK_RDY;// | AT91C_UDPHS_TX_PK_RDY;
    }

    pUdp->UDPHS_EPT[0].UDPHS_EPTCTLENB = AT91C_UDPHS_RX_SETUP | AT91C_UDPHS_TX_PK_RDY;

    /* clear all flags */
    TinyCLR_UsbClient_ClearEvent(usClientState, 0xFFFFFFFF); // clear all events on all endpoints

    for (int32_t ep = 0; ep < AT91_USB_ENDPOINT_COUNT; ep++) {
        usbDeviceDrivers[usClientState->controllerIndex].txRunning[ep] = false;
        usbDeviceDrivers[usClientState->controllerIndex].txNeedZLPS[ep] = false;
    }

    usClientState->deviceState = USB_DEVICE_STATE_DEFAULT;
    usClientState->address = 0;
    TinyCLR_UsbClient_StateCallback(usClientState);
}

void AT91_UsbDevice_ResumeEvent(UsClientState *usClientState) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    // The device enters Configured state
    // MCK + UDPCK must be on
    // Pull-Up must be connected
    // Transceiver must be enabled

    // TODO: will be replaced by PMC API
    AT91_UsbDevice_PmcEnableUsbClock();

    usClientState->deviceState = usbDeviceDrivers[usClientState->controllerIndex].previousDeviceState;

    TinyCLR_UsbClient_StateCallback(usClientState);

    // Enable end of reset and suspend interrupt
    pUdp->UDPHS_IEN |= AT91C_UDPHS_ENDOFRSM;// | AT91C_UDPHS_DET_SUSPD;

    // Disable Wakeup interrupt
    pUdp->UDPHS_IEN &= ~AT91C_UDPHS_WAKE_UP;
}

void AT91_UsbDevice_InterruptHandler(void *param) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    uint32_t USB_INTR = (pUdp->UDPHS_INTSTA & pUdp->UDPHS_IEN);
    uint32_t endpoint = 0;


    UsClientState *usClientState = (UsClientState*)param;

    // Handle all UDP interrupts
    while (USB_INTR != 0) {
        // Start Of Frame (SOF)
        if (USB_INTR & AT91C_UDPHS_IEN_SOF) {
            // Acknowledge interrupt
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_IEN_SOF;
            USB_INTR &= ~AT91C_UDPHS_IEN_SOF;
            // This interrupt should not happen, as it is not enabled.
        }

        // Suspend
        if (USB_INTR & AT91C_UDPHS_DET_SUSPD) {
            // Acknowledge interrupt
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_DET_SUSPD | AT91C_UDPHS_WAKE_UP;
            USB_INTR &= ~AT91C_UDPHS_DET_SUSPD;
        }

        // Resume or Wakeup
        if ((USB_INTR & AT91C_UDPHS_WAKE_UP) || (USB_INTR & AT91C_UDPHS_ENDOFRSM)) {
            AT91_UsbDevice_ResumeEvent(usClientState);

            // Acknowledge interrupt
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_WAKE_UP | AT91C_UDPHS_ENDOFRSM;
            USB_INTR &= ~(AT91C_UDPHS_WAKE_UP | AT91C_UDPHS_ENDOFRSM);
        }

        // End of bus reset
        if (USB_INTR & AT91C_UDPHS_ENDRESET) {
            // Acknowledge end of bus reset interrupt

            AT91_UsbDevice_ResetEvent(usClientState);
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_WAKE_UP | AT91C_UDPHS_DET_SUSPD | AT91C_UDPHS_ENDRESET;

            USB_INTR &= ~AT91C_UDPHS_ENDRESET;
        }

        if (USB_INTR & AT91C_UDPHS_UPSTR_RES) {
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_UPSTR_RES;
            USB_INTR &= ~AT91C_UDPHS_UPSTR_RES;
        }
        else //Endpoint Interrupt
        {
            uint32_t i = 0;
            USB_INTR >>= 8;
            while (USB_INTR != 0) {
                if (USB_INTR & 1) {
                    endpoint = i;
                    AT91_UsbDevice_EndpointIsr(usClientState, endpoint);

                }
                USB_INTR >>= 1;
                i++;
            }
        }
        USB_INTR = pUdp->UDPHS_INTSTA & pUdp->UDPHS_IEN;
    }
}

void AT91_UsbDevice_VbusInterruptHandler(UsClientState *usClientState, int32_t pin, bool pinState, void* param) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    // VBus High
    if (pinState) {
        // Enable USB clock
        AT91_UsbDevice_PmcEnableUsbClock();

        pUdp->UDPHS_CTRL |= AT91C_UDPHS_DETACH;

        // Reset and enable IP UDPHS
        pUdp->UDPHS_CTRL &= ~AT91C_UDPHS_EN_UDPHS;
        pUdp->UDPHS_CTRL |= AT91C_UDPHS_EN_UDPHS;

        // With OR without DMA !!!
        // Initialization of DMA
        for (int32_t i = 1; i <= ((pUdp->UDPHS_IPFEATURES & AT91C_UDPHS_DMA_CHANNEL_NBR) >> 4); i++) {
            // RESET endpoint canal DMA:
            // DMA stop channel command
            pUdp->UDPHS_DMA[i].UDPHS_DMACONTROL = 0;  // STOP command

            // Disable endpoint
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLDIS = 0xFFFFFFFF;

            // Reset endpoint config
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = 0;

            // Reset DMA channel (Buff count and Control field)
            pUdp->UDPHS_DMA[i].UDPHS_DMACONTROL = 0x02;  // NON STOP command

            // Reset DMA channel 0 (STOP)
            pUdp->UDPHS_DMA[i].UDPHS_DMACONTROL = 0;  // STOP command

            // Clear DMA channel status (read the register for clear it)
            pUdp->UDPHS_DMA[i].UDPHS_DMASTATUS = pUdp->UDPHS_DMA[i].UDPHS_DMASTATUS;

        }

        pUdp->UDPHS_IEN = AT91C_UDPHS_ENDOFRSM | AT91C_UDPHS_WAKE_UP;// | AT91C_UDPHS_DET_SUSPD;
        pUdp->UDPHS_CLRINT = 0xFE;

        // Pull up the DP line
        pUdp->UDPHS_CTRL &= ~AT91C_UDPHS_DETACH; // attach

        usClientState->deviceState = USB_DEVICE_STATE_ATTACHED;

        TinyCLR_UsbClient_StateCallback(usClientState);
    }
    else // VBus Low
    {
        // clear USB Txbuffer
        for (int32_t ep = 0; ep < AT91_USB_ENDPOINT_COUNT; ep++) {
            if (usClientState->isTxQueue[ep] && usClientState->queues[ep] != NULL)
                TinyCLR_UsbClient_ClearEndpoints(usClientState, ep);
        }

        pUdp->UDPHS_CTRL |= AT91C_UDPHS_DETACH;

        // TODO: will be replaced by PMC API
        // Disable  USB clock
        AT91_UsbDevice_PmcDisableUsbClock();

        usClientState->deviceState = USB_DEVICE_STATE_DETACHED;
        TinyCLR_UsbClient_StateCallback(usClientState);
    }
}

bool AT91_UsbDevice_ProtectPins(UsClientState *usClientState, bool On) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (usClientState) {
        if (On) {
            AT91_UsbDevice_VbusInterruptHandler(usClientState, 0, true, nullptr);

            struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;
            pUdp->UDPHS_CTRL &= ~AT91C_UDPHS_DETACH; // attach*)
        }
        else {
            for (int32_t ep = 0; ep < AT91_USB_ENDPOINT_COUNT; ep++) {
                if (usClientState->queues[ep] && usClientState->isTxQueue[ep])
                    AT91_UsbDevice_ClearTxQueue(usClientState, ep);

            }

            //detach D plus line
            struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;
            pUdp->UDPHS_CTRL |= AT91C_UDPHS_DETACH; // dettach

            //--//
            usClientState->deviceState = USB_DEVICE_STATE_DETACHED;
            TinyCLR_UsbClient_StateCallback(usClientState);
        }

        return true;
    }

    return false;
}

void AT91_UsbDevice_ClearTxQueue(UsClientState* usClientState, int32_t endpoint) {
    // it is much faster to just re-initialize the queue and it does the same thing
    if (usClientState->queues[endpoint] != NULL) {
        TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);
    }
}

#define USBH_TRANSFER_PACKET_TIMEOUT 4000
void AT91_UsbDevice_TxPacket(UsClientState* usClientState, int32_t endpoint) {
    int32_t timeout;

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    uint8_t * pDest = (uint8_t*)((((struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA)->UDPHS_READEPT0) + 16384 * endpoint);

    DISABLE_INTERRUPTS_SCOPED(irq);

    // transmit a packet on UsbPortNum, if there are no more packets to transmit, then die

    timeout = 0;
    while (!pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSTA & AT91C_UDPHS_TX_COMPLT) {
        timeout++;
        if (timeout > USBH_TRANSFER_PACKET_TIMEOUT) {
            return;
        }
    }
    timeout = 0;
    while (pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSTA & AT91C_UDPHS_TX_PK_RDY) {
        timeout++;
        if (timeout > USBH_TRANSFER_PACKET_TIMEOUT) {
            return;
        }
    }
    USB_PACKET64* Packet64;

    for (;;) {
        Packet64 = TinyCLR_UsbClient_TxDequeue(usClientState, endpoint);

        if (Packet64 == nullptr || Packet64->Size > 0) {
            break;
        }
    }

    if (Packet64) {
        int32_t i;

        AT91_UsbDevice_WriteEndPoint(endpoint, Packet64->Buffer, Packet64->Size);
        usbDeviceDrivers[usClientState->controllerIndex].txNeedZLPS[endpoint] = (Packet64->Size == USB_BULK_WMAXPACKETSIZE_EP_WRITE);
    }
    else {
        // send the zero leght packet since we landed on the FIFO boundary before
        // (and we queued a zero length packet to transmit)
        if (usbDeviceDrivers[usClientState->controllerIndex].txNeedZLPS[endpoint]) {
            pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSETSTA = AT91C_UDPHS_TX_PK_RDY;
            usbDeviceDrivers[usClientState->controllerIndex].txNeedZLPS[endpoint] = false;
        }

        // no more data
        usbDeviceDrivers[usClientState->controllerIndex].txRunning[endpoint] = false;
        pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLDIS = AT91C_UDPHS_TX_PK_RDY;
    }
}

void AT91_UsbDevice_ControlNext(UsClientState *usClientState) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    uint8_t *pFifo = (uint8_t *)&(((struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA)->UDPHS_READEPT0[0]);

    if (usClientState->dataCallback) {
        /* this call can't fail */
        usClientState->dataCallback(usClientState);

        if (usClientState->dataSize == 0) {
            AT91_UsbDevice_WriteEndPoint(0, (uint8_t*)NULL, 0);
        }
        else {
            AT91_UsbDevice_WriteEndPoint(0, usClientState->ptrData, usClientState->dataSize);

            // special handling the USB state set address test, cannot use the first descriptor as the ADDRESS state is handle in the hardware
            if (usbDeviceDrivers[usClientState->controllerIndex].firstDescriptorPacket) {
                usClientState->dataCallback = NULL;
            }
        }
    }
    else {
        pUdp->UDPHS_EPT[0].UDPHS_EPTCLRSTA = AT91C_UDPHS_TX_COMPLT;
        pUdp->UDPHS_EPT[0].UDPHS_EPTCTLDIS = AT91C_UDPHS_TX_PK_RDY;
    }
}

void AT91_UsbDevice_StallEndPoint(uint32_t ep) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    pUdp->UDPHS_EPT[ep].UDPHS_EPTSETSTA = UDPHS_EPTSETSTA_FRCESTALL;
}

uint32_t AT91_UsbDevice_WriteEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t cnt) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    uint8_t * pDest = (uint8_t*)((((struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA)->UDPHS_READEPT0) + 16384 * EPNum);

    memcpy(pDest, pData, cnt);

    pUdp->UDPHS_EPT[EPNum].UDPHS_EPTSETSTA = AT91C_UDPHS_TX_PK_RDY;

    return cnt;
}
uint32_t AT91_UsbDevice_ReadEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t len) {
    struct AT91_UDPHS_EPTFIFO *pFifo = (struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA;
    uint8_t *pDest = (uint8_t *)(pFifo->UDPHS_READEPT0 + 16384 * EPNum);

    memcpy(pData, pDest, len);
    return len;
}

void AT91_UsbDevice_ConfigEndPoint() {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    int32_t i;

    for (i = 1; i <= 2; i++) {
        if ((AT91_UsbDevice_EndpointAttr[i].Dir_Type & AT91C_UDPHS_EPT_DIR_OUT) == AT91C_UDPHS_EPT_DIR_OUT) {
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL | AT91C_UDPHS_RX_BK_RDY;
        }
        else if ((AT91_UsbDevice_EndpointAttr[i].Dir_Type & AT91C_UDPHS_EPT_DIR_IN) == AT91C_UDPHS_EPT_DIR_IN) {
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL;
        }
    }
    pUdp->UDPHS_EPT[4].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL;
}

void AT91_UsbDevice_EndpointIsr(UsClientState *usClientState, uint32_t endpoint) {
    uint32_t Status;

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    struct AT91_UDPHS_EPTFIFO *pFifo = (struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA;

    Status = pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSTA;

    // Control Endpoint
    if (endpoint == 0) {
        // ugly
        if (Status & AT91C_UDPHS_RX_BK_RDY) {
            while (pUdp->UDPHS_EPT[0].UDPHS_EPTSTA & AT91C_UDPHS_RX_BK_RDY)
                pUdp->UDPHS_EPT[0].UDPHS_EPTCLRSTA = AT91C_UDPHS_RX_BK_RDY;
        }

        // set up packet receive
        if (Status & AT91C_UDPHS_RX_SETUP) {
            uint8_t len = (pUdp->UDPHS_EPT[0].UDPHS_EPTSTA >> 20) & 0x7F;
            (*(uint32_t *)(&usClientState->controlEndpointBuffer[0])) = pFifo->UDPHS_READEPT0[0];
            (*(uint32_t *)(&usClientState->controlEndpointBuffer[4])) = pFifo->UDPHS_READEPT0[0];

            // special handling for the very first SETUP command - Getdescriptor[DeviceType], the host looks for 8 bytes data only
            TinyCLR_UsbClient_SetupPacket* Setup = (TinyCLR_UsbClient_SetupPacket*)&usClientState->controlEndpointBuffer[0];


            if ((Setup->Request == USB_GET_DESCRIPTOR) && (((Setup->Value & 0xFF00) >> 8) == USB_DEVICE_DESCRIPTOR_TYPE) && (Setup->Length != 0x12))
                usbDeviceDrivers[usClientState->controllerIndex].firstDescriptorPacket = true;
            else
                usbDeviceDrivers[usClientState->controllerIndex].firstDescriptorPacket = false;


            while (pUdp->UDPHS_EPT[0].UDPHS_EPTSTA & AT91C_UDPHS_RX_SETUP)
                pUdp->UDPHS_EPT[0].UDPHS_EPTCLRSTA = AT91C_UDPHS_RX_SETUP;

            /* send it to the upper layer */
            usClientState->ptrData = &usClientState->controlEndpointBuffer[0];
            usClientState->dataSize = len;

            pUdp->UDPHS_EPT[0].UDPHS_EPTCTLENB = AT91C_UDPHS_TX_PK_RDY;

            uint8_t result = TinyCLR_UsbClient_ControlCallback(usClientState);

            switch (result) {
            case USB_STATE_DATA:
                /* setup packet was handled and the upper layer has data to send */
                break;

            case USB_STATE_ADDRESS:
                /* upper layer needs us to change the address */
                // address stage handles in hardware
                break;

            case USB_STATE_DONE:
                usClientState->dataCallback = NULL;
                break;

            case USB_STATE_STALL:
                // since the setup command all handled in the hardware, should not have this state
                //
                // setup packet failed to process successfully
                // set stall condition on the default control
                // endpoint
                //
                AT91_UsbDevice_StallEndPoint(0);
                break;

            case USB_STATE_STATUS:
                // handle by hardware
                break;

            case USB_STATE_CONFIGURATION:
                /* USB spec 9.4.5 SET_CONFIGURATION resets halt conditions, resets toggle bits */
                AT91_UsbDevice_ConfigEndPoint();
                break;

            case USB_STATE_REMOTE_WAKEUP:
                // It is not using currently as the device side won't go into SUSPEND mode unless
                // the PC is purposely to select it to SUSPEND, as there is always SOF in the bus
                // to keeping the device from SUSPEND.
                break;

            default:
                return;
                // the status change is only seen and taken care in hardware
            }

            if (result != USB_STATE_STALL) {
                AT91_UsbDevice_ControlNext(usClientState);
                // If port is now configured, output any queued data
                if (result == USB_STATE_CONFIGURATION) {
                    for (int32_t ep = 1; ep < AT91_USB_ENDPOINT_COUNT; ep++) {
                        if (usClientState->queues[ep] && usClientState->isTxQueue[ep])
                            TinyCLR_UsbClient_StartOutput(usClientState, ep);
                    }
                }
            }
        }
        else if (!(Status & AT91C_UDPHS_TX_PK_RDY) && (pUdp->UDPHS_EPT[0].UDPHS_EPTCTL & AT91C_UDPHS_TX_PK_RDY)) {
            AT91_UsbDevice_ControlNext(usClientState);

            if (usClientState->address) {
                if (!(pUdp->UDPHS_CTRL & AT91C_UDPHS_FADDR_EN))
                    pUdp->UDPHS_CTRL |= AT91C_UDPHS_FADDR_EN | usClientState->address;
            }
        }
    }
    else {

        if ((AT91_UsbDevice_EndpointAttr[endpoint].Dir_Type & AT91C_UDPHS_EPT_DIR_IN) == AT91C_UDPHS_EPT_DIR_IN) {
            AT91_UsbDevice_TxPacket(usClientState, endpoint);
        }
        // OUT packet received
        else if (((Status & AT91C_UDPHS_RX_BK_RDY) != 0) && ((AT91_UsbDevice_EndpointAttr[endpoint].Dir_Type & AT91C_UDPHS_EPT_DIR_OUT) == AT91C_UDPHS_EPT_DIR_OUT)) {
            bool          DisableRx;

            uint32_t len = ((Status & AT91C_UDPHS_BYTE_COUNT) >> 20) & 0x7FF;
            uint8_t *pDest = (uint8_t *)(pFifo->UDPHS_READEPT0 + 16384 * endpoint);
            uint8_t block = len / USB_MAX_DATA_PACKET_SIZE;
            uint8_t rest = len % USB_MAX_DATA_PACKET_SIZE;
            while (block > 0) {
                USB_PACKET64* Packet64 = TinyCLR_UsbClient_RxEnqueue(usClientState, endpoint, DisableRx);
                if (!DisableRx) {

                    memcpy(&(Packet64->Buffer[0]), pDest, USB_MAX_DATA_PACKET_SIZE);
                    Packet64->Size = USB_MAX_DATA_PACKET_SIZE;
                    pDest += USB_MAX_DATA_PACKET_SIZE;
                    block--;
                }
            }
            if ((rest > 0) && (block == 0)) {
                USB_PACKET64* Packet64 = TinyCLR_UsbClient_RxEnqueue(usClientState, endpoint, DisableRx);
                if (!DisableRx) {
                    memcpy(&(Packet64->Buffer[0]), pDest, rest);
                    pDest += rest;
                    Packet64->Size = rest;
                }
            }

            pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCLRSTA = AT91C_UDPHS_RX_BK_RDY;

            if (DisableRx) {
                pUdp->UDPHS_IEN &= ~(1 << SHIFT_INTERUPT << endpoint);
                pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLDIS = AT91C_UDPHS_RX_BK_RDY;
            }
        }
    }
}

void AT91_UsbDevice_AddApi(const TinyCLR_Api_Manager* apiManager) {
    TinyCLR_UsbClient_AddApi(apiManager);

}
const TinyCLR_Api_Info* AT91_UsbDevice_GetRequiredApi() {
    return TinyCLR_UsbClient_GetRequiredApi();
}

void AT91_UsbDevice_Reset() {
    return TinyCLR_UsbClient_Reset(0);
}

void AT91_UsbDevice_InitializeConfiguration(UsClientState *usClientState) {
    auto controllerIndex = 0;

    if (usClientState != nullptr) {
        usClientState->controllerIndex = controllerIndex;

        usClientState->maxFifoPacketCountDefault = AT91_USB_PACKET_FIFO_COUNT;
        usClientState->totalEndpointsCount = AT91_USB_ENDPOINT_COUNT;
        usClientState->totalPipesCount = AT91_USB_PIPE_COUNT;

        // Update endpoint size DeviceDescriptor Configuration if device value is different to default value
        usClientState->deviceDescriptor.MaxPacketSizeEp0 = TinyCLR_UsbClient_GetEndpointSize(0);

        usbDeviceDrivers[controllerIndex].usClientState = usClientState;
    }
}

bool AT91_UsbDevice_Initialize(UsClientState *usClientState) {
    if (usClientState == nullptr)
        return false;

    auto controllerIndex = usClientState->controllerIndex;

    usbDeviceDrivers[controllerIndex].usClientState = usClientState;

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    struct AT91_UDPHS_EPT *pEp = (struct AT91_UDPHS_EPT *) (AT91C_BASE_UDP + 0x100);

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Enable USB device clock
    AT91_UsbDevice_PmcEnableUsbClock();

    // Enable the interrupt for  UDP
    AT91_InterruptInternal_Activate(AT91C_ID_UDP, (uint32_t*)&AT91_UsbDevice_InterruptHandler, (void*)usClientState);

    pUdp->UDPHS_IEN |= AT91C_UDPHS_EPT_INT_0;
    pEp->UDPHS_EPTCFG |= 0x00000043; //configuration info for control ep

    for (auto pipe = 0; pipe < AT91_USB_PIPE_COUNT; pipe++) {
        auto idx = 0;
        if (usClientState->pipes[pipe].RxEP != USB_ENDPOINT_NULL) {
            idx = usClientState->pipes[pipe].RxEP;
            AT91_UsbDevice_EndpointAttr[idx].Dir_Type = AT91C_UDPHS_EPT_TYPE_BUL_EPT;
            AT91_UsbDevice_EndpointAttr[idx].Dir_Type |= AT91C_UDPHS_EPT_DIR_OUT;
            pUdp->UDPHS_IEN |= (AT91C_UDPHS_EPT_INT_0 << idx);
        }

        if (usClientState->pipes[pipe].TxEP != USB_ENDPOINT_NULL) {
            idx = usClientState->pipes[pipe].TxEP;
            AT91_UsbDevice_EndpointAttr[idx].Dir_Type = AT91C_UDPHS_EPT_TYPE_BUL_EPT;
            AT91_UsbDevice_EndpointAttr[idx].Dir_Type |= AT91C_UDPHS_EPT_DIR_IN;
            pUdp->UDPHS_IEN |= (AT91C_UDPHS_EPT_INT_0 << idx);
        }
    }

    usClientState->firstGetDescriptor = true;

    AT91_UsbDevice_ProtectPins(usClientState, true);

    AT91_Time_Delay(nullptr, 100000); // 100ms

    return true;
}

bool AT91_UsbDevice_Uninitialize(UsClientState *usClientState) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_UsbDevice_PmcDisableUTMIBIAS();
    AT91_UsbDevice_PmcDisableUsbClock();

    AT91_InterruptInternal_Deactivate(AT91C_ID_UDP);

    if (usClientState != nullptr) {
        AT91_UsbDevice_ProtectPins(usClientState, false);
        usClientState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
    }

    return true;
}

bool AT91_UsbDevice_StartOutput(UsClientState* usClientState, int32_t endpoint) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;

    // If endpoint is not an output
    if (usClientState->queues[endpoint] == NULL || !usClientState->isTxQueue[endpoint])
        return false;

    pUdp->UDPHS_IEN |= 1 << SHIFT_INTERUPT << endpoint;
    pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLENB = AT91C_UDPHS_TX_PK_RDY;

    /* if the halt feature for this endpoint is set, then just
            clear all the characters */
    if (usClientState->endpointStatus[endpoint] & USB_STATUS_ENDPOINT_HALT) {
        AT91_UsbDevice_ClearTxQueue(usClientState, endpoint);
        return true;
    }

    //If txRunning, interrupts will drain the queue
    if (!(bool)((uint8_t)usbDeviceDrivers[usClientState->controllerIndex].txRunning[endpoint])) {

        usbDeviceDrivers[usClientState->controllerIndex].txRunning[endpoint] = true;

        AT91_UsbDevice_TxPacket(usClientState, endpoint);
    }
    else if (irq.IsDisabled()) // We should not call EP_TxISR after calling AT91_UsbDevice_TxPacket becuase it can cause a TX FIFO overflow error.
    {
        AT91_UsbDevice_EndpointIsr(usClientState, endpoint);
    }

    return true;
}

bool AT91_UsbDevice_RxEnable(UsClientState* usClientState, int32_t endpoint) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;

    pUdp->UDPHS_IEN |= 1 << SHIFT_INTERUPT << endpoint;
    pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLENB = AT91C_UDPHS_RX_BK_RDY;

    return true;
}

bool TinyCLR_UsbClient_Initialize(UsClientState* usClientState) {
    return AT91_UsbDevice_Initialize(usClientState);
}

bool TinyCLR_UsbClient_Uninitialize(UsClientState* usClientState) {
    return AT91_UsbDevice_Uninitialize(usClientState);
}

bool TinyCLR_UsbClient_StartOutput(UsClientState* usClientState, int32_t endpoint) {
    return AT91_UsbDevice_StartOutput(usClientState, endpoint);
}

bool TinyCLR_UsbClient_RxEnable(UsClientState* usClientState, int32_t endpoint) {
    return AT91_UsbDevice_RxEnable(usClientState, endpoint);
}

void TinyCLR_UsbClient_Delay(uint64_t microseconds) {
    AT91_Time_Delay(nullptr, microseconds);
}

uint64_t TinyCLR_UsbClient_Now() {
    return AT91_Time_GetCurrentProcessorTime();
}

void TinyCLR_UsbClient_InitializeConfiguration(UsClientState *usClientState) {
    AT91_UsbDevice_InitializeConfiguration(usClientState);
}

uint32_t TinyCLR_UsbClient_GetEndpointSize(int32_t endpoint) {
    return endpoint == 0 ? AT91_USB_ENDPOINT0_SIZE : AT91_USB_ENDPOINT_SIZE;
}

