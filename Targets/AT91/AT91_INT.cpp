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

#define VECTORING_GUARD  32
#define DEFINE_IRQ(index) { index, { NULL, (void*)(size_t)index } }
#define DISABLED_MASK 0x80

extern "C" {
    uint32_t    IRQ_LOCK_Release_asm();
    void        IRQ_LOCK_Probe_asm();
    uint32_t    IRQ_LOCK_GetState_asm();
    uint32_t    IRQ_LOCK_ForceDisabled_asm();
    uint32_t    IRQ_LOCK_ForceEnabled_asm();
    uint32_t    IRQ_LOCK_Disable_asm();
    void        IRQ_LOCK_Restore_asm();
}

typedef void(*AT91_Interrupt_Handler)(void* arg);

struct AT91_Interrupt_Callback {

public:
    void* EntryPoint;
    void* Argument;

public:
    void Initialize(uint32_t* EntryPoint, void* Argument) {
        this->EntryPoint = (void*)EntryPoint;
        this->Argument = Argument;
    }

    void Execute() const {
        AT91_Interrupt_Handler EntryPoint = (AT91_Interrupt_Handler)this->EntryPoint;

        void* Argument = this->Argument;

        if (EntryPoint) {
            EntryPoint(Argument);
        }
    }
};

struct AT91_Interrupt_Vectors {
    uint32_t                    Index;
    AT91_Interrupt_Callback    Handler;
};

static AT91_Interrupt_Vectors* AT91_Interrupt_IrqToVector(uint32_t Irq);

AT91_Interrupt_Vectors s_IsrTable[] =
{
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_WDT),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_SW),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_DBG_COM_RX),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_DBG_COM_TX),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_TIMER0),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_TIMER1),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_UART0),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_UART1),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_PWM_0_1),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_I2C0),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_SPI_SSP0),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_SSP1),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_PLL),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_RTC),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_EINT0),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_EINT1),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_EINT2_LCD),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_EINT3),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_ADC0),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_I2C1),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_BOD),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_EMAC),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_USB),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_CAN),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_SD),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_DMA),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_TIMER2),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_TIMER3),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_UART2),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_UART3),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_I2C2),
    DEFINE_IRQ(AT91XX_VIC::c_IRQ_INDEX_I2S),

    DEFINE_IRQ(VECTORING_GUARD),
};

AT91_Interrupt_Vectors* AT91_Interrupt_IrqToVector(uint32_t Irq) {
    AT91_Interrupt_Vectors* IsrVector = s_IsrTable;

    while (IsrVector->Index != VECTORING_GUARD) {
        if (IsrVector->Index == Irq) return IsrVector;

        IsrVector++;
    }

    return NULL;
}

void AT91_Interrupt_StubIrqVector(void* Param) {

}

TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Started;
TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Ended;

static TinyCLR_Interrupt_Provider interruptProvider;
static TinyCLR_Api_Info interruptApi;

const TinyCLR_Api_Info* AT91_Interrupt_GetApi() {
    interruptProvider.Parent = &interruptApi;
    interruptProvider.Index = 0;
    interruptProvider.Parent = &interruptApi;
    interruptProvider.Acquire = &AT91_Interrupt_Acquire;
    interruptProvider.Release = &AT91_Interrupt_Release;

    interruptProvider.IsDisabled = &AT91_Interrupt_GlobalIsDisabled;
    interruptProvider.Enable = &AT91_Interrupt_GlobalEnable;
    interruptProvider.Disable = &AT91_Interrupt_GlobalDisable;
    interruptProvider.Restore = &AT91_Interrupt_GlobalRestore;
    interruptProvider.WaitForInterrupt = &AT91_Interrupt_GlobalWaitForInterrupt;

    interruptApi.Author = "GHI Electronics, LLC";
    interruptApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.InterruptProvider";
    interruptApi.Type = TinyCLR_Api_Type::InterruptProvider;
    interruptApi.Version = 0;
    interruptApi.Count = 1;
    interruptApi.Implementation = &interruptProvider;

    return &interruptApi;
}

TinyCLR_Result AT91_Interrupt_Acquire(TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd) {
    AT91_Interrupt_Started = onInterruptStart;
    AT91_Interrupt_Ended = onInterruptEnd;

    AT91XX_VIC& VIC = AT91XX::VIC();

    // disable all interrupts
    VIC.INTENCLR = 0xFFFFFFFF;

    AT91_Interrupt_Vectors* IsrVector = s_IsrTable;

    while (IsrVector->Index != VECTORING_GUARD) {
        IsrVector->Handler.Initialize((uint32_t*)&AT91_Interrupt_StubIrqVector, (void*)(size_t)IsrVector->Index);

        IsrVector++;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Interrupt_Release() {
    return TinyCLR_Result::Success;
}

bool AT91_Interrupt_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param) {
    // figure out the interrupt
    AT91_Interrupt_Vectors* IsrVector = AT91_Interrupt_IrqToVector(Irq_Index);

    if (!IsrVector)
        return false;

    {
        GLOBAL_LOCK(irq);

        AT91XX_VIC& VIC = AT91XX::VIC();

        // disable this interrupt while we change it
        VIC.INTENCLR = 1 << IsrVector->Index;

        // set the vector
        IsrVector->Handler.Initialize(ISR, ISR_Param);

        // Use Vector Address register to identify the source of interrupt
        VIC.VECTADDR[Irq_Index] = Irq_Index;

        // enable the interrupt if we have a vector
        VIC.INTENABLE = 1 << IsrVector->Index;
    }

    return true;

}

bool AT91_Interrupt_Deactivate(uint32_t Irq_Index) {
    // figure out the interrupt
    AT91_Interrupt_Vectors* IsrVector = AT91_Interrupt_IrqToVector(Irq_Index);

    if (!IsrVector)
        return false;
    {
        GLOBAL_LOCK(irq);

        AT91XX_VIC& VIC = AT91XX::VIC();

        // disable this interrupt while we change it
        VIC.INTENCLR = 1 << IsrVector->Index;

        IsrVector->Handler.Initialize((uint32_t*)&AT91_Interrupt_StubIrqVector, (void*)(size_t)IsrVector->Index);
    }

    return true;
}

bool AT91_Interrupt_Enable(uint32_t Irq_Index) {
    AT91_Interrupt_Vectors* IsrVector = AT91_Interrupt_IrqToVector(Irq_Index);

    if (!IsrVector)
        return false;

    AT91XX_VIC& VIC = AT91XX::VIC();

    GLOBAL_LOCK(irq);

    bool WasEnabled = VIC.IsInterruptEnabled(Irq_Index);

    VIC.INTENABLE = 1 << IsrVector->Index;

    return WasEnabled;
}

bool AT91_Interrupt_Disable(uint32_t Irq_Index) {
    AT91_Interrupt_Vectors* IsrVector = AT91_Interrupt_IrqToVector(Irq_Index);

    if (!IsrVector)
        return 0;

    AT91XX_VIC& VIC = AT91XX::VIC();

    GLOBAL_LOCK(irq);

    bool WasEnabled = VIC.IsInterruptEnabled(Irq_Index);

    VIC.INTENCLR = 1 << IsrVector->Index;

    return WasEnabled;
}

bool AT91_Interrupt_EnableState(uint32_t Irq_Index) {
    AT91XX_VIC& VIC = AT91XX::VIC();

    return VIC.IsInterruptEnabled(Irq_Index);
}

bool AT91_Interrupt_InterruptState(uint32_t Irq_Index) {
    AT91XX_VIC& VIC = AT91XX::VIC();

    return VIC.GetInterruptState(Irq_Index);
}

bool AT91_SmartPtr_IRQ::WasDisabled() {
    return (m_state & DISABLED_MASK) == DISABLED_MASK;
}

void AT91_SmartPtr_IRQ::Acquire() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == DISABLED_MASK) {
        Disable();
    }
}

void AT91_SmartPtr_IRQ::Release() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        m_state = IRQ_LOCK_Release_asm();
    }
}

void AT91_SmartPtr_IRQ::Probe() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        IRQ_LOCK_Probe_asm();
    }
}

uint32_t AT91_SmartPtr_IRQ::GetState() {
    return IRQ_LOCK_GetState_asm();
}

void AT91_SmartPtr_IRQ::Disable() {
    m_state = IRQ_LOCK_Disable_asm();
}

void AT91_SmartPtr_IRQ::Restore() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        IRQ_LOCK_Restore_asm();
    }
}

//////////////////////////////////////////////////////////////////////////////
//Global Interrupt - Use in System Cote
//////////////////////////////////////////////////////////////////////////////

bool AT91_Interrupt_GlobalIsDisabled() {
    return (IRQ_LOCK_GetState_asm() & DISABLED_MASK);
}

bool AT91_Interrupt_GlobalEnable(bool force) {
    if (!force) {
        return (IRQ_LOCK_Release_asm() & DISABLED_MASK == 0);
    }

    return  (IRQ_LOCK_ForceEnabled_asm() & DISABLED_MASK == 0);
}

void AT91_Interrupt_GlobalRestore() {
    IRQ_LOCK_Restore_asm();
}

bool AT91_Interrupt_GlobalDisable(bool force) {
    if (!force) {
        return ((IRQ_LOCK_Disable_asm() & DISABLED_MASK) == DISABLED_MASK);
    }

    return ((IRQ_LOCK_ForceDisabled_asm() & DISABLED_MASK) == DISABLED_MASK);
}

void AT91_Interrupt_GlobalWaitForInterrupt() {
    IRQ_LOCK_Probe_asm();
}

extern "C" {
    void AT91_Interrupt_UndefHandler(unsigned int*, unsigned int, unsigned int) {
        volatile uint32_t debug = 0;

        while (debug == 0) {
            debug += debug;
        }
    }

    void AT91_Interrupt_AbortpHandler(unsigned int*, unsigned int, unsigned int) {
        volatile uint32_t debug = 0;

        while (debug == 0) {
            debug += debug;
        }
    }

    void AT91_Interrupt_AbortdHandler(unsigned int*, unsigned int, unsigned int) {
        volatile uint32_t debug = 0;
        while (debug == 0) {
            debug += debug;
        }
    }

    void __attribute__((interrupt("IRQ"))) IRQ_Handler(void *param) {
        // set before jumping elsewhere or allowing other interrupts
        INTERRUPT_START

            uint32_t index;

        AT91XX_VIC& VIC = AT91XX::VIC();

        index = VIC.ADDRESS;

        AT91_Interrupt_Vectors* IsrVector = &s_IsrTable[index];

        // In case the interrupt was forced, remove the flag.
        VIC.RemoveForcedInterrupt(index);

        IsrVector->Handler.Execute();

        INTERRUPT_END

            // Reset VIC priority hw logic.
            VIC.ADDRESS = 0xFF;
    }
}
