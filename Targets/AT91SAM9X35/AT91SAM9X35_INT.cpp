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

#define DISABLED_MASK 0x80

///////////////////////////////////////////////////////////////////////////////
#define DEFINE_IRQ(index, priority) { priority, { NULL, (void*)(size_t)index } }

extern "C" {
    uint32_t    IRQ_LOCK_Release_asm();
    void        IRQ_LOCK_Probe_asm();
    uint32_t    IRQ_LOCK_GetState_asm();
    uint32_t    IRQ_LOCK_ForceDisabled_asm();
    uint32_t    IRQ_LOCK_ForceEnabled_asm();
    uint32_t    IRQ_LOCK_Disable_asm();
    void        IRQ_LOCK_Restore_asm();
}

struct AT91_Interrupt_Vectors {
    uint32_t                    Priority;
    AT91_Interrupt_Callback    Handler;
};

static const uint32_t c_VECTORING_GUARD = 32;
static const uint32_t c_MaxInterruptIndex = 32;

AT91_Interrupt_Vectors s_IsrTable[] =
{
    DEFINE_IRQ(0,   7),      // Advanced Interrupt Controller
    DEFINE_IRQ(1,   7),      // System Peripherals
    DEFINE_IRQ(2,   1),      // Parallel IO Controller A
    DEFINE_IRQ(3,   1),      // Parallel IO Controller B
    DEFINE_IRQ(4,   1),      // Parallel IO Controller C
    DEFINE_IRQ(5,   1),      // Parallel IO Controller D
    DEFINE_IRQ(6,   5),      // USART 0
    DEFINE_IRQ(7,   5),      // USART 1
    DEFINE_IRQ(8,   5),      // USART 2
    DEFINE_IRQ(9,   5),      // USART 3
    DEFINE_IRQ(10,  0),      // Multimedia Card Interface
    DEFINE_IRQ(11,  6),      // Two-Wire Interface
    DEFINE_IRQ(12,  6),      // Two-Wire Interface
    DEFINE_IRQ(13,  5),      // Serial Peripheral Interface
    DEFINE_IRQ(14,  4),      // Serial Synchronous Controller 0
    DEFINE_IRQ(15,  4),      // Serial Synchronous Controller 1
    DEFINE_IRQ(16,  6),      // Timer Counter 0
    DEFINE_IRQ(17,  0),      // Timer Counter 1
    DEFINE_IRQ(18,  0),      // Timer Counter 2
    DEFINE_IRQ(19,  0),      // PWMC
    DEFINE_IRQ(20,  5),      // TSADCC
    DEFINE_IRQ(21,  6),      // DMAC
    DEFINE_IRQ(22,  6),      //High Speed USB
    DEFINE_IRQ(23,  3),      //LCDC
    DEFINE_IRQ(24,  3),      //AC97
    DEFINE_IRQ(25,  0),
    DEFINE_IRQ(26,  0),
    DEFINE_IRQ(27,  0),
    DEFINE_IRQ(28,  0),
    DEFINE_IRQ(29,  0),
    DEFINE_IRQ(30,  0),
    DEFINE_IRQ(31,  0),      // Advanced Interrupt Controller
};

TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Started;
TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Ended;

#define TOTAL_INTERRUPT_CONTROLLERS 1

struct InterruptState {
    uint32_t controllerIndex;
    bool tableInitialized;
};

const char* interruptApiNames[TOTAL_INTERRUPT_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.AT91.InterruptController\\0"
};

static TinyCLR_Interrupt_Controller interruptControllers[TOTAL_INTERRUPT_CONTROLLERS];
static TinyCLR_Api_Info interruptApi[TOTAL_INTERRUPT_CONTROLLERS];
static InterruptState interruptStates[TOTAL_INTERRUPT_CONTROLLERS];

void AT91_Interrupt_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_INTERRUPT_CONTROLLERS; i++) {
        if (interruptStates[i].tableInitialized)
            continue;

        interruptControllers[i].ApiInfo = &interruptApi[i];
        interruptControllers[i].Initialize = &AT91_Interrupt_Initialize;
        interruptControllers[i].Uninitialize = &AT91_Interrupt_Uninitialize;
        interruptControllers[i].Enable = &AT91_Interrupt_Enable;
        interruptControllers[i].Disable = &AT91_Interrupt_Disable;
        interruptControllers[i].WaitForInterrupt = &AT91_Interrupt_WaitForInterrupt;
        interruptControllers[i].IsDisabled = &AT91_Interrupt_IsDisabled;

        interruptApi[i].Author = "GHI Electronics, LLC";
        interruptApi[i].Name = interruptApiNames[i];
        interruptApi[i].Type = TinyCLR_Api_Type::InterruptController;
        interruptApi[i].Version = 0;
        interruptApi[i].Implementation = &interruptControllers[i];
        interruptApi[i].State = &interruptStates[i];

        interruptStates[i].controllerIndex = i;
        interruptStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* AT91_Interrupt_GetRequiredApi() {
    AT91_Interrupt_EnsureTableInitialized();

    return &interruptApi[0];
}

void AT91_Interrupt_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91_Interrupt_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_INTERRUPT_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &interruptApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::InterruptController, interruptApi[0].Name);
}


AT91_Interrupt_Vectors* AT91_Interrupt_IrqToVector(uint32_t Irq) {
    AT91_Interrupt_Vectors* IsrVector = s_IsrTable;

    if (Irq < c_VECTORING_GUARD) {
        return &IsrVector[Irq];
    }

    return nullptr;
}

void AT91_Interrupt_ForceInterrupt(uint32_t Irq_Index) {
    AT91_AIC &AIC = AT91::AIC();

    AIC.AIC_ISCR = (1 << Irq_Index);
}

void AT91_Interrupt_RemoveForcedInterrupt(uint32_t Irq_Index) {
    AT91_AIC &AIC = AT91::AIC();
    AIC.AIC_ICCR = (1 << Irq_Index);
}

void AT91_Interrupt_StubIrqVector(void* Param) {

}

TinyCLR_Result AT91_Interrupt_Initialize(const TinyCLR_Interrupt_Controller* self, TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd) {
    AT91_Interrupt_Started = onInterruptStart;
    AT91_Interrupt_Ended = onInterruptEnd;

    AT91_AIC &aic = AT91::AIC();
    aic.AIC_IDCR = AT91_AIC::AIC_IDCR_DIABLE_ALL;           // Disable the interrupt on the interrupt controller
    aic.AIC_ICCR = AT91_AIC::AIC_ICCR_CLEAR_ALL;            // Clear the interrupt on the Interrupt Controller ( if one is pending )

    for (int32_t i = 0; i < c_VECTORING_GUARD; ++i) {
        (void)aic.AIC_IVR;
        aic.AIC_EOICR = (uint32_t)i;
    }

    // set all priorities to the lowest
    AT91_Interrupt_Vectors* IsrVector = s_IsrTable;

    // set the priority level for each IRQ and stub the IRQ callback
    for (int32_t i = 0; i < c_VECTORING_GUARD; i++) {
        aic.AIC_SVR[i] = (uint32_t)i;

        if (i == AT91C_ID_TC0_TC1) {
            aic.AIC_SMR[i] = AT91_AIC::AIC_SRCTYPE_INT_POSITIVE_EDGE;
        }

        aic.AIC_SMR[i] &= ~AT91_AIC::AIC_PRIOR;
        aic.AIC_SMR[i] |= IsrVector[i].Priority;

        IsrVector[i].Handler.Initialize((uint32_t*)&AT91_Interrupt_StubIrqVector, (void*)(size_t)IsrVector[i].Priority);
    }

    // Set Spurious interrupt vector
    aic.AIC_SPU = c_VECTORING_GUARD;


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Interrupt_Uninitialize(const TinyCLR_Interrupt_Controller* self) {
    return TinyCLR_Result::Success;
}

bool AT91_InterruptInternal_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param) {
    // figure out the interrupt
    AT91_Interrupt_Vectors* IsrVector = AT91_Interrupt_IrqToVector(Irq_Index);

    if (!IsrVector)
        return false;

    AT91_AIC &aic = AT91::AIC();

    DISABLE_INTERRUPTS_SCOPED(irq);

    // disable this interrupt while we change it
    aic.AIC_IDCR = (0x01 << Irq_Index);

    // Clear the interrupt on the interrupt controller
    aic.AIC_ICCR = (0x01 << Irq_Index);

    // set the vector
    IsrVector->Handler.Initialize(ISR, ISR_Param);

    // enable the interrupt if we have a vector
    aic.AIC_IECR = 0x01 << Irq_Index;

    return true;

}

bool AT91_InterruptInternal_Deactivate(uint32_t Irq_Index) {
    // figure out the interrupt
    AT91_Interrupt_Vectors* IsrVector = 0; //IRQToIRQVector( Irq_Index );

    if (!IsrVector)
        return false;

    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_AIC &aic = AT91::AIC();

    // Clear the interrupt on the Interrupt Controller ( if one is pending )
    aic.AIC_ICCR = (1 << Irq_Index);

    // Disable the interrupt on the interrupt controller
    aic.AIC_IDCR = (1 << Irq_Index);

    // as it is stub, just put the Priority to the ISR parameter
    IsrVector->Handler.Initialize((uint32_t*)&AT91_Interrupt_StubIrqVector, (void*)(size_t)IsrVector->Priority);

    return true;
}

AT91_InterruptStarted_RaiiHelper::AT91_InterruptStarted_RaiiHelper() { AT91_Interrupt_Started(); };
AT91_InterruptStarted_RaiiHelper::~AT91_InterruptStarted_RaiiHelper() { AT91_Interrupt_Ended(); };

AT91_DisableInterrupts_RaiiHelper::AT91_DisableInterrupts_RaiiHelper() {
    state = IRQ_LOCK_Disable_asm();
}
AT91_DisableInterrupts_RaiiHelper::~AT91_DisableInterrupts_RaiiHelper() {
    uint32_t Cp = state;

    if ((Cp & DISABLED_MASK) == 0) {
        state = IRQ_LOCK_Release_asm();
    }
}

bool AT91_DisableInterrupts_RaiiHelper::IsDisabled() {
    return (state & DISABLED_MASK) == DISABLED_MASK;
}

void AT91_DisableInterrupts_RaiiHelper::Acquire() {
    uint32_t Cp = state;

    if ((Cp & DISABLED_MASK) == DISABLED_MASK) {
        state = IRQ_LOCK_Disable_asm();
    }
}

void AT91_DisableInterrupts_RaiiHelper::Release() {
    uint32_t Cp = state;

    if ((Cp & DISABLED_MASK) == 0) {
        state = IRQ_LOCK_Release_asm();
    }
}

//////////////////////////////////////////////////////////////////////////////
//Global Interrupt - Use in System Cote
//////////////////////////////////////////////////////////////////////////////
bool AT91_Interrupt_IsDisabled() {
    return ((IRQ_LOCK_GetState_asm() & DISABLED_MASK) == 0);
}

void AT91_Interrupt_Enable() {
    IRQ_LOCK_Release_asm();
}

void AT91_Interrupt_Disable() {
    IRQ_LOCK_Disable_asm();
}

void AT91_Interrupt_WaitForInterrupt() {
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
        INTERRUPT_STARTED_SCOPED(isr);

        uint32_t index;

        AT91_AIC &aic = AT91::AIC();

        while ((index = aic.AIC_IVR) < c_VECTORING_GUARD) {
            // Read IVR register (de-assert NIRQ) & check if we a spurous IRQ
            AT91_Interrupt_Vectors* IsrVector = &s_IsrTable[index];

            // In case the interrupt was forced, remove the flag.
            AT91_Interrupt_RemoveForcedInterrupt(index);


            IsrVector->Handler.Execute();

            // Mark end of Interrupt
            aic.AIC_EOICR = 1;
        }

        // Mark end of Interrupt (Last IVR read)
        aic.AIC_EOICR = 1;

    }
}
