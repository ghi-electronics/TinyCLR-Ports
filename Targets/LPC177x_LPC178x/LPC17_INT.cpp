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

#include "LPC17.h"

#define DISABLED_MASK  0x00000001

#define TOTAL_INTERRUPT_CONTROLLERS 1

TinyCLR_Interrupt_StartStopHandler LPC17_Interrupt_Started;
TinyCLR_Interrupt_StartStopHandler LPC17_Interrupt_Ended;

static TinyCLR_Interrupt_Controller interruptControllers[TOTAL_INTERRUPT_CONTROLLERS];
static TinyCLR_Api_Info interruptApi[TOTAL_INTERRUPT_CONTROLLERS];

const TinyCLR_Api_Info* LPC17_Interrupt_GetApi() {
    for (int32_t i = 0; i < TOTAL_INTERRUPT_CONTROLLERS; i++) {
        interruptControllers[i].ApiInfo = &interruptApi[i];
        interruptControllers[i].Initialize = &LPC17_Interrupt_Initialize;
        interruptControllers[i].Uninitialize = &LPC17_Interrupt_Uninitialize;
        interruptControllers[i].Enable = &LPC17_Interrupt_GlobalEnabled;
        interruptControllers[i].Disable = &LPC17_Interrupt_GlobalDisabled;
        interruptControllers[i].WaitForInterrupt = &LPC17_Interrupt_GlobalWaitForInterrupt;
        interruptControllers[i].IsDisabled = &LPC17_Interrupt_GlobalIsDisabled;
        interruptControllers[i].Restore = &LPC17_Interrupt_GlobalRestore;

        interruptApi[i].Author = "GHI Electronics, LLC";
        interruptApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.InterruptController";
        interruptApi[i].Type = TinyCLR_Api_Type::InterruptController;
        interruptApi[i].Version = 0;
        interruptApi[i].Implementation = &interruptControllers[i];
        interruptApi[i].State = nullptr;
    }

    return (const TinyCLR_Api_Info*)&interruptApi;
}

extern "C" {
    void SysTick_Handler(void *param);
    extern uint32_t __Vectors;
}

TinyCLR_Result LPC17_Interrupt_Initialize(const TinyCLR_Interrupt_Controller* self, TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd) {
    uint32_t *irq_vectors = (uint32_t*)&__Vectors;

    // disable all interrupts
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICER[2] = 0xFFFFFFFF;
    // clear pending bits
    NVIC->ICPR[0] = 0xFFFFFFFF;
    NVIC->ICPR[1] = 0xFFFFFFFF;
    NVIC->ICPR[2] = 0xFFFFFFFF;

    // force point to SysTick_Handler because GNU does not link to the function automatically
    irq_vectors[15] = (uint32_t)&SysTick_Handler;

    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) // unlock key
        | (7 << SCB_AIRCR_PRIGROUP_Pos);   // no priority group bits
    SCB->VTOR = (uint32_t)&__Vectors; // vector table base
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk  // enable faults
        | SCB_SHCSR_BUSFAULTENA_Msk
        | SCB_SHCSR_MEMFAULTENA_Msk;

    LPC17_Interrupt_Started = onInterruptStart;
    LPC17_Interrupt_Ended = onInterruptEnd;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Interrupt_Uninitialize(const TinyCLR_Interrupt_Controller* self) {
    return TinyCLR_Result::Success;
}

bool LPC17_Interrupt_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param) {
    int id = (int)Irq_Index;

    uint32_t *irq_vectors = (uint32_t*)&__Vectors;

    irq_vectors[id + 16] = (uint32_t)ISR; // exception = irq + 16
    NVIC->ICPR[id >> 5] = 1 << (id & 0x1F); // clear pending bit
    NVIC->ISER[id >> 5] = 1 << (id & 0x1F); // set enable bit

    return true;
}

bool LPC17_Interrupt_Deactivate(uint32_t Irq_Index) {
    int id = (int)Irq_Index;

    NVIC->ICER[id >> 5] = 1 << (id & 0x1F); // clear enable bit */

    return true;
}

LPC17_SmartPtr_Interrupt::LPC17_SmartPtr_Interrupt() { LPC17_Interrupt_Started(); };
LPC17_SmartPtr_Interrupt::~LPC17_SmartPtr_Interrupt() { LPC17_Interrupt_Ended(); };

LPC17_SmartPtr_IRQ::LPC17_SmartPtr_IRQ() { Disable(); }
LPC17_SmartPtr_IRQ::~LPC17_SmartPtr_IRQ() { Restore(); }

bool LPC17_Interrupt_Enable(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    uint32_t ier = NVIC->ISER[id >> 5]; // old state
    NVIC->ISER[id >> 5] = 1 << (id & 0x1F); // set enable bit

    return (ier >> (id & 0x1F)) & 1; // old enable bit
}

bool LPC17_Interrupt_Disable(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    uint32_t ier = NVIC->ISER[id >> 5]; // old state

    NVIC->ICER[id >> 5] = 1 << (id & 0x1F); // clear enable bit

    return (ier >> (id & 0x1F)) & 1; // old enable bit
}

bool LPC17_Interrupt_EnableState(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    // return enabled bit
    return (NVIC->ISER[id >> 5] >> (id & 0x1F)) & 1;
}

bool LPC17_Interrupt_InterruptState(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    // return pending bit
    return (NVIC->ISPR[id >> 5] >> (id & 0x1F)) & 1;
}

bool LPC17_SmartPtr_IRQ::IsDisabled() {
    return (m_state & DISABLED_MASK) == DISABLED_MASK;
}

void LPC17_SmartPtr_IRQ::Acquire() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == DISABLED_MASK)
        Disable();
}

void LPC17_SmartPtr_IRQ::Release() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        m_state = __get_PRIMASK();
        __enable_irq();
    }
}

void LPC17_SmartPtr_IRQ::Probe() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        LPC17_Interrupt_GlobalWaitForInterrupt();
    }
}

bool LPC17_SmartPtr_IRQ::GetState() {
    register uint32_t Cp = __get_PRIMASK();

    return (0 == (Cp & 1));
}

void LPC17_SmartPtr_IRQ::Disable() {
    m_state = __get_PRIMASK();

    __disable_irq();
}

void LPC17_SmartPtr_IRQ::Restore() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        __enable_irq();
    }
}

//////////////////////////////////////////////////////////////////////////////
//Global Interrupt - Use in System Cote
//////////////////////////////////////////////////////////////////////////////
bool LPC17_Interrupt_GlobalIsDisabled() {
    return (__get_PRIMASK() & DISABLED_MASK) == DISABLED_MASK;
}

bool LPC17_Interrupt_GlobalEnabled(bool force) {
    __enable_irq();

    return true;
}

bool LPC17_Interrupt_GlobalDisabled(bool force) {
    bool wasDisable = LPC17_Interrupt_GlobalIsDisabled();

    __disable_irq();

    return wasDisable;
}

void LPC17_Interrupt_GlobalWaitForInterrupt() {
    register uint32_t state = __get_PRIMASK();

    __enable_irq();

    // just to allow an interupt to an occur
    __WFI();

    // restore irq state
    __set_PRIMASK(state);
}

void LPC17_Interrupt_GlobalRestore() {
    __enable_irq();
}