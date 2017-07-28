// Copyright Microsoft Corporation
// Implementation for STM32F4: Copyright Oberon microsystems, Inc
// Copyright 2017 GHI Electronics, LLC
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

#define DISABLED_MASK  0x00000001

TinyCLR_Interrupt_StartStopHandler STM32F4_Interrupt_Started;
TinyCLR_Interrupt_StartStopHandler STM32F4_Interrupt_Ended;

static TinyCLR_Interrupt_Provider interruptProvider;
static TinyCLR_Api_Info interruptApi;

const TinyCLR_Api_Info* STM32F4_Interrupt_GetApi() {
    interruptProvider.Parent = &interruptApi;
    interruptProvider.Index = 0;
    interruptProvider.Parent = &interruptApi;
    interruptProvider.Acquire = &STM32F4_Interrupt_Acquire;
    interruptProvider.Release = &STM32F4_Interrupt_Release;
    interruptProvider.Enable = &STM32F4_Interrupt_GlobalEnabled;
    interruptProvider.Disable = &STM32F4_Interrupt_GlobalDisabled;
    interruptProvider.WaitForInterrupt = &STM32F4_Interrupt_GlobalWaitForInterrupt;
    interruptProvider.IsDisabled = &STM32F4_Interrupt_GlobalIsDisabled;
    interruptProvider.Restore = &STM32F4_Interrupt_GlobalRestore;

    interruptApi.Author = "GHI Electronics, LLC";
    interruptApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.InterruptProvider";
    interruptApi.Type = TinyCLR_Api_Type::InterruptProvider;
    interruptApi.Version = 0;
    interruptApi.Count = 1;
    interruptApi.Implementation = &interruptProvider;

    return &interruptApi;
}

extern "C" {
    void SysTick_Handler(void *param);
    extern uint32_t __Vectors;
}

TinyCLR_Result STM32F4_Interrupt_Acquire(TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd) {
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

    STM32F4_Interrupt_Started = onInterruptStart;
    STM32F4_Interrupt_Ended = onInterruptEnd;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Interrupt_Release() {
    return TinyCLR_Result::Success;
}

bool STM32F4_Interrupt_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param) {
    int id = (int)Irq_Index;

    uint32_t *irq_vectors = (uint32_t*)&__Vectors;

    irq_vectors[id + 16] = (uint32_t)ISR; // exception = irq + 16
    NVIC->ICPR[id >> 5] = 1 << (id & 0x1F); // clear pending bit
    NVIC->ISER[id >> 5] = 1 << (id & 0x1F); // set enable bit

    return true;
}

bool STM32F4_Interrupt_Deactivate(uint32_t Irq_Index) {
    int id = (int)Irq_Index;

    NVIC->ICER[id >> 5] = 1 << (id & 0x1F); // clear enable bit */

    return true;
}

bool STM32F4_Interrupt_Enable(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    uint32_t ier = NVIC->ISER[id >> 5]; // old state
    NVIC->ISER[id >> 5] = 1 << (id & 0x1F); // set enable bit

    return (ier >> (id & 0x1F)) & 1; // old enable bit
}

bool STM32F4_Interrupt_Disable(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    uint32_t ier = NVIC->ISER[id >> 5]; // old state

    NVIC->ICER[id >> 5] = 1 << (id & 0x1F); // clear enable bit

    return (ier >> (id & 0x1F)) & 1; // old enable bit
}

bool STM32F4_Interrupt_EnableState(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    // return enabled bit
    return (NVIC->ISER[id >> 5] >> (id & 0x1F)) & 1;
}

bool STM32F4_Interrupt_InterruptState(uint32_t Irq_Index) {
    int id = (int)Irq_Index;
    // return pending bit
    return (NVIC->ISPR[id >> 5] >> (id & 0x1F)) & 1;
}

bool STM32F4_SmartPtr_IRQ::WasDisabled() {
    return (m_state & DISABLED_MASK) == DISABLED_MASK;
}

void STM32F4_SmartPtr_IRQ::Acquire() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == DISABLED_MASK)
        Disable();
}

void STM32F4_SmartPtr_IRQ::Release() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        m_state = __get_PRIMASK();
        __enable_irq();
    }
}

void STM32F4_SmartPtr_IRQ::Probe() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        STM32F4_Interrupt_GlobalWaitForInterrupt();
    }
}

bool STM32F4_SmartPtr_IRQ::GetState() {
    register uint32_t Cp = __get_PRIMASK();

    return (0 == (Cp & 1));
}

void STM32F4_SmartPtr_IRQ::Disable() {
    m_state = __get_PRIMASK();

    __disable_irq();
}

void STM32F4_SmartPtr_IRQ::Restore() {
    uint32_t Cp = m_state;

    if ((Cp & DISABLED_MASK) == 0) {
        __enable_irq();
    }
}

//////////////////////////////////////////////////////////////////////////////
//Global Interrupt - Use in System Cote
//////////////////////////////////////////////////////////////////////////////
bool STM32F4_Interrupt_GlobalIsDisabled() {
    return (__get_PRIMASK() & DISABLED_MASK) == DISABLED_MASK;
}

bool STM32F4_Interrupt_GlobalEnabled(bool force) {
    __enable_irq();

    return true;
}

bool STM32F4_Interrupt_GlobalDisabled(bool force) {
    bool wasDisable = STM32F4_Interrupt_GlobalIsDisabled();

    __disable_irq();

    return wasDisable;
}


void STM32F4_Interrupt_GlobalWaitForInterrupt() {
    register uint32_t state = __get_PRIMASK();

    __enable_irq();

    // just to allow an interupt to an occur
    __WFI();

    // restore irq state
    __set_PRIMASK(state);
}

void STM32F4_Interrupt_GlobalRestore() {
    __enable_irq();
}