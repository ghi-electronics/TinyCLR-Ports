// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
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

#define DISABLED_MASK  0x00000001

#define TOTAL_INTERRUPT_CONTROLLERS 1

TinyCLR_Interrupt_StartStopHandler STM32F4_Interrupt_Started;
TinyCLR_Interrupt_StartStopHandler STM32F4_Interrupt_Ended;

static TinyCLR_Interrupt_Controller interruptControllers[TOTAL_INTERRUPT_CONTROLLERS];
static TinyCLR_Api_Info interruptApi[TOTAL_INTERRUPT_CONTROLLERS];

void STM32F4_Interrupt_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_INTERRUPT_CONTROLLERS; i++) {
        interruptControllers[i].ApiInfo = &interruptApi[i];
        interruptControllers[i].Initialize = &STM32F4_Interrupt_Initialize;
        interruptControllers[i].Uninitialize = &STM32F4_Interrupt_Uninitialize;
        interruptControllers[i].Enable = &STM32F4_Interrupt_Enable;
        interruptControllers[i].Disable = &STM32F4_Interrupt_Disable;
        interruptControllers[i].WaitForInterrupt = &STM32F4_Interrupt_WaitForInterrupt;
        interruptControllers[i].IsDisabled = &STM32F4_Interrupt_IsDisabled;
        interruptControllers[i].Restore = &STM32F4_Interrupt_Restore;

        interruptApi[i].Author = "GHI Electronics, LLC";
        interruptApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.InterruptController";
        interruptApi[i].Type = TinyCLR_Api_Type::InterruptController;
        interruptApi[i].Version = 0;
        interruptApi[i].Implementation = &interruptControllers[i];
        interruptApi[i].State = nullptr;
    }

    
}

extern "C" {
    void SysTick_Handler(void *param);
    extern uint32_t __Vectors;
}

TinyCLR_Result STM32F4_Interrupt_Initialize(const TinyCLR_Interrupt_Controller* self, TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd) {
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

    __DMB(); // ensure table is written

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

TinyCLR_Result STM32F4_Interrupt_Uninitialize(const TinyCLR_Interrupt_Controller* self) {
    return TinyCLR_Result::Success;
}

bool STM32F4_InterruptInternal_Activate(uint32_t index, uint32_t *isr, void* isrParam) {
    int id = (int)index;

    uint32_t *irq_vectors = (uint32_t*)&__Vectors;

    irq_vectors[id + 16] = (uint32_t)isr; // exception = irq + 16

    __DMB(); // ensure table is written

    NVIC->ICPR[id >> 5] = 1 << (id & 0x1F); // clear pending bit
    NVIC->ISER[id >> 5] = 1 << (id & 0x1F); // set enable bit

    return true;
}

bool STM32F4_InterruptInternal_Deactivate(uint32_t index) {
    int id = (int)index;

    NVIC->ICER[id >> 5] = 1 << (id & 0x1F); // clear enable bit */

    return true;
}
STM32F4_InterruptStarted_RaiiHelper::STM32F4_InterruptStarted_RaiiHelper() { STM32F4_Interrupt_Started(); };
STM32F4_InterruptStarted_RaiiHelper::~STM32F4_InterruptStarted_RaiiHelper() { STM32F4_Interrupt_Ended(); };

STM32F4_DisableInterrupts_RaiiHelper::STM32F4_DisableInterrupts_RaiiHelper() {
    state = __get_PRIMASK();

    __disable_irq();
}
STM32F4_DisableInterrupts_RaiiHelper::~STM32F4_DisableInterrupts_RaiiHelper() {
    uint32_t Cp = state;

    if ((Cp & DISABLED_MASK) == 0) {
        __enable_irq();
    }
}

bool STM32F4_DisableInterrupts_RaiiHelper::IsDisabled() {
    return (state & DISABLED_MASK) == DISABLED_MASK;
}

void STM32F4_DisableInterrupts_RaiiHelper::Acquire() {
    uint32_t Cp = state;

    if ((Cp & DISABLED_MASK) == DISABLED_MASK) {
        state = __get_PRIMASK();

        __disable_irq();
    }
}

void STM32F4_DisableInterrupts_RaiiHelper::Release() {
    uint32_t Cp = state;

    if ((Cp & DISABLED_MASK) == 0) {
        state = __get_PRIMASK();
        __enable_irq();
    }
}

//////////////////////////////////////////////////////////////////////////////
//Global Interrupt - Use in System Cote
//////////////////////////////////////////////////////////////////////////////
bool STM32F4_Interrupt_IsDisabled() {
    return (__get_PRIMASK() & DISABLED_MASK) == DISABLED_MASK;
}

bool STM32F4_Interrupt_Enable(bool force) {
    __enable_irq();

    return true;
}

bool STM32F4_Interrupt_Disable(bool force) {
    bool wasDisable = STM32F4_Interrupt_IsDisabled();

    __disable_irq();

    return wasDisable;
}


void STM32F4_Interrupt_WaitForInterrupt() {
    register uint32_t state = __get_PRIMASK();

    __enable_irq();

    // just to allow an interupt to an occur
    __WFI();

    // restore irq state
    __set_PRIMASK(state);
}

void STM32F4_Interrupt_Restore() {
    __enable_irq();
}