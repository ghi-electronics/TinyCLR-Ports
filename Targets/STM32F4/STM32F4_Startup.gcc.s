@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@
@  Copyright Microsoft Corporation
@  Implementation for STM32: Copyright Oberon microsystems, Inc
@  Copyright 2017 GHI Electronics, LLC
@
@  Licensed under the Apache License, Version 2.0 (the "License");
@  you may not use this file except in compliance with the License.
@  You may obtain a copy of the License at
@
@      http://www.apache.org/licenses/LICENSE-2.0
@
@  Unless required by applicable law or agreed to in writing, software
@  distributed under the License is distributed on an "AS IS" BASIS,
@  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@  See the License for the specific language governing permissions and
@  limitations under the License.
@
@  CORTEX-Mx Standard Entry Code
@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@
@ This version of the "boot entry" support is used when the application is
@ loaded or otherwise started from a bootloader. (e.g. this application isn't
@ a boot loader). More specifically this version is used whenever the application
@ does NOT run from the power on reset vector because some other code is already
@ there.
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@
    .syntax unified
    .arch armv7-m
    .thumb

    .global  EntryPoint

    .global StackBottom
    .global StackTop
    .global HeapBegin
    .global HeapEnd
    .global __initial_sp
    .global Reset_Handler

    .extern main
    .extern SystemInit


    @*************************************************************************

    .section SectionForStackBottom, "w", %nobits
StackBottom:
       .word 0

    .section SectionForStackTop, "w", %nobits
__initial_sp:
StackTop:
      .word 0

    .section SectionForHeapBegin, "w", %nobits
HeapBegin:
     .word 0

    .section SectionForHeapEnd, "w", %nobits
HeapEnd:
    .word 0

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Dummy Exception Handlers (infinite loops which can be overloaded since they are exported with weak linkage )

    .section SectionForBootstrapOperations, "ax", %progbits

    .align 1
    .thumb_func
    .weak Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b  .
    .size Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_irq_handler    handler_name
    .weak    \handler_name
    .set    \handler_name, Default_Handler
    .endm

    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler

@ SOC Specific default handlers
    def_irq_handler    WWDG_IRQHandler
    def_irq_handler    PVD_IRQHandler
    def_irq_handler    TAMP_STAMP_IRQHandler
    def_irq_handler    RTC_WKUP_IRQHandler
    def_irq_handler    FLASH_IRQHandler
    def_irq_handler    RCC_IRQHandler
    def_irq_handler    EXTI0_IRQHandler
    def_irq_handler    EXTI1_IRQHandler
    def_irq_handler    EXTI2_IRQHandler
    def_irq_handler    EXTI3_IRQHandler
    def_irq_handler    EXTI4_IRQHandler
    def_irq_handler    DMA1_Stream0_IRQHandler
    def_irq_handler    DMA1_Stream1_IRQHandler
    def_irq_handler    DMA1_Stream2_IRQHandler
    def_irq_handler    DMA1_Stream3_IRQHandler
    def_irq_handler    DMA1_Stream4_IRQHandler
    def_irq_handler    DMA1_Stream5_IRQHandler
    def_irq_handler    DMA1_Stream6_IRQHandler
    def_irq_handler    ADC_IRQHandler
    def_irq_handler    CAN1_TX_IRQHandler
    def_irq_handler    CAN1_RX0_IRQHandler
    def_irq_handler    CAN1_RX1_IRQHandler
    def_irq_handler    CAN1_SCE_IRQHandler
    def_irq_handler    EXTI9_5_IRQHandler
    def_irq_handler    TIM1_BRK_TIM9_IRQHandler
    def_irq_handler    TIM1_UP_TIM10_IRQHandler
    def_irq_handler    TIM1_TRG_COM_TIM11_IRQHandler
    def_irq_handler    TIM1_CC_IRQHandler
    def_irq_handler    TIM2_IRQHandler
    def_irq_handler    TIM3_IRQHandler
    def_irq_handler    TIM4_IRQHandler
    def_irq_handler    I2C1_EV_IRQHandler
    def_irq_handler    I2C1_ER_IRQHandler
    def_irq_handler    I2C2_EV_IRQHandler
    def_irq_handler    I2C2_ER_IRQHandler
    def_irq_handler    SPI1_IRQHandler
    def_irq_handler    SPI2_IRQHandler
    def_irq_handler    USART1_IRQHandler
    def_irq_handler    USART2_IRQHandler
    def_irq_handler    USART3_IRQHandler
    def_irq_handler    EXTI15_10_IRQHandler
    def_irq_handler    RTC_Alarm_IRQHandler
    def_irq_handler    OTG_FS_WKUP_IRQHandler
    def_irq_handler    TIM8_BRK_TIM12_IRQHandler
    def_irq_handler    TIM8_UP_TIM13_IRQHandler
    def_irq_handler    TIM8_TRG_COM_TIM14_IRQHandler
    def_irq_handler    TIM8_CC_IRQHandler
    def_irq_handler    DMA1_Stream7_IRQHandler
    def_irq_handler    FMC_IRQHandler
    def_irq_handler    SDIO_IRQHandler
    def_irq_handler    TIM5_IRQHandler
    def_irq_handler    SPI3_IRQHandler
    def_irq_handler    UART4_IRQHandler
    def_irq_handler    UART5_IRQHandler
    def_irq_handler    TIM6_DAC_IRQHandler
    def_irq_handler    TIM7_IRQHandler
    def_irq_handler    DMA2_Stream0_IRQHandler
    def_irq_handler    DMA2_Stream1_IRQHandler
    def_irq_handler    DMA2_Stream2_IRQHandler
    def_irq_handler    DMA2_Stream3_IRQHandler
    def_irq_handler    DMA2_Stream4_IRQHandler
    def_irq_handler    ETH_IRQHandler
    def_irq_handler    ETH_WKUP_IRQHandler
    def_irq_handler    CAN2_TX_IRQHandler
    def_irq_handler    CAN2_RX0_IRQHandler
    def_irq_handler    CAN2_RX1_IRQHandler
    def_irq_handler    CAN2_SCE_IRQHandler
    def_irq_handler    OTG_FS_IRQHandler
    def_irq_handler    DMA2_Stream5_IRQHandler
    def_irq_handler    DMA2_Stream6_IRQHandler
    def_irq_handler    DMA2_Stream7_IRQHandler
    def_irq_handler    USART6_IRQHandler
    def_irq_handler    I2C3_EV_IRQHandler
    def_irq_handler    I2C3_ER_IRQHandler
    def_irq_handler    OTG_HS_EP1_OUT_IRQHandler
    def_irq_handler    OTG_HS_EP1_IN_IRQHandler
    def_irq_handler    OTG_HS_WKUP_IRQHandler
    def_irq_handler    OTG_HS_IRQHandler
    def_irq_handler    DCMI_IRQHandler
    def_irq_handler    HASH_RNG_IRQHandler
    def_irq_handler    FPU_IRQHandler

    .section i.EntryPoint, "ax", %progbits

   @ENTRY

EntryPoint:

@ The first word has a dual role:
@ - It is the entry point of the application loaded or discovered by
@   the bootloader and therfore must be valid executable code
@ - it contains a signature word used to identify application blocks
@   in TinyBooter (see: Tinybooter_ProgramWordCheck() for more details )
@ * The additional entries in this table are completely ignored and
@   remain for backwards compatibility. Since the boot loader is hard
@   coded to look for the signature, half of which is an actual relative
@   branch instruction, removing the unused entries would require all
@   bootloaders to be updated as well. [sic.]
@   [ NOTE:
@     In the next major release where we can afford to break backwards
@     compatibility this will almost certainly change, as the whole
@     init/startup for NETMF is overly complex. The various tools used
@     for building the CLR have all come around to supporting simpler
@     init sequences we should leverage directly
@   ]
    .word     __initial_sp                      @ Top of Stack
    .word     Reset_Handler+1                   @ Reset Handler
    .word     NMI_Handler                       @ NMI Handler
    .word     HardFault_Handler                 @ Hard Fault Handler
    .word     MemManage_Handler                 @ MPU Fault Handler
    .word     BusFault_Handler                  @ Bus Fault Handler
    .word     UsageFault_Handler                @ Usage Fault Handler
    .word     0                                 @ Reserved
    .word     0                                 @ Reserved
    .word     0                                 @ Reserved
    .word     0                                 @ Reserved
    .word     SVC_Handler                       @ SVCall Handler
    .word     DebugMon_Handler                  @ Debug Monitor Handler
    .word     0                                 @ Reserved
    .word     PendSV_Handler                    @ PendSV Handler
    .word     SysTick_Handler                   @ SysTick Handler

@ External Interrupts
@ The remaining entries all SOC specific
@ NOTE: Each SOC has a fixed bumber of interrrupt
@ sources so the actual number of entries is not architecturally
@ defined for all systems (but there is a MAX defined)
    .word  WWDG_IRQHandler
    .word  PVD_IRQHandler
    .word  TAMP_STAMP_IRQHandler
    .word  RTC_WKUP_IRQHandler
    .word  FLASH_IRQHandler
    .word  RCC_IRQHandler
    .word  EXTI0_IRQHandler
    .word  EXTI1_IRQHandler
    .word  EXTI2_IRQHandler
    .word  EXTI3_IRQHandler
    .word  EXTI4_IRQHandler
    .word  DMA1_Stream0_IRQHandler
    .word  DMA1_Stream1_IRQHandler
    .word  DMA1_Stream2_IRQHandler
    .word  DMA1_Stream3_IRQHandler
    .word  DMA1_Stream4_IRQHandler
    .word  DMA1_Stream5_IRQHandler
    .word  DMA1_Stream6_IRQHandler
    .word  ADC_IRQHandler
    .word  CAN1_TX_IRQHandler
    .word  CAN1_RX0_IRQHandler
    .word  CAN1_RX1_IRQHandler
    .word  CAN1_SCE_IRQHandler
    .word  EXTI9_5_IRQHandler
    .word  TIM1_BRK_TIM9_IRQHandler
    .word  TIM1_UP_TIM10_IRQHandler
    .word  TIM1_TRG_COM_TIM11_IRQHandler
    .word  TIM1_CC_IRQHandler
    .word  TIM2_IRQHandler
    .word  TIM3_IRQHandler
    .word  TIM4_IRQHandler
    .word  I2C1_EV_IRQHandler
    .word  I2C1_ER_IRQHandler
    .word  I2C2_EV_IRQHandler
    .word  I2C2_ER_IRQHandler
    .word  SPI1_IRQHandler
    .word  SPI2_IRQHandler
    .word  USART1_IRQHandler
    .word  USART2_IRQHandler
    .word  USART3_IRQHandler
    .word  EXTI15_10_IRQHandler
    .word  RTC_Alarm_IRQHandler
    .word  OTG_FS_WKUP_IRQHandler
    .word  TIM8_BRK_TIM12_IRQHandler
    .word  TIM8_UP_TIM13_IRQHandler
    .word  TIM8_TRG_COM_TIM14_IRQHandler
    .word  TIM8_CC_IRQHandler
    .word  DMA1_Stream7_IRQHandler
    .word  FMC_IRQHandler
    .word  SDIO_IRQHandler
    .word  TIM5_IRQHandler
    .word  SPI3_IRQHandler
    .word  UART4_IRQHandler
    .word  UART5_IRQHandler
    .word  TIM6_DAC_IRQHandler
    .word  TIM7_IRQHandler
    .word  DMA2_Stream0_IRQHandler
    .word  DMA2_Stream1_IRQHandler
    .word  DMA2_Stream2_IRQHandler
    .word  DMA2_Stream3_IRQHandler
    .word  DMA2_Stream4_IRQHandler
    .word  ETH_IRQHandler
    .word  ETH_WKUP_IRQHandler
    .word  CAN2_TX_IRQHandler
    .word  CAN2_RX0_IRQHandler
    .word  CAN2_RX1_IRQHandler
    .word  CAN2_SCE_IRQHandler
    .word  OTG_FS_IRQHandler
    .word  DMA2_Stream5_IRQHandler
    .word  DMA2_Stream6_IRQHandler
    .word  DMA2_Stream7_IRQHandler
    .word  USART6_IRQHandler
    .word  I2C3_EV_IRQHandler
    .word  I2C3_ER_IRQHandler
    .word  OTG_HS_EP1_OUT_IRQHandler
    .word  OTG_HS_EP1_IN_IRQHandler
    .word  OTG_HS_WKUP_IRQHandler
    .word  OTG_HS_IRQHandler
    .word  DCMI_IRQHandler
    .word  HASH_RNG_IRQHandler
    .word  FPU_IRQHandler

Reset_Handler:
    @@ reload the stack pointer as there's no returning to the loader
    ldr     sp, =__initial_sp
    bl  SystemInit
    b   main

    .pool
    .size    Reset_Handler, . - Reset_Handler

    .balign   4

    .global  __Vectors

@ Vector Table For the application
@
@ bootloaders place this at offset 0 and the hardware uses
@ it from there at power on reset. Applications (or the boot
@  loader itself) can place a copy in RAM to allow dynamically
@ "hooking" interrupts at run-time
@
@ It is expected ,though not required, that the .externed handlers
@ have a default empty implementation declared with WEAK linkage
@ thus allowing applications to override the default by simply
@ defining a function with the same name and proper behavior
@ [ NOTE:
@   This standardized handler naming is an essential part of the
@   CMSIS-Core specification. It is relied upon by the CMSIS-RTX
@   implementation as well as much of the mbed framework.
@ ]
    .section VectorTable
    .align 9

@ The first 16 entries are all architecturally defined by ARM
__Vectors:
    .long     __initial_sp                      @ Top of Stack
    .long     Reset_Handler                     @ Reset Handler
    .long     NMI_Handler                       @ NMI Handler
    .long     HardFault_Handler                 @ Hard Fault Handler
    .long     MemManage_Handler                 @ MPU Fault Handler
    .long     BusFault_Handler                  @ Bus Fault Handler
    .long     UsageFault_Handler                @ Usage Fault Handler
    .long     0                                 @ Reserved
    .long     0                                 @ Reserved
    .long     0                                 @ Reserved
    .long     0                                 @ Reserved
    .long     SVC_Handler                       @ SVCall Handler
    .long     DebugMon_Handler                  @ Debug Monitor Handler
    .long     0                                 @ Reserved
    .long     PendSV_Handler                    @ PendSV Handler
    .long     SysTick_Handler                   @ SysTick Handler

@ External Interrupts
@ The remaining entries all SOC specific
@ NOTE: Each SOC has a fixed bumber of interrrupt
@ sources so the actual number of entries is not architecturally
@ defined for all systems (but there is a MAX defined)
    .long     WWDG_IRQHandler                   @ Window WatchDog
    .long     PVD_IRQHandler                    @ PVD through EXTI Line detection
    .long     TAMP_STAMP_IRQHandler             @ Tamper and TimeStamps through the EXTI line
    .long     RTC_WKUP_IRQHandler               @ RTC Wakeup through the EXTI line
    .long     FLASH_IRQHandler                  @ FLASH
    .long     RCC_IRQHandler                    @ RCC
    .long     EXTI0_IRQHandler                  @ EXTI Line0
    .long     EXTI1_IRQHandler                  @ EXTI Line1
    .long     EXTI2_IRQHandler                  @ EXTI Line2
    .long     EXTI3_IRQHandler                  @ EXTI Line3
    .long     EXTI4_IRQHandler                  @ EXTI Line4
    .long     DMA1_Stream0_IRQHandler           @ DMA1 Stream 0
    .long     DMA1_Stream1_IRQHandler           @ DMA1 Stream 1
    .long     DMA1_Stream2_IRQHandler           @ DMA1 Stream 2
    .long     DMA1_Stream3_IRQHandler           @ DMA1 Stream 3
    .long     DMA1_Stream4_IRQHandler           @ DMA1 Stream 4
    .long     DMA1_Stream5_IRQHandler           @ DMA1 Stream 5
    .long     DMA1_Stream6_IRQHandler           @ DMA1 Stream 6
    .long     ADC_IRQHandler                    @ ADC1, ADC2 and ADC3s
    .long     CAN1_TX_IRQHandler                @ CAN1 TX
    .long     CAN1_RX0_IRQHandler               @ CAN1 RX0
    .long     CAN1_RX1_IRQHandler               @ CAN1 RX1
    .long     CAN1_SCE_IRQHandler               @ CAN1 SCE
    .long     EXTI9_5_IRQHandler                @ External Line[9:5]s
    .long     TIM1_BRK_TIM9_IRQHandler          @ TIM1 Break and TIM9
    .long     TIM1_UP_TIM10_IRQHandler          @ TIM1 Update and TIM10
    .long     TIM1_TRG_COM_TIM11_IRQHandler     @ TIM1 Trigger and Commutation and TIM11
    .long     TIM1_CC_IRQHandler                @ TIM1 Capture Compare
    .long     TIM2_IRQHandler                   @ TIM2
    .long     TIM3_IRQHandler                   @ TIM3
    .long     TIM4_IRQHandler                   @ TIM4
    .long     I2C1_EV_IRQHandler                @ I2C1 Event
    .long     I2C1_ER_IRQHandler                @ I2C1 Error
    .long     I2C2_EV_IRQHandler                @ I2C2 Event
    .long     I2C2_ER_IRQHandler                @ I2C2 Error
    .long     SPI1_IRQHandler                   @ SPI1
    .long     SPI2_IRQHandler                   @ SPI2
    .long     USART1_IRQHandler                 @ USART1
    .long     USART2_IRQHandler                 @ USART2
    .long     USART3_IRQHandler                 @ USART3
    .long     EXTI15_10_IRQHandler              @ External Line[15:10]s
    .long     RTC_Alarm_IRQHandler              @ RTC Alarm (A and B) through EXTI Line
    .long     OTG_FS_WKUP_IRQHandler            @ USB OTG FS Wakeup through EXTI line
    .long     TIM8_BRK_TIM12_IRQHandler         @ TIM8 Break and TIM12
    .long     TIM8_UP_TIM13_IRQHandler          @ TIM8 Update and TIM13
    .long     TIM8_TRG_COM_TIM14_IRQHandler     @ TIM8 Trigger and Commutation and TIM14
    .long     TIM8_CC_IRQHandler                @ TIM8 Capture Compare
    .long     DMA1_Stream7_IRQHandler           @ DMA1 Stream7
    .long     FMC_IRQHandler                    @ FMC
    .long     SDIO_IRQHandler                   @ SDIO
    .long     TIM5_IRQHandler                   @ TIM5
    .long     SPI3_IRQHandler                   @ SPI3
    .long     UART4_IRQHandler                  @ UART4
    .long     UART5_IRQHandler                  @ UART5
    .long     TIM6_DAC_IRQHandler               @ TIM6 and DAC1&2 underrun errors
    .long     TIM7_IRQHandler                   @ TIM7
    .long     DMA2_Stream0_IRQHandler           @ DMA2 Stream 0
    .long     DMA2_Stream1_IRQHandler           @ DMA2 Stream 1
    .long     DMA2_Stream2_IRQHandler           @ DMA2 Stream 2
    .long     DMA2_Stream3_IRQHandler           @ DMA2 Stream 3
    .long     DMA2_Stream4_IRQHandler           @ DMA2 Stream 4
    .long     ETH_IRQHandler                    @ Ethernet
    .long     ETH_WKUP_IRQHandler               @ Ethernet Wakeup through EXTI line
    .long     CAN2_TX_IRQHandler                @ CAN2 TX
    .long     CAN2_RX0_IRQHandler               @ CAN2 RX0
    .long     CAN2_RX1_IRQHandler               @ CAN2 RX1
    .long     CAN2_SCE_IRQHandler               @ CAN2 SCE
    .long     OTG_FS_IRQHandler                 @ USB OTG FS
    .long     DMA2_Stream5_IRQHandler           @ DMA2 Stream 5
    .long     DMA2_Stream6_IRQHandler           @ DMA2 Stream 6
    .long     DMA2_Stream7_IRQHandler           @ DMA2 Stream 7
    .long     USART6_IRQHandler                 @ USART6
    .long     I2C3_EV_IRQHandler                @ I2C3 event
    .long     I2C3_ER_IRQHandler                @ I2C3 error
    .long     OTG_HS_EP1_OUT_IRQHandler         @ USB OTG HS End Point 1 Out
    .long     OTG_HS_EP1_IN_IRQHandler          @ USB OTG HS End Point 1 In
    .long     OTG_HS_WKUP_IRQHandler            @ USB OTG HS Wakeup through EXTI
    .long     OTG_HS_IRQHandler                 @ USB OTG HS
    .long     DCMI_IRQHandler                   @ DCMI
    .long     0                                 @ Reserved
    .long     HASH_RNG_IRQHandler               @ Hash and Rng
    .long     FPU_IRQHandler                    @ FPU

.end
