@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@
@  Copyright Microsoft Corporation
@  Copyright Oberon microsystems, Inc
@  Copyright GHI Electronics, LLC
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

    .section SectionForHeapBegin, "aw", %nobits
HeapBegin:
     .word 0

    .section SectionForHeapEnd, "aw", %nobits
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
.end
