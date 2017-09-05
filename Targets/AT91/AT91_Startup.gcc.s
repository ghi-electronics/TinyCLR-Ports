@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Copyright (c) Microsoft Corporation.  All rights reserved.
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


    .global  EntryPoint
    .global  PreStackInit_Exit_Pointer
    .global  PreStackInit
    .global  ARM_Vectors
    .global AT91_CPU_InvalidateTLBs_asm
    .global AT91_CPU_EnableMMU_asm
    .global AT91_CPU_DisableMMU_asm
    .global AT91_CPU_IsMMUEnabled_asm

    .global IRQ_LOCK_Release_asm
    .global IRQ_LOCK_Probe_asm
    .global IRQ_LOCK_GetState_asm
    .global IRQ_LOCK_ForceDisabled_asm
    .global IRQ_LOCK_ForceEnabled_asm
    .global IRQ_LOCK_Disable_asm
    .global IRQ_LOCK_Restore_asm
    .global IDelayLoop

    @ .extern  PreStackInit


    .extern AT91_Interrupt_UndefHandler               @ void AT91_Interrupt_UndefHandler  (unsigned int*, unsigned int, unsigned int)
    .extern AT91_Interrupt_AbortpHandler              @ void AT91_Interrupt_AbortpHandler (unsigned int*, unsigned int, unsigned int)
    .extern AT91_Interrupt_AbortdHandler              @ void AT91_Interrupt_AbortdHandler (unsigned int*, unsigned int, unsigned int)



    .extern  SystemInit
    .extern  main


@   PRESERVE8

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

PSR_MODE_USER       =     0xD0
PSR_MODE_FIQ        =     0xD1
PSR_MODE_IRQ        =     0xD2
PSR_MODE_SUPERVISOR =     0xD3
PSR_MODE_ABORT      =     0xD7
PSR_MODE_UNDEF      =     0xDB
PSR_MODE_SYSTEM     =     0xDF


STACK_MODE_ABORT    =     16
STACK_MODE_UNDEF    =     16
    .ifdef FIQ_SAMPLING_PROFILER
STACK_MODE_FIQ      =     2048
    .else
STACK_MODE_IRQ      =     2048
    .endif

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    .section SectionForStackBottom,       "a", %progbits
StackBottom:
    .word   0
    .section SectionForStackTop,          "a", %progbits
StackTop:
    .word   0
    .section SectionForHeapBegin,         "a", %progbits
HeapBegin:
    .word   0
    .section SectionForHeapEnd,           "a", %progbits
HeapEnd:
    .word   0


    .global StackBottom
    .global StackTop
    .global HeapBegin
    .global HeapEnd
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    .section    SectionForBootstrapOperations, "xa", %progbits


AT91_CPU_InvalidateTLBs_asm:
    mov		r0, #0
    mcr		p15, 0, r0, c8, c7, 0
    mrc     p15, 0, r1, c2, c0, 0
    nop

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

AT91_CPU_EnableMMU_asm:
    mcr     p15, 0, r0, c2, c0, 0		@ Set the TTB address location to CP15
    mrc     p15, 0, r1, c2, c0, 0
    nop
    mrc     p15, 0, r1, c1, c0, 0
    orr     r1, r1, #0x0001             @ Enable MMU
    mcr     p15, 0, r1, c1, c0, 0
    mrc     p15, 0, r1, c2, c0, 0
    nop

    @ Note that the 2 preceeding instruction would still be prefetched
    @ under the physical address space instead of in virtual address space.
    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

AT91_CPU_DisableMMU_asm:
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #0x0001           @ Disable MMU
    mcr     p15, 0, r0, c1, c0, 0
    mrc     p15, 0, r0, c2, c0, 0
    nop

    @ Note that the 2 preceeding instruction would still be prefetched
    @ under the physical address space instead of in virtual address space.
    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

AT91_CPU_IsMMUEnabled_asm:
    mrc     p15, 0, r0, c1, c0, 0
    mrc     p15, 0, r1, c2, c0, 0
    nop

    and		r0, r0, #1
    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    .section   SectionForFlashOperations,"xa", %progbits       @  void IDelayLoop(UINT32 count)

IDelayLoop:

IDelayLoop__Fi_b:

    subs    r0, r0, #4          @@ 1 cycle
    bgt     IDelayLoop__Fi_b    @@ 3 cycles, expect the last round, which is 1 cycle.

    mov     pc, lr              @@ 3 cycles, expect the last round, which is 1 cycle.

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

IRQ_LOCK_Release_asm:
    mrs     r0, CPSR
    bic     r1, r0, #0x80
    msr     CPSR_c, r1

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

IRQ_LOCK_Probe_asm:
    mrs		r0, CPSR
    bic		r1, r0, #0x80
    msr		CPSR_c, r1
    msr		CPSR_c, r0

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

IRQ_LOCK_GetState_asm:
    mrs		r0, CPSR
    mvn		r0, r0
    and		r0, r0, #0x80

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

IRQ_LOCK_ForceDisabled_asm:
    mrs		r0, CPSR
    orr		r1, r0, #0x80
    msr		CPSR_c, r1
    mvn		r0, r0
    and		r0, r0, #0x80

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

IRQ_LOCK_ForceEnabled_asm:
    mrs		r0, CPSR
    bic		r1, r0, #0x80
    msr		CPSR_c, r1
    mvn		r0, r0
    and		r0, r0, #0x80

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

IRQ_LOCK_Disable_asm:
    mrs		r0, CPSR
    orr		r1, r0, #0x80
    msr		CPSR_c, r1

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

IRQ_LOCK_Restore_asm:
    mrs		r0, CPSR
    bic		r0, r0, #0x80
    msr		CPSR_c, r0

    mov     pc, lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    .section SectionForBootstrapOperations, "xa", %progbits

PreStackInit:


    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    @
    @ TODO: Enter your pre stack initialization code here (if needed)
    @       e.g. SDRAM initialization if you don't have/use SRAM for the stack
    @

    @ << ADD CODE HERE >>

    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    @ DO NOT CHANGE THE FOLLOWING CODE! we can not use pop to return because we
    @ loaded the PC register to get here (since the stack has not been initialized).
    @ Make sure the PreStackInit_Exit_Pointer is within range and
    @ in the SectionForBootstrapOperations
    @ go back to the firstentry(_loader) code
    @


PreStackEnd:
    B     PreStackInit_Exit_Pointer


    @
    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    .section    .text.UNDEF_SubHandler, "xa", %progbits
    UNDEF_SubHandler:
@ on entry, were are in UNDEF mode, without a usable stack
    stmfd   r13!, {r0}                  @ store the r0 at undef stack first
    mrs     r0, spsr                    @ get the previous mode.
    orr     r0, r0, #0xC0               @ keep interrupts disabled.
    msr     cpsr_c, r0                  @ go back to the previous mode.

    stmfd   r13!, {r1-r12}              @ push unbanked registers on the stack
    msr     cpsr_c, #PSR_MODE_UNDEF     @ go back into UNDEF mode, but keep IRQs off
    mov     r3,r0
    mrs     r0, spsr                    @ now save the spsr_UNDEF register
    mov     r1, r14                     @ now save the r14_UNDEF  register
    LDMFD   r13!,{r2}                   @ r2 <= previous r0

    msr     cpsr_c, r3                  @ go back to the previous mode.
    stmfd   r13!, {r0-r2}               @ push spsr_UNDEF and r14_UNDEF on stack and the old r0 value

    mov     r0, r13                     @ ARG1 of handler: the stack location of registers
    add     r1, r13, #60                @ ARG2 of handler: SYSTEM mode stack at time of exception (without saved registers = 15*4 back in stack)
    mov     r2, r14                     @ ARG3 of handler: get the link register of SYSTEM mode: r14_SYSTEM

    ldr     pc,UNDEF_Handler_Ptr        @ address of vector routine in C to jump to, never expect to return

UNDEF_Handler_Ptr:
    .word   AT91_Interrupt_UndefHandler

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

@ the ABORT conditions on the ARM7TDMI are catastrophic, so there is no return, but it is supported in non-RTM builds

    .section    .text.ABORTP_SubHandler, "xa", %progbits

  .ifdef COMPILE_THUMB
    .arm
  .endif

ABORTP_SubHandler:
    @ on entry, were are in ABORT mode, without a usable stack

    stmfd   r13!, {r0}                  @ store the r0 at undef stack first
    mrs     r0, spsr                    @ get the previous mode.
    orr     r0, r0, #0xC0               @ keep interrupts disabled.
    msr     cpsr_c, r0                  @ go back to the previous mode.

    stmfd   r13!, {r1-r12}              @ push unbanked registers on the stack
    msr     cpsr_c, #PSR_MODE_ABORT     @ go back into ABORT mode, but keep IRQs off
    mov     r3,r0
    mrs     r0, spsr                    @ now save the spsr_ABORT register
    mov     r1, r14                     @ now save the r14_ABORT  register
    LDMFD   r13!,{r2}                   @ r2 <= previous r0

    msr     cpsr_c, r3                  @ go back to the previous mode.
    stmfd   r13!, {r0-r2}               @ push spsr_ABORT and r14_ABORT on stack and the old r0 value

    mov     r0, r13                     @ ARG1 of handler: the stack location of registers
    add     r1, r13, #60                @ ARG2 of handler: SYSTEM mode stack at time of exception (without saved registers = 15*4 back in stack)
    mov     r2, r14                     @ ARG3 of handler: get the link register of SYSTEM mode: r14_SYSTEM

    ldr     pc,ABORTP_Handler_Ptr       @ address of vector routine in C to jump to, never expect to return

ABORTP_Handler_Ptr:
    .word   AT91_Interrupt_AbortpHandler

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    .section    .text.ABORTD_SubHandler, "xa", %progbits

  .ifdef COMPILE_THUMB
    .arm
  .endif

ABORTD_SubHandler:
    @ on entry, were are in ABORT mode, without a usable stack

    stmfd   r13!, {r0}                  @ store the r0 at undef stack first
    mrs     r0, spsr                    @ get the previous mode.
    orr     r0, r0, #0xC0               @ keep interrupts disabled.
    msr     cpsr_c, r0                  @ go back to the previous mode.

    stmfd   r13!, {r1-r12}              @ push unbanked registers on the stack
    msr     cpsr_c, #PSR_MODE_ABORT     @ go back into ABORT mode, but keep IRQs off
    mov     r3,r0
    mrs     r0, spsr                    @ now save the spsr_ABORT register
    mov     r1, r14                     @ now save the r14_ABORT  register
    LDMFD   r13!,{r2}                   @ r2 <= previous r0

    msr     cpsr_c, r3                  @ go back to the previous mode.
    stmfd   r13!, {r0-r2}               @ push spsr_ABORT and r14_ABORT on stack and the old r0 value

    mov     r0, r13                     @ ARG1 of handler: the stack location of registers
    add     r1, r13, #60                @ ARG2 of handler: SYSTEM mode stack at time of exception (without saved registers = 15*4 back in stack)
    mov     r2, r14                     @ ARG3 of handler: get the link register of SYSTEM mode: r14_SYSTEM

    ldr     pc,ABORTD_Handler_Ptr       @ address of vector routine in C to jump to, never expect to return

ABORTD_Handler_Ptr:
    .word   AT91_Interrupt_AbortdHandler


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

   .section VectorsTrampolines, "xa", %progbits

    .ifdef COMPILE_THUMB
        .arm
    .endif

ARM_Vectors:

    @ RESET
RESET_VECTOR:
    b       UNDEF_VECTOR

    @ UNDEF INSTR
UNDEF_VECTOR:
    ldr     pc, UNDEF_SubHandler_Trampoline

    @ SWI
SWI_VECTOR:
    .word   0xbaadf00d

    @ PREFETCH ABORT
PREFETCH_VECTOR:
    ldr     pc, ABORTP_SubHandler_Trampoline

    @ DATA ABORT
DATA_VECTOR:
    ldr     pc, ABORTD_SubHandler_Trampoline

    @ unused
USED_VECTOR:
    .word   0xbaadf00d

    @ IRQ
IRQ_VECTOR:
    ldr     pc, IRQ_SubHandler_Trampoline

    @ FIQ
    @ we place the FIQ handler where it was designed to go, immediately at the end of the vector table
    @ this saves an additional 3+ clock cycle branch to the handler
FIQ_Handler:
    .ifdef FIQ_SAMPLING_PROFILER
    ldr     pc,FIQ_SubHandler_Trampoline

FIQ_SubHandler_Trampoline:
    .word   FIQ_SubHandler
    .endif

UNDEF_SubHandler_Trampoline:
    .word   UNDEF_SubHandler

ABORTP_SubHandler_Trampoline:
    .word   ABORTP_SubHandler

ABORTD_SubHandler_Trampoline:
    .word   ABORTD_SubHandler


        @ route the normal interupt handler to the proper lowest level driver
IRQ_SubHandler_Trampoline:
    .word  	IRQ_Handler

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


    .section i.EntryPoint, "xa", %progbits

    .arm

EntryPoint:


    .ifdef  HAL_REDUCESIZE
    .ifndef TARGETLOCATION_RAM
    @ -----------------------------------------------
    @ ADD BOOT MARKER HERE IF YOU NEED ONE
    @ -----------------------------------------------
    .ifdef  PLATFORM_ARM_LPC22XX
     orr     r0, pc,#0x80000000
     mov     pc, r0
    .endif

    .endif
    .endif


    @ designed to be a vector area for ARM7
    @ RESET
    @ keep PortBooter signature the same

    msr     cpsr_c, #PSR_MODE_SYSTEM    @ go into System mode, interrupts off

    @--------------------------------------------------------------------------------
    @ ALLOW pre-stack initilization
    @ Use the relative address of PreStackInit (because memory may need to be remapped)
    @--------------------------------------------------------------------------------


    .ifdef PLATFORM_ARM_AT91XX
    @ AT91XX on chip bootloader requires valid checksum in internal Flash
    @ location 0x14 ( ARM reserved vector ).
    B       PreStackEntry
    .space 20
    .endif


PreStackEntry:
    B       PreStackInit


PreStackInit_Exit_Pointer:

    ldr     r0, =StackTop               @ new SYS stack pointer for a full decrementing stack

    msr     cpsr_c, #PSR_MODE_ABORT     @ go into ABORT mode, interrupts off
    mov     sp, r0                      @ stack top
    sub     r0, r0, #STACK_MODE_ABORT   @ ( take the size of abort stack off )

    msr     cpsr_c, #PSR_MODE_UNDEF     @ go into UNDEF mode, interrupts off
    mov     sp, r0                      @ stack top - abort stack
    sub     r0, r0, #STACK_MODE_UNDEF   @


    .ifdef FIQ_SAMPLING_PROFILER
    msr     cpsr_c, #PSR_MODE_FIQ       @ go into FIQ mode, interrupts off
    mov     sp, r0                      @ stack top - abort stack - undef stack
    sub     r0, r0, #STACK_MODE_FIQ
    .endif

    msr     cpsr_c, #PSR_MODE_IRQ       @ go into IRQ mode, interrupts off
    mov     sp, r0                      @ stack top - abort stack - undef stack (- FIQ stack)
    sub     r0, r0, #STACK_MODE_IRQ

    msr     cpsr_c, #PSR_MODE_SYSTEM    @ go into System mode, interrupts off
    mov     sp,r0                       @ stack top - abort stack - undef stack (- FIQ stack) - IRQ stack


        @******************************************************************************************
        @ This ensures that we execute from the real location, regardless of any remapping scheme *
        @******************************************************************************************

    ldr     pc, EntryPoint_Restart_Pointer
EntryPoint_Restart_Pointer:
    .word   EntryPoint_Restart
EntryPoint_Restart:

        @*********************************************************************

    bl      SystemInit

    ldr     r0, =StackTop               @ new svc stack pointer for a full decrementing stack

    .ifdef FIQ_SAMPLING_PROFILER
    sub     sp, r0, #STACK_MODE_FIQ     @
    .else
    sub     sp, r0, #STACK_MODE_IRQ     @
    .endif


  .ifdef COMPILE_THUMB
        ldr     r0,BootEntryPointer
        bx      r0
   .else
        ldr     pc,BootEntryPointer
    .endif
BootEntryPointer:
        .word   main


  .ifdef COMPILE_THUMB
    .thumb
  .endif

    .end

