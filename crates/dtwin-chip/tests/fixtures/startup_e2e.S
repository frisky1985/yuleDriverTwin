/*=============================================================================
 * startup_e2e.S -- dtwin A9 压力固件启动代码（Cortex-M4 / ARMv7E-M）
 *
 * 源自 yuleASR QEMU 固件 startup.S（该仓库为只读参考），按 M4 配置：
 *  - 向量表在 0x0（Flash 起始）
 *  - 首字 = 初始 SP（_estack），次字 = Reset_Handler
 *  - Reset_Handler：拷贝 .data → SRAM、清零 .bss、调 SystemInit + main
 *  - main 返回后空转（dtwin 侧将执行至指令上限 LimitReached）
 *
 * 注意：不写 CPACR（dtwin 引擎默认 CPACR=0xF00000 FPU 已使能；
 *       dtwin 内存模型不映射 0xE000ED88，写它会触发 BusFault）。
 *============================================================================*/

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global _start
.global Reset_Handler
.global Default_Handler
.global HardFault_Handler
.global MemManage_Handler
.global BusFault_Handler
.global UsageFault_Handler
.global SVC_Handler
.global PendSV_Handler
.global SysTick_Handler

/* Weak default handlers -- override in application code */
.weak   HardFault_Handler
.weak   MemManage_Handler
.weak   BusFault_Handler
.weak   UsageFault_Handler
.weak   SVC_Handler
.weak   PendSV_Handler
.weak   SysTick_Handler

/*=====================================================================
 * Vector Table
 *====================================================================*/
.section .vector_table, "a", %progbits
.type _start, %object
_start:
    .word _estack                       /* 0: Initial SP */
    .word Reset_Handler                 /* 1: Reset */
    .word NMI_Handler                   /* 2: NMI */
    .word HardFault_Handler             /* 3: HardFault */
    .word MemManage_Handler             /* 4: MemManage */
    .word BusFault_Handler              /* 5: BusFault */
    .word UsageFault_Handler            /* 6: UsageFault */
    .word 0                             /* 7: Reserved */
    .word 0                             /* 8: Reserved */
    .word 0                             /* 9: Reserved */
    .word 0                             /* 10: Reserved */
    .word SVC_Handler                   /* 11: SVCall */
    .word DebugMon_Handler              /* 12: DebugMonitor */
    .word 0                             /* 13: Reserved */
    .word PendSV_Handler                /* 14: PendSV */
    .word SysTick_Handler               /* 15: SysTick */
    /* External interrupt vectors (all default) */
    .space 240 * 4                      /* 16-255: IRQs */
.size _start, . - _start

/*=====================================================================
 * Default handler -- infinite loop
 *====================================================================*/
.section .text.Default_Handler, "ax", %progbits
.type Default_Handler, %function
Default_Handler:
    b Default_Handler
.size Default_Handler, . - Default_Handler

/* Empty handler stubs */
.weak NMI_Handler
.thumb_set NMI_Handler, Default_Handler

.weak DebugMon_Handler
.thumb_set DebugMon_Handler, Default_Handler

.thumb_func
HardFault_Handler:
    b HardFault_Handler

.thumb_func
MemManage_Handler:
    b MemManage_Handler

.thumb_func
BusFault_Handler:
    b BusFault_Handler

.thumb_func
UsageFault_Handler:
    b UsageFault_Handler

.thumb_func
SVC_Handler:
    b SVC_Handler

.thumb_func
PendSV_Handler:
    b PendSV_Handler

.thumb_func
SysTick_Handler:
    b SysTick_Handler

/*=====================================================================
 * Reset_Handler
 *====================================================================*/
.section .text.Reset_Handler, "ax", %progbits
.type Reset_Handler, %function
Reset_Handler:
    /* Set stack pointer (already set by hardware, but be safe) */
    ldr r0, =_estack
    msr msp, r0

    /* Copy .data from FLASH to SRAM */
    ldr r1, =__data_load_start
    ldr r2, =__data_init_start
    ldr r3, =__data_init_end
    subs r3, r3, r2
    beq .L_zero_bss
.L_copy_data:
    subs r3, r3, #4
    ldr r0, [r1, r3]
    str r0, [r2, r3]
    bne .L_copy_data

    /* Zero .bss */
.L_zero_bss:
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__
    subs r2, r2, r1
    beq .L_call_main
    mov r0, #0
.L_clear_bss:
    subs r2, r2, #4
    str r0, [r1, r2]
    bne .L_clear_bss

    /* Call SystemInit (weak -- override in project code) */
.L_call_main:
    bl SystemInit
    bl main

    /* If main returns, loop forever */
.L_infinite:
    b .L_infinite
.size Reset_Handler, . - Reset_Handler

/*=====================================================================
 * SystemInit -- weak default
 *====================================================================*/
.section .text.SystemInit, "ax", %progbits
.type SystemInit, %function
.weak SystemInit
SystemInit:
    /* Minimal: nothing to initialize in QEMU */
    bx lr
.size SystemInit, . - SystemInit

.end
