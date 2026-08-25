/*=============================================================================
 * startup_freertos.S — FreeRTOS 镜像启动代码（Cortex-M4F / ARM_CM4F port）
 *
 * 向量表（0x0，VTOR=0）：
 *   [0] 初始 MSP = _estack（SRAM 顶）
 *   [1] Reset_Handler（拷贝 .data、清零 .bss、调 main）
 *   [11] vPortSVCHandler_Dispatch（自定义 SVC 分发，见下）
 *   [14] xPortPendSVHandler（FreeRTOS port 提供）
 *   [15] xPortSysTickHandler（FreeRTOS port 提供）
 *
 * SVC 分发（FRT-FW-03）：FreeRTOS 启动用 svc 0（线程模式 SPSEL=0 → 现场帧在 MSP，
 * EXC_RETURN=F9，bit2=0）；任务内自定义 svc #N（线程模式 SPSEL=1 → 现场帧在 PSP，
 * EXC_RETURN=FD，bit2=1）。用 tst r14,#4 判别：
 *   - F9（bit2=0）→ b vPortSVCHandler（复刻调度器启动语义）
 *   - FD（bit2=1）→ 读 PSP 现场帧 PC 槽（return_pc），回退 2 字节取 svc 立即数，
 *     调 C 函数 vApplicationSvcHandler(imm) 打印后 bx r4 异常返回。
 *============================================================================*/

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.section .isr_vector, "a", %progbits
.type freertos_vector_table, %object
freertos_vector_table:
    .word _estack                    /* 0: 初始 MSP */
    .word Reset_Handler              /* 1: Reset */
    .word Default_Handler            /* 2: NMI */
    .word Default_Handler            /* 3: HardFault */
    .word Default_Handler            /* 4: MemManage */
    .word Default_Handler            /* 5: BusFault */
    .word Default_Handler            /* 6: UsageFault */
    .word 0                          /* 7 */
    .word 0                          /* 8 */
    .word 0                          /* 9 */
    .word 0                          /* 10 */
    .word vPortSVCHandler_Dispatch   /* 11: SVCall（自定义分发） */
    .word Default_Handler            /* 12: DebugMonitor */
    .word 0                          /* 13 */
    .word xPortPendSVHandler         /* 14: PendSV */
    .word xPortSysTickHandler        /* 15: SysTick */

.text
.thumb_func
.global Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    /* 拷贝 .data：LMA(_sidata) → VMA(_sdata.._edata) */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
copy_loop:
    cmp r0, r1
    bge copy_done
    ldr r3, [r2], #4
    str r3, [r0], #4
    b copy_loop
copy_done:
    /* 清零 .bss */
    ldr r0, =_sbss
    ldr r1, =_ebss
zero_loop:
    cmp r0, r1
    bge zero_done
    movs r3, #0
    str r3, [r0], #4
    b zero_loop
zero_done:
    bl main
    b .

.thumb_func
.global Default_Handler
.type Default_Handler, %function
Default_Handler:
    b .

/* SVC 分发（见文件头注释） */
.thumb_func
.global vPortSVCHandler_Dispatch
.type vPortSVCHandler_Dispatch, %function
vPortSVCHandler_Dispatch:
    tst r14, #4                     /* EXC_RETURN bit2：0=MSP(启动) 1=PSP(任务) */
    beq 1f
    /* 任务内自定义 SVC：现场帧在 PSP；PC 槽 = return_pc（svc 指令地址 + 2） */
    mov r4, lr                      /* 保存 EXC_RETURN（bl 会破坏 lr；r4 为 callee-saved） */
    mrs r0, psp
    ldr r1, [r0, #24]               /* 帧 PC 槽 */
    sub r1, r1, #2                  /* svc 指令地址（16 位 0xDFxx） */
    ldrb r1, [r1]                   /* 立即数（低字节） */
    mov r0, r1                      /* AAPCS：第一参数在 r0（mrs 后 r0=PSP 需覆盖） */
    bl vApplicationSvcHandler
    bx r4                           /* 异常返回（弹 PSP 帧恢复现场） */
1:
    b vPortSVCHandler               /* 调度器启动路径（复刻 port 语义） */

.end
