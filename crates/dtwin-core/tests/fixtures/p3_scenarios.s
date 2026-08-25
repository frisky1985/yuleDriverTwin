@ =============================================================================
@ p3_scenarios.s — P3 异常机制集成测试固件（exception_mech.rs 的二进制来源）
@
@ 重建说明：原 .s 未入库（仅 .bin 被 include_bytes! 引用），本文件由
@ objdump 反汇编逐字节重建，汇编后与 p3_scenarios.bin 完全一致（256B）。
@ 重建命令：
@   arm-none-eabi-as -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb \
@     p3_scenarios.s -o /tmp/p3.o && arm-none-eabi-objcopy -O binary /tmp/p3.o p3_scenarios.bin
@
@ 符号（exception_mech.rs 引用）：scn_a_start=0x4A scn_a_ret=0x50
@ scn_b_start=0x54 task1_body=0x76 task2_body=0x7A scn_c_start=0x7E
@ scn_c_systick_handler=0x8E scn_d_start=0x96 scn_e_start=0xBC
@ scn_e_pendsv_handler=0xC6 scn_f_start=0xD2 scn_g_start=0xD8
@ scn_g_svc_handler=0xE2
@
@ 注：向量表按原始二进制使用偶数地址（无 T 位，0x40/0x42/0x6E/0x94），
@ 与引擎 vector_base+偏移*4 的取值方式一致。
@ =============================================================================
	.syntax unified
	.thumb
	.cpu cortex-m4
	.fpu fpv4-sp-d16

	.section .text
	.align 2

@ ---------------- 向量表（0x00-0x3F） ----------------
	.word 0x20002000			@ 0x00 MSP
	.word 0x00000040			@ 0x04 异常 0（Reset → 0x40 跳板）
	.word 0, 0, 0, 0			@ 0x08-0x17 异常 1-4（未用）
	.word 0, 0, 0, 0			@ 0x18-0x27 异常 5-8（未用）
	.word 0				@ 0x28-0x2B 异常 9-10（未用，共 9 个零 word）
	.word 0x00000042			@ 0x2C 异常 11（SVCall → 0x42）
	.word 0, 0				@ 0x30-0x37 异常 12-13（未用）
	.word 0x0000006E			@ 0x38 异常 14（PendSV → 0x6E）
	.word 0x00000094			@ 0x3C 异常 15（SysTick → 0x94）

@ ---------------- 0x40 Reset 跳板 → scn_a_start ----------------
reset_trampoline:
	b	scn_a_start

@ ---------------- 0x42 公共 SVC 处理器（异常 11 向量指向） ----------------
common_svc_handler:
	mrs	r2, IPSR
	movs	r3, #51			@ 0x33
	bx	lr

@ ---------------- 0x4A 场景 A：SVC 0 入口/返回 ----------------
scn_a_start:
	movs	r0, #17			@ 0x11
	movs	r1, #34			@ 0x22
	svc	0
scn_a_ret:
	movs	r4, #68			@ 0x44
	b	scn_a_ret

@ ---------------- 0x54 场景 B：切 PSP + CONTROL（SPSEL=1） ----------------
scn_b_start:
	ldr	r0, psp_literal		@ 0xF0：新 PSP 值
	msr	PSP, r0
	movs	r0, #1
	msr	CONTROL, r0
	isb	sy
	ldr	r1, icsr_addr		@ 0xF4：ICSR
	movs	r2, #1
	lsls	r2, r2, #28		@ PENDSVSET
	str	r2, [r1, #0]
	b	.				@ 等待 PendSV

@ ---------------- 0x6E 公共 PendSV 处理器（异常 14 向量指向） ----------------
common_pendsv_handler:
	ldr	r0, psp_literal2	@ 0xF8：回切 PSP
	msr	PSP, r0
	bx	lr

@ ---------------- 0x76/0x7A 任务体（task1/task2） ----------------
task1_body:
	movs	r4, #238		@ 0xEE
	b	task1_body
task2_body:
	movs	r5, #119		@ 0x77
	b	task2_body

@ ---------------- 0x7E 场景 C：使能 SysTick 后 SVC ----------------
scn_c_start:
	ldr	r1, systick_addr	@ 0xFC：SysTick
	movs	r0, #7
	str	r0, [r1, #0]		@ CTRL = 7（TICKINT|ENABLE|CLKSOURCE）
	movs	r0, #50			@ 0x32
	str	r0, [r1, #4]		@ LOAD = 50
	movs	r0, #0
	str	r0, [r1, #8]		@ VAL = 0
	b	.
scn_c_systick_handler:
	svc	1
	movs	r7, #199		@ 0xC7
	bx	lr

@ ---------------- 0x94 公共 SysTick 处理器（异常 15 向量指向） ----------------
common_systick_handler:
	bx	lr

@ ---------------- 0x96 场景 D：BASEPRI 屏蔽下自旋等待 ----------------
scn_d_start:
	movs	r0, #5
	msr	BASEPRI, r0
	ldr	r1, systick_addr	@ 0xFC：SysTick
	movs	r0, #7
	str	r0, [r1, #0]
	movs	r0, #20			@ 0x14
	str	r0, [r1, #4]		@ LOAD = 20
	movs	r0, #0
	str	r0, [r1, #8]
scn_d_spin:
	mov.w	r1, #0x20000000
	ldr	r2, [r1, #0]
	cmp	r2, #1
	bne	scn_d_spin		@ 等待 [0x20000000] == 1（回跳 mov.w @0xAA）
	movs	r0, #0
	msr	BASEPRI, r0
	b	.

@ ---------------- 0xBC 场景 E：PendSV 挂起（PENDSVSET） ----------------
scn_e_start:
	ldr	r1, icsr_addr		@ 0xF4
	movs	r2, #1
	lsls	r2, r2, #28
	str	r2, [r1, #0]
	b	.
scn_e_pendsv_handler:
	ldr	r1, icsr_addr		@ 0xF4
	movs	r2, #1
	lsls	r2, r2, #26		@ PENDSVCLR
	str	r2, [r1, #0]
	movs	r6, #230		@ 0xE6
	bx	lr

@ ---------------- 0xD2 场景 F：SVC 后循环 ----------------
scn_f_start:
	movs	r0, #250		@ 0xFA
	svc	0
	b	.

@ ---------------- 0xD8 场景 G：FPU 指令 + SVC（scn_g_svc_handler @0xE2） ----------------
scn_g_start:
	vadd.f32	s0, s0, s1
	movs	r0, #119		@ 0x77
	svc	0
	b	.
scn_g_svc_handler:
	mrs	r2, IPSR
	ldr	r3, [sp, #32]		@ 异常帧偏移检查
	ldr	r4, [sp, #96]		@ 0x60
	movs	r5, #153		@ 0x99
	bx	lr

	.balign 4, 0
psp_literal:	.word 0x20001000	@ 0xF0
icsr_addr:	.word 0xE000ED04	@ 0xF4
psp_literal2:	.word 0x20000800	@ 0xF8
systick_addr:	.word 0xE000E010	@ 0xFC
