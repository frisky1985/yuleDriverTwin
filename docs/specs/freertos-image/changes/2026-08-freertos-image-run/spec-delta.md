# spec-delta: FreeRTOS 镜像跑通（SysTick / PendSV / SVC 调度）

> **Change ID**: `2026-08-freertos-image-run`
> **能力**: `docs/specs/freertos-image/spec.md`
> **日期**: 2026-08-16 | **作者**: 小马（质量架构师）| **状态**: ⏳ 待评审
> **可行性依据**: `docs/specs/freertos-image/feasibility.md`（能力缺口逐项证据）
> **评审通过后**: 小克按 P1-P4 拆包开发（见可行性分析 §3）

---

## 1. 背景与目标

yuleDriverTwin 已完成 M4F 引擎 + yuleASR 打通（E2E 27/27）+ E2E 固件 70 项 PASS。
下一步关键验证：在 dtwin 上运行 **FreeRTOS V11.1.0（ARM_CM4F 移植）** 镜像，
验证 SysTick 定时器、SVC/PendSV 异常、多任务上下文切换调度，并与 QEMU
（mps2-an386 + cortex-m4）黄金输出对照。这是"可验证驱动代码"承诺的 OS 底座。

本变更新增能力要求 ID 前缀：**FRT-**（分组 EXC/SYS/INS/CHIP/FW/AC）。

---

## 2. 需求变更（ADDED）

### 2.1 异常机制（FRT-EXC）

**ADDED FRT-EXC-01 — SHALL 异常入口压栈跳转**：引擎接受任一异常（SVC/PendSV/SysTick/外部 IRQ）时，须：① 从内存读取向量地址 `(VTOR 值 + 4 × 异常号)`；② 向选中栈压 8 字现场帧（r0, r1, r2, r3, r12, lr, pc, xpsr，小端、低地址在前）；③ 栈选择：线程模式按 CONTROL.SPSEL（0→MSP，1→PSP），Handler 模式恒 MSP；④ LR←EXC_RETURN（线程+PSP→0xFFFFFFFD，线程+MSP→0xFFFFFFF9，Handler→0xFFFFFFF1；FPU 上下文→对应 FPU 变体）；⑤ xPSR.IPSR←异常号；⑥ 切换 Handler 模式（SP 别名 MSP）；⑦ ITSTATE 清零。

**ADDED FRT-EXC-02 — SHALL 异常返回弹栈恢复**：执行 BX 且目标为 EXC_RETURN 值（F1/F9/FD/E1/E9/ED）时，须按 EXC_RETURN bit2 选择 MSP/PSP 弹 8 字帧，恢复 r0-r3/r12/r14/PC/xPSR；切回线程模式；CONTROL.SPSEL 按 EXC_RETURN 更新（bit2=1→SPSEL=1）；xPSR.IPSR←0。非 EXC_RETURN 值的 BX 保持普通分支语义不变。

**ADDED FRT-EXC-03 — SHALL 现场帧内容语义**：PC 槽 = 被中断流的下一指令地址（当前 PC 值）；LR 槽 = 被中断上下文当前的 r14 寄存器值；xPSR 槽 = 被中断时的 xPSR（含 T 位原样保存/恢复）。语义与 QEMU v11.0.2 `m_helper.c v7m_push_stack` 一致（LR 槽为寄存器值，非 PC+4）。

**ADDED FRT-EXC-04 — SHALL 压栈对齐**：CCR.STKALIGN=1（复位默认）时，异常入口若 SP 未 8 字节对齐（SP & 4），先 SP-=4 并置 xPSR bit9（SPREALIGN），出口弹栈后恢复 SP 并清该位。

**ADDED FRT-EXC-05 — SHALL 优先级仲裁**：异常被接受条件 = 其优先级数值 < 当前执行优先级（线程模式基线 256，Handler 模式 = 当前异常优先级）；同优先级不抢占（先挂起者保持挂起，如 PendSV(255) 不抢占 SysTick(255)）；系统异常优先级取自 SHPR1-3，外部 IRQ 取自 NVIC priority 数组。

**ADDED FRT-EXC-06 — SHALL 屏蔽生效**：异常仲裁须受 PRIMASK/FAULTMASK/BASEPRI 约束——PRIMASK=1 屏蔽全部可配置优先级异常；FAULTMASK=1 额外屏蔽 HardFault；BASEPRI=N 屏蔽优先级 ≥ N 的可配置异常（BASEPRI=0 不屏蔽）；MRS/MSR 对这三者的写入在下一指令边界立即生效。

**ADDED FRT-EXC-07 — SHALL SVC 执行语义**：执行 SVC 指令 → 按 FRT-EXC-01 触发异常 11（SVCall），不再返回 `UnimplementedInstr` Fault。

**ADDED FRT-EXC-08 — SHALL MRS IPSR 维护**：Handler 模式 MRS IPSR 读回当前异常号（xPSR bits[8:0]），线程模式读 0；异常入口置位、出口清零。

**ADDED FRT-EXC-09 — SHOULD FPU 扩展帧**：任务使用浮点（CONTROL.FPCA=1，引擎跟踪首次 VFP 指令置位）时，异常入口压扩展帧（S0-S15 + FPSCR，基本帧后追加 0x48 字节），EXC_RETURN 使用 FPU 变体（bit4=0：E1/E9/ED）；出口恢复 FPU 寄存器与 FPCA。允许 eager 保存实现（与硬件懒压栈行为等价；懒压栈为性能语义，可不实现）。

**ADDED FRT-EXC-10 — SHOULD 行为等价优化边界**：尾链（tail-chaining）与懒压栈为性能优化，不要求实现；验收只比对行为与输出序列，不比对周期数。实现先出栈再入栈亦可。

### 2.2 SysTick 与系统寄存器（FRT-SYS）

**ADDED FRT-SYS-01 — SHALL SysTick 寄存器模型**：0xE000E010-0xE000E01C 建模——CTRL（bit0 ENABLE、bit1 TICKINT、bit2 CLKSOURCE、bit16 COUNTFLAG 只读）、LOAD、VAL、CALIB；写 LOAD 更新重载值；写 VAL 清 COUNTFLAG 且计数器归零（ENABLE=1 时下一周期自 LOAD 重载）；读 CTRL 的 COUNTFLAG 后硬件自动清零（读即清，ARM/QEMU 语义）。

**ADDED FRT-SYS-02 — SHALL SysTick 周期触发**：ENABLE=1 时计数器每模拟周期减 1（当前引擎周期模型：1 指令 = 1 周期，exec.rs cycle_count）；减至 0 → 置 COUNTFLAG、若 TICKINT=1 则挂起异常 15、自 LOAD 重载；LOAD=0 时计数器保持 0 不触发。

**ADDED FRT-SYS-03 — SHALL ICSR 模型**：0xE000ED04——PENDSVSET(w1s→挂起异常 14)、PENDSVCLEAR(w1c)、PENDSTSET(w1s→挂起异常 15)、PENDSTCLR(w1c)、VECTACTIVE[8:0] 读 = 当前异常号（线程模式 0）；整字写仅对置 1 位生效（w1s/w1c 语义，写 0 位不变）。

**ADDED FRT-SYS-04 — SHALL SHPR1-3 模型**：0xE000ED18/1C/20 字节字段——SVCall[SHPR2 bit31:24]、PendSV[SHPR3 bit23:16]、SysTick[SHPR3 bit31:24]、MemManage/BusFault/UsageFault[SHPR1]；读写保留，值参与 FRT-EXC-05 仲裁。

**ADDED FRT-SYS-05 — SHALL VTOR/CPUID/CPACR/FPCCR 模型**：VTOR(0xE000ED08) 读 = 向量基址（复位 0，本阶段支持读 0 即可）；CPUID(0xE000ED00) 读 = 0x410FC241（Cortex-M4 r0p1）；CPACR(0xE000ED88) 读写作用于现有 `fpu_enabled()` 门控（默认 0x00F00000 保持）；FPCCR(0xE000EF34) 写存储 ASPEN/LSPEN 位（供 FRT-EXC-09 判断）。

**ADDED FRT-SYS-06 — SHOULD 周期模型声明**：SysTick 以引擎周期驱动，不要求与 QEMU 周期数一致；黄金对照只比对输出序列与行为（可行性 §2.5 论证序列级对比成立的前提：任务每迭代工作量 < 500 指令，tick ≈ 25000 周期）。

### 2.3 指令（FRT-INS）

**ADDED FRT-INS-01 — SHALL CPSIE/CPSID 解码执行**：16 位 CPSIE i(0xB662)/CPSID i(0xB672)/CPSIE f/CPSID f——分别置/清 PRIMASK、置/清 FAULTMASK；不再落入 Unimplemented。

**ADDED FRT-INS-02 — SHALL DSB/ISB/DMB 解码**：0xF3BF 8F4F/8F5F/8F6F 解码为屏障指令，单核顺序模拟语义下无操作（不 fault、不改变可观测状态）。

**ADDED FRT-INS-03 — SHALL 32 位 LDM/STM 全家族**：0xE8xx/0xE9xx IA/DB 形式 + 回写 + 寄存器列表 r0-r15 任意组合（含 r14、PC）；PC 装载按 Branch 语义（清 bit0、不 +width）；与既有 16 位 LDM/STM 行为一致。

**ADDED FRT-INS-04 — SHALL CLZ**：32 位 CLZ 解码执行（前导零计数，Rd=31-位置）。

**ADDED FRT-INS-05 — SHOULD 编译器常用位操作指令**：UBFX/SBFX/BFI/BFC/REV/REV16/REVSH/RBIT/LDREX/STREX/UMULL/SMULL/SMLAL 按需解码执行（-O2 固件暴露后补齐，每指令配 golden 测试；流程见可行性 §4 风险 1）。

**ADDED FRT-INS-06 — MAY 提示类指令**：SEV/WFE/WFI/YIELD 解码为无操作（固件出现再补，非阻塞）。

### 2.4 芯片与内存（FRT-CHIP）

**ADDED FRT-CHIP-01 — SHALL SYSTEM 内存区**：S32K312 profile（或 `memory_from_profile` 统一逻辑）必须包含 0xE0000000-0xE0100000 的 SYSTEM 区（读写、不可执行），使系统寄存器地址不再 BusFault（当前 s32k312.rs 仅 FLASH/SRAM/PERIPH 三区）。

**ADDED FRT-CHIP-02 — SHALL 系统寄存器挂接机制**：SysTick/SCB 寄存器经既有 `BusDevice`/`attach_peripheral` 机制注册到 Memory（与 CmsdkUart 同模式），读路由/写路由生效；`Peripheral::tick`（或等价机制）接入引擎 run 循环驱动周期行为。

**ADDED FRT-CHIP-03 — SHOULD 向量表取址一致性**：异常入口从内存读向量（VTOR=0 → flash 0x0 的 ELF 向量表），与 loader 烧录行为一致（Nvic 内部 vector_table 结构体不再承担取址职责，或明确同步）。

### 2.5 固件与 E2E（FRT-FW）

**ADDED FRT-FW-01 — SHALL FreeRTOS 最小固件 fixture**：`crates/dtwin-chip/tests/fixtures/freertos/` 下建立 V11.1.0 工程——ARM_CM4F port（port.c + portmacro.h，vendor 入库）+ 内核（tasks/queue/list/heap_4）+ startup + 链接脚本 + FreeRTOSConfig：configCPU_CLOCK_HZ=25MHz、configSYSTICK_CLOCK_HZ=25MHz、configTICK_RATE_HZ=1000、configUSE_PREEMPTION=1、configUSE_TIME_SLICING=1、configMAX_PRIORITIES=8、configUSE_PORT_OPTIMISED_TASK_SELECTION=1、configMAX_SYSCALL_INTERRUPT_PRIORITY=5、configUSE_TICKLESS_IDLE=0、configTOTAL_HEAP_SIZE=32KB、configSUPPORT_STATIC_ALLOCATION=1；链接布局代码 0x0 / SRAM 0x20000000；输出走 CMSDK UART 0x40004000（无 libc）。

**ADDED FRT-FW-02 — SHALL 任务集与输出**：任务 HIGH(pri2)/MID(pri1)/LOW(pri0) 各自打印 N≥5 次带计数行（格式 `[TASK] name seq`）后 vTaskDelay(2/3/5)；2 个同优先级任务（pri2）在无阻塞循环中交替打印（验证时间片）；每个任务每次迭代工作量 < 500 条指令（远小于 1 tick ≈ 25000 周期，保证序列确定性）。全部输出行前缀统一（如 `[TASK]`/`[TS]`/`[SVC]`/`[CRIT]`/`[PASS]`）。

**ADDED FRT-FW-03 — SHALL 自定义 SVC 用例**：任务内执行 `svc #N`（N 自选非 0），自定义 SVC 处理器打印标记并正确返回（验证 SVC 入口/出口/现场恢复；该路径独立于 FreeRTOS 的 PendSV yield 路径）。设计提示：SVC 向量只有 1 个，demo 的自定义 naked SVC 处理器需兼管两路——SVC 立即数 == 0 时走调度器启动路径（复刻 port 的 vPortSVCHandler 语义），非 0 时调 C 函数打印后 `bx lr` 返回（约 20 行汇编）。

**ADDED FRT-FW-04 — SHALL 临界区用例**：任务用 `taskENTER_CRITICAL()/taskEXIT_CRITICAL()` 保护共享计数递增并打印结果（验证 BASEPRI 屏蔽下 SysTick 不抢占临界区，计数无丢失/无重入）。

**ADDED FRT-FW-05 — SHALL 构建脚本**：`scripts/build_freertos_demo.sh` 单命令产出 `fixtures/build/freertos_demo.elf` + `.elf.dat`（复刻 build_driver_stress.sh 模式；port/内核文件 vendor 入库，离线可复现；构建参数 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -ffreestanding -nostdlib）。

**ADDED FRT-FW-06 — SHALL QEMU 黄金脚本**：`scripts/run_qemu_golden_freertos.sh` 产出黄金输出（`qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel freertos_demo.elf`，FreeRTOS port 自带 vPortEnableVFP，无需 CPACR gdb 补丁；输出归一化剔除 QEMU 终止提示行）。

**ADDED FRT-FW-07 — SHALL 双跑对比**：`scripts/e2e_freertos.sh`（CLI 层）+ `crates/dtwin-chip/tests/e2e_freertos.rs`（集成测试层，include_bytes fixture，复刻 e2e_driver_stress.rs 模式）：归一化逐行对比 + 核心检查行命中统计（缺一行即失败）。

---

## 3. 验收矩阵（GIVEN / WHEN / THEN）

> 判定命令统一前缀：`export PATH="$HOME/.cargo/bin:$PATH"; cd ~/.openclaw/workspace/yuleDriverTwin`
> 每条验收可机械执行：给定前置 → 执行命令 → 断言结果。

| ID | GIVEN | WHEN | THEN |
|----|-------|------|------|
| **FRT-AC-01** 多任务不同优先级轮转 | FreeRTOS 固件含 HIGH(pri2,delay2)/MID(pri1,delay3)/LOW(pri0,delay5) 三任务，各打印 ≥5 次 | `scripts/build_freertos_demo.sh && scripts/e2e_freertos.sh` | 输出含全部任务行且顺序符合优先级/延迟语义（HIGH 先于 MID 先于 LOW 启动，各自 seq 单调递增）；与 QEMU 黄金输出对应行逐字一致 |
| **FRT-AC-02** 时间片轮转 | 固件含 2 个同优先级(pri2)任务，循环内打印后不阻塞 | 同上 | `[TS]` 两任务行交替出现（每 tick 轮换，configUSE_TIME_SLICING=1），交替序列与黄金输出一致 |
| **FRT-AC-03** SysTick 周期中断驱动节拍 | 固件启动即使能 SysTick（vPortSetupTimerInterrupt） | 引擎运行至输出完整（max-instructions 上限内） | SysTick 异常 15 被周期触发：任务按 tick 节奏运行（delay 语义正确），引擎侧 SysTick 计数器行为与黄金输出可观察效果一致（无死锁、无 tick 丢失导致的序列偏差） |
| **FRT-AC-04** SVC 启动调度器 + 自定义 SVC | 固件 vTaskStartScheduler 走 svc 0；任务内另有 svc #N 用例 | 同上 | 首个任务正常启动（调度器 SVC#0 出口后执行任务代码）；`[SVC]` 标记打印且 SVC 处理器返回后任务现场正确（后续 seq 连续）；与黄金输出一致 |
| **FRT-AC-05** PendSV 上下文切换现场正确 | 固件任务用 r4-r11 工作寄存器 + 栈变量，经多次延迟切换 | 同上 | 切换后各任务计数器/局部状态无错乱（输出 seq 无跳变/重复）；引擎 fault 数 = 0；与黄金输出一致 |
| **FRT-AC-06** FPU 场景 A（任务无浮点） | 任务不含浮点指令（场景 A），port 仍使能 VFP/FPCCR | 同上 | 运行正确；PendSV 按 `tst r14,#0x10` 跳过 s16-s31 保存（EXC_RETURN 非 FPU 变体）；输出与黄金一致 |
| **FRT-AC-07** FPU 场景 B（任务用浮点，SHOULD） | 固件另含浮点任务（VADD/VCVT 工作负载）变体 | 构建 FPU 变体固件双跑 | FPU 上下文切换正确：任务浮点累计结果打印与黄金一致（EXC_RETURN FPU 变体 + S0-S31/FPSCR 现场正确） |
| **FRT-AC-08** QEMU 黄金双跑序列一致 | 同一 ELF + 两侧运行 | `scripts/e2e_freertos.sh` | 归一化后 dtwin 输出与黄金输出逐行一致（diff 0 差异）；核心检查行（`[TASK]`/`[TS]`/`[SVC]`/`[CRIT]`/`[PASS]`）全命中 |
| **FRT-AC-09** 临界区（BASEPRI 屏蔽）正确 | 固件含 taskENTER_CRITICAL 保护计数用例 | 同上 | `[CRIT]` 行打印的计数无丢失、无重入错误；与黄金输出一致（BASEPRI 屏蔽期间 SysTick 不抢占） |
| **FRT-AC-10** 全量回归 | 仓库现有 191 单测 + 2 个既有 E2E（yuleASR 27/27、driver_stress 70 项） | `cargo test` | 全绿；新增测试后总数 ≥ 191 且既有断言无一修改为弱化（如有语义修正须在变更说明中列明） |
| **FRT-AC-11** 边界确认（本阶段不做） | 固件源码 | `grep -E "xSemaphore|xQueue|xEventGroup|xTimer|xTaskNotify" fixtures/freertos/main_freertos.c` | 无内核对象 API 使用（互斥量/信号量/队列/事件组/定时器/任务通知明确不在本阶段）；固件未启用 tickless 与 MPU 特性 |

---

## 4. 本阶段不做（明确边界）

以下**明确不在本变更范围内**（后续阶段候选，避免范围蔓延）：

1. **内核对象**：互斥量/递归互斥量/信号量（二值/计数）/队列/流缓冲/消息缓冲/事件组/软件定时器/任务通知——固件不编译对应源文件、不调用对应 API（FRT-AC-11 机械核对）。
2. **低功耗 tickless**：configUSE_TICKLESS_IDLE=0（避开 vPortSuppressTicksAndSleep 的 cpsid 复杂路径）。
3. **MPU/特权模式**：CONTROL.nPRIV 不建模；FreeRTOS 非 MPU 移植全程特权态运行（与真实默认配置一致）。
4. **GDB 调试链路**：dtwin-gdb 保持现状，不动。
5. **周期精确性**：指令周期模型（1 指令=1 周期）不变；不追求与 QEMU 周期数一致（输出序列为准）。
6. **尾链/懒压栈性能优化**：行为等价即可（FRT-EXC-10）。
7. **真实 S32K312 板级 FreeRTOS 工程**（P2 排期）与本阶段无关；本阶段固件为 QEMU MPS2 布局，与 S32K312 profile 内存映射巧合兼容（0x0 代码 / 0x20000000 SRAM / 0x40004000 UART）。

---

## 5. 变更影响与兼容性约束

| 影响面 | 说明 | 约束 |
|--------|------|------|
| Engine::run 异常检查 | 现"仅外部 IRQ 记账"路径（engine.rs:157-163）被统一异常仲裁取代 | 既有外部 IRQ 挂起/使能/优先级语义保留并接入仲裁 |
| Nvic API | enter_exception/exit_exception 语义从"记账"升级为"真实入口/出口" | nvic.rs 既有单测（irq_pend_enable、exception_nesting 等）须同步更新并保持语义正确 |
| SVC 行为 | 从 `Fault(UnimplementedInstr)` 变为异常入口（exec.rs:842-850） | 属于有意的行为变更，需在变更说明中列明并配新测试 |
| 指令解码 | 新增 CPSIE/CPSID/DSB/ISB/DMB/32 位 LDM/STM/CLZ | 只增不改既有解码路径；Unimplemented 兜底保留 |
| 内存模型 | profile 增加 SYSTEM 区 | 与 profile validate（无重叠）校验兼容；m4f_default 已有 SYSTEM 区不受影响 |
| 既有 E2E | yuleASR 27/27、driver_stress 70 项（无异常路径固件） | 必须保持全绿（异常机制对无异常固件不可观测） |
| 单测总数 | 新增指令级/异常级/寄存器级单测 + e2e_freertos.rs | ≥ 191，只增不减 |

---

## 6. 验收命令（可复现路径）

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd ~/.openclaw/workspace/yuleDriverTwin

# 0. 构建 FreeRTOS 最小固件（M4F, V11.1.0, ARM_CM4F port）
scripts/build_freertos_demo.sh
#    → crates/dtwin-chip/tests/fixtures/build/freertos_demo.elf (+.elf.dat 提交用)

# 1. QEMU 黄金输出（mps2-an386 + cortex-m4，同一 ELF）
scripts/run_qemu_golden_freertos.sh
#    → /tmp/freertos_qemu_golden.txt（含 [TASK]/[TS]/[SVC]/[CRIT]/[PASS] 检查行）

# 2. dtwin 双跑 + 逐行对比（退出码 0 = 通过）
scripts/e2e_freertos.sh
#    内部：dtwin run freertos_demo.elf --chip S32K312 --uart-base 0x40004000 \
#          --max-instructions 20000000
#    断言：核心检查行全命中 + 归一化 diff 0 差异 + 引擎 faults=0

# 3. Rust 集成测试（fixtures 内嵌，CI 可复现；等价于步骤 2 的测试层）
cargo test --test e2e_freertos

# 4. 全量回归
cargo test      # 全绿（≥191 测试 + 既有 2 个 E2E）
```

**判定汇总**：FRT-AC-01 至 AC-06、AC-08 至 AC-11 全部通过 = 本阶段验收通过；
FRT-AC-07（FPU 场景 B）为 SHOULD，未通过不阻塞验收但须在结论中如实标注。

---

## 7. 诚实标注（不确定处）

1. **QEMU 无现成 M4F FreeRTOS demo**（已核实官方仓库目录，仅有 M3 QEMU demo）→ 采用自建最小工程方案；构建链细节（新布局内核 + 经典布局 port 混用）存在中等风险，退路为整体使用 qemu_m33 同款经典布局内核（可行性 §4）。
2. **-O2 编译器指令缺口不可完全预知**（UBFX/SBFX/BFI 等）→ 采用"先构建先跑、UnimplementedInstr 清单补码"的诚实迭代流程，每补一条配 golden 测试；不预写全部指令。
3. **异常帧 LR 槽语义**已按 QEMU v11.0.2 源码（LR 槽 = 被中断 r14 寄存器值）定稿；FreeRTOS 不依赖该槽位值，双跑一致可兜底。
4. **SysTick 周期驱动**采用引擎周期模型（1 指令=1 周期）；与 QEMU 周期数必然存在差异，但序列级对比成立（前提 FRT-FW-02 的工作量约束，验收时核对）。
5. **周期/尾链/懒压栈**为性能语义，本阶段以行为等价为准，不做周期数断言。
