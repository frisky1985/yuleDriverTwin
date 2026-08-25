# OpenSpec: FreeRTOS 镜像支持（Cortex-M 调度）

> **能力目录**: `docs/specs/freertos-image/`
> **版本**: v0.1（基线）
> **状态**: ✅ 已合入（变更 `2026-08-freertos-image-run` 于 2026-08-16 评审通过，P1-P4 已实施；验收复核中，见登记表 AC 状态）
> **最后更新**: 2026-08-25

## 概述

本能力定义 dtwin（yuleDriverTwin 的 ARM Cortex-M4F 行为模拟器）对 FreeRTOS 镜像的
支持范围：在模拟器上运行真实 FreeRTOS V11.1.0 内核固件，验证 SysTick 定时器、
SVC/PendSV 异常与多任务调度（上下文切换）行为，并以 QEMU（mps2-an386 + cortex-m4）
黄金输出为对照基准。这是"可验证驱动代码"承诺的 OS 底座验证——FreeRTOS 是后续
真实驱动开发（任务、中断、外设驱动）的运行环境。

## 范围

- **覆盖**：FreeRTOS V11.1.0 内核在 dtwin 上的启动（SVC）、节拍（SysTick）、
  上下文切换（PendSV）、多任务轮转调度、QEMU 黄金对照。
- **不覆盖**（本能力当前阶段）：内核对象（互斥量/信号量/队列/事件组/定时器/任务通知）、
  低功耗 tickless、MPU 特权隔离、GDB 调试、真实 S32K312 板级固件。

## 相关规范

| 文档 | 关系 |
|------|------|
| `docs/specs/m4f-engine-architecture.md` | 引擎架构与验收状态（M4F-04 异常嵌套/惰性压栈未达成，本能力依赖） |
| `docs/requirements/` | PRD v1.0（驱动孪生） |
| `docs/specs/freertos-image/feasibility.md` | 可行性分析（能力缺口清单、镜像方案、外部事实核实） |

## 要求登记表（Requirement Registry）

> 本表按变更 `2026-08-freertos-image-run`（2026-08-16 评审通过）的 ADDED 清单回填。
> 详细 SHALL/SHOULD/MAY 语义见 `changes/2026-08-freertos-image-run/spec-delta.md`。

### 需求（FRT-EXC / FRT-SYS / FRT-INS / FRT-CHIP / FRT-FW）

| 需求 ID | 级别 | 摘要 | 来源变更 | 状态 |
|---------|------|------|----------|------|
| FRT-EXC-01 | SHALL | 异常入口压栈跳转：向量取址、8 字现场帧、MSP/PSP 栈选择、LR←EXC_RETURN、IPSR←异常号、切 Handler 模式、ITSTATE 清零 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-02 | SHALL | 异常返回弹栈恢复：BX EXC_RETURN 识别、弹帧恢复、SPSEL 更新、IPSR 清零；非 EXC_RETURN 保持普通分支 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-03 | SHALL | 现场帧内容语义：PC 槽=下一指令、LR 槽=被中断 r14 寄存器值（QEMU v7m_push_stack 一致） | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-04 | SHALL | 压栈对齐：CCR.STKALIGN=1 时 SP&4 先 SP-=4 并置 xPSR bit9（SPREALIGN），出口恢复 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-05 | SHALL | 优先级仲裁：线程基线/Handler 当前优先级、同优先级不抢占、SHPR1-3/NVIC 优先级参与 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-06 | SHALL | 屏蔽生效：PRIMASK/FAULTMASK/BASEPRI 约束仲裁，MSR 写入下一指令边界立即生效 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-07 | SHALL | SVC 执行语义：触发异常 11（SVCall），不再返回 UnimplementedInstr Fault | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-08 | SHALL | MRS IPSR 维护：Handler 读回异常号，线程读 0；入口置位/出口清零 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-09 | SHOULD | FPU 扩展帧：FPCA=1 时压 S0-S15+FPSCR 扩展帧，EXC_RETURN FPU 变体，出口恢复（eager 允许） | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-EXC-10 | SHOULD | 行为等价优化边界：尾链/懒压栈可不实现，只比对行为与输出序列 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-SYS-01 | SHALL | SysTick 寄存器模型：CTRL/LOAD/VAL/CALIB、COUNTFLAG 读即清、写 VAL 清标志归零 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-SYS-02 | SHALL | SysTick 周期触发：每模拟周期减 1，至 0 置 COUNTFLAG、TICKINT 挂异常 15、自 LOAD 重载 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-SYS-03 | SHALL | ICSR 模型：PENDSVSET/CLEAR、PENDSTSET/CLEAR（w1s/w1c）、VECTACTIVE 读 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-SYS-04 | SHALL | SHPR1-3 模型：SVCall/PendSV/SysTick 字节字段，值参与仲裁 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-SYS-05 | SHALL | VTOR/CPUID/CPACR/FPCCR 模型：读基址/0x410FC241/FPU 门控/ASPEN-LSPEN 存储 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-SYS-06 | SHOULD | 周期模型声明：SysTick 引擎周期驱动，黄金对照只比对输出序列与行为 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-INS-01 | SHALL | CPSIE/CPSID 解码执行：置/清 PRIMASK、FAULTMASK | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-INS-02 | SHALL | DSB/ISB/DMB 解码：单核顺序模拟下无操作 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-INS-03 | SHALL | 32 位 LDM/STM 全家族：IA/DB+回写+r0-r15 任意组合，PC 按 Branch 语义 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-INS-04 | SHALL | CLZ 解码执行 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-INS-05 | SHOULD | 位操作指令族：UBFX/SBFX/BFI/BFC/REV 家族/RBIT/LDREX/STREX/UMULL/SMULL/SMLAL | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-INS-06 | MAY | 提示类指令：SEV/WFE/WFI/YIELD 解码为无操作 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-CHIP-01 | SHALL | SYSTEM 内存区：S32K312 profile 含 0xE0000000-0xE0100000（可读写不可执行） | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-CHIP-02 | SHALL | 系统寄存器挂接：SysTick/SCB 经 BusDevice/attach_peripheral 注册，Peripheral::tick 接入 run 循环 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-CHIP-03 | SHOULD | 向量表取址一致性：异常入口从内存读向量，与 loader 烧录一致 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-FW-01 | SHALL | FreeRTOS 最小固件 fixture：V11.1.0 ARM_CM4F port+内核+启动+链接脚本+FreeRTOSConfig（25MHz/1000Hz 等） | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-FW-02 | SHALL | 任务集与输出：HIGH/MID/LOW 延迟打印 + 2 同优先级时间片任务，行前缀统一，迭代 < 500 指令 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-FW-03 | SHALL | 自定义 SVC 用例：任务内 svc #N，处理器打印标记并正确返回 | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-FW-04 | SHALL | 临界区用例：taskENTER/EXIT_CRITICAL 保护共享计数（BASEPRI 屏蔽验证） | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-FW-05 | SHALL | 构建脚本：build_freertos_demo.sh 单命令产出 ELF+.elf.dat（离线可复现） | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-FW-06 | SHALL | QEMU 黄金脚本：run_qemu_golden_freertos.sh 产出黄金输出（mps2-an386+cortex-m4） | 2026-08-freertos-image-run | ✅ 已合入 |
| FRT-FW-07 | SHALL | 双跑对比：e2e_freertos.sh + e2e_freertos.rs 归一化逐行对比 + 核心行命中统计 | 2026-08-freertos-image-run | ✅ 已合入 |

### 验收项（FRT-AC-01~11，判定细则见 spec-delta §3）

| 需求 ID | 级别 | 摘要 | 来源变更 | 状态 |
|---------|------|------|----------|------|
| FRT-AC-01 | 验收 | 多任务不同优先级轮转，与 QEMU 黄金对应行逐字一致 | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-02 | 验收 | 同优先级时间片轮转（[TS] 交替序列与黄金一致） | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-03 | 验收 | SysTick 周期中断驱动节拍（delay 语义正确、无 tick 丢失） | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-04 | 验收 | SVC 启动调度器 + 自定义 SVC（[SVC] 标记、现场恢复） | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-05 | 验收 | PendSV 上下文切换现场正确（seq 无跳变/重复，faults=0） | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-06 | 验收 | FPU 场景 A（任务无浮点，PendSV 跳过 s16-s31 保存） | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-07 | 验收(SHOULD) | FPU 场景 B（浮点任务上下文切换与黄金一致） | 2026-08-freertos-image-run | ⏳ 复核中（VLDM-VSTM 门控阻塞，小克修复后复验） |
| FRT-AC-08 | 验收 | QEMU 黄金双跑归一化 diff 0 差异，核心检查行全命中 | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-09 | 验收 | 临界区 BASEPRI 屏蔽正确（[CRIT] 计数无丢失/重入） | 2026-08-freertos-image-run | ✅ 通过 |
| FRT-AC-10 | 验收 | 全量回归 ≥191 全绿，既有断言无弱化 | 2026-08-freertos-image-run | ✅ 通过（当前 251 tests 全绿） |
| FRT-AC-11 | 验收 | 边界确认：固件无内核对象 API/tickless/MPU 使用（grep 机械核对） | 2026-08-freertos-image-run | ✅ 通过 |

## 验收入口

本能力的验收命令与判定标准见
`changes/2026-08-freertos-image-run/spec-delta.md`（验收矩阵 + 可复现命令）。
