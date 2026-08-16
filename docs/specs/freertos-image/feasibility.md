# 可行性分析：dtwin 跑通 FreeRTOS 镜像（SysTick / PendSV / SVC 调度）

> 日期: 2026-08-16 | 作者: 小马（质量架构师）| 仓库: yuleDriverTwin @ main (HEAD=0a28393)
> 关联: docs/specs/freertos-image/spec.md + changes/2026-08-freertos-image-run/spec-delta.md
> 状态: 待评审（评审通过后派小克开发）

---

## 0. 结论摘要（TL;DR）

**结论：条件可行。** FreeRTOS V11.1.0（ARM_CM4F 移植）可以在 dtwin 上跑通，但引擎存在 **3 类硬缺口**（异常机制缺失、SysTick/系统寄存器未建模、4 类关键指令未解码），需先补齐；FreeRTOS 镜像**无现成 M4F QEMU demo 可用**（已核实官方仓库），但可按 yuleASR `tests/qemu_m33` 的成熟模式自建最小工程（工具链已就绪），确定性风险低。

**能力缺口分级：**

| 级别 | 缺口 | 证据 | 工作量估计 |
|------|------|------|-----------|
| 🔴 硬阻塞（MUST） | 异常入口/出口机制（压栈/弹栈/向量跳转/EXC_RETURN/模式切换） | engine.rs run 循环仅记账不跳转；exec.rs SVC 直接 Fault | 大（核心重构） |
| 🔴 硬阻塞（MUST） | SysTick 外设（0xE000E010-1C）+ 周期中断 | SYSTEM 区读 0 写忽略；peripheral::tick 从未被调用 | 中 |
| 🔴 硬阻塞（MUST） | ICSR / SHPR2 / SHPR3 / VTOR / CPUID / CPACR 系统寄存器 | SYSTEM 区无模型；port 读写这些地址 | 中 |
| 🔴 硬阻塞（MUST） | 指令：CPSIE/CPSID、DSB/ISB/DMB、32 位 LDM/STM、CLZ | decode.rs 均落 Unimplemented | 小-中 |
| 🔴 硬阻塞（MUST） | S32K312 profile 缺 SYSTEM 内存区 | s32k312.rs memory 仅 3 区；0xE000E010 会 BusFault | 小 |
| 🟡 必须（MUST） | BASEPRI/PRIMASK 屏蔽生效（临界区语义） | MRS/MSR 可读写但无仲裁效果 | 中（含在异常机制内） |
| 🟡 必须（MUST） | xPSR.IPSR 维护（xPortIsInsideInterrupt 读 MRS IPSR） | MRS IPSR 恒 0 | 小 |
| 🟢 建议（SHOULD） | UBFX/SBFX/BFI/BFC、REV/REV16/REVSH、LDREX/STREX 等编译器常用指令 | decode.rs 无对应解码 | 小-中 |
| 🟢 建议（SHOULD） | FPU 上下文切换（FPCA 跟踪 + 扩展帧 + EXC_RETURN FPU 变体） | M4F-04 未达成（m4f-engine-architecture.md 6 节） | 中 |
| 🟢 建议（SHOULD） | 尾链（tail-chaining） | 无证据（行为等价优化，非功能必需） | 可不做 |

**已验证的外部事实：**
- QEMU 11.0.2 提供 `mps2-an386`（Cortex-M4）机器，**SYSCLK=25MHz、SysTick 参考时钟 1MHz**（QEMU 源码 `hw/arm/mps2.c`：`SYSCLK_FRQ 25000000`、`REFCLK_FRQ 1000000`）。
- FreeRTOS 官方仓库（FreeRTOS/FreeRTOS main）**没有 Cortex-M4F 的 QEMU demo**；QEMU demo 仅 M3（CORTEX_MPS2_QEMU_IAR_GCC / CORTEX_MPU_M3_MPS2_QEMU_GCC / CORTEX_LM3S6965_GCC_QEMU）。→ 必须自建镜像。
- FreeRTOS-Kernel **V11.1.0 tag 存在 `portable/GCC/ARM_CM4F/port.c` + `portmacro.h`**（两文件，约 52KB，MIT）→ 可直接取用。
- yuleASR 已 vendor FreeRTOS **V11.1.0** 内核（`third_party/freertos`，新目录布局 src/+include/）并有 QEMU 测试工程 `tests/qemu_m33`（build.sh 单命令构建 FreeRTOS 镜像）→ 可直接复刻模式，内核零下载。
- 本机工具链就绪：`qemu-system-arm` 11.0.2、`arm-none-eabi-gcc` 16.1.0、`arm-none-eabi-as`（已实测可用）。
- 现有 E2E 已证明 mps2-an386 + CMSDK UART(0x40004000) + dtwin S32K312 profile 是**同一内存布局**（代码 0x0 / SRAM 0x20000000），同一固件双跑可行（e2e_driver_stress 27/27 先例）。

---

## 1. 引擎能力盘点（逐项证据）

### 1.1 已具备 ✅

| 能力 | 证据（文件:行） | 说明 |
|------|----------------|------|
| SVC/PendSV/SysTick 异常号定义 | `crates/dtwin-core/src/nvic.rs:20-27`（SvCall=11, PendSv=14, SysTick=15） | 枚举齐全 |
| EXC_RETURN 6 种变体识别 | `nvic.rs:36-57`（F1/F9/FD + FPU 变体 E1/E9/ED，uses_psp/to_thread） | 有识别，无使用 |
| 外部 IRQ 挂起/使能/活跃/优先级存储 | `nvic.rs:86-100, 139-155`（pending/enabled/active/priority 数组） | 仅记账 |
| MRS/MSR 全部特殊寄存器 | `exec.rs:789-840`（MSP/PSP/PRIMASK/FAULTMASK/BASEPRI/BASEPRI_MAX/CONTROL/APSR 家族/IPSR/EPSR）+ golden 测试 `exec.rs:2514-2598` | 读写正确（含 BASEPRI_MAX min 语义） |
| CONTROL.SPSEL 的 SP 别名同步 | `exec.rs:1688-1696`（sync_sp）+ 测试 `exec.rs:2192-2242` | 线程模式别名正确 |
| LDM/STM 执行（含 r14/PC 写回） | `exec.rs:610-673`（Ldm/Stm 通用 16 位寄存器位图实现） | 执行层就绪 |
| VLDM/VSTM（S/D、IA/DB、回写） | `decode.rs:340, 744-749, 1357-1359` | PendSV 的 vstmdbeq/vldmiaeq 可解码 |
| IT 块条件门控（含 VFP 指令） | `exec.rs:126-147`（ITSTATE 状态机，Skipped 语义） | `it eq; vstmdbeq` 语义正确 |
| TST（寄存器/立即数） | A1 修复 + E2E [TST] 覆盖（README 与 e2e_driver_stress.c） | PendSV 的 `tst r14,#0x10` 就绪 |
| CMSDK UART 行为模型 | `crates/dtwin-core/src/uart.rs`（DATA 写捕获 + 回显） | 0x40004000 兼容 |
| ELF32 loader（LMA/VMA、SP/PC、T 位） | `loader.rs:50-51, 365-373` | 向量表 SP/PC 初始化正确 |
| 引擎防死循环上限 | `engine.rs:142`（max_instructions） | 空转固件可终止 |
| 周期计数（1 指令 = 1 周期） | `exec.rs:126`（cycle_count += 1） | 可作为 SysTick 递减源 |
| E2E 双跑对照模式 | `crates/dtwin-chip/tests/e2e_driver_stress.rs`（include_bytes fixture + 逐行对比） | 验收模式已建立 |

### 1.2 部分具备 ⚠️

| 能力 | 现状 | 证据 | 缺口 |
|------|------|------|------|
| 异常入口 | `Nvic::enter_exception` 只做记账（current_exception/nesting_depth/events），**不压栈、不跳向量、不切模式** | `nvic.rs:226-243`；`engine.rs:146-149`（run 循环 `nvic.enter_exception((irq+16) as u8)` 后继续执行原 PC） | 整个异常机制缺失 |
| 异常返回 | `Instruction::ExceptionReturn` 已定义、exec 返回 `ExecOutcome::ExceptionReturn`，engine 仅计数后 Halted | `decode.rs:205`（**无任何解码路径产出该指令**）；`exec.rs:864`；`engine.rs:119-122` | 死代码；BX r14(0xFFFFFFFD) 会按普通分支跳到 0xFFFFFFFC → BusFault |
| NVIC 优先级 | priority 数组存在但**未参与仲裁**；系统异常优先级（SHPR）无模型 | `nvic.rs:98`；`engine.rs` run 循环无优先级比较 | 需要完整仲裁 |
| MRS IPSR | 读 `cpu.xpsr & 0x1FF`，引擎从不置 IPSR 位 → 恒 0 | `exec.rs:797`；`lib.rs:19-21`（xpsr 注释无 IPSR 语义） | 异常入口需置/清 IPSR（FreeRTOS xPortIsInsideInterrupt 依赖） |
| PRIMASK/BASEPRI/FAULTMASK | 可读写但**无屏蔽效果**（run 循环不查询） | `exec.rs:828-837`；`engine.rs` 异常检查仅看 `current_exception==0` | 需接入仲裁 |

### 1.3 缺失 ❌（硬阻塞）

**A. 异常入口/出口机制（最大缺口）**
- 无：向量地址从内存读取（`mem.read_u32(vt_base + 4*异常号)`，vt_base 来自 VTOR）→ 当前 Nvic.vector_table 是独立结构体且未被 run 循环使用（`nvic.rs:128, 155-157` 默认表；实际向量表在 flash 0x0 由 loader 写入）。
- 无：8 字现场压栈（r0-r3, r12, lr, pc, xpsr）到当前 SP（线程模式按 SPSEL 选 MSP/PSP；Handler 模式恒 MSP）。
- 无：异常入口置 LR=EXC_RETURN、置 IPSR、切 Handler 模式（SP 别名切 MSP）、ITSTATE 清零。
- 无：异常返回（BX EXC_RETURN）弹栈、恢复现场、切回线程模式、更新 CONTROL.SPSEL。
- 无：栈对齐（STKALIGN / xPSR.SPREALIGN，ARMv7-M CCR.STKALIGN 复位=1；FreeRTOS AAPCS 8 字节对齐依赖）。
- 无：优先级仲裁（线程模式基线 256；Handler 用当前异常优先级；同优先级不抢占；PRIMASK/FAULTMASK/BASEPRI 屏蔽）。

**B. 外设与系统寄存器**
- SysTick（0xE000E010-0xE000E01C）无模型：SYSTEM 区（无存储，`storage()` 对 Code/Sram/Ccm 外返回 None）读返回 0、写忽略（`memory.rs:278` 外设/系统区读 0；`memory.rs:369-377` 无存储区写忽略）。
- `Peripheral::tick(cycles)` trait 存在但**全仓库无调用点**（`peripheral.rs:25`；grep 仅定义处）。
- ICSR(0xE000ED04)/SHPR2(0xE000ED1C)/SHPR3(0xE000ED20)/VTOR(0xE000ED08)/CPUID(0xE000ED00)/AIRCR(0xE000ED0C)/CPACR(0xE000ED88)/FPCCR(0xE000EF34) 均无模型 —— FreeRTOS CM4F port 全部直接读写（见 2.2 节清单）。
- **S32K312 profile 内存映射缺 SYSTEM 区**：`crates/dtwin-chip/src/s32k312.rs:33-64` 仅 FLASH/SRAM/PERIPH 三区 → 0xE000E010 落在任何区域外 → **BusFault**（`memory.rs:255-256` region_at None → BusFault）。`Memory::m4f_default` 有 SYSTEM 区（`memory.rs:144`）但 chip profile 路径没有（`chip/src/lib.rs:52-58` 只按 profile.memory 建区）。

**C. 指令（decode 缺失 → UnimplementedInstr Fault）**
| 指令 | 用途（FreeRTOS CM4F port） | 证据 |
|------|---------------------------|------|
| `cpsie i` / `cpsie f` | prvPortStartFirstTask 开启中断（port.c:292-293） | decode.rs `decode_b_misc` 0xB6xx 落 `_ => Unimplemented`（0xB000-0xB7FF 仅处理 ADD/SUB SP、CBZ、PUSH） |
| `cpsid i` / `cpsid f` | tickless 路径（本阶段关 tickless 可不出现）；通用引擎应支持 | 同上 |
| `dsb` / `isb` / `dmb`（0xF3BF 8Fxx） | 临界区 vPortRaiseBASEPRI 每次 `isb; dsb`（portmacro.h:213-227）；vPortEnableVFP；prvPortStartFirstTask | decode_word 0xF3BF 无匹配 → Unimplemented |
| 32 位 `ldmia`/`stmdb`（0xE8xx/0xE9xx） | vPortSVCHandler `ldmia r0!,{r4-r11,r14}`；xPortPendSVHandler `stmdb r0!,{r4-r11,r14}`、`stmdb sp!,{r0,r3}`、`ldmia sp!,{r0,r3}` | decode_word 仅处理 LDRD/STRD 0xE9D0/0xE9C0；0xE8B0/0xE900 等无匹配（16 位 LDM/STM 只支持 r0-r7 位图，含不了 r14） |
| `clz` | portGET_HIGHEST_PRIORITY（configUSE_PORT_OPTIMISED_TASK_SELECTION=1 默认开，portmacro.h:155,171） | decode 无 Clz |
| `mrs %0, ipsr` | xPortIsInsideInterrupt（portmacro.h:188） | MRS 已实现 ✅（缺 IPSR 值维护） |

**D. 编译器 -O2 常用指令（SHOULD，遇缺再补）**
UBFX/SBFX/BFI/BFC、REV/REV16/REVSH、LDREX/STREX、RBIT、UMULL/SMULL/SMLAL —— decode.rs 全无。FreeRTOS 内核 C 代码（tasks.c/queue.c/list.c/heap_4.c）以 -O2 编译时大概率出现 UBFX/SBFX/BFI（位提取/插入模式）。**开发流程建议：固件先构建、引擎先跑，UnimplementedInstr 清单作为补齐输入（诚实迭代，不预写全部）。**

---

## 2. FreeRTOS 版本与镜像方案

### 2.1 版本建议：V11.1.0（与 yuleASR 一致）

- yuleASR `third_party/freertos/include/task.h:56` 确认 `tskKERNEL_VERSION_NUMBER "V11.1.0"`，本地已有新布局内核源码（src/ + include/ + portable/ + heap_4.c）→ **内核零下载**。
- FreeRTOS-Kernel V11.1.0 tag 有 `portable/GCC/ARM_CM4F/`（已通过 GitHub API 核实：port.c 41,903B + portmacro.h 10,427B，MIT 许可）→ 仅需获取这 2 个文件。
- 与 qemu_m33 测试工程（同为 V11.1.0）行为/配置语义一致，可交叉参考 FreeRTOSConfig.h。

### 2.2 ARM_CM4F port 对硬件的精确依赖（已逐行核对 V11.1.0 port.c/portmacro.h）

| 硬件点 | 地址/行为 | port 用法 | 引擎需求 |
|--------|-----------|-----------|----------|
| SysTick CTRL/LOAD/VAL | 0xE000E010/14/18 | vPortSetupTimerInterrupt 写；tickless 读 VAL/COUNTFLAG（本阶段 tickless=0） | **MUST：SysTick 外设**（CLKSOURCE=1 每周期减 1；到 0 → COUNTFLAG + TICKINT→挂起异常 15 + 重载 LOAD；写 VAL 清 COUNTFLAG） |
| ICSR | 0xE000ED04 | yield：`ICSR = PENDSVSET`（port.c:578）；PENDSTSET/CLR 读（tickless）；vPortEnterCritical 断言 `VECTACTIVE==0`（port.c:488） | **MUST：ICSR 寄存器**（PENDSVSET w1s 挂起异常 14；VECTACTIVE 读=当前异常号） |
| SHPR2 | 0xE000ED1C | `SHPR2 = 0` → SVCall 优先级 0（port.c:436） | **MUST：SHPR1-3 字节字段**（参与仲裁） |
| SHPR3 | 0xE000ED20 | PendSV/SysTick 优先级 255（port.c:434-435） | 同上 |
| VTOR | 0xE000ED08 | prvPortStartFirstTask 读定位向量表（port.c:286-288） | MUST：读=0（向量表基址 0x0）；异常入口从内存取向量 |
| CPUID | 0xE000ED00 | 断言非 M7 r0p1（port.c:408-409） | MUST：读返回非 M7 ID（0 即可通过断言；建议 0x410FC241） |
| CPACR | 0xE000ED88 | vPortEnableVFP `CPACR |= 0x00F00000`（port.c:449 附近） | MUST：读写建模（dtwin 默认已使能，读写一致即可） |
| FPCCR | 0xE000EF34 | `FPCCR |= ASPEN|LSPEN (0xC0000000)`（port.c:449） | MUST：写存储；SHOULD：LSPEN 语义（懒压栈，见 2.3） |
| NVIC IP 寄存器 | 0xE000E3F0+ | 仅 configASSERT 时 vPortValidateInterruptPriority 读 | MAY（无外部 IRQ 时不被读） |
| BASEPRI（指令） | — | 临界区= `mov #imm; msr basepri; isb; dsb`（portmacro.h:213-227）；`msr basepri, #0`（port.c:299,570） | **MUST：BASEPRI 屏蔽生效** |
| PRIMASK（指令） | — | 仅 tickless 用 cpsid i（本阶段关）；prvPortStartFirstTask cpsie i | MUST：cpsie/cpsid 解码 + PRIMASK 生效 |
| SVC #0 | 向量 11 | 调度器启动 prvPortStartFirstTask → svc 0（port.c:296） | **MUST：SVC 异常入口**（入口压栈 + 跳向量；现场经 TCB 栈恢复后 bx r14 出口） |
| PendSV | 向量 14 | xPortPendSVHandler：`mrs r0,psp` → `tst r14,#0x10` + `vstmdbeq {s16-s31}` → `stmdb {r4-r11,r14}` → 存 TCB → BASEPRI 屏蔽 → vTaskSwitchContext → 恢复 → `bx r14`（port.c:534-593） | **MUST：PendSV 入口/出口 + PSP 语义**；SHOULD：FPU 变体 |
| SysTick 异常 | 向量 15 | xPortSysTickHandler → xTaskIncrementTick → 需要切换则 ICSR=PENDSVSET（port.c:612-648） | **MUST：SysTick 异常入口** |
| 首次任务启动 | — | pxPortInitialiseStack 预置 xPSR=0x01000000/PC=入口/LR=prvTaskExitError/R0=参数/EXC_RETURN=0xFFFFFFFD（port.c:202-229）→ vPortSVCHandler `ldmia r0!,{r4-r11,r14}; msr psp,r0; msr basepri,#0; bx r14`（port.c:260-305） | **MUST：异常返回弹 8 字帧（r0-r3,r12,lr,pc,xpsr）且 LR 槽恢复 r14、PC 槽恢复 PC、xPSR 恢复（T 位校验）** |

### 2.3 FPU 上下文（诚实边界）

- port **无条件** 使能 VFP + 置 FPCCR（V11.1.0 无 configENABLE_FPU 开关，port.c:449）。
- 硬件懒压栈语义：任务执行 VFP 指令 → CONTROL.FPCA=1 → 异常入口压扩展帧（S0-S15+FPSCR，帧 0x68 字节）+ EXC_RETURN 用 FPU 变体（bit4=0：0xFFFFFFE1/E9/ED）→ PendSV 据此 `vstmdbeq r0!,{s16-s31}`。
- **场景 A（本阶段 SHALL 验收）**：任务不使用浮点 → FPCA 恒 0 → 32 字节基本帧 + EXC_RETURN FD → PendSV 跳过 s16-s31。**引擎无需任何 FPU 帧逻辑即可跑通。**
- **场景 B（SHOULD）**：任务用浮点 → 引擎需 FPCA 跟踪 + 扩展帧 + EXC_RETURN 变体。**可先实现 eager 保存（行为与懒压栈等价，懒压栈是性能语义），尾链同理（行为等价优化）。**
- QEMU mps2-an386 侧：cortex-m4 带 FPU（vfpv4），CPACR 复位 0 由 port 的 vPortEnableVFP 使能 → 同一固件 QEMU 侧行为一致 ✓（driver_stress 先例已证明 CPACR 路径可用）。

### 2.4 镜像构建方案（推荐：自建最小工程，复刻 qemu_m33 模式）

**为什么不用现成 demo**：官方无 M4F QEMU demo（已核实）；官方 M3 demo（CORTEX_MPS2_QEMU_IAR_GCC）是 IAR/GCC 混合工程且不含 FPU 上下文场景，构建链复杂。yuleASR `tests/qemu_m33/build.sh` 已证明"内核 + port + startup + 链接脚本单命令交叉编译"模式成熟可用，直接复刻到 M4F 成本最低、可控性最高。

```
yuleDriverTwin/crates/dtwin-chip/tests/fixtures/freertos/
├── FreeRTOSConfig.h        # M4F 配置（CPU 25MHz / tick 1000Hz / 优先级 8 / 堆 32KB / 优化任务选择开）
├── main_freertos.c         # 3 任务：HIGH(pri2)/MID(pri1)/LOW(pri0) 各打印 N 次后 vTaskDelay；
│                           # + 2 个同优先级任务验证时间片；+ 自定义 SVC 用例；+ 临界区计数用例
├── startup_freertos.S      # 复位/向量表 + .data 拷贝（参照 startup_e2e.S）
├── link_freertos.ld        # 代码 0x0 / SRAM 0x20000000（与 mps2-an386 及 S32K312 profile 同布局）
├── port/                   # ARM_CM4F port.c + portmacro.h（V11.1.0，vendor 入库，MIT）
└── kernel/                 # FreeRTOS V11.1.0 内核（复用 yuleASR third_party/freertos，或 vendor 副本）
```

构建命令（脚本 `scripts/build_freertos_demo.sh`，参照 `scripts/build_driver_stress.sh`）：
```bash
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -O2 -ffreestanding -nostdlib -Wall -Werror \
  -I fixtures/freertos -I fixtures/freertos/kernel/include -I fixtures/freertos/port \
  -T fixtures/freertos/link_freertos.ld \
  -o fixtures/build/freertos_demo.elf \
  fixtures/freertos/kernel/tasks.c .../queue.c .../list.c .../heap_4.c \
  fixtures/freertos/port/port.c \
  fixtures/freertos/startup_freertos.S fixtures/freertos/main_freertos.c
```

**关键配置**（与 QEMU 时钟对齐，源：QEMU mps2.c `SYSCLK_FRQ 25000000`）：
```c
#define configCPU_CLOCK_HZ         25000000UL  // QEMU mps2-an386 SYSCLK
#define configSYSTICK_CLOCK_HZ     25000000UL  // 与 CPU 同频 → port 置 CLKSOURCE=1
#define configTICK_RATE_HZ         1000        // LOAD = 24999
#define configUSE_PREEMPTION       1
#define configUSE_TIME_SLICING     1           // 同优先级时间片轮转
#define configMAX_PRIORITIES       8
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1  // 依赖 CLZ
#define configMAX_SYSCALL_INTERRUPT_PRIORITY   5   // 临界区 BASEPRI 值
#define configUSE_TICKLESS_IDLE    0           // 本阶段关闭（避开 cpsid/tickless 路径）
#define configTOTAL_HEAP_SIZE      (32*1024)
#define configSUPPORT_STATIC_ALLOCATION 1      // idle 任务栈
```

### 2.5 黄金对照方案

同一 ELF 双跑（沿用 e2e_driver_stress 模式）：
- **QEMU 侧**：`qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel freertos_demo.elf`（无需 CPACR gdb 补丁——FreeRTOS port 自带 vPortEnableVFP，与 driver_stress 不同）。
- **dtwin 侧**：`dtwin run freertos_demo.elf --chip S32K312 --uart-base 0x40004000`（引擎升级后）。
- **对比**：归一化后逐行对比 + 核心检查行命中统计（复用 e2e_driver_stress.rs 模式）。
- **诚实标注**：dtwin 周期模型为 1 指令=1 周期，QEMU 为真实周期计数 → 两边 tick 边界在指令流中的相位会漂移。**但只要每个任务每次迭代的工作量 << 一个 tick 的指令量，输出序列（谁在哪个 tick 打印）由 FreeRTOS 调度决策唯一确定，与相位无关** → 序列级对比成立。验收矩阵以此为前提设计（任务每迭代一次打印 + 一次延迟，迭代体 < 500 条指令，tick ≈ 25000 周期）。

---

## 3. 引擎改造范围（供小克拆包参考）

建议拆 4 包（每包可独立验证，参照 yuleDriverTwin 既有小步拆包纪律）：

| 包 | 内容 | 验证 |
|----|------|------|
| P1 指令补齐 | CPSIE/CPSID(i/f)、DSB/ISB/DMB（单核模拟器语义 = NOP 屏障）、32 位 LDM/STM（0xE8xx/0xE9xx 全家族 IA/DB + 回写 + r14/PC）、CLZ、REV 家族、UBFX/SBFX/BFI/BFC、LDREX/STREX | 指令级 golden 测试（arm-none-eabi-as 编码实测）；cargo test 191 不破坏 |
| P2 系统寄存器 + SysTick | SYSTEM 区注册：SysTick(CTRL/LOAD/VAL/CALIB)、ICSR、SHPR1-3、VTOR、CPUID、CPACR、FPCCR；SysTick 周期驱动（引擎周期递减 → 挂起异常 15）；`Peripheral::tick` 接入 run 循环 | 寄存器读写单测 + SysTick 触发单测 |
| P3 异常机制（核心） | 异常仲裁（系统+外部、优先级、PRIMASK/FAULTMASK/BASEPRI）→ 入口（选栈/压 8 字帧/EXC_RETURN/IPSR/模式切换/向量取址）→ SVC 语义 → 出口（BX EXC_RETURN 弹栈/恢复/切线程模式/SPSEL 更新）；STKALIGN；ITSTATE 清零；MRS IPSR 维护 | 单元测试：SVC 入口出口、PendSV 上下文切换、嵌套、BASEPRI 屏蔽、EXC_RETURN 三种返回 |
| P4 芯片 profile + 固件 + E2E | S32K312 profile 加 SYSTEM 区（或 memory_from_profile 统一补）；FreeRTOS 最小工程 fixtures + build 脚本 + QEMU 黄金脚本 + dtwin 对比脚本 + e2e_freertos.rs 集成测试 | e2e_freertos 双跑一致；cargo test 全绿 |

**兼容性约束（SHALL）**：现有 191 测试与 2 个 E2E（yuleASR 27/27、driver_stress 70 项）不得回归——异常机制重构需保持 `Nvic::enter_exception/exit_exception` 等既有 API 兼容或同步更新其单测。

---

## 4. 风险与诚实标注

| 风险 | 等级 | 说明与对策 |
|------|------|-----------|
| -O2 编译器指令缺口（UBFX 等） | 中 | 不可避免会有少量未解码指令暴露；流程上"先构建先跑，UnimplementedInstr 清单补码"，每补一条配 golden 测试 |
| 异常帧 LR 槽语义（保存 r14 寄存器值 vs PC+4） | 低 | 已核对 QEMU v11.0.2 `target/arm/tcg/m_helper.c` `v7m_push_stack`：LR 槽 = **当前 r14 寄存器值**、PC 槽 = 下一指令地址（实证）；且 FreeRTOS 不依赖该槽位具体值（现场经 TCB 栈保存/恢复），双跑一致性可兜底 |
| SysTick 时钟频率与 QEMU 对齐 | 低 | configCPU_CLOCK_HZ=25MHz 与 QEMU SYSCLK 一致；即便有偏差也不影响输出序列（见 2.5） |
| QEMU cortex-m4 的 FPU/懒压栈行为与真实硬件差异 | 低 | 场景 A 不涉 FPU；场景 B（SHOULD）以 QEMU 为黄金参照，引擎 eager 保存行为等价 |
| FreeRTOS demo 构建的构建链细节（新布局内核 + 经典布局 port 混用） | 中 | qemu_m33 已有新旧两套布局构建先例；若新布局内核与 CM4F port 混用出问题，退路是整体使用经典布局内核（qemu_m33/third_party/FreeRTOS-Kernel 同版本） |
| `svc 0` 启动调度器后 MSP 现场"泄漏"（不弹栈） | 无 | 硬件与 FreeRTOS 设计如此（调度器永不返回），非缺陷；引擎按"入口压栈 + 出口弹栈"对称实现即可 |
| 本机无 cargo（PATH 未配） | 低 | `~/.cargo/bin` 存在，验收脚本需 `export PATH="$HOME/.cargo/bin:$PATH"` |
| Web 搜索不可用（无法二次核验外部链接） | 低 | 所有外部事实均以 GitHub API 直接拉取核实（FreeRTOS-Kernel V11.1.0 port 文件、FreeRTOS demo 目录、QEMU mps2.c/m_helper.c 源码），已落盘 /tmp 备查 |

---

## 5. 验收路径预览（详见 spec-delta.md）

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd ~/.openclaw/workspace/yuleDriverTwin

# 1. 构建 FreeRTOS 最小固件（M4F）
scripts/build_freertos_demo.sh
#    → crates/dtwin-chip/tests/fixtures/build/freertos_demo.elf (+.elf.dat)

# 2. QEMU 黄金输出（mps2-an386, cortex-m4, 同一 ELF）
scripts/run_qemu_golden_freertos.sh
#    → /tmp/freertos_qemu_golden.txt

# 3. dtwin 双跑 + 逐行对比（核心检查行命中 + 全量 diff）
scripts/e2e_freertos.sh

# 4. Rust 集成测试（fixtures 内嵌，CI 可复现）
cargo test --test e2e_freertos

# 5. 全量回归
cargo test   # ≥ 191（新增测试后只增不减）
```

---

*本文件所有代码证据均来自仓库当前 HEAD（0a28393）实测，外部事实来自 GitHub API 与 QEMU 源码（v11.0.2）直读，未凭记忆。*
