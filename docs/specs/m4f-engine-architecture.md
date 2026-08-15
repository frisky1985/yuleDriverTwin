# M4F 内核引擎架构规划（Cortex-M4F / ARMv7E-M）

> 日期: 2026-08-14 | 决策: 明天华 | 目标: yuleDriverTwin MVP 内核引擎
> 芯片锚点: **STM32F407VG（Cortex-M4F, 168MHz）** — PRD P0 芯片

## 1. 目标

实现 ARMv7E-M 指令集精确模拟，支持：
- **Thumb-2 整数指令集**（M3 基础，约 130+ 条）
- **DSP 扩展**（饱和运算/SIMD 双半字，约 40 条）
- **FPU**（单/双精度浮点，约 55 条 + 寄存器文件 S0-S31 + FPSCR）

验收锚点：`dtwin create --chip STM32F407VG` → 加载 FPU 浮点驱动 ELF → 寄存器/内存/中断行为与真实芯片一致。

## 2. 架构分层

```
┌─────────────────────────────────────────────┐
│ dtwin-cli（CLI 入口）                        │
├─────────────────────────────────────────────┤
│ dtwin-core                                  │
│  ├── engine/      指令解码 + 执行循环        │
│  │   ├── decode.rs    Thumb-2 指令解码       │
│  │   ├── exec.rs      指令执行               │
│  │   ├── fpu.rs       FPU 寄存器 + 浮点指令  │
│  │   └── dsp.rs       DSP 扩展指令           │
│  ├── register.rs   寄存器模型（位域/副作用） │
│  ├── memory.rs     内存模型（MPU/Flash/WP）  │
│  ├── nvic.rs       中断系统（优先级/嵌套）   │
│  └── peripheral.rs 外设行为模型              │
├─────────────────────────────────────────────┤
│ dtwin-chip（STM32F407VG TOML profile + SVD） │
├─────────────────────────────────────────────┤
│ dtwin-gdb（GDB RSP 调试）                    │
└─────────────────────────────────────────────┘
```

## 3. CPU 状态模型（ARMv7E-M）

| 状态 | 说明 |
|------|------|
| R0-R15 | 通用寄存器（R13=SP 别名 MSP/PSP） |
| xPSR | APSR + IPSR + EPSR（含 Q 标志 — DSP 饱和） |
| MSP/PSP | 主栈/进程栈指针 |
| PRIMASK/FAULTMASK/BASEPRI/CONTROL | 特殊寄存器 |
| **S0-S31 + FPSCR** | **FPU 寄存器文件（单精度 32 个）+ 浮点状态** |
| FPCCR | 浮点上下文控制（惰性压栈） |

## 4. 指令集实现顺序（小步拆解）

### Phase 1 — 核心整数（M3 基础，PRD 6 周的主干）
1. 数据传送：MOV/MOVW/MOVT/LDR/STR（含立即数/寄存器寻址）
2. 算术逻辑：ADD/SUB/AND/ORR/EOR/BIC/MUL/UDIV/SDIV
3. 移位循环：LSL/LSR/ASR/ROR/RRX
4. 分支：B/BX/BLX/CBZ/CBNZ/TBB/TBH
5. 比较测试：CMP/CMN/TST/TEQ
6. 压栈出栈：PUSH/POP（含多寄存器）

### Phase 2 — 内存与异常
7. 加载存储多寄存器：LDM/STM
8. 异常处理：异常入口/出口、向量表、NVIC 集成
9. 特权/非特权切换：MRS/MSR、CONTROL 操作

### Phase 3 — DSP 扩展（ARMv7E-M 增量）
10. 饱和运算：SSAT/USAT/QADD/QSUB/QSADD/QSUSB
11. SIMD 双半字：SADD16/SMUAD/SMLAD/SMLALD/SDIV 等
12. 乘加融合：MLA/MLS/SMLAL/UMULL/UMLAL

### Phase 4 — FPU（M4F 特性）
13. FPU 寄存器访问：VMOV（GPR↔S）/VPUSH/VPOP
14. 浮点运算：VADD/VSUB/VMUL/VDIV/VSQRT/VMLA/VNMLA
15. 转换指令：VCVT（浮点↔整数，舍入模式）
16. 比较与条件：VCMP/VMRS/VMSR + FPSCR 标志
17. 双精度扩展：VADD.F64/VLDRD/VSTRD（D0-D15）

## 5. 设计约束

- **#![deny(unsafe_code)]**：全部安全代码，禁止 unsafe
- **性能基线**：解释器模式 ≥ 10 MIPS（M4F@168MHz 全速）；后续热点用 Cranelift JIT
- **可测性**：每条指令有独立测试（指令级 golden 测试 + 芯片级 E2E）
- **命名**：按 .ai-rules.md AUTOSAR 命名（模块前缀 + 语义）

## 6. 验收标准

> 状态更新（2026-08-16）：M4F-01/02/03 已达成；M4F-04/05 未达成（异常嵌套为骨架、GDB 集成未实现）。详见下方状态列。

| 编号 | 场景 | 预期 | 状态 |
|------|------|------|------|
| M4F-01 | `dtwin create --chip STM32F407VG` | 实例创建，内核=Cortex-M4F，168MHz | ✅ 达成（S32K312 profile 亦可用） |
| M4F-02 | 加载 FPU 驱动 ELF（含 VADD/VCVT） | FPU 指令正确执行，FPSCR 更新 | ✅ 达成（Phase 4 + E2E 固件） |
| M4F-03 | DSP 饱和指令（SSAT/QADD） | 饱和行为与硬件一致，Q 标志置位 | ✅ 达成（Phase 3 + golden 测试） |
| M4F-04 | 中断嵌套（FPU 上下文切换） | 惰性压栈正确，浮点状态保存/恢复 | ❌ 未达成（异常入口/出口为骨架，FPU 上下文切换未实现） |
| M4F-05 | GDB 连接查看 S0-S31 | 浮点寄存器可读写 | ❌ 未达成（dtwin-gdb crate 仅骨架，GDB RSP 未实现） |

### 6.1 E2E 覆盖边界（2026-08-16 记录）

现有 E2E 固件（yuleASR QEMU 兼容固件 + e2e_driver_stress）覆盖：
- 16/32 位 LDR/STR 家族（含 LDRSH/LDRSB 符号扩展、LDRD/STRD 双字）
- DSP（SSAT/USAT/QADD/SADD16）、FPU（VADD/VMUL/VCVT）、IT 块、MRS/MSR、TST.W 位测试、移位链
- UART 输出与 QEMU 黄金逐行一致（27/27 检查行）

未覆盖：异常/中断嵌套路径、FPU 上下文切换（M4F-04 依赖）；GDB 调试链路（M4F-05）。

### 6.2 参考分歧记录（2026-08-16）

- **SADD16 GE 标志**：与 QEMU `op_addsub.h` 交叉核对一致（小克 2620e2e 已按 QEMU 语义实现）
- **MRS SYSm 表**：M-profile 0x01/0x02 = IAPSR/EAPSR（非 A-profile APSR_nzcvqg，P1-7 修复）
- **SUB 编码 op4**：实测 SUB=0xD（0xB 为 SBC，与部分文档记忆相反，以汇编器为准）
