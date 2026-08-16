# OpenSpec: FreeRTOS 镜像支持（Cortex-M 调度）

> **能力目录**: `docs/specs/freertos-image/`
> **版本**: v0.1（基线）
> **状态**: 变更提案阶段（当前生效要求 = 无）
> **最后更新**: 2026-08-16

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

> 本能力为新建立能力，当前基线无生效要求。首个变更
> `changes/2026-08-freertos-image-run/spec-delta.md` 提出全部要求（FRT-*）。
> 变更评审通过并合入后，本表按 delta 的 ADDED 清单回填。

| 需求 ID | 级别 | 摘要 | 来源变更 | 状态 |
|---------|------|------|----------|------|
| （待回填） | — | — | 2026-08-freertos-image-run | ⏳ 提案中 |

## 验收入口

本能力的验收命令与判定标准见
`changes/2026-08-freertos-image-run/spec-delta.md`（验收矩阵 + 可复现命令）。
