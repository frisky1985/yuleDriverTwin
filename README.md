# 🚗 yuleDriverTwin — 驱动孪生（Driver Twin）

> 芯片级精度的 ARM Cortex-M 行为模拟器：无硬件环境下验证嵌入式驱动代码

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## 🎯 定位

解决嵌入式驱动开发"硬件依赖"困境——驱动代码必须烧录到目标芯片后才能验证正确性，硬件不到位则开发停滞（硬件等待占周期 30%–50%）。

**驱动孪生** = 专为驱动代码验证设计的芯片级行为模拟器：无硬件即可完成寄存器读写验证、外设通信协议模拟、中断处理逻辑测试、内存边界检查，将硬件等待时间从开发周期中消除。

## ✨ 核心能力

> 状态标注（2026-08-16）：✅ 已实现 / 🚧 部分实现 / ⬜ 规划中（README 与实际对齐，避免夸大声明）

| 模块 | 能力 | 状态 |
|------|------|------|
| **内核引擎** | ARMv7E-M（M4/M4F）指令集精确模拟：整数 + DSP（SSAT/USAT/SIMD/乘加）+ FPU（VFPv4 单精度 + F64 骨架） | ✅ 182/182 单测全绿 |
| **寄存器模型** | 按位/字节/半字/字读写、位域访问、副作用建模（W1C）、事务日志 | ✅ |
| **内存模型** | 标准 Cortex-M 映射 + MPU + Flash 擦写行为 + watchpoint + 非对齐检测 | ✅ |
| **外设模型** | UART（CMSDK/LPUART0 双模型，TX 输出捕获）；GPIO/SPI/I2C/ADC 等 🚧 骨架 | 🚧 |
| **NVIC 中断** | 优先级模型、中断挂起/使能、异常返回映射；嵌套/惰性压栈 🚧 骨架 | 🚧 |
| **GDB 集成** | GDB RSP（localhost:3333）⬜ 未实现（dtwin-gdb crate 骨架） | ⬜ |
| **芯片配置** | S32K312 profile（内存映射 + 10 外设基址）；STM32F407VG 待补 | ✅（S32K312） |
| **固件加载** | ELF32 loader（armv7e-m 段映射/SP/PC 初始化） | ✅ |
| **CLI** | `dtwin load/run/create/list-chips`，`--uart-model cmsdk\|lpuart0` | ✅ |
| **E2E 验证** | 跑 yuleASR QEMU 固件，输出与 QEMU 黄金逐行一致（27/27 检查行） | ✅ |
| **Web 界面 / CI/CD** | ⬜ 规划中 | ⬜ |
| **测试框架** | Rust 单测（指令级 golden + 芯片级 E2E）；C 测试 API 🚧 未实现 | 🚧 |

## 🚀 快速开始

```bash
# 创建模拟实例
dtwin create --chip STM32F407VG

# 加载固件并运行
dtwin load firmware.elf
dtwin run

# 调试
dtwin debug --gdb        # 或 arm-none-eabi-gdb → target remote localhost:3333

# 寄存器/内存操作
dtwin reg read GPIOA MODER
dtwin mem read 0x20000000 64

# 运行测试
dtwin test run --coverage
```

## 📦 安装

> 待发布：CLI 二进制（macOS/Ubuntu/Windows）+ Docker 镜像

## 🗂 工程结构

```
├── src/                          # 核心源码
│   ├── engine/                   # 内核模拟器（指令集/CPU 状态/双栈）
│   ├── register/                 # 寄存器模型（位域/副作用/事务日志）
│   ├── memory/                   # 内存模型（MPU/Flash/watchpoint）
│   ├── peripheral/               # 外设行为模型（GPIO/UART/Timer/SPI/I2C/ADC...）
│   ├── nvic/                     # 中断与异常系统
│   ├── gdb/                      # GDB RSP 调试集成
│   ├── chip/                     # 芯片配置系统（TOML/SVD/overlay）
│   └── cli/                      # CLI 工具链
├── configs/chips/                # 芯片配置文件（STM32F4/F1/H7/L4...）
├── tests/                        # 测试用例
├── docs/
│   ├── requirements/             # PRD 与需求文档
│   ├── specs/                    # 技术设计文档
│   └── changes/                  # 需求变更记录
├── scripts/                      # 工具脚本
├── output/                       # 构建/报告输出（gitignore）
└── .ai-rules.md                  # AI 辅助开发规范（项目级系统提示词）
```

## 🛣 路线图

| 阶段 | 内容 | 预估 |
|------|------|------|
| **MVP** | Cortex-M3+M4 内核、寄存器/内存/GPIO/UART/Timer、NVIC、GDB、芯片配置、CLI | ~4 个月 |
| **V1.1** | M0+/M4F FPU、SPI/I2C/ADC、Web 界面、测试框架、CI/CD | ~3 个月 |
| **V2.0** | M33+TrustZone、DMA、反向调试、社区平台、企业版 | ~3 个月 |

## 📄 文档

- [PRD：驱动孪生（Driver Twin）](docs/requirements/PRD_驱动孪生_DriverTwin.md)
- [AI 辅助开发规范](.ai-rules.md)

## 📜 License

MIT © 2026 Shanghai Yule Electronics Technology Co., Ltd.
