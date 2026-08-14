# 🚗 yuleDriverTwin — 驱动孪生（Driver Twin）

> 芯片级精度的 ARM Cortex-M 行为模拟器：无硬件环境下验证嵌入式驱动代码

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## 🎯 定位

解决嵌入式驱动开发"硬件依赖"困境——驱动代码必须烧录到目标芯片后才能验证正确性，硬件不到位则开发停滞（硬件等待占周期 30%–50%）。

**驱动孪生** = 专为驱动代码验证设计的芯片级行为模拟器：无硬件即可完成寄存器读写验证、外设通信协议模拟、中断处理逻辑测试、内存边界检查，将硬件等待时间从开发周期中消除。

## ✨ 核心能力

| 模块 | 能力 |
|------|------|
| **内核引擎** | ARMv6-M / ARMv7-M / ARMv7E-M / ARMv8-M（M0/M3/M4/M4F/M33）指令集精确模拟 |
| **寄存器模型** | 按位/字节/半字/字读写、位域访问、副作用建模（W1C/toggle/auto-clear）、事务日志 |
| **内存模型** | 标准 Cortex-M 映射 + 芯片特有区域（CCM/DTCM/ITCM）+ MPU + Flash 擦写行为 + watchpoint |
| **外设模型** | GPIO/UART/Timer（P0）、SPI/I2C/ADC（P1）、DMA/RTC/CAN（P2），外设互联 + 录制回放 |
| **NVIC 中断** | 完整优先级模型（-3~255）、嵌套中断、中断延迟、向量表重映射、中断追踪 |
| **GDB 集成** | GDB RSP 协议（localhost:3333），兼容 arm-none-eabi-gdb + VS Code Cortex-Debug |
| **芯片配置** | TOML 配置文件 + SVD 一键导入 + overlay 继承 + 社区共享 |
| **Web 界面** | 仪表盘/实例监控/芯片配置管理/测试报告（WebSocket 实时 <500ms） |
| **CI/CD** | Docker 镜像 + GitHub Action/GitLab CI 模板 + JUnit/TAP 报告 + 结构化 JSON 输出 |
| **测试框架** | C 语言测试 API（DTWIN_ASSERT_* / DTWIN_INJECT_*）+ 覆盖率统计 |

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
