//! dtwin — Driver Twin 命令行工具
//!
//! 芯片级精度 ARM Cortex-M 行为模拟器 CLI。
//!
//! 主要命令：
//! - `dtwin load <elf> --chip S32K312`：加载固件 ELF，打印 SP/PC/段摘要
//! - `dtwin run  <elf> --chip S32K312 [--max-instructions N] [--uart-base ADDR]`：
//!   加载并全速执行，UART 输出回显到 stdout，结束时打印引擎统计与退出码
//! - `dtwin create/list-chips/chip ...`：芯片配置管理（原有命令）

use std::path::PathBuf;
use std::process::ExitCode;

use anyhow::{bail, Context};
use clap::{Parser, Subcommand};
use dtwin_chip::memory_from_profile;
use dtwin_chip::S32K312;
use dtwin_core::engine::{Engine, EngineResult};
use dtwin_core::loader::Loader;
use dtwin_core::memory::Memory;
use dtwin_core::nvic::Nvic;
use dtwin_core::uart::{CmsdkUart, Lpuart0Uart};
use dtwin_core::CpuState;

/// Driver Twin — 芯片级精度 ARM Cortex-M 行为模拟器
#[derive(Parser)]
#[command(name = "dtwin", version, about = "Driver Twin: ARM Cortex-M behavior simulator")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// 加载固件 ELF 到内存并初始化 CPU（打印 SP/PC/段摘要）
    Load {
        /// 固件 ELF 路径（ELF32 little-endian ARM EXEC）
        elf: PathBuf,
        /// 芯片型号（如 S32K312）
        #[arg(long, default_value = "S32K312")]
        chip: String,
    },
    /// 加载并全速运行固件（UART 输出回显，结束打印统计与退出码）
    Run {
        /// 固件 ELF 路径（ELF32 little-endian ARM EXEC）
        elf: PathBuf,
        /// 芯片型号（如 S32K312）
        #[arg(long, default_value = "S32K312")]
        chip: String,
        /// 指令数上限（防死循环）
        #[arg(long, default_value_t = 1_000_000)]
        max_instructions: u64,
        /// UART 基地址（默认 S32K312 LPUART0 0x40180000；
        /// yuleASR QEMU 兼容固件实际访问 0x40004000，需显式指定）
        #[arg(long, value_parser = parse_hex_u32)]
        uart_base: Option<u32>,
        /// UART 行为模型：cmsdk（默认，E2E 兼容）/ lpuart0（真实 S32K312 LPUART0）
        #[arg(long, value_parser = clap::value_parser!(UartModel), default_value_t = UartModel::Cmsdk)]
        uart_model: UartModel,
    },
    /// 创建模拟实例
    Create {
        /// 芯片型号（如 STM32F407VG）
        #[arg(long)]
        chip: String,
    },
    /// 列出支持的芯片型号
    ListChips,
    /// 查看芯片配置
    Chip {
        #[command(subcommand)]
        command: ChipCommands,
    },
}

#[derive(Subcommand)]
enum ChipCommands {
    /// 导入 SVD 文件生成芯片配置
    ImportSvd { path: String },
    /// 校验芯片配置文件
    Validate { path: String },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match cli.command {
        Commands::Load { elf, chip } => match cmd_load(&elf, &chip) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("[load] 错误: {e:#}");
                ExitCode::FAILURE
            }
        },
        Commands::Run {
            elf,
            chip,
            max_instructions,
            uart_base,
            uart_model,
        } => match cmd_run(&elf, &chip, max_instructions, uart_base, uart_model) {
            Ok(rc) => ExitCode::from(rc),
            Err(e) => {
                eprintln!("[run] 错误: {e:#}");
                ExitCode::FAILURE
            }
        },
        Commands::Create { chip } => {
            println!("创建模拟实例: {}", chip);
            println!("  内核: 解析芯片型号 -> Cortex-M4F");
            println!("  状态: Ready");
            ExitCode::SUCCESS
        }
        Commands::ListChips => {
            println!("支持的芯片型号:");
            println!("  - S32K312 (Cortex-M4F)");
            println!("  - STM32F407VG (Cortex-M4F)");
            println!("  - STM32F103   (Cortex-M3)");
            ExitCode::SUCCESS
        }
        Commands::Chip { command } => match command {
            ChipCommands::ImportSvd { path } => {
                println!("导入 SVD: {} (待实现)", path);
                ExitCode::SUCCESS
            }
            ChipCommands::Validate { path } => {
                println!("校验配置: {} (待实现)", path);
                ExitCode::SUCCESS
            }
        },
    }
}

/// UART 行为模型选择
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
#[clap(rename_all = "lower")]
enum UartModel {
    /// CMSDK APB UART（QEMU MPS2 兼容，yuleASR QEMU 固件用）
    Cmsdk,
    /// 真实 S32K312 LPUART0（0x40180000，TDRE/TC/RDRF 位定义）
    Lpuart0,
}

impl std::fmt::Display for UartModel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UartModel::Cmsdk => write!(f, "cmsdk"),
            UartModel::Lpuart0 => write!(f, "lpuart0"),
        }
    }
}

/// 解析十六进制/十进制地址（CLI 友好：支持 0x 前缀）
fn parse_hex_u32(s: &str) -> Result<u32, String> {
    let t = s.trim();
    let v = if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16)
    } else {
        t.parse::<u32>()
    };
    v.map_err(|e| format!("非法地址 '{s}': {e}"))
}

/// 按型号构造芯片 profile（当前仅 S32K312 有行为模型）
fn chip_profile(name: &str) -> anyhow::Result<dtwin_chip::ChipProfile> {
    match name {
        "S32K312" => Ok(S32K312::new()),
        other => bail!("未支持的芯片型号: {other}（当前支持 S32K312）"),
    }
}

/// 从芯片 profile 构造内存并挂接 UART 行为模型
fn memory_with_uart(profile: &dtwin_chip::ChipProfile, uart_base: u32, model: UartModel) -> Memory {
    let mut mem = memory_from_profile(profile);
    match model {
        UartModel::Cmsdk => {
            mem.attach_peripheral(CmsdkUart::with_echo(uart_base, true));
        }
        UartModel::Lpuart0 => {
            mem.attach_peripheral(Lpuart0Uart::with_echo(uart_base, true));
        }
    }
    mem
}

/// `dtwin load <elf> --chip S32K312`
fn cmd_load(elf: &PathBuf, chip: &str) -> anyhow::Result<()> {
    let profile = chip_profile(chip)?;
    let mut mem = memory_from_profile(&profile);
    let mut cpu = CpuState::default();
    let summary = Loader::load_elf(elf, &mut mem, &mut cpu)
        .with_context(|| format!("加载固件 {}", elf.display()))?;

    println!("[load] {} -> SP={:#010x} PC={:#010x}", elf.display(), summary.initial_sp, summary.entry_pc);
    println!("[load] 段摘要 ({} 个 PT_LOAD):", summary.segments.len());
    for (i, seg) in summary.segments.iter().enumerate() {
        println!(
            "  [{i}] vaddr={:#010x} paddr={:#010x} filesz={:#x} memsz={:#x} flags={:#x}{}",
            seg.vaddr,
            seg.paddr,
            seg.filesz,
            seg.memsz,
            seg.flags,
            if seg.memsz > seg.filesz {
                " (含 BSS 零填充)"
            } else {
                ""
            }
        );
    }
    Ok(())
}

/// `dtwin run <elf> --chip S32K312 [--max-instructions N] [--uart-base ADDR] [--uart-model cmsdk|lpuart0]`
fn cmd_run(
    elf: &PathBuf,
    chip: &str,
    max_instructions: u64,
    uart_base: Option<u32>,
    uart_model: UartModel,
) -> anyhow::Result<u8> {
    let profile = chip_profile(chip)?;
    // 默认 UART 基址 = S32K312 LPUART0；yuleASR QEMU 兼容固件需 --uart-base 0x40004000
    let base = uart_base.unwrap_or(0x4018_0000);
    let mut mem = memory_with_uart(&profile, base, uart_model);
    let mut cpu = CpuState::default();

    let summary = Loader::load_elf(elf, &mut mem, &mut cpu)
        .with_context(|| format!("加载固件 {}", elf.display()))?;
    println!(
        "[run] 加载 {} -> SP={:#010x} PC={:#010x} (chip={}, uart={:#x}, uart_model={:?})",
        elf.display(),
        summary.initial_sp,
        summary.entry_pc,
        chip,
        base,
        uart_model
    );

    let mut nvic = Nvic::new();
    let mut engine = Engine::new();
    engine.max_instructions = max_instructions;

    println!("[run] 开始执行 (max_instructions={max_instructions})...");
    let result = engine.run(&mut cpu, &mut mem, &mut nvic);

    // 结束统计
    let stats = engine.stats;
    println!();
    println!(
        "[run] 结果: {:?} | instructions={} cycles={} faults={} exceptions={} fpu_frames={} fpu_exc_returns={}",
        result,
        stats.instructions,
        stats.cycles,
        stats.faults,
        stats.exceptions,
        stats.fpu_frames,
        stats.fpu_exc_returns
    );

    // 退出码：Fault → 1；其余（Halted/LimitReached）→ 0
    let code = match result {
        EngineResult::Fault { .. } => 1,
        _ => 0,
    };
    Ok(code)
}
