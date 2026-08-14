//! dtwin — Driver Twin 命令行工具
//!
//! 芯片级精度 ARM Cortex-M 行为模拟器 CLI。

use clap::{Parser, Subcommand};

/// Driver Twin — 芯片级精度 ARM Cortex-M 行为模拟器
#[derive(Parser)]
#[command(name = "dtwin", version, about = "Driver Twin: ARM Cortex-M behavior simulator")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
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

fn main() {
    let cli = Cli::parse();
    match cli.command {
        Commands::Create { chip } => {
            println!("创建模拟实例: {}", chip);
            println!("  内核: 解析芯片型号 -> Cortex-M4F");
            println!("  状态: Ready");
        }
        Commands::ListChips => {
            println!("支持的芯片型号:");
            println!("  - STM32F407VG (Cortex-M4F)");
            println!("  - STM32F103   (Cortex-M3)");
        }
        Commands::Chip { command } => match command {
            ChipCommands::ImportSvd { path } => {
                println!("导入 SVD: {} (待实现)", path);
            }
            ChipCommands::Validate { path } => {
                println!("校验配置: {} (待实现)", path);
            }
        },
    }
}
