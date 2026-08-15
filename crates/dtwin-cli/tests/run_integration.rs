//! CLI 集成测试 — `dtwin run` 端到端（E3）
//!
//! 加载 fixtures 里的最小 ELF 固件，断言 UART 输出包含 `PASS` 行：
//! - `minimal_lpuart0.elf.dat`：真实 S32K312 LPUART0（0x40180000，STAT/CTRL/DATA
//!   真实位定义，轮询 STAT.TDRE 后写 DATA）→ `--uart-model lpuart0`
//! - `minimal_cmsdk.elf.dat`：CMSDK APB UART（0x40004000，QEMU 兼容）→ 默认 cmsdk
//!
//! 固件用 arm-none-eabi-as 汇编（16 位指令集，规避 32-bit LDR/STR 未建模），
//! 向量表位于 0x0（SP=0x20080000，Reset=_start|1），输出 "PASS\n" 后 while(1) 空转。

use std::process::Command;

/// dtwin 可执行文件路径（cargo 注入）
fn dtwin_bin() -> &'static str {
    env!("CARGO_BIN_EXE_dtwin")
}

const FIXTURES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/");

#[test]
fn run_minimal_lpuart0_prints_pass() {
    let elf = format!("{FIXTURES}minimal_lpuart0.elf.dat");
    let out = Command::new(dtwin_bin())
        .args(["run", &elf, "--uart-model", "lpuart0", "--max-instructions", "300"])
        .output()
        .expect("dtwin run 应成功启动");
    assert!(out.status.success(), "退出码应成功: {:?}", out.status);
    let text = String::from_utf8_lossy(&out.stdout);
    assert!(
        text.contains("PASS"),
        "UART 输出应包含 PASS 行（真实 LPUART0 模型）:\n{text}"
    );
    assert!(
        text.contains("LimitReached"),
        "固件 while(1) 空转应达指令上限:\n{text}"
    );
    assert!(
        text.contains("faults=0"),
        "执行不应产生故障:\n{text}"
    );
}

#[test]
fn run_minimal_cmsdk_prints_pass() {
    let elf = format!("{FIXTURES}minimal_cmsdk.elf.dat");
    let out = Command::new(dtwin_bin())
        .args([
            "run",
            &elf,
            "--uart-base",
            "0x40004000",
            "--max-instructions",
            "300",
        ])
        .output()
        .expect("dtwin run 应成功启动");
    assert!(out.status.success(), "退出码应成功: {:?}", out.status);
    let text = String::from_utf8_lossy(&out.stdout);
    assert!(
        text.contains("PASS"),
        "UART 输出应包含 PASS 行（默认 cmsdk 模型）:\n{text}"
    );
    assert!(
        text.contains("LimitReached"),
        "固件 while(1) 空转应达指令上限:\n{text}"
    );
}

/// lpuart0 模型下 TE 未使能时写 DATA 不产生输出（真实语义）：用第二个 fixture 变体验证
/// —— 直接构造：CLI 不支持注入缺失 TE，故用 lpuart0 模型跑 cmsdk fixture（访问不同
/// 基址，模型不捕获 → 输出无 PASS），验证模型隔离性
#[test]
fn run_lpuart0_model_ignores_cmsdk_fixture() {
    let elf = format!("{FIXTURES}minimal_cmsdk.elf.dat");
    let out = Command::new(dtwin_bin())
        .args(["run", &elf, "--uart-model", "lpuart0", "--max-instructions", "300"])
        .output()
        .expect("dtwin run 应成功启动");
    let text = String::from_utf8_lossy(&out.stdout);
    assert!(
        !text.contains("PASS"),
        "lpuart0 模型不响应 0x40004000 的 CMSDK 访问:\n{text}"
    );
}
