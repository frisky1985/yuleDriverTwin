//! ELF 加载器集成测试 — 使用真实固件 yuleASR（快照自
//! `~/.openclaw/workspace/yuleASR/qemu/build/yuleasr_qemu.elf`，armv7e-m hard-float）
//!
//! 快照内容（arm-none-eabi-readelf/objdump 核对）：
//! - 2 个 PT_LOAD：flash { vaddr 0x0, filesz/memsz 0x10b4, R E }、sram { vaddr 0x20000000, filesz 0, memsz 0x8000, RW }
//! - 向量表：初始 SP = 0x20008000，Reset_Handler = 0x845（符号 0x844 | Thumb）

use dtwin_core::loader::{LoadedSegment, Loader};
use dtwin_core::memory::Memory;
use dtwin_core::CpuState;

/// 固件快照（重命名 .elf → .elf.dat 以避开仓库 *.elf 忽略规则）
const FIRMWARE: &[u8] = include_bytes!("fixtures/yuleasr_qemu.elf.dat");

#[test]
fn load_real_yuleasr_firmware() {
    let mut mem = Memory::m4f_default();
    let mut cpu = CpuState::default();

    let summary = Loader::load_elf_bytes(FIRMWARE, &mut mem, &mut cpu).expect("加载 yuleASR 固件");

    // 段映射摘要
    assert_eq!(
        summary.segments,
        vec![
            LoadedSegment {
                vaddr: 0x0000_0000,
                paddr: 0x0000_0000,
                filesz: 0x10b4,
                memsz: 0x10b4,
                flags: 5
            }, // R|X
            LoadedSegment {
                vaddr: 0x2000_0000,
                paddr: 0x2000_0000,
                filesz: 0x0000,
                memsz: 0x8000,
                flags: 6
            }, // R|W
        ]
    );

    // 向量表字面量
    assert_eq!(mem.read_u32(0x0000_0000).unwrap(), 0x2000_8000);
    assert_eq!(mem.read_u32(0x0000_0004).unwrap(), 0x845);

    // 初始 SP / PC（Reset_Handler 清 Thumb 位）
    assert_eq!(summary.initial_sp, 0x2000_8000);
    assert_eq!(summary.entry_pc, 0x844);
    assert_eq!(cpu.msp, 0x2000_8000);
    assert_eq!(cpu.regs[13], 0x2000_8000);
    assert_eq!(cpu.regs[15], 0x844);
    assert_ne!(cpu.xpsr & (1 << 24), 0, "T 位应置 1");

    // text 段首条指令（0x400 处真实字节：80 b4 00 af ...）
    assert_eq!(mem.read_u8(0x0000_0400).unwrap(), 0x80);
    assert_eq!(mem.read_u8(0x0000_0401).unwrap(), 0xB4);

    // Reset_Handler 入口指令（0x844: 0c 48 = ldr r0, [pc, #48]）
    assert_eq!(mem.read_u8(0x0000_0844).unwrap(), 0x0C);
    assert_eq!(mem.read_u8(0x0000_0845).unwrap(), 0x48);

    // SRAM 段（纯 BSS/堆栈）零填充
    assert_eq!(mem.read_u32(0x2000_0000).unwrap(), 0);
    assert_eq!(mem.read_u32(0x2000_7FFC).unwrap(), 0);

    // flash 其余区域保持 0xFF（未覆盖区）
    assert_eq!(mem.read_u8(0x0000_2000).unwrap(), 0xFF);
}

#[test]
fn loader_error_contains_context_for_bad_fixture_copy() {
    // 防御性：若固件快照被替换导致断言失效，先检查文件确实是 ELF
    assert_eq!(&FIRMWARE[0..4], b"\x7fELF", "fixture 必须是 ELF 文件");
    assert_eq!(FIRMWARE[4], 1, "ELFCLASS32");
    assert_eq!(FIRMWARE[5], 1, "小端");
}
