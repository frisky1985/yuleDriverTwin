//! ELF32 加载器 — 将链接好的 Cortex-M 固件 ELF 载入 `Memory` 并初始化 `CpuState`
//!
//! 仅支持 ELF32 little-endian ARM `EXEC`（链接完成的可执行文件，无需重定位）。
//! 解析流程：
//! 1. 校验 ELF 头（magic / class / endian / machine / type）
//! 2. 遍历程序头，将 `PT_LOAD` 段内容写入 Memory 对应地址（`memsz > filesz` 部分按 BSS 零填充）
//! 3. 从向量表（地址 `0x0`）读初始 SP 与 Reset_Handler 地址，设置 `CpuState`
//!
//! 本模块纯手工解析 ELF32（固定 52 字节头 + 32 字节程序头），无第三方 ELF 依赖，
//! 严格遵守 crate 级 `#![deny(unsafe_code)]`。遇到不支持的段类型/格式如实报错。

use crate::memory::{Memory, MemoryFault};
use crate::CpuState;
use std::fmt;
use std::fs;
use std::path::Path;
use thiserror::Error;

// ---- ELF32 常量 ----
const ELF_MAGIC: [u8; 4] = [0x7F, b'E', b'L', b'F'];
/// ELFCLASS32
const ELFCLASS32: u8 = 1;
/// ELFDATA2LSB（小端）
const ELFDATA2LSB: u8 = 1;
/// EM_ARM
const EM_ARM: u16 = 40;
/// ET_EXEC（链接完成的可执行文件）
const ET_EXEC: u16 = 2;
/// PT_LOAD
const PT_LOAD: u32 = 1;
/// ELF32 固定头长度
const ELF32_EHDR_SIZE: usize = 52;
/// ELF32 程序头长度
const ELF32_PHDR_SIZE: usize = 32;

/// e_ident 偏移
const EI_CLASS: usize = 4;
const EI_DATA: usize = 5;

/// 向量表偏移：Reset_Handler 位于第二个字
const VECTOR_SP_OFFSET: u32 = 0x0000_0000;
const VECTOR_RESET_OFFSET: u32 = 0x0000_0004;
/// xPSR Thumb 位
const XPSR_T: u32 = 1 << 24;

/// ELF 加载错误
#[derive(Debug, Error)]
pub enum LoaderError {
    /// 读取文件失败（IO 错误）
    #[error("读取 ELF 文件失败: {0}")]
    ReadFile(#[from] std::io::Error),
    /// 文件小于最小 ELF 头
    #[error("文件过小 ({0} 字节)，不是合法 ELF")]
    TooSmall(usize),
    /// magic 不符
    #[error("不是 ELF 文件: magic = {0:02x?}")]
    BadMagic([u8; 4]),
    /// 仅支持 ELF32
    #[error("仅支持 ELF32 (EI_CLASS={0})")]
    NotElf32(u8),
    /// 仅支持小端
    #[error("仅支持小端 (EI_DATA={0})")]
    NotLittleEndian(u8),
    /// 仅支持 ARM 架构
    #[error("仅支持 ARM 架构 (e_machine={0:#x})")]
    NotArm(u16),
    /// 仅支持 EXEC 类型（固件为链接好的可执行文件，无重定位需求）
    #[error("仅支持 EXEC 可执行文件 (e_type={0})")]
    NotExec(u16),
    /// 程序头表越界
    #[error("程序头表越界: phoff={0:#x} phentsize={1} phnum={2} file={3} 字节")]
    PhTableOutOfBounds(u32, u16, u16, usize),
    /// 段文件数据越界
    #[error("段 {0} 文件数据越界: offset={1:#x} filesz={2:#x}")]
    SegmentDataOutOfBounds(usize, u32, u32),
    /// 遇到不支持的段类型
    #[error("不支持的段类型 p_type={1:#x} (程序头 {0})")]
    UnsupportedSegment(usize, u32),
    /// 段写入内存失败
    #[error("段写入内存失败 @ {addr:#x}: {desc}")]
    MemoryWrite { addr: u32, desc: String },
    /// 向量表读取失败
    #[error("向量表读取失败 @ {addr:#x}: {desc}")]
    VectorTable { addr: u32, desc: String },
    /// Reset_Handler 不是 Thumb 地址（bit0=0）
    #[error("Reset_Handler 不是 Thumb 地址 (bit0=0): {0:#x}")]
    NotThumb(u32),
}

impl LoaderError {
    fn from_fault(addr: u32, f: MemoryFault) -> Self {
        LoaderError::MemoryWrite {
            addr,
            desc: format!("{}", f),
        }
    }
}

/// `MemoryFault` 的展示（`MemoryFault` 未实现 Display，此处本地格式化）
impl fmt::Display for MemoryFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MemoryFault::BusFault { address } => write!(f, "BusFault @ {address:#x}"),
            MemoryFault::MemManage { address } => write!(f, "MemManage @ {address:#x}"),
            MemoryFault::UnalignedAccess { address } => write!(f, "Unaligned @ {address:#x}"),
            MemoryFault::ReadOnlyWrite { address } => write!(f, "ReadOnlyWrite @ {address:#x}"),
        }
    }
}

/// ELF32 头（仅解析需要的字段；e_type/e_machine 在 parse 时校验，不保留）
struct Elf32Header {
    e_phoff: u32,
    e_phentsize: u16,
    e_phnum: u16,
}

impl Elf32Header {
    fn parse(data: &[u8]) -> Result<Self, LoaderError> {
        if data.len() < ELF32_EHDR_SIZE {
            return Err(LoaderError::TooSmall(data.len()));
        }
        if data[0..4] != ELF_MAGIC {
            let mut magic = [0u8; 4];
            magic.copy_from_slice(&data[0..4]);
            return Err(LoaderError::BadMagic(magic));
        }
        if data[EI_CLASS] != ELFCLASS32 {
            return Err(LoaderError::NotElf32(data[EI_CLASS]));
        }
        if data[EI_DATA] != ELFDATA2LSB {
            return Err(LoaderError::NotLittleEndian(data[EI_DATA]));
        }
        let e_type = u16::from_le_bytes([data[16], data[17]]);
        let e_machine = u16::from_le_bytes([data[18], data[19]]);
        let e_phoff = u32::from_le_bytes([data[28], data[29], data[30], data[31]]);
        let e_phentsize = u16::from_le_bytes([data[42], data[43]]);
        let e_phnum = u16::from_le_bytes([data[44], data[45]]);
        if e_machine != EM_ARM {
            return Err(LoaderError::NotArm(e_machine));
        }
        if e_type != ET_EXEC {
            return Err(LoaderError::NotExec(e_type));
        }
        Ok(Self {
            e_phoff,
            e_phentsize,
            e_phnum,
        })
    }
}

/// ELF32 程序头（仅解析需要的字段）
struct ProgramHeader {
    p_type: u32,
    p_offset: u32,
    p_vaddr: u32,
    p_filesz: u32,
    p_memsz: u32,
    p_flags: u32,
}

impl ProgramHeader {
    /// 从程序头表读取第 `index` 个程序头（ELF32 程序头固定 32 字节）
    fn read(data: &[u8], hdr: &Elf32Header, index: usize) -> Result<Self, LoaderError> {
        let base = hdr.e_phoff as usize + index * hdr.e_phentsize as usize;
        // 程序头表整体范围在 load_elf_bytes 中已校验，此处仅做防御性检查
        if base + ELF32_PHDR_SIZE > data.len() {
            return Err(LoaderError::PhTableOutOfBounds(
                hdr.e_phoff,
                hdr.e_phentsize,
                hdr.e_phnum,
                data.len(),
            ));
        }
        let u32_at = |off: usize| -> u32 {
            u32::from_le_bytes([
                data[base + off],
                data[base + off + 1],
                data[base + off + 2],
                data[base + off + 3],
            ])
        };
        Ok(Self {
            p_type: u32_at(0),
            p_offset: u32_at(4),
            p_vaddr: u32_at(8),
            p_filesz: u32_at(16),
            p_memsz: u32_at(20),
            p_flags: u32_at(24),
        })
    }
}

/// 已加载段摘要
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoadedSegment {
    /// 段虚拟地址（= 物理地址，Cortex-M 统一编址）
    pub vaddr: u32,
    /// 文件中数据长度
    pub filesz: u32,
    /// 内存中占用长度（> filesz 部分零填充，如 BSS）
    pub memsz: u32,
    /// 段权限标志（PF_X=1, PF_W=2, PF_R=4）
    pub flags: u32,
}

/// ELF 加载结果摘要
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoadSummary {
    /// 复位后 PC（Reset_Handler，已清 Thumb 位）
    pub entry_pc: u32,
    /// 初始主栈指针（向量表第一个字）
    pub initial_sp: u32,
    /// 已加载的 LOAD 段列表（按程序头顺序）
    pub segments: Vec<LoadedSegment>,
}

/// ELF32 加载器
pub struct Loader;

impl Loader {
    /// 从文件加载 ELF 固件到内存，并初始化 CPU 状态
    ///
    /// - 所有 `PT_LOAD` 段按 `p_vaddr` 写入 `Memory`（Flash 只读区使用"烧录"语义直写）
    /// - 向量表第一个字 → 初始 SP（`msp` 与 `regs[13]`），第二个字 → PC（`regs[15]`，清 Thumb 位）
    /// - xPSR T 位置 1（Cortex-M 仅支持 Thumb）
    pub fn load_elf(
        path: &Path,
        mem: &mut Memory,
        cpu: &mut CpuState,
    ) -> Result<LoadSummary, LoaderError> {
        let data = fs::read(path)?;
        Self::load_elf_bytes(&data, mem, cpu)
    }

    /// 从字节切片加载 ELF 固件（供测试及内存场景直接使用）
    pub fn load_elf_bytes(
        data: &[u8],
        mem: &mut Memory,
        cpu: &mut CpuState,
    ) -> Result<LoadSummary, LoaderError> {
        let hdr = Elf32Header::parse(data)?;

        // 程序头表整体范围校验
        let stride = hdr.e_phentsize as usize;
        if stride < ELF32_PHDR_SIZE {
            return Err(LoaderError::PhTableOutOfBounds(
                hdr.e_phoff,
                hdr.e_phentsize,
                hdr.e_phnum,
                data.len(),
            ));
        }
        let table_end = (hdr.e_phoff as usize)
            .checked_add(hdr.e_phnum as usize * stride)
            .ok_or(LoaderError::PhTableOutOfBounds(
                hdr.e_phoff,
                hdr.e_phentsize,
                hdr.e_phnum,
                data.len(),
            ))?;
        if table_end > data.len() {
            return Err(LoaderError::PhTableOutOfBounds(
                hdr.e_phoff,
                hdr.e_phentsize,
                hdr.e_phnum,
                data.len(),
            ));
        }

        // 遍历程序头：仅支持 PT_LOAD，其余段类型如实报错
        let mut segments = Vec::new();
        for i in 0..hdr.e_phnum as usize {
            let ph = ProgramHeader::read(data, &hdr, i)?;
            match ph.p_type {
                PT_LOAD => {
                    // 段文件数据范围校验
                    let start = ph.p_offset as usize;
                    let end = start.checked_add(ph.p_filesz as usize).ok_or(
                        LoaderError::SegmentDataOutOfBounds(i, ph.p_offset, ph.p_filesz),
                    )?;
                    if end > data.len() {
                        return Err(LoaderError::SegmentDataOutOfBounds(
                            i,
                            ph.p_offset,
                            ph.p_filesz,
                        ));
                    }
                    // 段内存范围防溢出（memsz 决定最终占用）
                    ph.p_vaddr
                        .checked_add(ph.p_memsz)
                        .ok_or(LoaderError::MemoryWrite {
                            addr: ph.p_vaddr,
                            desc: "段地址溢出 u32".into(),
                        })?;

                    if ph.p_filesz > 0 {
                        mem.load_bytes(ph.p_vaddr, &data[start..end])
                            .map_err(|f| LoaderError::from_fault(ph.p_vaddr, f))?;
                    }
                    // BSS/堆栈零填充：memsz > filesz 部分（分块写入，避免超大临时分配）
                    if ph.p_memsz > ph.p_filesz {
                        let mut addr = ph.p_vaddr.wrapping_add(ph.p_filesz);
                        let mut remaining = (ph.p_memsz - ph.p_filesz) as usize;
                        let zeros = [0u8; 256];
                        while remaining > 0 {
                            let n = remaining.min(zeros.len());
                            mem.load_bytes(addr, &zeros[..n])
                                .map_err(|f| LoaderError::from_fault(addr, f))?;
                            addr = addr.wrapping_add(n as u32);
                            remaining -= n;
                        }
                    }
                    segments.push(LoadedSegment {
                        vaddr: ph.p_vaddr,
                        filesz: ph.p_filesz,
                        memsz: ph.p_memsz,
                        flags: ph.p_flags,
                    });
                }
                other => return Err(LoaderError::UnsupportedSegment(i, other)),
            }
        }

        // 向量表 → 初始 SP / Reset_Handler（Thumb 地址，bit0=1）
        let initial_sp = mem
            .read_u32(VECTOR_SP_OFFSET)
            .map_err(|f| LoaderError::VectorTable {
                addr: VECTOR_SP_OFFSET,
                desc: f.to_string(),
            })?;
        let reset = mem
            .read_u32(VECTOR_RESET_OFFSET)
            .map_err(|f| LoaderError::VectorTable {
                addr: VECTOR_RESET_OFFSET,
                desc: f.to_string(),
            })?;
        if reset & 1 == 0 {
            return Err(LoaderError::NotThumb(reset));
        }
        let entry_pc = reset & !1;

        cpu.msp = initial_sp;
        cpu.regs[13] = initial_sp;
        cpu.regs[15] = entry_pc;
        cpu.xpsr |= XPSR_T; // Cortex-M 强制 Thumb

        Ok(LoadSummary {
            entry_pc,
            initial_sp,
            segments,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::Memory;
    use crate::CpuState;

    /// 手工构造最小合法 ELF32：flash LOAD 段（0x0，含向量表）+ sram LOAD 段（0x20000000，纯 BSS）
    fn synthetic_elf() -> Vec<u8> {
        let mut elf = vec![0u8; ELF32_EHDR_SIZE + 2 * ELF32_PHDR_SIZE];
        // ELF 头
        elf[0..4].copy_from_slice(&ELF_MAGIC);
        elf[EI_CLASS] = ELFCLASS32;
        elf[EI_DATA] = ELFDATA2LSB;
        elf[6] = 1; // EI_VERSION
        elf[16..18].copy_from_slice(&ET_EXEC.to_le_bytes());
        elf[18..20].copy_from_slice(&EM_ARM.to_le_bytes());
        elf[24..28].copy_from_slice(&0x400u32.to_le_bytes()); // e_entry（Cortex-M 不使用）
        elf[28..32].copy_from_slice(&(ELF32_EHDR_SIZE as u32).to_le_bytes()); // e_phoff
        elf[42..44].copy_from_slice(&(ELF32_PHDR_SIZE as u16).to_le_bytes()); // e_phentsize
        elf[44..46].copy_from_slice(&2u16.to_le_bytes()); // e_phnum

        // 程序头 0：flash LOAD，文件偏移 0x1000，vaddr 0x0，filesz=memsz=0x10b4，PF_R|PF_X
        let ph0 = ELF32_EHDR_SIZE;
        elf[ph0..ph0 + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
        elf[ph0 + 4..ph0 + 8].copy_from_slice(&0x1000u32.to_le_bytes());
        elf[ph0 + 8..ph0 + 12].copy_from_slice(&0x0000_0000u32.to_le_bytes());
        elf[ph0 + 16..ph0 + 20].copy_from_slice(&0x10b4u32.to_le_bytes());
        elf[ph0 + 20..ph0 + 24].copy_from_slice(&0x10b4u32.to_le_bytes());
        elf[ph0 + 24..ph0 + 28].copy_from_slice(&5u32.to_le_bytes()); // R|X

        // 程序头 1：sram LOAD（纯 BSS），vaddr 0x20000000，filesz=0，memsz=0x8000，PF_R|PF_W
        let ph1 = ELF32_EHDR_SIZE + ELF32_PHDR_SIZE;
        elf[ph1..ph1 + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
        elf[ph1 + 4..ph1 + 8].copy_from_slice(&0x1000u32.to_le_bytes());
        elf[ph1 + 8..ph1 + 12].copy_from_slice(&0x2000_0000u32.to_le_bytes());
        elf[ph1 + 16..ph1 + 20].copy_from_slice(&0u32.to_le_bytes());
        elf[ph1 + 20..ph1 + 24].copy_from_slice(&0x8000u32.to_le_bytes());
        elf[ph1 + 24..ph1 + 28].copy_from_slice(&6u32.to_le_bytes()); // R|W

        // 文件载荷（偏移 0x1000 起）：向量表 + 少量代码
        elf.resize(0x1000 + 0x10b4, 0);
        elf[0x1000..0x1004].copy_from_slice(&0x2000_8000u32.to_le_bytes()); // 初始 SP
        elf[0x1004..0x1008].copy_from_slice(&0x845u32.to_le_bytes()); // Reset_Handler | Thumb
        elf[0x1400] = 0x0c; // 0x400 处首条指令低字节（与真实固件 Reset_Handler 无关，仅占位）
        elf
    }

    #[test]
    fn load_synthetic_elf_sets_state_and_maps_segments() {
        let elf = synthetic_elf();
        let mut mem = Memory::m4f_default();
        let mut cpu = CpuState::default();
        let summary = Loader::load_elf_bytes(&elf, &mut mem, &mut cpu).expect("加载合成 ELF");

        // 段映射摘要
        assert_eq!(
            summary.segments,
            vec![
                LoadedSegment {
                    vaddr: 0x0000_0000,
                    filesz: 0x10b4,
                    memsz: 0x10b4,
                    flags: 5
                },
                LoadedSegment {
                    vaddr: 0x2000_0000,
                    filesz: 0x0000,
                    memsz: 0x8000,
                    flags: 6
                },
            ]
        );
        // 向量表字节已写入 flash
        assert_eq!(mem.read_u32(0x0000_0000).unwrap(), 0x2000_8000);
        assert_eq!(mem.read_u32(0x0000_0004).unwrap(), 0x845);
        // BSS 零填充
        assert_eq!(mem.read_u32(0x2000_0000).unwrap(), 0);
        assert_eq!(mem.read_u32(0x2000_7FFC).unwrap(), 0);
        // CPU 状态：SP / PC / T 位
        assert_eq!(summary.initial_sp, 0x2000_8000);
        assert_eq!(summary.entry_pc, 0x844);
        assert_eq!(cpu.msp, 0x2000_8000);
        assert_eq!(cpu.regs[13], 0x2000_8000);
        assert_eq!(cpu.regs[15], 0x844);
        assert_ne!(cpu.xpsr & XPSR_T, 0);
    }

    #[test]
    fn load_rejects_non_elf() {
        let mut mem = Memory::test_ram();
        let mut cpu = CpuState::default();
        // 长度 >= 52 字节但 magic 不符
        let r = Loader::load_elf_bytes(&[0u8; 64], &mut mem, &mut cpu);
        assert!(matches!(r, Err(LoaderError::BadMagic(_))));
    }

    #[test]
    fn load_rejects_non_arm_and_non_exec() {
        let mut mem = Memory::test_ram();
        let mut cpu = CpuState::default();
        // 合法头但 machine = x86
        let mut elf = synthetic_elf();
        elf[18..20].copy_from_slice(&3u16.to_le_bytes());
        assert!(matches!(
            Loader::load_elf_bytes(&elf, &mut mem, &mut cpu),
            Err(LoaderError::NotArm(3))
        ));
        // 合法头但 type = DYN
        let elf = synthetic_elf();
        let mut dyn_elf = elf.clone();
        dyn_elf[16..18].copy_from_slice(&3u16.to_le_bytes());
        assert!(matches!(
            Loader::load_elf_bytes(&dyn_elf, &mut mem, &mut cpu),
            Err(LoaderError::NotExec(3))
        ));
    }

    #[test]
    fn load_rejects_unsupported_segment_type() {
        let mut elf = synthetic_elf();
        let ph1 = ELF32_EHDR_SIZE + ELF32_PHDR_SIZE;
        elf[ph1..ph1 + 4].copy_from_slice(&0x7000_0001u32.to_le_bytes()); // PT_ARM_EXIDX
                                                                          // 用 m4f_default（flash 0x80000）保证段 0 能先正常加载，错误出自段 1 的类型检查
        let mut mem = Memory::m4f_default();
        let mut cpu = CpuState::default();
        assert!(matches!(
            Loader::load_elf_bytes(&elf, &mut mem, &mut cpu),
            Err(LoaderError::UnsupportedSegment(1, 0x7000_0001))
        ));
    }

    #[test]
    fn load_rejects_non_thumb_reset() {
        let mut elf = synthetic_elf();
        elf[0x1004..0x1008].copy_from_slice(&0x844u32.to_le_bytes()); // bit0 = 0
        let mut mem = Memory::m4f_default();
        let mut cpu = CpuState::default();
        assert!(matches!(
            Loader::load_elf_bytes(&elf, &mut mem, &mut cpu),
            Err(LoaderError::NotThumb(0x844))
        ));
    }

    #[test]
    fn load_rejects_truncated_ph_table() {
        let mut elf = synthetic_elf();
        elf.truncate(ELF32_EHDR_SIZE + 10); // 程序头被截断
        let mut mem = Memory::test_ram();
        let mut cpu = CpuState::default();
        assert!(matches!(
            Loader::load_elf_bytes(&elf, &mut mem, &mut cpu),
            Err(LoaderError::PhTableOutOfBounds(_, _, _, _))
        ));
    }
}
