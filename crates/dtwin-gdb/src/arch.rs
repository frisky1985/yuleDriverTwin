//! Cortex-M4F 自定义架构定义：寄存器文件、寄存器 ID 与 target.xml。
//!
//! 设计要点：
//! - 寄存器布局严格对齐 GDB 标准 M-profile 特性：
//!   `org.gnu.gdb.arm.m-profile`（r0-r15 + xpsr）、
//!   `org.gnu.gdb.arm.m-system`（msp/psp/primask/basepri/faultmask/control）、
//!   `org.gnu.gdb.arm.vfp`（d0-d15 + fpscr，GDB 据此合成 s0-s31）。
//! - `g` 报文顺序与 XML 中 `<reg>` 出现顺序一致，中间不插入 padding 寄存器：
//!   r0-r12 → sp → lr → pc → xpsr → msp..control → d0-d15 → fpscr。
//! - xPSR 使用 M-profile 原生位布局（T 位在 bit24），与 GDB M-profile 期望一致，
//!   无需像 Armv4t CPSR 那样做 bit5↔bit24 转换。

use core::num::NonZeroUsize;

use gdbstub::arch::{Arch, RegId, Registers};
use gdbstub_arch::arm::ArmBreakpointKind;

/// Cortex-M4F 目标描述 XML。
///
/// 显式标注关键寄存器的 `regnum` 以匹配 GDB 标准编号：
/// xpsr=25、msp=26、psp=27、primask=28、basepri=29、faultmask=30、control=31、
/// d0-d15=32-47、fpscr=48。r0-r15 不标注 regnum，由 GDB 顺序分配 0-15。
pub const CORTEXM4F_TARGET_XML: &str = r#"<target version="1.0">
  <architecture>arm</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32" type="uint32"/>
    <reg name="r1" bitsize="32" type="uint32"/>
    <reg name="r2" bitsize="32" type="uint32"/>
    <reg name="r3" bitsize="32" type="uint32"/>
    <reg name="r4" bitsize="32" type="uint32"/>
    <reg name="r5" bitsize="32" type="uint32"/>
    <reg name="r6" bitsize="32" type="uint32"/>
    <reg name="r7" bitsize="32" type="uint32"/>
    <reg name="r8" bitsize="32" type="uint32"/>
    <reg name="r9" bitsize="32" type="uint32"/>
    <reg name="r10" bitsize="32" type="uint32"/>
    <reg name="r11" bitsize="32" type="uint32"/>
    <reg name="r12" bitsize="32" type="uint32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32" type="uint32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32" regnum="25" type="uint32"/>
  </feature>
  <feature name="org.gnu.gdb.arm.m-system">
    <reg name="msp" bitsize="32" regnum="26" type="data_ptr"/>
    <reg name="psp" bitsize="32" regnum="27" type="data_ptr"/>
    <reg name="primask" bitsize="32" regnum="28" type="uint32"/>
    <reg name="basepri" bitsize="32" regnum="29" type="uint32"/>
    <reg name="faultmask" bitsize="32" regnum="30" type="uint32"/>
    <reg name="control" bitsize="32" regnum="31" type="uint32"/>
  </feature>
  <feature name="org.gnu.gdb.arm.vfp">
    <reg name="d0" bitsize="64" regnum="32" type="ieee_double"/>
    <reg name="d1" bitsize="64" regnum="33" type="ieee_double"/>
    <reg name="d2" bitsize="64" regnum="34" type="ieee_double"/>
    <reg name="d3" bitsize="64" regnum="35" type="ieee_double"/>
    <reg name="d4" bitsize="64" regnum="36" type="ieee_double"/>
    <reg name="d5" bitsize="64" regnum="37" type="ieee_double"/>
    <reg name="d6" bitsize="64" regnum="38" type="ieee_double"/>
    <reg name="d7" bitsize="64" regnum="39" type="ieee_double"/>
    <reg name="d8" bitsize="64" regnum="40" type="ieee_double"/>
    <reg name="d9" bitsize="64" regnum="41" type="ieee_double"/>
    <reg name="d10" bitsize="64" regnum="42" type="ieee_double"/>
    <reg name="d11" bitsize="64" regnum="43" type="ieee_double"/>
    <reg name="d12" bitsize="64" regnum="44" type="ieee_double"/>
    <reg name="d13" bitsize="64" regnum="45" type="ieee_double"/>
    <reg name="d14" bitsize="64" regnum="46" type="ieee_double"/>
    <reg name="d15" bitsize="64" regnum="47" type="ieee_double"/>
    <reg name="fpscr" bitsize="32" regnum="48" type="int"/>
  </feature>
</target>"#;

/// Cortex-M4F 寄存器文件。
///
/// 字段顺序即 `g` 报文序列化顺序。
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CortexM4FRegs {
    /// 通用寄存器 R0-R12
    pub r: [u32; 13],
    /// 栈指针 R13
    pub sp: u32,
    /// 链接寄存器 R14
    pub lr: u32,
    /// 程序计数器 R15
    pub pc: u32,
    /// 程序状态寄存器 xPSR（M-profile 原生布局，T 位 bit24）
    pub xpsr: u32,
    /// 主栈指针
    pub msp: u32,
    /// 进程栈指针
    pub psp: u32,
    /// 中断屏蔽寄存器
    pub primask: u32,
    /// 基础优先级屏蔽寄存器
    pub basepri: u32,
    /// 故障屏蔽寄存器
    pub faultmask: u32,
    /// 控制寄存器
    pub control: u32,
    /// VFP 双精度寄存器 D0-D15
    pub d: [u64; 16],
    /// 浮点状态与控制寄存器
    pub fpscr: u32,
}

impl CortexM4FRegs {
    /// `g` 报文总字节数（RLE 解压后）。
    pub const PACKET_BYTES: usize = 13 * 4 + 4 + 4 + 4 + 4 + 6 * 4 + 16 * 8 + 4; // 224
}

impl Registers for CortexM4FRegs {
    type ProgramCounter = u32;

    fn pc(&self) -> Self::ProgramCounter {
        self.pc
    }

    fn gdb_serialize(&self, mut write_byte: impl FnMut(Option<u8>)) {
        macro_rules! write_u32 {
            ($v:expr) => {
                for b in $v.to_le_bytes() {
                    write_byte(Some(b));
                }
            };
        }
        macro_rules! write_u64 {
            ($v:expr) => {
                for b in $v.to_le_bytes() {
                    write_byte(Some(b));
                }
            };
        }

        for reg in self.r.iter() {
            write_u32!(reg);
        }
        write_u32!(&self.sp);
        write_u32!(&self.lr);
        write_u32!(&self.pc);
        write_u32!(&self.xpsr);
        write_u32!(&self.msp);
        write_u32!(&self.psp);
        write_u32!(&self.primask);
        write_u32!(&self.basepri);
        write_u32!(&self.faultmask);
        write_u32!(&self.control);
        for reg in self.d.iter() {
            write_u64!(reg);
        }
        write_u32!(&self.fpscr);
    }

    fn gdb_deserialize(&mut self, bytes: &[u8]) -> Result<(), ()> {
        if bytes.len() != Self::PACKET_BYTES {
            return Err(());
        }

        let mut pos = 0;
        let mut take = |n: usize| -> Result<&[u8], ()> {
            if pos + n > bytes.len() {
                return Err(());
            }
            let s = &bytes[pos..pos + n];
            pos += n;
            Ok(s)
        };

        for reg in self.r.iter_mut() {
            *reg = u32::from_le_bytes(take(4)?.try_into().unwrap());
        }
        self.sp = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.lr = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.pc = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.xpsr = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.msp = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.psp = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.primask = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.basepri = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.faultmask = u32::from_le_bytes(take(4)?.try_into().unwrap());
        self.control = u32::from_le_bytes(take(4)?.try_into().unwrap());
        for reg in self.d.iter_mut() {
            let b = take(8)?;
            let lo = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
            let hi = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
            *reg = (hi as u64) << 32 | lo as u64;
        }
        self.fpscr = u32::from_le_bytes(take(4)?.try_into().unwrap());

        if pos != bytes.len() {
            return Err(());
        }
        Ok(())
    }
}

/// Cortex-M4F 寄存器标识符（用于单寄存器读写 `p`/`P` 报文）。
#[derive(Debug, Clone, Copy)]
pub enum CortexM4FRegId {
    /// 通用寄存器 R0-R12
    Gpr(u8),
    /// 栈指针 R13
    Sp,
    /// 链接寄存器 R14
    Lr,
    /// 程序计数器 R15
    Pc,
    /// 程序状态寄存器 xPSR
    Xpsr,
    /// 主栈指针
    Msp,
    /// 进程栈指针
    Psp,
    /// PRIMASK
    Primask,
    /// BASEPRI
    Basepri,
    /// FAULTMASK
    Faultmask,
    /// CONTROL
    Control,
    /// VFP 双精度寄存器 D0-D15
    D(u8),
    /// FPSCR
    Fpscr,
}

impl RegId for CortexM4FRegId {
    fn from_raw_id(id: usize) -> Option<(Self, Option<NonZeroUsize>)> {
        let (reg, size) = match id {
            0..=12 => (Self::Gpr(id as u8), 4),
            13 => (Self::Sp, 4),
            14 => (Self::Lr, 4),
            15 => (Self::Pc, 4),
            25 => (Self::Xpsr, 4),
            26 => (Self::Msp, 4),
            27 => (Self::Psp, 4),
            28 => (Self::Primask, 4),
            29 => (Self::Basepri, 4),
            30 => (Self::Faultmask, 4),
            31 => (Self::Control, 4),
            32..=47 => (Self::D((id - 32) as u8), 8),
            48 => (Self::Fpscr, 4),
            _ => return None,
        };
        Some((reg, Some(NonZeroUsize::new(size)?)))
    }
}

/// Cortex-M4F 架构（零变体枚举）。
pub enum CortexM4F {}

impl Arch for CortexM4F {
    type Usize = u32;
    type Registers = CortexM4FRegs;
    type RegId = CortexM4FRegId;
    type BreakpointKind = ArmBreakpointKind;

    fn target_description_xml() -> Option<&'static str> {
        Some(CORTEXM4F_TARGET_XML)
    }
}
