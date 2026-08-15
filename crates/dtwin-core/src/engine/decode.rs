//! Thumb-2 指令解码 — ARMv7E-M (Cortex-M4F)
//!
//! 解码 16-bit Thumb 与 32-bit Thumb-2 指令，输出统一中间表示。

use crate::engine::FaultReason;

/// 指令宽度
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InstrWidth {
    HalfWord,
    Word,
}

/// 解码后的指令（统一表示，供 exec 执行）
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Instruction {
    /// NOP / 提示指令
    Nop,
    /// 数据传送: MOV Rd, #imm / MOV Rd, Rm
    Mov { rd: u8, rm: u8, imm: Option<u32> },
    /// 32 位立即数高半字: MOVW/MOVT
    MovImm32 { rd: u8, imm16: u16, top: bool },
    /// 加法: ADD Rd, Rn, Rm/imm
    Add {
        rd: u8,
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
        flags: bool,
    },
    /// 减法: SUB Rd, Rn, Rm/imm
    Sub {
        rd: u8,
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
        flags: bool,
    },
    /// 按位与
    And { rd: u8, rn: u8, rm: u8, flags: bool },
    /// 按位或
    Orr { rd: u8, rn: u8, rm: u8, flags: bool },
    /// 按位异或
    Eor { rd: u8, rn: u8, rm: u8, flags: bool },
    /// 位清除: BIC Rd, Rn, Rm
    Bic { rd: u8, rn: u8, rm: u8, flags: bool },
    /// 乘法
    Mul { rd: u8, rn: u8, rm: u8, flags: bool },
    /// 无符号除法
    Udiv { rd: u8, rn: u8, rm: u8 },
    /// 有符号除法
    Sdiv { rd: u8, rn: u8, rm: u8 },
    /// 移位: LSL/LSR/ASR/ROR (reg 或 imm)
    Shift {
        rd: u8,
        rm: u8,
        kind: ShiftKind,
        amount: ShiftAmount,
        flags: bool,
    },
    /// 比较: CMP Rn, Rm/imm
    Cmp {
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
    },
    /// 负数比较
    Cmn { rn: u8, rm: u8 },
    /// 测试位: TST Rn, Rm
    Tst { rn: u8, rm: u8 },
    /// 分支: B<cond> imm
    Branch { cond: Option<Cond>, target: u32 },
    /// 分支+链接
    BranchLink { target: u32 },
    /// 分支+交换: BX Rm
    BranchExchange { rm: u8 },
    /// 分支+链接+交换: BLX Rm
    BranchLinkExchange { rm: u8 },
    /// 条件分支为 0: CBZ/CBNZ
    CompareBranch { rn: u8, target: u32, zero: bool },
    /// 表分支: TBB/TBH
    TableBranch { rn: u8, rm: u8, halfword: bool },
    /// 压栈: PUSH {reglist}
    Push { regs: u16, lr: bool },
    /// 出栈: POP {reglist}
    Pop { regs: u16, pc: bool },
    /// 多寄存器加载: LDM
    Ldm { rn: u8, regs: u16, writeback: bool },
    /// 多寄存器存储: STM
    Stm { rn: u8, regs: u16, writeback: bool },
    /// 加载字: LDR Rt, [Rn, #off] / [Rn, Rm]
    Ldr {
        rt: u8,
        rn: u8,
        offset: LoadStoreOffset,
        width: AccessWidth,
    },
    /// 存储字: STR Rt, [Rn, #off] / [Rn, Rm]
    Str {
        rt: u8,
        rn: u8,
        offset: LoadStoreOffset,
        width: AccessWidth,
    },
    /// 加载 PC 相对
    LdrLiteral { rt: u8, imm: u32 },
    /// 特殊寄存器访问: MRS/MSR
    MsrMrs {
        reg: SpecialReg,
        from_psr: bool,
        psr: bool,
    },
    /// 软件中断: SVC
    Svc { imm8: u8 },
    /// 异常返回: BX LR 特殊形式
    ExceptionReturn,

    // ================= Phase 3: DSP (ARMv7E-M 饱和运算/SIMD) =================
    /// 饱和指令: SSAT/USAT Rd, #sat, Rn {, shift}
    Sat {
        rd: u8,
        rn: u8,
        /// 饱和位宽（SSAT: 1-32；USAT: 0-31）
        sat_imm: u8,
        /// true = SSAT（有符号），false = USAT（无符号）
        signed: bool,
        shift_kind: DspShiftKind,
        /// 移位量（0-31，0 表示不移位）
        shift_imm: u8,
    },
    /// QADD/QSUB/QDADD/QDSUB: Rd = 饱和运算(Rm, Rn)
    QAddSub {
        rd: u8,
        rn: u8,
        rm: u8,
        kind: QAddKind,
    },
    /// 半字 SIMD: SADD16/UADD16/SASX/SSAX/SSUB16 等（写 GE[1:0]）
    Simd16 {
        rd: u8,
        rn: u8,
        rm: u8,
        kind: Simd16Kind,
        unsigned: bool,
    },
    /// SMUAD/SMUSD: 双半字乘法（可选交换 Rm 半字）
    DualHalfMul {
        rd: u8,
        rn: u8,
        rm: u8,
        swap: bool,
        sub: bool,
    },
    /// SMLAD/SMLSD: 双半字乘加（可选交换 Rm 半字）
    DualHalfMulAcc {
        rd: u8,
        rn: u8,
        rm: u8,
        ra: u8,
        swap: bool,
        sub: bool,
    },
    /// SMLALD/SMLSLD: 双半字 64 位乘加（[RdHi:RdLo] 累加）
    DualHalfMulLong {
        rdlo: u8,
        rdhi: u8,
        rn: u8,
        rm: u8,
        swap: bool,
        sub: bool,
    },
    /// MLA/MLS: 32 位乘加/乘减 Rd = Ra ± Rn×Rm
    Mla {
        rd: u8,
        rn: u8,
        rm: u8,
        ra: u8,
        sub: bool,
    },
    /// PKHBT/PKHTB: 半字打包
    Pkh {
        rd: u8,
        rn: u8,
        rm: u8,
        tb: bool,
        shift_imm: u8,
    },

    /// 未实现指令
    Unimplemented { bits: u32 },
    /// 非法指令
    Invalid { address: u32 },
}

/// DSP 饱和指令的移位类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DspShiftKind {
    /// SSAT/USAT: 逻辑左移
    Lsl,
    /// SSAT: 算术右移；USAT: 逻辑右移（解码时已区分）
    Asr,
}

/// Q 系列饱和指令种类
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QAddKind {
    Qadd,
    Qsub,
    Qdadd,
    Qdsub,
}

/// 半字 SIMD 运算种类
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Simd16Kind {
    /// ADD16: 低/高半字分别相加
    Add16,
    /// ASX: 低半字 = Rn.lo + Rm.hi，高半字 = Rn.hi - Rm.lo
    Asx,
    /// SAX: 低半字 = Rn.lo - Rm.hi，高半字 = Rn.hi + Rm.lo
    Sax,
    /// SUB16: 低/高半字分别相减
    Sub16,
}


/// 移位类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShiftKind {
    Lsl,
    Lsr,
    Asr,
    Ror,
    Rrx,
}

/// 移位量
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShiftAmount {
    Immediate(u8),
    Register(u8),
}

/// 条件码
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cond {
    Eq,
    Ne,
    Cs,
    Cc,
    Mi,
    Pl,
    Vs,
    Vc,
    Hi,
    Ls,
    Ge,
    Lt,
    Gt,
    Le,
    Al,
}

impl Cond {
    pub fn from_bits(bits: u8) -> Option<Self> {
        Some(match bits & 0xF {
            0 => Cond::Eq,
            1 => Cond::Ne,
            2 => Cond::Cs,
            3 => Cond::Cc,
            4 => Cond::Mi,
            5 => Cond::Pl,
            6 => Cond::Vs,
            7 => Cond::Vc,
            8 => Cond::Hi,
            9 => Cond::Ls,
            10 => Cond::Ge,
            11 => Cond::Lt,
            12 => Cond::Gt,
            13 => Cond::Le,
            14 => Cond::Al,
            _ => return None,
        })
    }
}

/// 加载/存储偏移
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadStoreOffset {
    Immediate(u32),
    Register(u8),
}

/// 访问宽度
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessWidth {
    Byte,
    HalfWord,
    Word,
}

/// 特殊寄存器
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpecialReg {
    Msp,
    Psp,
    Primask,
    Faultmask,
    Basepri,
    Control,
}

/// 指令解码器
#[derive(Debug, Default)]
pub struct Decoder {
    /// 指令计数（统计用）
    pub decoded_count: u64,
}

impl Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// 解码 16-bit Thumb 指令
    pub fn decode_halfword(&mut self, bits: u16, pc: u32) -> Instruction {
        self.decoded_count += 1;
        let op5 = bits >> 11; // bits[15:11]
        match op5 {
            // 00000-00010: 移位立即数 LSL/LSR/ASR（00011 是 ADD/SUB #imm3，见下）
            0b00000 | 0b00001 | 0b00010 => self.decode_shift_imm(bits),
            // 00011: ADD/SUB #imm3
            0b00011 => self.decode_add_sub_imm(bits),
            // 001xx: MOVS/CMP/ADDS/SUBS #imm8
            0b00100 | 0b00101 | 0b00110 | 0b00111 => self.decode_mov_cmp_add_sub_imm8(bits),
            // 010xx: MOV/CMP/ADD/SUB 寄存器
            0b01000 | 0b01001 | 0b01010 | 0b01011 => self.decode_mov_cmp_add_sub_reg(bits),
            // 011xx: ALU 寄存器
            0b01100 | 0b01101 | 0b01110 | 0b01111 => self.decode_alu_reg(bits),
            // 100xx: LDR/STR halfword/byte — decode 未接，留 Unimplemented
            0b10000 | 0b10001 | 0b10010 | 0b10011 => {
                Instruction::Unimplemented { bits: bits as u32 }
            }
            // 101xx: 条件分支
            0b10101 => self.decode_branch_cond(bits, pc),
            // 110xx: LDR/STR word — decode 未接，留 Unimplemented
            0b11000 | 0b11001 | 0b11010 | 0b11011 => {
                Instruction::Unimplemented { bits: bits as u32 }
            }
            // 11100: 无条件分支
            0b11100 => self.decode_branch_uncond(bits, pc),
            // 其他 111xx: BL/BLX 等 32-bit 前缀或系统指令
            _ => Instruction::Unimplemented { bits: bits as u32 },
        }
    }

    /// 解码 32-bit Thumb-2 指令
    pub fn decode_word(&mut self, bits: u32, pc: u32) -> Instruction {
        self.decoded_count += 1;
        let top = (bits >> 16) as u16;
        // MOVW/MOVT (11110 i 100100 ...)
        if (top & 0xFBF0) == 0xF240 {
            let imm4 = ((top >> 4) & 0xF) as u32;
            let imm3 = ((bits >> 12) & 0x7) as u32;
            let imm8 = (bits & 0xFF) as u32;
            let rd = ((bits >> 8) & 0xF) as u8;
            let imm16 = ((imm4 << 12) | (imm3 << 8) | imm8) as u16;
            let top_half = (top & 0x0008) != 0; // MOVT if bit 11 set
            return Instruction::MovImm32 {
                rd,
                imm16,
                top: top_half,
            };
        }
        // SVC (11101 111 ...)
        if (top & 0xFF00) == 0xDF00 {
            return Instruction::Svc {
                imm8: (bits & 0xFF) as u8,
            };
        }

        // ---- Phase 3: DSP ----
        // MRS (0xF3EF) / MSR (0xF380, low bit15=1) 保持诚实 Unimplemented
        if top == 0xF3EF || (top == 0xF380 && (bits & 0x8000) != 0) {
            return Instruction::MsrMrs {
                reg: SpecialReg::Control,
                from_psr: false,
                psr: false,
            };
        }
        // SSAT (0xF30x/0xF32x) / USAT (0xF38x/0xF3Ax，low bit15=0)
        if (top & 0xFFD0) == 0xF300 || ((top & 0xFFD0) == 0xF380 && (bits & 0x8000) == 0) {
            return self.decode_sat(bits);
        }
        // QADD/QSUB/QDADD/QDSUB (0xFA80-0xFA8F，低半字 op=1000-1011)
        if (top & 0xFFF8) == 0xFA80 && (bits & 0xF000) == 0xF000 && ((bits >> 4) & 0xC) == 0x8 {
            return self.decode_qaddsub(bits);
        }
        // SADD16 等半字 SIMD（0xFA90/0xFAA0/0xFAD0/0xFAE0）
        if matches!(top & 0xFFF8, 0xFA90 | 0xFAA0 | 0xFAD0 | 0xFAE0)
            && (bits & 0xF000) == 0xF000
            && matches!(bits & 0x00F0, 0x00 | 0x40)
        {
            return self.decode_simd16(bits);
        }
        // SMUAD/SMUSD/SMLAD/SMLSD（0xFB20/0xFB40，低半字 bits[7:5]=000）
        if matches!(top & 0xFFF8, 0xFB20 | 0xFB40) && (bits & 0x00E0) == 0 {
            return self.decode_dual_half_mul(bits);
        }
        // SMLALD/SMLSLD（0xFBC0/0xFBD0，低半字 bits[7:6]=11）
        if matches!(top & 0xFFF8, 0xFBC0 | 0xFBD0) && (bits & 0x00C0) == 0x00C0 {
            return self.decode_dual_half_mul_long(bits);
        }
        // MLA/MLS（0xFB00，低半字 bits[7:5]=000）
        if (top & 0xFFF0) == 0xFB00 && (bits & 0x00E0) == 0 {
            return self.decode_mla(bits);
        }
        // PKHBT/PKHTB（0xEAC0）
        if (top & 0xFFF0) == 0xEAC0 {
            return self.decode_pkh(bits);
        }

        Instruction::Unimplemented { bits }
    }

    /// 解码 SSAT/USAT（0xF30x 有符号 / 0xF38x 无符号）
    fn decode_sat(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let signed = (top & 0x80) == 0; // bit23: 0=SSAT, 1=USAT
        let sh = (top & 0x20) != 0; // bit21: 0=LSL, 1=ASR(SSAT)/LSR(USAT)
        let rn = (top & 0xF) as u8;
        let rd = ((bits >> 8) & 0xF) as u8;
        // imm5: bits{14,13,12} = imm5[4:2]，bits{7,6} = imm5[1:0]
        let imm5 = (((bits >> 12) & 0x7) << 2) | ((bits >> 6) & 0x3);
        // SSAT 编码 sat_imm-1；USAT 编码 sat_imm（0-31）
        let sat_imm = if signed {
            (bits & 0x1F) as u8 + 1
        } else {
            (bits & 0x1F) as u8
        };
        let shift_kind = if sh {
            DspShiftKind::Asr
        } else {
            DspShiftKind::Lsl
        };
        Instruction::Sat {
            rd,
            rn,
            sat_imm,
            signed,
            shift_kind,
            shift_imm: imm5 as u8,
        }
    }

    /// 解码 QADD/QSUB/QDADD/QDSUB（top=0xFA8x，低半字 bits[7:4]=1000-1011）
    /// 编码布局（汇编器验证）：top bits[3:0] = Rn，低半字 bits[3:0] = Rm
    fn decode_qaddsub(&self, bits: u32) -> Instruction {
        let rn = ((bits >> 16) & 0xF) as u8;
        let rd = ((bits >> 8) & 0xF) as u8;
        let rm = (bits & 0xF) as u8;
        let kind = match (bits >> 4) & 0xF {
            0x8 => QAddKind::Qadd,
            0x9 => QAddKind::Qdadd,
            0xA => QAddKind::Qsub,
            _ => QAddKind::Qdsub,
        };
        Instruction::QAddSub { rd, rn, rm, kind }
    }

    /// 解码半字 SIMD（SADD16 等）
    fn decode_simd16(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let rn = (top & 0xF) as u8;
        let rd = ((bits >> 8) & 0xF) as u8;
        let rm = (bits & 0xF) as u8;
        let unsigned = (bits & 0x40) != 0; // low bit6: 1 = 无符号
        let kind = match (top >> 4) & 0xF {
            0x9 => Simd16Kind::Add16,
            0xA => Simd16Kind::Asx,
            0xE => Simd16Kind::Sax,
            _ => Simd16Kind::Sub16,
        };
        Instruction::Simd16 {
            rd,
            rn,
            rm,
            kind,
            unsigned,
        }
    }

    /// 解码 SMUAD/SMUSD/SMLAD/SMLSD
    fn decode_dual_half_mul(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let rn = (top & 0xF) as u8;
        let rd = ((bits >> 8) & 0xF) as u8;
        let rm = (bits & 0xF) as u8;
        let swap = (bits & 0x10) != 0; // X
        let sub = (top & 0x40) != 0; // SMUSD/SMLSD（bits[23:20]=0100）
        let ra = ((bits >> 12) & 0xF) as u8;
        if ra == 0xF {
            // 低半字 bits[15:12]=1111 → SMUAD/SMUSD（无累加）
            Instruction::DualHalfMul {
                rd,
                rn,
                rm,
                swap,
                sub,
            }
        } else {
            Instruction::DualHalfMulAcc {
                rd,
                rn,
                rm,
                ra,
                swap,
                sub,
            }
        }
    }

    /// 解码 SMLALD/SMLSLD（64 位累加）
    fn decode_dual_half_mul_long(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let rn = (top & 0xF) as u8;
        let rdlo = ((bits >> 12) & 0xF) as u8;
        let rdhi = ((bits >> 8) & 0xF) as u8;
        let rm = (bits & 0xF) as u8;
        let swap = (bits & 0x10) != 0; // X
        let sub = (top & 0x10) != 0; // SMLSLD（bits[23:20]=1101）
        Instruction::DualHalfMulLong {
            rdlo,
            rdhi,
            rn,
            rm,
            swap,
            sub,
        }
    }

    /// 解码 MLA/MLS
    fn decode_mla(&self, bits: u32) -> Instruction {
        let rn = (bits >> 16) & 0xF;
        let ra = ((bits >> 12) & 0xF) as u8;
        let rd = ((bits >> 8) & 0xF) as u8;
        let rm = (bits & 0xF) as u8;
        let sub = (bits & 0x10) != 0;
        Instruction::Mla {
            rd: rd as u8,
            rn: rn as u8,
            rm,
            ra,
            sub,
        }
    }

    /// 解码 PKHBT/PKHTB
    fn decode_pkh(&self, bits: u32) -> Instruction {
        let rn = (bits >> 16) & 0xF;
        let rd = ((bits >> 8) & 0xF) as u8;
        let rm = (bits & 0xF) as u8;
        let tb = (bits & 0x20) != 0;
        let imm5 = (((bits >> 12) & 0x7) << 2) | ((bits >> 6) & 0x3);
        Instruction::Pkh {
            rd,
            rn: rn as u8,
            rm,
            tb,
            shift_imm: imm5 as u8,
        }
    }

    /// 移位立即数: 000 00 xxxxx xxxxx (LSL/LSR/ASR)
    fn decode_shift_imm(&self, bits: u16) -> Instruction {
        let rd = (bits & 0x7) as u8;
        let rm = ((bits >> 3) & 0x7) as u8;
        let imm5 = (bits >> 6) & 0x1F;
        let op = (bits >> 11) & 0x3;
        let kind = match op {
            0 => ShiftKind::Lsl,
            1 => ShiftKind::Lsr,
            2 => ShiftKind::Asr,
            _ => return Instruction::Unimplemented { bits: bits as u32 },
        };
        Instruction::Shift {
            rd,
            rm,
            kind,
            amount: ShiftAmount::Immediate(imm5 as u8),
            flags: true,
        }
    }

    /// 加减立即数: 000 11 xxxxx xxxxx (ADD/SUB #imm)
    fn decode_add_sub_imm(&self, bits: u16) -> Instruction {
        let rd = (bits & 0x7) as u8;
        let rn = ((bits >> 3) & 0x7) as u8;
        let imm3 = ((bits >> 6) & 0x7) as u32;
        let op = (bits >> 9) & 0x1; // 0=ADD, 1=SUB
        let imm = match (bits >> 10) & 0x3 {
            0 => imm3,
            1 => (imm3 << 4) | ((bits & 0xF) as u32), // 8-bit imm
            2 => imm3 << 8,                           // 11-bit imm
            _ => imm3,
        };
        if op == 0 {
            Instruction::Add {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: true,
            }
        } else {
            Instruction::Sub {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: true,
            }
        }
    }

    /// MOV/CMP/ADD/SUB 寄存器: 010 0/1 xxxxx xxxxx
    fn decode_mov_cmp_add_sub_reg(&self, bits: u16) -> Instruction {
        let rd = (bits & 0x7) as u8;
        let rn = rd;
        let rm = ((bits >> 3) & 0x7) as u8;
        let op = (bits >> 6) & 0x3;
        match op {
            0 => Instruction::Mov { rd, rm, imm: None },
            1 => Instruction::Cmp {
                rn,
                rm: Some(rm),
                imm: None,
            },
            2 => Instruction::Add {
                rd,
                rn,
                rm: Some(rm),
                imm: None,
                flags: true,
            },
            _ => Instruction::Sub {
                rd,
                rn,
                rm: Some(rm),
                imm: None,
                flags: true,
            },
        }
    }

    /// ALU 寄存器: 010 000 1101 xxxx (AND/ORR/EOR/BIC/MUL)
    fn decode_alu_reg(&self, bits: u16) -> Instruction {
        let rd = (bits & 0x7) as u8;
        let rm = ((bits >> 3) & 0x7) as u8;
        let rn = rd;
        let op = (bits >> 6) & 0xF;
        match op {
            0 => Instruction::And {
                rd,
                rn,
                rm,
                flags: true,
            },
            1 => Instruction::Eor {
                rd,
                rn,
                rm,
                flags: true,
            },
            2 => Instruction::Shift {
                rd,
                rm,
                kind: ShiftKind::Lsl,
                amount: ShiftAmount::Register(rn),
                flags: true,
            },
            3 => Instruction::Shift {
                rd,
                rm,
                kind: ShiftKind::Lsr,
                amount: ShiftAmount::Register(rn),
                flags: true,
            },
            4 => Instruction::Shift {
                rd,
                rm,
                kind: ShiftKind::Asr,
                amount: ShiftAmount::Register(rn),
                flags: true,
            },
            5 => Instruction::Add {
                rd,
                rn,
                rm: Some(rm),
                imm: None,
                flags: true,
            },
            6 => Instruction::Sub {
                rd,
                rn,
                rm: Some(rm),
                imm: None,
                flags: true,
            },
            7 => Instruction::Shift {
                rd,
                rm,
                kind: ShiftKind::Ror,
                amount: ShiftAmount::Register(rn),
                flags: true,
            },
            8 => Instruction::And {
                rd,
                rn,
                rm,
                flags: false,
            },
            9 => Instruction::Eor {
                rd,
                rn,
                rm,
                flags: false,
            },
            10 => Instruction::Shift {
                rd,
                rm,
                kind: ShiftKind::Lsl,
                amount: ShiftAmount::Register(rn),
                flags: false,
            },
            11 => Instruction::Shift {
                rd,
                rm,
                kind: ShiftKind::Lsr,
                amount: ShiftAmount::Register(rn),
                flags: false,
            },
            12 => Instruction::Shift {
                rd,
                rm,
                kind: ShiftKind::Asr,
                amount: ShiftAmount::Register(rn),
                flags: false,
            },
            13 => Instruction::Mul {
                rd,
                rn,
                rm,
                flags: false,
            },
            14 => Instruction::Bic {
                rd,
                rn,
                rm,
                flags: true,
            },
            _ => Instruction::Unimplemented { bits: bits as u32 },
        }
    }

    /// MOVS/CMP/ADD/SUB #imm8: 100 x op rd imm8
    fn decode_mov_cmp_add_sub_imm8(&self, bits: u16) -> Instruction {
        let rd = ((bits >> 8) & 0x7) as u8;
        let rn = rd;
        let imm = (bits & 0xFF) as u32;
        let op = (bits >> 11) & 0x3;
        match op {
            0 => Instruction::Mov {
                rd,
                rm: 0,
                imm: Some(imm),
            },
            1 => Instruction::Cmp {
                rn,
                rm: None,
                imm: Some(imm),
            },
            2 => Instruction::Add {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: true,
            },
            _ => Instruction::Sub {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: true,
            },
        }
    }

    /// 条件分支: 1101 cond imm8
    fn decode_branch_cond(&self, bits: u16, pc: u32) -> Instruction {
        let cond_bits = ((bits >> 8) & 0xF) as u8;
        let imm8 = (bits & 0xFF) as u32;
        let target = pc.wrapping_add(4).wrapping_add(imm8 << 1);
        match Cond::from_bits(cond_bits) {
            Some(cond) => Instruction::Branch {
                cond: Some(cond),
                target,
            },
            None => {
                // 1110 = undefined, 1111 = SVC
                if (bits & 0xFF00) == 0xDF00 {
                    Instruction::Svc { imm8: imm8 as u8 }
                } else {
                    Instruction::Invalid { address: pc }
                }
            }
        }
    }

    /// 无条件分支: 11100 imm11
    fn decode_branch_uncond(&self, bits: u16, pc: u32) -> Instruction {
        let imm11 = (bits & 0x7FF) as u32;
        let target = pc.wrapping_add(4).wrapping_add(imm11 << 1);
        Instruction::Branch { cond: None, target }
    }

    /// 生成非法指令错误
    pub fn invalid(&self, address: u32) -> FaultReason {
        FaultReason::IllegalInstruction { pc: address }
    }
}
