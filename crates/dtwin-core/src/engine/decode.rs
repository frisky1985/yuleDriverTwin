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
    /// 数据传送: MOV Rd, #imm / MOV Rd, Rm（flags=true 时更新 N/Z，即 16 位 MOVS 形式）
    Mov {
        rd: u8,
        rm: u8,
        imm: Option<u32>,
        flags: bool,
    },
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
    /// 按位与（Rd = Rn & Rm/imm）
    And {
        rd: u8,
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
        flags: bool,
    },
    /// 按位或（Rd = Rn | Rm/imm）
    Orr {
        rd: u8,
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
        flags: bool,
    },
    /// 按位异或（Rd = Rn ^ Rm/imm）
    Eor {
        rd: u8,
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
        flags: bool,
    },
    /// 位清除: BIC Rd, Rn, Rm/imm（Rd = Rn & ~Rm）
    Bic {
        rd: u8,
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
        flags: bool,
    },
    /// 带进位加法: ADC Rd, Rn, Rm（Rd = Rn + Rm + C）
    Adc {
        rd: u8,
        rn: u8,
        rm: u8,
        flags: bool,
    },
    /// 带借位减法: SBC Rd, Rn, Rm（Rd = Rn - Rm - NOT(C)）
    Sbc {
        rd: u8,
        rn: u8,
        rm: u8,
        flags: bool,
    },
    /// 取负: NEG Rd, Rm（= RSB Rd, Rm, #0，Rd = 0 - Rm）
    Neg {
        rd: u8,
        rn: u8,
        flags: bool,
    },
    /// 按位取反: MVN Rd, Rm（Rd = ~Rm）
    Mvn {
        rd: u8,
        rm: u8,
        flags: bool,
    },
    /// 乘法
    Mul { rd: u8, rn: u8, rm: u8, flags: bool },
    /// 64 位长乘法: UMULL/SMULL/SMLAL（[RdHi:RdLo] = Rn×Rm [+ [RdHi:RdLo]]）
    MullLong {
        rdlo: u8,
        rdhi: u8,
        rn: u8,
        rm: u8,
        signed: bool,
        accumulate: bool,
    },
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
    /// 负数比较: CMN Rn, Rm/imm
    Cmn {
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
    },
    /// 测试位: TST Rn, Rm/imm
    Tst {
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
    },
    /// 测试相等（异或）: TEQ Rn, Rm/imm（仅更新标志）
    Teq {
        rn: u8,
        rm: Option<u8>,
        imm: Option<u32>,
    },
    /// 分支: B<cond> imm
    Branch { cond: Option<Cond>, target: u32 },
    /// 分支+链接
    BranchLink { target: u32 },
    /// 分支+链接+交换（立即数形式，32-bit BLX）
    BranchLinkExchangeImm { target: u32 },
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
    /// 多寄存器加载: LDM（descending=true 为 DB 形式：先减后访存）
    Ldm {
        rn: u8,
        regs: u16,
        writeback: bool,
        descending: bool,
    },
    /// 多寄存器存储: STM（descending=true 为 DB 形式：先减后访存）
    Stm {
        rn: u8,
        regs: u16,
        writeback: bool,
        descending: bool,
    },
    /// 数据屏障: DSB/ISB/DMB（单核顺序模拟 = 无操作）
    Barrier { kind: BarrierKind },
    /// CPSIE/CPSID: 修改 PRIMASK/FAULTMASK（disable=true 为 CPSID）
    Cps { disable: bool, i: bool, f: bool },
    /// 前导零计数: CLZ Rd, Rm
    Clz { rd: u8, rm: u8 },
    /// 位反转: RBIT Rd, Rm
    Rbit { rd: u8, rm: u8 },
    /// 字节序反转: REV/REV16/REVSH
    Rev { rd: u8, rm: u8, kind: RevKind },
    /// 位域操作: UBFX/SBFX/BFI/BFC
    BitField {
        rd: u8,
        rn: u8,
        lsb: u8,
        width: u8,
        kind: BitFieldKind,
    },
    /// 独占加载: LDREX/LDREXB/LDREXH（单核语义 = LDR）
    Ldrex {
        rt: u8,
        rn: u8,
        imm: u32,
        width: AccessWidth,
    },
    /// 独占存储: STREX/STREXB/STREXH（单核语义 = STR 且 Rd=0 成功）
    Strex {
        rd: u8,
        rt: u8,
        rn: u8,
        imm: u32,
        width: AccessWidth,
    },
    /// 加载字: LDR Rt, [Rn, #off] / [Rn, Rm]
    Ldr {
        rt: u8,
        rn: u8,
        offset: LoadStoreOffset,
        width: AccessWidth,
    },
    /// 有符号加载（符号扩展）: LDRSB/LDRSH Rt, [Rn, Rm]
    LdrSignExtend {
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
    /// 双字加载: LDRD Rt, Rt2, [Rn, #imm8×4]
    LdrD { rt: u8, rt2: u8, rn: u8, imm: u32 },
    /// 双字存储: STRD Rt, Rt2, [Rn, #imm8×4]
    StrD { rt: u8, rt2: u8, rn: u8, imm: u32 },
    /// 特殊寄存器访问: MRS/MSR
    MsrMrs {
        /// 核心寄存器（MRS 的 Rd / MSR 的 Rn）
        rt: u8,
        /// 目标特殊寄存器
        reg: SpecialReg,
        /// true = MRS（特殊寄存器 → 核心寄存器）；false = MSR（核心寄存器 → 特殊寄存器）
        read: bool,
    },
    /// 软件中断: SVC
    Svc { imm8: u8 },
    /// 调试断点: BKPT #imm8
    Breakpoint { imm8: u8 },
    /// If-Then 条件块: IT{cond} mask（ITSTATE 状态机，后续 1-4 条指令按条件执行）
    It { cond: Cond, mask: u8 },
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
    /// 8-bit SIMD: SADD8/SSUB8/UADD8/USUB8/SHADD8/SHSUB8/UHADD8/UHSUB8（写 GE[3:0]）
    Simd8 {
        rd: u8,
        rn: u8,
        rm: u8,
        /// true = 无符号（U 前缀）
        unsigned: bool,
        /// true = 减半（H 前缀）
        halving: bool,
        /// true = 减法家族（SSUB8/USUB8/SHSUB8/UHSUB8）
        sub: bool,
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

    // ================= Phase 4: FPU (VFPv4-SP + F64 骨架) =================
    /// VMOV Sd, Sm / Dd, Dm（寄存器间传送）
    FpVmovReg { sd: u8, sm: u8, double: bool },
    /// VMOV Sn, Rt / Rt, Sn（核心寄存器与单精度互传）
    FpVmovCore { rt: u8, sn: u8, to_core: bool },
    /// VMOV Dd, Rt, Rt2 / Rt, Rt2, Dd（双精度与两个核心寄存器）
    FpVmovCoreD {
        rt: u8,
        rt2: u8,
        dn: u8,
        to_core: bool,
    },
    /// VMOV.F32/F64 Sd, #imm（已展开为位模式）
    FpVmovImm { sd: u8, imm: u64, double: bool },
    /// VFP 三寄存器运算（VADD/VSUB/VMUL/VDIV/VMLA/VMLS/VNMLS/VNMLA）
    FpArith3 {
        op: FpArithOp,
        vd: u8,
        vn: u8,
        vm: u8,
        double: bool,
    },
    /// VFP 二寄存器一元运算（VABS/VNEG/VSQRT）
    FpUnary {
        op: FpUnaryOp,
        vd: u8,
        vm: u8,
        double: bool,
    },
    /// VCMP/VCMPE（含 #0.0 形式），写 FPSCR N/Z/C/V
    FpCmp {
        vd: u8,
        vm: u8,
        double: bool,
        /// VCMPE（对 NaN 触发 Invalid Operation）
        e: bool,
        /// 与 +0.0 比较
        zero: bool,
    },
    /// VCVT 转换（S32/U32/F32/F64 之间）
    FpCvt { op: FpCvtOp, vd: u8, vm: u8 },
    /// VLDR/VSTR: 内存访问
    FpLoadStore {
        rt: u8,
        rn: u8,
        /// 已按符号展开的字节偏移
        offset: u32,
        load: bool,
        double: bool,
    },
    /// VLDM/VSTM: 多寄存器加载/存储（S 或 D 列表，IA/DB + 回写）
    FpLoadStoreMulti {
        /// 起始寄存器索引（SP: S[vd..]；DP: D[vd..]）
        vd: u8,
        rn: u8,
        /// 寄存器个数（imm8 按字计算：SP = imm8，DP = imm8/2）
        count: u32,
        load: bool,
        double: bool,
        /// true = DB（先减后访存），false = IA
        decrement: bool,
        writeback: bool,
    },
    /// VCVT 定点转换（Vd 与源同寄存器，仅单精度）
    FpCvtFixed {
        vd: u8,
        /// 小数位数 fbits
        fbits: u8,
        /// 整数宽度：16 或 32
        width: u8,
        signed: bool,
        /// true = 定点 → 浮点；false = 浮点 → 定点
        to_float: bool,
    },

    /// 未实现指令
    Unimplemented { bits: u32 },
    /// 非法指令
    Invalid { address: u32 },
}

/// 数据屏障种类（FRT-INS-02：单核顺序模拟下均为无操作）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BarrierKind {
    Dsb,
    Isb,
    Dmb,
}

/// 字节序反转种类（16 位 REV 家族）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RevKind {
    Rev,
    Rev16,
    RevSh,
}

/// 位域操作种类（UBFX/SBFX/BFI/BFC）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitFieldKind {
    Ubfx,
    Sbfx,
    Bfi,
    Bfc,
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

/// VFP 三寄存器运算种类
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FpArithOp {
    Vmla,
    Vmls,
    Vnmls,
    Vnmla,
    Vmul,
    Vnmul,
    Vadd,
    Vsub,
    Vdiv,
}

/// VFP 一元运算种类
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FpUnaryOp {
    Vabs,
    Vneg,
    Vsqrt,
}

/// VCVT 转换种类（vd/vm 已按方向解码为正确索引）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FpCvtOp {
    /// 有符号 32 位整数 → F32（按 FPSCR 舍入模式）
    S32ToF32,
    /// 无符号 32 位整数 → F32
    U32ToF32,
    /// F32 → 有符号 32 位整数（朝零舍入）
    F32ToS32,
    /// F32 → 无符号 32 位整数（朝零舍入）
    F32ToU32,
    /// F32 → 有符号 32 位整数（就近舍入，VCVTR）
    F32ToS32R,
    /// F32 → 无符号 32 位整数（就近舍入，VCVTR）
    F32ToU32R,
    /// F64 → F32
    F64ToF32,
    /// F32 → F64（精确）
    F32ToF64,
    /// 有符号 32 位整数 → F64
    S32ToF64,
    /// 无符号 32 位整数 → F64
    U32ToF64,
    /// F64 → 有符号 32 位整数（朝零舍入）
    F64ToS32,
    /// F64 → 无符号 32 位整数（朝零舍入）
    F64ToU32,
    /// F64 → 有符号 32 位整数（就近舍入）
    F64ToS32R,
    /// F64 → 无符号 32 位整数（就近舍入）
    F64ToU32R,
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

    /// 条件码数值（与 from_bits 互逆）
    pub fn to_bits(self) -> u8 {
        match self {
            Cond::Eq => 0,
            Cond::Ne => 1,
            Cond::Cs => 2,
            Cond::Cc => 3,
            Cond::Mi => 4,
            Cond::Pl => 5,
            Cond::Vs => 6,
            Cond::Vc => 7,
            Cond::Hi => 8,
            Cond::Ls => 9,
            Cond::Ge => 10,
            Cond::Lt => 11,
            Cond::Gt => 12,
            Cond::Le => 13,
            Cond::Al => 14,
        }
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

/// 特殊寄存器（MRS/MSR 目标）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpecialReg {
    Msp,
    Psp,
    Primask,
    Faultmask,
    Basepri,
    /// BASEPRI_MAX：写时仅当新值更小（提高屏蔽）生效
    BasepriMax,
    Control,
    /// APSR（NZCV+Q，xPSR bits[31:27]）
    Apsr,
    /// APSR_nzcvqg：NZCV+Q+GE（xPSR bits[31:27]+[19:16]）
    ApsrGe,
    /// IAPSR：IPSR + APSR（xPSR bits[31:27]+[8:0]）
    Iapsr,
    /// EAPSR：EPSR + APSR（xPSR bits[31:27]+[24:16]）
    Eapsr,
    /// IPSR（xPSR bits[8:0]，异常号）
    Ipsr,
    /// EPSR（xPSR bits[24:16]，T+GE）
    Epsr,
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
            // 00011: ADD/SUB（bit10=0 寄存器形式 / bit10=1 立即数形式）
            0b00011 => self.decode_add_sub_imm(bits),
            // 001xx: MOVS/CMP/ADDS/SUBS #imm8
            0b00100 | 0b00101 | 0b00110 | 0b00111 => self.decode_mov_cmp_add_sub_imm8(bits),
            // 01000: 寄存器数据处理（bits[9:8]=00）+ 高寄存器 ADD/CMP/MOV + BX/BLX
            0b01000 => self.decode_data_proc_high(bits),
            // 01001: LDR literal（PC 相对字面量）
            0b01001 => self.decode_ldr_literal(bits),
            // 01010/01011: STR/STRB、LDR/LDRB 寄存器偏移（真实编码：bit10 = B）
            0b01010 | 0b01011 => self.decode_ldr_str_reg(bits),
            // 01100-01111: STR/LDR/STRB/LDRB 立即数偏移（真实编码：imm5）
            0b01100 | 0b01101 | 0b01110 | 0b01111 => self.decode_ldr_str_imm(bits),
            // 10000/10001: STRH/LDRH 立即数偏移（imm5×2）
            0b10000 | 0b10001 => self.decode_ldr_str_imm(bits),
            // 10010/10011: STR/LDR SP 相对（imm8×4）
            0b10010 | 0b10011 => self.decode_sp_relative(bits),
            // 10100/10101: ADD Rd, PC/SP, #imm8×4
            0b10100 | 0b10101 => self.decode_add_pc_sp_imm(bits),
            // 10110: ADD/SUB SP、CBZ、PUSH（0xB000-0xB7FF）
            0b10110 => self.decode_b_misc(bits, pc),
            // 10111: CBNZ、POP、BKPT/IT/提示（0xB800-0xBFFF）
            0b10111 => self.decode_b_top(bits, pc),
            // 11000/11001: STMIA/LDMIA（16 位形式固定回写）
            0b11000 => self.decode_stmia(bits),
            0b11001 => self.decode_ldmia(bits),
            // 11010/11011: B<cond>
            0b11010 | 0b11011 => self.decode_branch_cond(bits, pc),
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
        // MOVW/MOVT (11110 i 100100/101100 imm4 0 imm3 Rd imm8)
        // A9 实测修正：imm4 = top[3:0]（原 (top>>4)&0xF 取错位）、
        // imm16 = imm4:i:imm3:imm8（原漏 i 位）、MOVT 判别 = top bit7
        // （原 top&0x0008 误判，导致 MOVW 被当 MOVT 写入高半字）。
        if (top & 0xFB70) == 0xF240 {
            let imm4 = (top & 0xF) as u32; // bits[19:16]
            let i = ((top >> 10) & 1) as u32; // bit26
            let imm3 = ((bits >> 12) & 0x7) as u32; // bits[14:12]
            let imm8 = (bits & 0xFF) as u32;
            let rd = ((bits >> 8) & 0xF) as u8;
            let imm16 = ((imm4 << 12) | (i << 11) | (imm3 << 8) | imm8) as u16;
            let top_half = (top & 0x0080) != 0; // MOVT：bit23=1（bit7 of top）
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

        // BL（11110 S imm10 → 11111 J1 J2 imm11）；BLX（低半字 11101 开头）见下一分支
        if (top & 0xF800) == 0xF000 && (bits & 0xF800) == 0xF800 {
            return self.decode_bl(bits, pc);
        }
        // BLX 32-bit（11110 S imm10 → 1110 1 imm11[10:0]）：低半字 11101 开头
        if (top & 0xF800) == 0xF000 && (bits & 0xF800) == 0xE800 {
            return self.decode_blx(bits, pc);
        }

        // ---- Phase 3: DSP ----
        // MRS (0xF3EF，低半字 bit15=1)：读特殊寄存器到 Rd
        if top == 0xF3EF && (bits & 0x8000) != 0 {
            return self.decode_mrs(bits);
        }
        // MSR (0xF38x，低半字 bit15=1)：写特殊寄存器（Rn 在 top bits[3:0]）
        if (top & 0xFFF0) == 0xF380 && (bits & 0x8000) != 0 {
            return self.decode_msr(bits);
        }
        // SSAT (0xF30x/0xF32x) / USAT (0xF38x/0xF3Ax，low bit15=0)
        if (top & 0xFFD0) == 0xF300 || ((top & 0xFFD0) == 0xF380 && (bits & 0x8000) == 0) {
            return self.decode_sat(bits);
        }
        // DSB/ISB/DMB（0xF3BF 8Fxx，FRT-INS-02）：编码（as 实测）
        // dsb sy=0xF3BF 8F4F / dmb sy=0xF3BF 8F5F / isb sy=0xF3BF 8F6F——
        // 单核顺序模拟语义 = 无操作屏障（不 fault、不改变可观测状态）
        if top == 0xF3BF && (bits & 0xFF0F) == 0x8F0F {
            let kind = match (bits >> 4) & 0xF {
                0x4 => BarrierKind::Dsb,
                0x5 => BarrierKind::Dmb,
                0x6 => BarrierKind::Isb,
                // 其他选项（DSB 域/共享性变体）同为屏障
                _ => BarrierKind::Dsb,
            };
            return Instruction::Barrier { kind };
        }
        // UBFX/SBFX/BFI/BFC（位域操作，FRT-INS-05 SHOULD）：
        // 编码（as 实测）：ubfx=0xF3C1/sbfx=0xF341/bfi=0xF361/bfc=0xF36F（Rn=1111）——
        // 1111 0 0111 op[2:0] Rn 0 imm3 Rd imm2 0 msb/lsb
        if (top & 0xFFF0) == 0xF3C0
            || (top & 0xFFF0) == 0xF340
            || (top & 0xFFF0) == 0xF360
        {
            return self.decode_bitfield(bits);
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
        // 8-bit SIMD（SADD8/SSUB8 家族：top 0xFA8x 加 / 0xFACx 减，低半字 0xF0xx）
        // bits[7:6] != 10 排除 QADD8/QSUB8 等（QADD 家族判别位）
        if matches!(top & 0xFFF8, 0xFA80 | 0xFAC0)
            && (bits & 0xF000) == 0xF000
            && (bits & 0x00C0) != 0x0080
        {
            return self.decode_simd8(bits);
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
        // UMULL/SMULL/SMLAL（FRT-INS-05 SHOULD，-O2 固件 64 位乘加）：
        // 编码（as 实测）：umull r0,r1,r2,r3=0xFBA2 0103 / smull=0xFB86 4507 /
        // smlal=0xFBC2 0103——top 0xFBA0/0xFB80/0xFBC0 家族，低半字 bits[7:4]=0000。
        // 与 SMLALD/SMLSLD（低半字 bits[7:6]=11）及 SMUAD（0xFB20/0xFB40）互斥。
        if matches!(top & 0xFFF0, 0xFBA0 | 0xFB80 | 0xFBC0) && (bits & 0x00F0) == 0 {
            let rdlo = ((bits >> 12) & 0xF) as u8;
            let rdhi = ((bits >> 8) & 0xF) as u8;
            let rn = (top & 0xF) as u8;
            let rm = (bits & 0xF) as u8;
            let signed = (top & 0xFFF0) == 0xFB80;
            let accumulate = (top & 0xFFF0) == 0xFBC0;
            return Instruction::MullLong {
                rdlo,
                rdhi,
                rn,
                rm,
                signed,
                accumulate,
            };
        }
        // PKHBT/PKHTB（0xEAC0）
        if (top & 0xFFF0) == 0xEAC0 {
            return self.decode_pkh(bits);
        }

        // CLZ（0xFAB0，FRT-INS-04）：编码（as 实测）clz r0,r1=0xFAB1 F081——
        // 1111 1010 1011 Rn 1111 Rd 1000 Rm
        if (top & 0xFFF0) == 0xFAB0 && (bits & 0xF0F0) == 0xF080 {
            return Instruction::Clz {
                rd: ((bits >> 8) & 0xF) as u8,
                rm: (bits & 0xF) as u8,
            };
        }
        // RBIT（0xFA90，FRT-INS-05 SHOULD）：编码（as 实测）rbit r0,r1=0xFA91 F0A1——
        // 1111 1010 1001 Rn 1111 Rd 1010 Rm（低半字 bits[7:4]=1010 与 SADD16 区分）
        if (top & 0xFFF0) == 0xFA90 && (bits & 0xF0F0) == 0xF0A0 {
            return Instruction::Rbit {
                rd: ((bits >> 8) & 0xF) as u8,
                rm: (bits & 0xF) as u8,
            };
        }

        // ---- 32-bit LDR/STR 家族（A8：真实固件刚需）----
        // 编码（arm-none-eabi-as 实测）：
        //   LDR.W/STR.W imm12: 0xF8D0/0xF8C0 + Rn；LDRH/STRH: 0xF8B0/0xF8A0；
        //   LDRB/STRB: 0xF890/0xF880；LDRSH/LDRSB: 0xF9B0/0xF990；
        //   LDR/STR reg 偏移: 0xF850/0xF840；LDR literal: 0xF8DF；LDRD/STRD: 0xE9D0/0xE9C0
        if (top & 0xF800) == 0xF800 {
            if let Some(instr) = self.try_decode_ldr_str_32(bits) {
                return instr;
            }
        }
        // LDRD/STRD（0xE9D0/0xE9C0）——须先于 32 位 LDM/STM 检查（同为 0xE8xx/0xE9xx 族）
        if (top & 0xFFF0) == 0xE9D0 || (top & 0xFFF0) == 0xE9C0 {
            return self.decode_ldrd_strd(bits);
        }
        // ---- 32-bit LDM/STM + LDREX/STREX（FRT-INS-03/05，0xE8xx/0xE9xx）----
        // 编码（arm-none-eabi-as 实测）：P=bit24、U=bit23、W(回写)=bit21、L=bit20；
        // bit22=0 → LDM/STM（IA: P=0,U=1；DB: P=1,U=0）；bit22=1 → LDREX/STREX 家族。
        // 例：stmia.w r0!,{r4-r11,lr}=0xE8A0 4FF0 / ldmia.w r0!,{..}=0xE8B0 4FF0 /
        //     stmdb r0!,{..}=0xE920 4FF0 / ldmdb r0!,{..}=0xE930 4FF0
        if (top & 0xFE00) == 0xE800 {
            if (top & 0x0040) == 0 {
                // bit22=0：LDM/STM（32 位 IA/DB 全家族，寄存器列表 r0-r15 任意组合）
                let p = (top >> 8) & 1; // bit24
                let u = (top >> 7) & 1; // bit23
                let w = (top >> 5) & 1; // bit21
                let l = (top >> 4) & 1; // bit20
                if !(p == 0 && u == 1) && !(p == 1 && u == 0) {
                    // P=U（IA+DB 之外的保留组合，如 0xE800）→ 诚实 Unimplemented
                    return Instruction::Unimplemented { bits };
                }
                let rn = (top & 0xF) as u8;
                let regs = (bits & 0xFFFF) as u16;
                let descending = p == 1 && u == 0; // DB；IA = P=0,U=1
                if l == 1 {
                    return Instruction::Ldm {
                        rn,
                        regs,
                        writeback: w == 1,
                        descending,
                    };
                } else {
                    return Instruction::Stm {
                        rn,
                        regs,
                        writeback: w == 1,
                        descending,
                    };
                }
            } else {
                // bit22=1：LDREX/STREX 家族（单核语义，FRT-INS-05 SHOULD）
                return self.decode_ldrex_strex(bits);
            }
        }

        // ---- 32-bit 数据处理（修改立即数）: 11110 i 0 op4 S Rn / 0 imm3 Rd imm8 ----
        if (top & 0xFA00) == 0xF000 && (bits & 0x8000) == 0 {
            return self.decode_data_proc_imm(bits);
        }

        // ---- Phase 4: FPU ----
        if (top & 0xFF00) == 0xEE00 {
            if let Some(instr) = self.try_decode_fpu(bits) {
                return instr;
            }
        }
        // VLDM/VSTM（0xECxx IA 家族；0xED2x/0xED3x DB 家族需 W=1，避开 VLDR/VSTR）
        if ((top & 0xFF00) == 0xEC00 || ((top & 0xFF00) == 0xED00 && (top & 0x20) != 0))
            && (bits & 0x0E00) == 0x0A00
            && (bits & 0x00F0) == 0
        {
            return self.decode_vldm_vstm(bits);
        }
        // VLDR/VSTR（0xED00，bit21=0）
        if (top & 0xFF00) == 0xED00
            && (top & 0x20) == 0
            && (bits & 0x0E00) == 0x0A00
            && (bits & 0x00F0) == 0
        {
            return self.decode_fpu_loadstore(bits);
        }
        // VMOV Dd, Rt, Rt2 / Rt, Rt2, Dd（0xEC4x/0xEC5x）
        if (top & 0xFF00) == 0xEC00
            && (top & 0x20) == 0
            && (bits & 0x0F00) == 0x0B00
            && (bits & 0x10) == 0x10
            && (bits & 0x60) == 0
        {
            return self.decode_vmov_core_d(bits);
        }

        Instruction::Unimplemented { bits }
    }

    /// 32-bit LDR/STR 家族解码（A8）
    /// 编码（arm-none-eabi-as 实测）：
    ///   LDR.W imm12: 0xF8D0+rn | rt imm12；STR.W: 0xF8C0+rn
    ///   LDRH.W: 0xF8B0+rn；STRH.W: 0xF8A0+rn；LDRB.W: 0xF890+rn；STRB.W: 0xF880+rn
    ///   LDRSH.W: 0xF9B0+rn；LDRSB.W: 0xF990+rn（符号扩展）
    ///   LDR/STR reg 偏移: 0xF850/0xF840+rn | rt imm2 rm
    ///   LDR literal: 0xF8DF（Rn=1111）| rt imm12
    fn try_decode_ldr_str_32(&self, bits: u32) -> Option<Instruction> {
        let top = (bits >> 16) as u16;
        let rt = ((bits >> 12) & 0xF) as u8;
        let rn = (top & 0xF) as u8;
        let imm12 = bits & 0xFFF;
        // 寄存器偏移形式：top & 0x0FF0 == 0x0850(LDR)/0x0840(STR)
        // （imm12 形式为 0x08D0/0x08C0/0x08B0/0x08A0/0x0890/0x0880/0x09B0/0x0990，勿误判）
        let is_reg_offset = (top & 0x0FF0) == 0x0850 || (top & 0x0FF0) == 0x0840;
        if is_reg_offset {
            let rm = (bits & 0xF) as u8;
            let is_load = (top & 0x0010) != 0;
            let offset = LoadStoreOffset::Register(rm);
            return Some(if is_load {
                Instruction::Ldr {
                    rt,
                    rn,
                    offset,
                    width: AccessWidth::Word,
                }
            } else {
                Instruction::Str {
                    rt,
                    rn,
                    offset,
                    width: AccessWidth::Word,
                }
            });
        }
        match top & 0xFFF0 {
            // LDR.W (imm12)；Rn=1111 → LDR literal
            0xF8D0 if rn == 0xF => Some(Instruction::LdrLiteral { rt, imm: imm12 }),
            0xF8D0 => Some(Instruction::Ldr {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::Word,
            }),
            // STR.W
            0xF8C0 => Some(Instruction::Str {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::Word,
            }),
            // LDRH.W
            0xF8B0 => Some(Instruction::Ldr {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::HalfWord,
            }),
            // STRH.W
            0xF8A0 => Some(Instruction::Str {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::HalfWord,
            }),
            // LDRB.W
            0xF890 => Some(Instruction::Ldr {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::Byte,
            }),
            // STRB.W
            0xF880 => Some(Instruction::Str {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::Byte,
            }),
            // LDRSH.W（符号扩展半字）
            0xF9B0 => Some(Instruction::LdrSignExtend {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::HalfWord,
            }),
            // LDRSB.W（符号扩展字节）
            0xF990 => Some(Instruction::LdrSignExtend {
                rt,
                rn,
                offset: LoadStoreOffset::Immediate(imm12),
                width: AccessWidth::Byte,
            }),
            _ => None,
        }
    }

    /// LDRD/STRD 双字（0xE9D0/0xE9C0 + rn | rt rt2 imm8×4）
    fn decode_ldrd_strd(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let rn = (top & 0xF) as u8;
        let rt = ((bits >> 12) & 0xF) as u8;
        let rt2 = ((bits >> 8) & 0xF) as u8;
        let imm = ((bits & 0xFF) as u32) * 4;
        if (top & 0xFFF0) == 0xE9D0 {
            Instruction::LdrD { rt, rt2, rn, imm }
        } else {
            Instruction::StrD { rt, rt2, rn, imm }
        }
    }

    /// 解码 MRS（0xF3EF，低半字 10R0 Rd 0000 0000 SYSm）
    /// SYSm 编码表（ARM DDI 0406C B3.3）：0x00=APSR、0x05=IPSR、0x06=EPSR、
    /// 0x08=MSP、0x09=PSP、0x10=PRIMASK、0x11=BASEPRI、0x13=FAULTMASK、0x14=CONTROL。
    /// 不支持的 SYSm（含 0x12 BASEPRI_MAX，读为 UNPREDICTABLE）→ 诚实 Unimplemented。
    fn decode_mrs(&self, bits: u32) -> Instruction {
        let rd = ((bits >> 8) & 0xF) as u8;
        let sysm = (bits & 0xFF) as u8;
        let reg = match sysm {
            0x00 => SpecialReg::Apsr,
            // M-profile SYSm：0x01=IAPSR（IPSR+APSR）、0x02=EAPSR（EPSR+APSR）
            // （A-profile 的 APSR_nzcvqg 表不适用，P1-7 修复）
            0x01 => SpecialReg::Iapsr,
            0x02 => SpecialReg::Eapsr,
            0x05 => SpecialReg::Ipsr,
            0x06 => SpecialReg::Epsr,
            0x08 => SpecialReg::Msp,
            0x09 => SpecialReg::Psp,
            0x10 => SpecialReg::Primask,
            0x11 => SpecialReg::Basepri,
            0x13 => SpecialReg::Faultmask,
            0x14 => SpecialReg::Control,
            _ => return Instruction::Unimplemented { bits },
        };
        Instruction::MsrMrs { rt: rd, reg, read: true }
    }

    /// 解码 MSR（0xF38x，低半字 10R0 Rn 0000 mask SYSm）
    /// mask bit10（低半字 bit10）=1 时 APSR 同时写 GE（APSR_nzcvqg）。
    fn decode_msr(&self, bits: u32) -> Instruction {
        let rn = ((bits >> 16) & 0xF) as u8;
        let sysm = (bits & 0xFF) as u8;
        let ge = (bits & 0x0400) != 0;
        let reg = match sysm {
            0x00 => {
                if ge {
                    SpecialReg::ApsrGe
                } else {
                    SpecialReg::Apsr
                }
            }
            0x08 => SpecialReg::Msp,
            0x09 => SpecialReg::Psp,
            0x10 => SpecialReg::Primask,
            0x11 => SpecialReg::Basepri,
            0x12 => SpecialReg::BasepriMax,
            0x13 => SpecialReg::Faultmask,
            0x14 => SpecialReg::Control,
            _ => return Instruction::Unimplemented { bits },
        };
        Instruction::MsrMrs { rt: rn, reg, read: false }
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
    }    /// 解码 QADD/QSUB/QDADD/QDSUB（top=0xFA8x，低半字 bits[7:4]=1000-1011）
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

    /// 解码 8-bit SIMD（SADD8/SSUB8/UADD8/USUB8/SHADD8/SHSUB8/UHADD8/UHSUB8）
    /// 编码（汇编器实测）：top = 0xFA8x（加家族）/ 0xFACx（减家族，bit22=1），
    /// 低半字 0xF0xx：bit21 = 无符号（U），bit20 = 减半（H）
    fn decode_simd8(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let rn = (top & 0xF) as u8;
        let rd = ((bits >> 8) & 0xF) as u8;
        let rm = (bits & 0xF) as u8;
        let unsigned = (bits & 0x40) != 0; // bit21
        let halving = (bits & 0x20) != 0; // bit20
        let sub = (top & 0x40) != 0; // bit22：0xFA8x 加 / 0xFACx 减
        Instruction::Simd8 {
            rd,
            rn,
            rm,
            unsigned,
            halving,
            sub,
        }
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
    }    /// 解码 SMUAD/SMUSD/SMLAD/SMLSD
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

    /// UBFX/SBFX/BFI/BFC 位域操作解码（FRT-INS-05 SHOULD）
    /// 编码（arm-none-eabi-as 实测）：
    ///   ubfx r0,r1,#3,#5 = 0xF3C1 00C4；sbfx = 0xF341 00C4；bfi = 0xF361 00C7；bfc = 0xF36F 00C7
    /// UBFX/SBFX：lsb = imm3:imm2（bits[14:12]:bits[7:6]），width = widthm1(bits[4:0])+1
    /// BFI/BFC：bits[4:0] = msb，lsb = imm3:imm2，width = msb-lsb+1；BFC 的 Rn=1111
    fn decode_bitfield(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let imm3 = ((bits >> 12) & 0x7) as u8;
        let rd = ((bits >> 8) & 0xF) as u8;
        let imm2 = ((bits >> 6) & 0x3) as u8;
        let rn = (top & 0xF) as u8;
        match top & 0xFFF0 {
            // UBFX：无符号位域提取
            0xF3C0 => {
                let lsb = (imm3 << 2) | imm2;
                let width = (bits & 0x1F) as u8 + 1;
                Instruction::BitField {
                    rd,
                    rn,
                    lsb,
                    width,
                    kind: BitFieldKind::Ubfx,
                }
            }
            // SBFX：有符号位域提取（结果符号扩展）
            0xF340 => {
                let lsb = (imm3 << 2) | imm2;
                let width = (bits & 0x1F) as u8 + 1;
                Instruction::BitField {
                    rd,
                    rn,
                    lsb,
                    width,
                    kind: BitFieldKind::Sbfx,
                }
            }
            // BFI/BFC：位域插入/清除（Rn=1111 → BFC，无源寄存器）
            _ => {
                let msb = (bits & 0x1F) as u8;
                let lsb = (imm3 << 2) | imm2;
                let width = msb - lsb + 1;
                let kind = if rn == 0xF {
                    BitFieldKind::Bfc
                } else {
                    BitFieldKind::Bfi
                };
                Instruction::BitField {
                    rd,
                    rn,
                    lsb,
                    width,
                    kind,
                }
            }
        }
    }

    /// LDREX/STREX 家族解码（FRT-INS-05 SHOULD，单核语义）
    /// 编码（arm-none-eabi-as 实测）：
    ///   LDREX Rt,[Rn,#imm8×4] = 0xE850|Rn 低= Rt:1111 1001 imm8
    ///   STREX Rd,Rt,[Rn,#imm8×4] = 0xE840|Rn 低= Rt:Rd:1111 1001 imm8
    ///   LDREXB = 0xE8D0|Rn 低= Rt:1111 0100 1111；LDREXH 低= Rt:1111 0101 1111
    ///   STREXB = 0xE8C0|Rn 低= Rt:1111 0100 Rd；STREXH 低= Rt:1111 0101 Rd
    fn decode_ldrex_strex(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let rn = (top & 0xF) as u8;
        let rt = ((bits >> 12) & 0xF) as u8;
        match (top & 0xFFF0, bits & 0x0FFF, bits & 0x0FF0) {
            (0xE850, _, _) => Instruction::Ldrex {
                rt,
                rn,
                imm: (bits & 0xFF) * 4,
                width: AccessWidth::Word,
            },
            (0xE840, _, _) => Instruction::Strex {
                rd: ((bits >> 8) & 0xF) as u8,
                rt,
                rn,
                imm: (bits & 0xFF) * 4,
                width: AccessWidth::Word,
            },
            // LDREXB/LDREXH：低半字固定 1111 0100/0101 1111
            (0xE8D0, 0x0F4F, _) => Instruction::Ldrex {
                rt,
                rn,
                imm: 0,
                width: AccessWidth::Byte,
            },
            (0xE8D0, 0x0F5F, _) => Instruction::Ldrex {
                rt,
                rn,
                imm: 0,
                width: AccessWidth::HalfWord,
            },
            // STREXB/STREXH：低半字固定 1111 0100/0101 + bits[3:0]=Rd
            (0xE8C0, _, 0x0F40) => Instruction::Strex {
                rd: (bits & 0xF) as u8,
                rt,
                rn,
                imm: 0,
                width: AccessWidth::Byte,
            },
            (0xE8C0, _, 0x0F50) => Instruction::Strex {
                rd: (bits & 0xF) as u8,
                rt,
                rn,
                imm: 0,
                width: AccessWidth::HalfWord,
            },
            _ => Instruction::Unimplemented { bits },
        }
    }

    /// 解码 VFP 指令（top 0xEE00 家族），返回 None 表示非 VFP/未实现
    fn try_decode_fpu(&self, bits: u32) -> Option<Instruction> {
        let top = (bits >> 16) as u16;
        let low = bits as u16;
        let sz = (low >> 8) & 1; // X = size: 0=SP, 1=DP
        let sz_is_double = sz == 1;

        // ---- VMOV Sn, Rt / Rt, Sn（核心寄存器互传）：top bit21=0，低半字 bit4=1 ----
        if (top & 0x20) == 0 && (low & 0x10) != 0 && (low & 0x60) == 0 && (low & 0x0E00) == 0x0A00 {
            let rt = ((low >> 12) & 0xF) as u8;
            let sn = (((top & 0xF) as u8) << 1) | ((low >> 7) & 1) as u8;
            let to_core = (top & 0x10) != 0;
            return Some(Instruction::FpVmovCore { rt, sn, to_core });
        }

        let opc = (((top >> 7) & 1) << 2) | (((top >> 5) & 1) << 1) | ((top >> 4) & 1);
        // ---- 三寄存器运算（VMLA..VDIV）：低半字 bit4=0，bits[11:9]=101 ----
        if (low & 0x10) == 0 && (low & 0x0E00) == 0x0A00 {
            match opc {
                0..=3 => {
                    // 单精度: Vd=(bits[15:12]<<1)|bit22, Vn=(bits[19:16]<<1)|bit7, Vm=(bits[3:0]<<1)|bit5
                    let vd = if sz_is_double {
                        ((low >> 12) & 0xF) as u8
                    } else {
                        (((low >> 12) as u8) << 1) | ((top >> 6) & 1) as u8
                    };
                    let vn = if sz_is_double {
                        (top & 0xF) as u8
                    } else {
                        (((top & 0xF) as u8) << 1) | ((low >> 7) & 1) as u8
                    };
                    let vm = if sz_is_double {
                        (low & 0xF) as u8
                    } else {
                        (((low & 0xF) as u8) << 1) | ((low >> 5) & 1) as u8
                    };
                    let sub = (low & 0x40) != 0;
                    let op = match (opc, sub) {
                        (0, false) => FpArithOp::Vmla,
                        (0, true) => FpArithOp::Vmls,
                        (1, false) => FpArithOp::Vnmls,
                        (1, true) => FpArithOp::Vnmla,
                        (2, false) => FpArithOp::Vmul,
                        (2, true) => FpArithOp::Vnmul,
                        (3, false) => FpArithOp::Vadd,
                        _ => FpArithOp::Vsub,
                    };
                    return Some(Instruction::FpArith3 {
                        op,
                        vd,
                        vn,
                        vm,
                        double: sz_is_double,
                    });
                }
                4 if (top & 0x30) == 0 => {
                    // VDIV（bit23=1，bits[21:20]=00）
                    let vd = if sz_is_double {
                        ((low >> 12) & 0xF) as u8
                    } else {
                        (((low >> 12) as u8) << 1) | ((top >> 6) & 1) as u8
                    };
                    let vn = if sz_is_double {
                        (top & 0xF) as u8
                    } else {
                        (((top & 0xF) as u8) << 1) | ((low >> 7) & 1) as u8
                    };
                    let vm = if sz_is_double {
                        (low & 0xF) as u8
                    } else {
                        (((low & 0xF) as u8) << 1) | ((low >> 5) & 1) as u8
                    };
                    return Some(Instruction::FpArith3 {
                        op: FpArithOp::Vdiv,
                        vd,
                        vn,
                        vm,
                        double: sz_is_double,
                    });
                }
                7 if (top & 0x30) == 0x30 => {
                    // ---- 二寄存器家族（VMOV/VABS/VNEG/VSQRT/VCMP/VCVT/VMOV-imm）----
                    return Some(self.decode_fpu_2reg(bits));
                }
                _ => {}
            }
        }
        None
    }

    /// 解码 VFP 二寄存器指令（top 0xEEB0+，bits[21:20]=11）
    fn decode_fpu_2reg(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let low = bits as u16;
        let sz = (low >> 8) & 1;
        let sz_is_double = sz == 1;
        // opc5 = bits[19:16] + bit7
        let op5 = (((top & 0xF) as u8) << 1) | ((low >> 7) & 1) as u8;
        // 单精度寄存器解码
        let sp_vd = |w: u32| (((w >> 12) & 0xF) as u8) << 1 | ((w >> 22) & 1) as u8;
        let sp_vm = |w: u32| (((w & 0xF) as u8) << 1) | ((w >> 5) & 1) as u8;
        let dp_vd = |w: u32| ((w >> 12) & 0xF) as u8;
        let dp_vm = |w: u32| (w & 0xF) as u8;
        let (vd, vm) = if sz_is_double {
            (dp_vd(bits), dp_vm(bits))
        } else {
            (sp_vd(bits), sp_vm(bits))
        };

        // ---- VCVT 定点（0xEEBA/0xEEBB = 定点→F32，0xEEBE/0xEEBF = F32→定点）----
        // bits[7:4] = 0x4（16 位）/ 0xC（32 位），fbits = N - (bits[3:0] << 1)；
        // Vd 与 Vm 同寄存器（编码无独立 Vm 字段）。
        if matches!(top & 0xF, 0xA | 0xB | 0xE | 0xF) {
            let width_marker = (bits >> 4) & 0xF;
            if width_marker == 0x4 || width_marker == 0xC {
                let width = if width_marker == 0x4 { 16u8 } else { 32u8 };
                let n = width as u32;
                let fb = (bits & 0xF) << 1;
                if fb < n {
                    let to_float = matches!(top & 0xF, 0xA | 0xB);
                    let signed = matches!(top & 0xF, 0xA | 0xE);
                    return Instruction::FpCvtFixed {
                        vd,
                        fbits: (n - fb) as u8,
                        width,
                        signed,
                        to_float,
                    };
                }
            }
            return Instruction::Unimplemented { bits };
        }

        match op5 {
            _ if (low & 0xF0) == 0 => {
                // VMOV（立即数）：bits[7:4]=0000，imm8 = bits[19:16]<<4 | bits[3:0]
                let imm8 = (((top & 0xF) << 4) | (low & 0xF)) as u8;
                let imm = crate::engine::fpu::vfp_expand_imm(imm8, sz_is_double);
                Instruction::FpVmovImm {
                    sd: vd,
                    imm,
                    double: sz_is_double,
                }
            }
            0b00000 => Instruction::FpVmovReg {
                sd: vd,
                sm: vm,
                double: sz_is_double,
            },
            0b00001 => Instruction::FpUnary {
                op: FpUnaryOp::Vabs,
                vd,
                vm,
                double: sz_is_double,
            },
            0b00010 => Instruction::FpUnary {
                op: FpUnaryOp::Vneg,
                vd,
                vm,
                double: sz_is_double,
            },
            0b00011 => Instruction::FpUnary {
                op: FpUnaryOp::Vsqrt,
                vd,
                vm,
                double: sz_is_double,
            },
            0b01000 => Instruction::FpCmp {
                vd,
                vm,
                double: sz_is_double,
                e: false,
                zero: false,
            },
            0b01001 => Instruction::FpCmp {
                vd,
                vm,
                double: sz_is_double,
                e: true,
                zero: false,
            },
            0b01010 => Instruction::FpCmp {
                vd,
                vm,
                double: sz_is_double,
                e: false,
                zero: true,
            },
            0b01011 => Instruction::FpCmp {
                vd,
                vm,
                double: sz_is_double,
                e: true,
                zero: true,
            },
            0b01111 => {
                // VCVT F64↔F32：sz 决定方向；目标寄存器字段也随方向变化
                if sz_is_double {
                    // VCVT.F32.F64 Sd, Dm
                    let sd = sp_vd(bits);
                    let dm = dp_vm(bits);
                    Instruction::FpCvt {
                        op: FpCvtOp::F32ToF64,
                        vd: sd,
                        vm: dm,
                    }
                } else {
                    // VCVT.F64.F32 Dd, Sm
                    let dd = dp_vd(bits);
                    let sm = sp_vm(bits);
                    Instruction::FpCvt {
                        op: FpCvtOp::F64ToF32,
                        vd: dd,
                        vm: sm,
                    }
                }
            }
            0b10000 | 0b10001 | 0b11000 | 0b11001 | 0b11010 | 0b11011 => {
                // VCVT 整数↔浮点（1100x = U32，1101x = S32）
                if sz_is_double {
                    // 整数→F64: Dd ← Sm；F64→整数: Sd ← Dm
                    if op5 == 0b10000 || op5 == 0b10001 {
                        let dd = dp_vd(bits);
                        let sm = sp_vm(bits);
                        let op = if op5 == 0b10000 {
                            FpCvtOp::U32ToF64
                        } else {
                            FpCvtOp::S32ToF64
                        };
                        Instruction::FpCvt { op, vd: dd, vm: sm }
                    } else {
                        let sd = sp_vd(bits);
                        let dm = dp_vm(bits);
                        let op = match op5 {
                            0b11000 => FpCvtOp::F64ToU32R,
                            0b11001 => FpCvtOp::F64ToU32,
                            0b11010 => FpCvtOp::F64ToS32R,
                            _ => FpCvtOp::F64ToS32,
                        };
                        Instruction::FpCvt { op, vd: sd, vm: dm }
                    }
                } else {
                    let op = match op5 {
                        0b10000 => FpCvtOp::U32ToF32,
                        0b10001 => FpCvtOp::S32ToF32,
                        0b11000 => FpCvtOp::F32ToU32R,
                        0b11001 => FpCvtOp::F32ToU32,
                        0b11010 => FpCvtOp::F32ToS32R,
                        _ => FpCvtOp::F32ToS32,
                    };
                    Instruction::FpCvt { op, vd, vm }
                }
            }
            _ => Instruction::Unimplemented { bits },
        }
    }

    /// 解码 VLDM/VSTM（编码 1110 110P U D L Rn Vd 101x imm8）
    /// SP 寄存器索引 = (Vd<<1)|D；DP 索引 = D0-D15（D=1 → D16+ 未支持，诚实 Unimplemented）
    fn decode_vldm_vstm(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let p = (top >> 8) & 1 == 1; // bit24
        let u = (top >> 7) & 1 == 1; // bit23
        let d = (top >> 6) & 1 == 1; // bit22
        let w = (top >> 5) & 1 == 1; // bit21
        let l = (top >> 4) & 1 == 1; // bit20
        let rn = (top & 0xF) as u8;
        let vd_field = ((bits >> 12) & 0xF) as u8;
        let double = (bits >> 8) & 1 == 1; // sz
        let imm8 = bits & 0xFF;
        // 仅 IA（P=0,U=1）与 DB（P=1,U=0）为合法组合
        if p == u {
            return Instruction::Unimplemented { bits };
        }
        let decrement = !u; // DB
        // 寄存器个数：imm8 为字数；SP 每寄存器 1 字，DP 每寄存器 2 字
        let count = imm8 / if double { 2 } else { 1 };
        let vd = if double {
            if d {
                // D16-D31 超出 FpuRegisters D0-D15 骨架
                return Instruction::Unimplemented { bits };
            }
            vd_field
        } else {
            (vd_field << 1) | d as u8
        };
        Instruction::FpLoadStoreMulti {
            vd,
            rn,
            count,
            load: l,
            double,
            decrement,
            writeback: w,
        }
    }

    /// 解码 VLDR/VSTR
    fn decode_fpu_loadstore(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let sz = (bits >> 8) & 1;
        let u = (top >> 7) & 1; // bit23: 偏移符号
        let load = (top >> 4) & 1 == 1; // bit20: L
        let rn = (top & 0xF) as u8;
        let imm = (bits & 0xFF) as u32 * 4; // imm8 × 4（单/双精度一致）
        let offset = if u == 1 { imm } else { imm.wrapping_neg() };
        let rt = if sz == 1 {
            ((bits >> 12) & 0xF) as u8
        } else {
            (((bits >> 12) & 0xF) as u8) << 1 | ((top >> 6) & 1) as u8
        };
        Instruction::FpLoadStore {
            rt,
            rn,
            offset,
            load,
            double: sz == 1,
        }
    }

    /// 解码 VMOV Dd, Rt, Rt2 / Rt, Rt2, Dd
    fn decode_vmov_core_d(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let rt = ((bits >> 12) & 0xF) as u8;
        let rt2 = (top & 0xF) as u8;
        let dn = ((((top >> 6) & 1) as u8) << 4) | ((bits & 0xF) as u8); // D:bits[3:0]
        let to_core = (top & 0x10) != 0;
        Instruction::FpVmovCoreD {
            rt,
            rt2,
            dn,
            to_core,
        }
    }

    /// 16-bit LDR/STR 立即数偏移：01100-01111（STR/LDR/STRB/LDRB）与
    /// 10000/10001（STRH/LDRH）。真实 Thumb-1 编码（objdump 实测）：
    /// - 01100 STR (imm5×4)、01101 LDR (imm5×4)
    /// - 01110 STRB (imm5)、01111 LDRB (imm5)
    /// - 10000 STRH (imm5×2)、10001 LDRH (imm5×2)
    fn decode_ldr_str_imm(&self, bits: u16) -> Instruction {
        let rt = (bits & 0x7) as u8;
        let rn = ((bits >> 3) & 0x7) as u8;
        let imm5 = ((bits >> 6) & 0x1F) as u32;
        let (load, width, scale) = match bits >> 11 {
            0b01100 => (false, AccessWidth::Word, 4),     // STR
            0b01101 => (true, AccessWidth::Word, 4),      // LDR
            0b01110 => (false, AccessWidth::Byte, 1),     // STRB
            0b01111 => (true, AccessWidth::Byte, 1),      // LDRB
            0b10000 => (false, AccessWidth::HalfWord, 2), // STRH
            _ => (true, AccessWidth::HalfWord, 2),        // LDRH
        };
        let offset = LoadStoreOffset::Immediate(imm5 * scale);
        if load {
            Instruction::Ldr {
                rt,
                rn,
                offset,
                width,
            }
        } else {
            Instruction::Str {
                rt,
                rn,
                offset,
                width,
            }
        }
    }

    /// 16-bit LDR/STR 寄存器偏移：0101 op2[2:0] Rm Rn Rt（真实编码，GNU as 实测）
    /// op2：000=STR、001=STRH、010=STRB、011=LDRSB（符号扩展）、100=LDR、
    ///      101=LDRH、110=LDRB、111=UNDEF（保留）
    fn decode_ldr_str_reg(&self, bits: u16) -> Instruction {
        let rt = (bits & 0x7) as u8;
        let rn = ((bits >> 3) & 0x7) as u8;
        let rm = ((bits >> 6) & 0x7) as u8;
        let offset = LoadStoreOffset::Register(rm);
        match ((bits >> 9) & 0x7) as u8 {
            // 000: STR（字）
            0 => Instruction::Str {
                rt,
                rn,
                offset,
                width: AccessWidth::Word,
            },
            // 001: STRH（半字）
            1 => Instruction::Str {
                rt,
                rn,
                offset,
                width: AccessWidth::HalfWord,
            },
            // 010: STRB（字节）
            2 => Instruction::Str {
                rt,
                rn,
                offset,
                width: AccessWidth::Byte,
            },
            // 011: LDRSB（有符号字节加载）
            3 => Instruction::LdrSignExtend {
                rt,
                rn,
                offset,
                width: AccessWidth::Byte,
            },
            // 100: LDR（字）
            4 => Instruction::Ldr {
                rt,
                rn,
                offset,
                width: AccessWidth::Word,
            },
            // 101: LDRH（半字）
            5 => Instruction::Ldr {
                rt,
                rn,
                offset,
                width: AccessWidth::HalfWord,
            },
            // 110: LDRB（字节）
            6 => Instruction::Ldr {
                rt,
                rn,
                offset,
                width: AccessWidth::Byte,
            },
            // 111: 保留（UNDEF）→ 诚实 Invalid
            _ => Instruction::Invalid { address: 0 },
        }
    }

    /// 16-bit SP 相对 STR/LDR（10010/10011，imm8×4，Rt 在 bits[10:8]）
    fn decode_sp_relative(&self, bits: u16) -> Instruction {
        let rt = ((bits >> 8) & 0x7) as u8;
        let imm = ((bits & 0xFF) as u32) * 4;
        let offset = LoadStoreOffset::Immediate(imm);
        if (bits >> 11) & 1 == 1 {
            Instruction::Ldr {
                rt,
                rn: 13, // SP
                offset,
                width: AccessWidth::Word,
            }
        } else {
            Instruction::Str {
                rt,
                rn: 13,
                offset,
                width: AccessWidth::Word,
            }
        }
    }

    /// 16-bit STMIA（11000，Rn 为 R0-R7，固定回写）
    fn decode_stmia(&self, bits: u16) -> Instruction {
        let rn = ((bits >> 8) & 0x7) as u8;
        let regs = (bits & 0xFF) as u16;
        Instruction::Stm {
            rn,
            regs,
            writeback: true,
            descending: false,
        }
    }

    /// 16-bit LDMIA（11001，Rn 为 R0-R7，固定回写）
    fn decode_ldmia(&self, bits: u16) -> Instruction {
        let rn = ((bits >> 8) & 0x7) as u8;
        let regs = (bits & 0xFF) as u16;
        Instruction::Ldm {
            rn,
            regs,
            writeback: true,
            descending: false,
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
        // ARMv7-M：LSR/ASR 立即数 imm5=0 编码表示 #32（移满 → LSR=0 / ASR=符号填充），
        // 必须在 decode 端转换为 32，exec 才能按 n>=32 语义执行；LSL #0 为不移位，保持原值。
        let amount = match (kind, imm5) {
            (ShiftKind::Lsr | ShiftKind::Asr, 0) => ShiftAmount::Immediate(32),
            _ => ShiftAmount::Immediate(imm5 as u8),
        };
        Instruction::Shift {
            rd,
            rm,
            kind,
            amount,
            flags: true,
        }
    }    /// 加减: 000 11 x xxxxx xxxxx
    ///   bit10=0 寄存器形式: 00011 0 op Rm Rn Rd → ADD/SUB Rd, Rn, Rm
    ///   bit10=1 立即数形式: 00011 1 op imm3 Rn Rd → ADD/SUB Rd, Rn, #imm3
    /// （编码位布局经 arm-none-eabi-as 实测：0x1A9B = subs r3, r3, r2；0x1C5A = adds r2, r3, #1）
    fn decode_add_sub_imm(&self, bits: u16) -> Instruction {
        let rd = (bits & 0x7) as u8;
        let rn = ((bits >> 3) & 0x7) as u8;
        let op = (bits >> 9) & 0x1; // 0=ADD, 1=SUB
        let (rm, imm) = if (bits >> 10) & 1 == 0 {
            // 寄存器形式：Rm = bits[8:6]
            (Some(((bits >> 6) & 0x7) as u8), None)
        } else {
            // 立即数形式：imm3 = bits[8:6]
            (None, Some(((bits >> 6) & 0x7) as u32))
        };
        // A9 实测修正：立即数形式恒置标志（ADDS/SUBS）；寄存器形式仅
        // SUB（SUBS，0x1A00-0x1A7F）置标志，ADD（0x1800-0x187F）不置标志。
        let flags = ((bits >> 10) & 1) == 1 || op == 1;
        if op == 0 {
            Instruction::Add {
                rd,
                rn,
                rm,
                imm,
                flags,
            }
        } else {
            Instruction::Sub {
                rd,
                rn,
                rm,
                imm,
                flags,
            }
        }
    }

    /// 0x4000-0x47FF：寄存器数据处理（bit10=0）+ 高寄存器 ADD/CMP/MOV + BX/BLX（bit10=1）
    ///
    /// bit10=0（0x4000-0x43FF）为真实 Thumb-1 寄存器数据处理组：
    /// `010000 op[3:0] Rm Rd`（op = bits[9:6]），十六种操作：
    ///   AND/EOR/LSL(reg)/LSR(reg)/ASR(reg)/ADC/SBC/ROR(reg)/TST/NEG/CMP/CMN/ORR/MUL/BIC/MVN
    /// 除 MUL 外全部更新标志（ARMv7E-M 语义：MUL 不更新 flags；编码与 GNU as 实测一致）。
    /// 寄存器移位（LSL/LSR/ASR/ROR reg 形式）的移位量 Rs = bits[5:3]，Rd 既是源也是目的。
    ///
    /// bit10=1 时（0x4400-0x47FF）：
    ///   op = bits[9:8]：00=ADD、01=CMP、10=MOV（H1=bit7 / H2=bit6 扩展高位寄存器）、
    ///   11=BX（bit5=0）/ BLX（bit5=1）
    fn decode_data_proc_high(&self, bits: u16) -> Instruction {
        if (bits >> 10) & 1 == 0 {
            let rd = (bits & 0x7) as u8;
            let rm = ((bits >> 3) & 0x7) as u8;
            match ((bits >> 6) & 0xF) as u8 {
                // 0000 ANDS Rd, Rd, Rm
                0x0 => Instruction::And {
                    rd,
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                    flags: true,
                },
                // 0001 EORS Rd, Rd, Rm
                0x1 => Instruction::Eor {
                    rd,
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                    flags: true,
                },
                // 0010 LSLS Rd, Rd, Rs（寄存器移位：Rd 同时是源与目的，Rs = bits[5:3]）
                0x2 => Instruction::Shift {
                    rd,
                    rm: rd,
                    kind: ShiftKind::Lsl,
                    amount: ShiftAmount::Register(rm),
                    flags: true,
                },
                // 0011 LSRS Rd, Rd, Rs
                0x3 => Instruction::Shift {
                    rd,
                    rm: rd,
                    kind: ShiftKind::Lsr,
                    amount: ShiftAmount::Register(rm),
                    flags: true,
                },
                // 0100 ASRS Rd, Rd, Rs
                0x4 => Instruction::Shift {
                    rd,
                    rm: rd,
                    kind: ShiftKind::Asr,
                    amount: ShiftAmount::Register(rm),
                    flags: true,
                },
                // 0101 ADCS Rd, Rd, Rm
                0x5 => Instruction::Adc {
                    rd,
                    rn: rd,
                    rm,
                    flags: true,
                },
                // 0110 SBCS Rd, Rd, Rm
                0x6 => Instruction::Sbc {
                    rd,
                    rn: rd,
                    rm,
                    flags: true,
                },
                // 0111 RORS Rd, Rd, Rs
                0x7 => Instruction::Shift {
                    rd,
                    rm: rd,
                    kind: ShiftKind::Ror,
                    amount: ShiftAmount::Register(rm),
                    flags: true,
                },
                // 1000 TST Rd, Rm
                0x8 => Instruction::Tst {
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                },
                // 1001 NEGS Rd, Rm（= RSB Rd, Rm, #0）
                0x9 => Instruction::Neg {
                    rd,
                    rn: rm,
                    flags: true,
                },
                // 1010 CMP Rd, Rm
                0xA => Instruction::Cmp {
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                },
                // 1011 CMN Rd, Rm
                0xB => Instruction::Cmn {
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                },
                // 1100 ORRS Rd, Rd, Rm
                0xC => Instruction::Orr {
                    rd,
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                    flags: true,
                },
                // 1101 MULS Rd, Rm, Rd（ARMv7E-M：不更新 flags）
                0xD => Instruction::Mul {
                    rd,
                    rn: rd,
                    rm,
                    flags: false,
                },
                // 1110 BICS Rd, Rd, Rm
                0xE => Instruction::Bic {
                    rd,
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                    flags: true,
                },
                // 1111 MVNS Rd, Rm
                _ => Instruction::Mvn { rd, rm, flags: true },
            }
        } else {
            let h1 = (bits >> 7) & 1;
            let h2 = (bits >> 6) & 1;
            let rd = ((h1 << 3) | (bits & 0x7)) as u8;
            let rm = ((h2 << 3) | ((bits >> 3) & 0x7)) as u8;
            match (bits >> 8) & 0x3 {
                // ADD（高寄存器形式不更新标志）
                0 => Instruction::Add {
                    rd,
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                    flags: false,
                },
                // CMP
                1 => Instruction::Cmp {
                    rn: rd,
                    rm: Some(rm),
                    imm: None,
                },
                // MOV（高寄存器形式不更新标志）
                2 => Instruction::Mov {
                    rd,
                    rm,
                    imm: None,
                    flags: false,
                },
                // BX / BLX（bit7=0 → BX，bit7=1 → BLX；Rm = H2:bits[5:3]）
                _ => {
                    if (bits >> 7) & 1 == 0 {
                        Instruction::BranchExchange { rm }
                    } else {
                        Instruction::BranchLinkExchange { rm }
                    }
                }
            }
        }
    }

    /// LDR literal（PC 相对字面量）: 01001 Rt imm8
    /// 执行地址 = Align(PC+4, 4) + imm8×4（PC 为指令地址，exec 端统一计算）
    fn decode_ldr_literal(&self, bits: u16) -> Instruction {
        let rt = ((bits >> 8) & 0x7) as u8;
        let imm = ((bits & 0xFF) as u32) << 2;
        Instruction::LdrLiteral { rt, imm }
    }

    /// ADD Rd, PC/SP, #imm8×4: 10100=PC（ADR 语义），10101=SP
    fn decode_add_pc_sp_imm(&self, bits: u16) -> Instruction {
        let rd = ((bits >> 8) & 0x7) as u8;
        let rn = if (bits >> 11) & 1 == 0 { 15 } else { 13 };
        let imm = ((bits & 0xFF) as u32) * 4;
        Instruction::Add {
            rd,
            rn,
            rm: None,
            imm: Some(imm),
            flags: false,
        }
    }

    /// 0xB000-0xB7FF：ADD/SUB SP（000）、CBZ（001）、PUSH（100/101）
    fn decode_b_misc(&self, bits: u16, pc: u32) -> Instruction {
        match (bits >> 8) & 0x7 {
            // ADD SP, #imm7×4（bit7=0）/ SUB SP, #imm7×4（bit7=1）
            0 => {
                let imm = ((bits & 0x7F) as u32) * 4;
                if (bits >> 7) & 1 == 0 {
                    Instruction::Add {
                        rd: 13,
                        rn: 13,
                        rm: None,
                        imm: Some(imm),
                        flags: false,
                    }
                } else {
                    Instruction::Sub {
                        rd: 13,
                        rn: 13,
                        rm: None,
                        imm: Some(imm),
                        flags: false,
                    }
                }
            }
            // CBZ（0xB100-0xB1FF）：i=bit9，imm5=bits[7:3]，Rn=bits[2:0]
            1 => {
                let rn = (bits & 0x7) as u8;
                let imm5 = ((bits >> 3) & 0x1F) as u32;
                let i = ((bits >> 9) & 1) as u32;
                let target = pc.wrapping_add(4).wrapping_add((i << 6) | (imm5 << 1));
                Instruction::CompareBranch {
                    rn,
                    target,
                    zero: true,
                }
            }
            // PUSH {regs}（bit8=0）/ PUSH {regs, lr}（bit8=1）
            4 => Instruction::Push {
                regs: (bits & 0xFF) as u16,
                lr: false,
            },
            5 => Instruction::Push {
                regs: (bits & 0xFF) as u16,
                lr: true,
            },
            // CPSIE/CPSID（0xB660-0xB67F，FRT-INS-01）：
            // 编码（arm-none-eabi-as 实测）：cpsie i=0xB662/cpsid i=0xB672/
            // cpsie f=0xB661/cpsid f=0xB671——bit4=imod（0=IE 使能/1=ID 禁止），
            // bit1=i（PRIMASK）、bit0=f（FAULTMASK）
            6 => {
                if (bits & 0xFFE0) == 0xB660 {
                    Instruction::Cps {
                        disable: (bits >> 4) & 1 == 1,
                        i: bits & 0x2 != 0,
                        f: bits & 0x1 != 0,
                    }
                } else {
                    Instruction::Unimplemented {
                        bits: bits as u32,
                    }
                }
            }
            _ => Instruction::Unimplemented {
                bits: bits as u32,
            },
        }
    }

    /// 0xB800-0xBFFF：CBNZ（001）、POP（100/101）、BKPT（110）、IT/提示（111）
    fn decode_b_top(&self, bits: u16, pc: u32) -> Instruction {
        match (bits >> 8) & 0x7 {
            // CBNZ（0xB900-0xB9FF）
            1 => {
                let rn = (bits & 0x7) as u8;
                let imm5 = ((bits >> 3) & 0x1F) as u32;
                let i = ((bits >> 9) & 1) as u32;
                let target = pc.wrapping_add(4).wrapping_add((i << 6) | (imm5 << 1));
                Instruction::CompareBranch {
                    rn,
                    target,
                    zero: false,
                }
            }
            // REV/REV16/REVSH（0xBA00-0xBAFF，FRT-INS-05 SHOULD）：
            // 编码（as 实测）：REV=0xBA08/REV16=0xBA48/REVSH=0xBAC8——
            // 1011 1010 op[1:0] Rm Rd（op: 00=REV, 01=REV16, 11=REVSH；10 保留）
            2 => {
                let rd = (bits & 0x7) as u8;
                let rm = ((bits >> 3) & 0x7) as u8;
                match (bits >> 6) & 0x3 {
                    0 => Instruction::Rev {
                        rd,
                        rm,
                        kind: RevKind::Rev,
                    },
                    1 => Instruction::Rev {
                        rd,
                        rm,
                        kind: RevKind::Rev16,
                    },
                    3 => Instruction::Rev {
                        rd,
                        rm,
                        kind: RevKind::RevSh,
                    },
                    // 10（0xBA80-0xBABF）：保留，诚实 Unimplemented
                    _ => Instruction::Unimplemented {
                        bits: bits as u32,
                    },
                }
            }
            // POP {regs}（bit8=0）/ POP {regs, pc}（bit8=1）
            4 => Instruction::Pop {
                regs: (bits & 0xFF) as u16,
                pc: false,
            },
            5 => Instruction::Pop {
                regs: (bits & 0xFF) as u16,
                pc: true,
            },
            // BKPT #imm8（0xBE00-0xBEFF，imm8 = bits[7:0]）→ 调试事件
            6 => Instruction::Breakpoint {
                imm8: (bits & 0xFF) as u8,
            },
            // 0xBF00-0xBFFF：提示指令或 IT 条件块
            7 => {
                if bits & 0xF == 0 {
                    // 提示指令：NOP/WFI/WFE/SEV/YIELD（无副作用，WFI 建模为 NOP）
                    Instruction::Nop
                } else {
                    // IT：firstcond = bits[7:4]，mask = bits[3:0]
                    let cond_bits = ((bits >> 4) & 0xF) as u8;
                    let mask = (bits & 0xF) as u8;
                    match Cond::from_bits(cond_bits) {
                        // AL/NV 作为 IT 条件 UNPREDICTABLE；mask 非法（非前导 1 后 0 模式）按 Invalid
                        Some(cond) if mask != 0 && cond != Cond::Al => {
                            Instruction::It { cond, mask }
                        }
                        _ => Instruction::Invalid { address: pc },
                    }
                }
            }
            _ => Instruction::Unimplemented {
                bits: bits as u32,
            },
        }
    }

    /// MOVS/CMP/ADD/SUB #imm8: 100 x op rd imm8
    fn decode_mov_cmp_add_sub_imm8(&self, bits: u16) -> Instruction {
        let rd = ((bits >> 8) & 0x7) as u8;
        let rn = rd;
        let imm = (bits & 0xFF) as u32;
        let op = (bits >> 11) & 0x3;
        match op {
            // 000: MOVS Rd, #imm8（Thumb-1 立即数传送，更新 N/Z）
            0 => Instruction::Mov {
                rd,
                rm: 0,
                imm: Some(imm),
                flags: true,
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
        // imm8 为有符号 8 位（向后分支为负），目标 = PC+4 + imm8×2
        let imm8 = (bits & 0xFF) as i8 as i32;
        let target = pc.wrapping_add(4).wrapping_add((imm8 << 1) as u32);
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

    /// 无条件分支: 11100 imm11（有符号 11 位）
    fn decode_branch_uncond(&self, bits: u16, pc: u32) -> Instruction {
        let imm11 = (bits & 0x7FF) as i16;
        let imm11 = if imm11 & 0x400 != 0 { imm11 - 0x800 } else { imm11 };
        let target = pc.wrapping_add(4).wrapping_add((imm11 << 1) as u32);
        Instruction::Branch {
            cond: None,
            target,
        }
    }

    /// 生成非法指令错误
    pub fn invalid(&self, address: u32) -> FaultReason {
        FaultReason::IllegalInstruction { pc: address }
    }

    /// 32-bit BL: 11110 S imm10 → 11111 J1 J2 imm11
    ///
    /// I1 = J1 XOR NOT(S)，I2 = J2 XOR NOT(S)；
    /// imm25 = S:imm10:I1:I2:imm11:0（25 位有符号，PC = 指令地址 + 4）。
    /// 编码位布局经固件字节实测：0xF000 0xF80F @0x86E → 0x890；0xF7FF 0xFF93 @0x54A → 0x474。
    fn decode_bl(&self, bits: u32, pc: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let low = bits as u16;
        // BL 低半字以 11111 开头；11101 开头为 BLX（decode_blx，A-profile 语义，M-profile UNDEF）
        if low & 0xF800 != 0xF800 {
            return Instruction::Unimplemented { bits };
        }
        let s = ((top >> 10) & 1) as u32;
        let imm10 = (top & 0x3FF) as u32;
        let j1 = ((low >> 14) & 1) as u32;
        let j2 = ((low >> 13) & 1) as u32;
        let imm11 = (low & 0x7FF) as u32;
        let i1 = j1 ^ (s ^ 1);
        let i2 = j2 ^ (s ^ 1);
        let imm25 = (s << 24) | (imm10 << 14) | (i1 << 13) | (i2 << 12) | (imm11 << 1);
        // 25 位有符号扩展
        let offset = if imm25 & (1 << 24) != 0 {
            imm25 | 0xFF00_0000
        } else {
            imm25
        };
        let target = pc.wrapping_add(4).wrapping_add(offset);
        Instruction::BranchLink { target }
    }

    /// 32-bit BLX: 11110 S imm10 → 1110 1 imm11[10:0]
    ///
    /// imm22 = S:imm10:imm11（22 位有符号），目标 = Align(PC,4) + 4 + (imm22 << 1)，
    /// LR = (PC+4)|1。编码位布局经 GNU as 19 个样本实测（含正/负/远偏移）：
    /// 低半字固定 11101 开头（区别于 BL 的 11111），imm11 = bits[10:0]，
    /// 无独立 J1/J2 位（A-profile BLX T2 布局；binutils 实测如此）。
    /// 诚实注：BLX(立即数) 仅 A-profile 有效，ARMv7-M 上该编码空间 UNDEF
    /// （QEMU translate.c 注释确认），此处建模供完整性/工具链使用；目标 bit0
    /// 恒清（引擎仅 Thumb 态）。
    fn decode_blx(&self, bits: u32, pc: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let low = bits as u16;
        let s = ((top >> 10) & 1) as u32;
        let imm10 = (top & 0x3FF) as u32;
        let imm11 = (low & 0x7FF) as u32;
        // 22 位有符号 → 32 位符号扩展
        let imm22 = (s << 21) | (imm10 << 11) | imm11;
        let offset = if imm22 & (1 << 21) != 0 {
            (imm22 | 0xFFC0_0000) as i32
        } else {
            imm22 as i32
        };
        // 目标 = Align(PC,4) + 4 + (imm22 << 1)；清 bit0（Thumb 态）
        let target = (pc & !3)
            .wrapping_add(4)
            .wrapping_add((offset.wrapping_shl(1)) as u32)
            & !1;
        Instruction::BranchLinkExchangeImm { target }
    }

    /// 32-bit 数据处理（修改立即数）: 11110 i 0 op4 S Rn / 0 imm3 Rd imm8
    ///
    /// op4：0=AND、1=BIC、2=ORR（Rn=1111 → MOV）、3=ORN（Rn=1111 → MVN）、
    /// 4=EOR、8=ADD、B=SUB、D=CMP、E=CMN；其余（ADC/SBC/TEQ/RSB 等）未建模。
    fn decode_data_proc_imm(&self, bits: u32) -> Instruction {
        let top = (bits >> 16) as u16;
        let i = ((top >> 10) & 1) as u32;
        let op4 = ((top >> 5) & 0xF) as u8;
        let s = ((top >> 4) & 1) == 1;
        let rn = (top & 0xF) as u8;
        let imm3 = ((bits >> 12) & 0x7) as u32;
        let rd = ((bits >> 8) & 0xF) as u8;
        let imm8 = bits & 0xFF;
        let imm12 = (i << 11) | (imm3 << 8) | imm8;
        let imm = Self::thumb_expand_imm(imm12);
        match op4 {
            // AND/ANDS；Rd=1111 → TST（只更新标志，不写 PC）
            0x0 if rd == 0xF => Instruction::Tst {
                rn,
                rm: None,
                imm: Some(imm),
            },
            // AND/ANDS
            0x0 => Instruction::And {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: s,
            },
            // BIC/BICS
            0x1 => Instruction::Bic {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: s,
            },
            // ORR（Rn=1111 → MOV）/ MOVS.W（S=1）
            // ARMv7-M：op4=0x2 且 Rn=1111 恒为 MOV（imm），与 S 无关；
            // MOVS.W r15,#imm（Rd=1111 且 S=1）为 UNPREDICTABLE → 诚实 Unimplemented，
            // 不静默写 PC（P0-1 家族残留修复）。
            0x2 if rn == 0xF && rd == 0xF && s => Instruction::Unimplemented { bits },
            0x2 if rn == 0xF => Instruction::Mov {
                rd,
                rm: 0,
                imm: Some(imm),
                flags: s,
            },
            0x2 => Instruction::Orr {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: s,
            },
            // ORN / MVN 未建模
            0x3 => Instruction::Unimplemented { bits },
            // EOR/EORS；Rd=1111 → TEQ（只更新标志，不写 PC）
            0x4 if rd == 0xF => Instruction::Teq {
                rn,
                rm: None,
                imm: Some(imm),
            },
            // EOR/EORS
            0x4 => Instruction::Eor {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: s,
            },
            // ADD/ADDS；Rd=1111 → CMN（只更新标志，不写 PC）
            0x8 if rd == 0xF => Instruction::Cmn {
                rn,
                rm: None,
                imm: Some(imm),
            },
            // ADD/ADDS
            0x8 => Instruction::Add {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: s,
            },
            // SUB/SUBS；Rd=1111 → CMP（只更新标志，不写 PC）
            // （汇编实测：SUB 编码 op4=0xD；旧代码误用 0xB=SBC）
            0xD if rd == 0xF => Instruction::Cmp {
                rn,
                rm: None,
                imm: Some(imm),
            },
            // SUB/SUBS
            0xD => Instruction::Sub {
                rd,
                rn,
                rm: None,
                imm: Some(imm),
                flags: s,
            },
            // 其余（ADC/SBC/RSB/ORN/MVN 等）无 imm 变体 → 诚实 Unimplemented
            // 0xA=ADC、0xB=SBC、0xE=RSB（汇编实测 0xF143/0xF163/0xF1C3）
            _ => Instruction::Unimplemented { bits },
        }
    }

    /// Thumb-2 修改立即数展开（与 QEMU t32_expandimm / GNU objdump 实测一致）
    ///
    /// imm12 = i:imm3:imm8：
    /// - bits[11:8]=0: 0x000000XX
    /// - bits[11:8]=1: 0x00XX00XX
    /// - bits[11:8]=2: 0xXX00XX00
    /// - bits[11:8]=3: 0xXXXXXXXX
    /// - 其余: ROR(imm8|0x80, bits[11:7])
    fn thumb_expand_imm(imm12: u32) -> u32 {
        let imm8 = imm12 & 0xFF;
        let mode = (imm12 >> 8) & 0xF;
        let rot = if imm12 & 0xC00 != 0 {
            (imm12 >> 7) & 0x1F
        } else {
            0
        };
        let imm: u32 = match mode {
            0 => imm8,
            1 => imm8 * 0x0001_0001,
            2 => imm8 * 0x0100_0100,
            3 => imm8 * 0x0101_0101,
            _ => imm8 | 0x80,
        };
        if rot != 0 {
            imm.rotate_right(rot)
        } else {
            imm
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 16-bit LDR/STR 立即数偏移解码（编码经 arm-none-eabi-as 实测）
    #[test]
    fn decode_16bit_ldr_str_imm() {
        let mut d = Decoder::new();
        // STR r0, [r1, #12] = 0x60C8 → Word 偏移 12
        assert_eq!(
            d.decode_halfword(0x60C8, 0),
            Instruction::Str {
                rt: 0,
                rn: 1,
                offset: LoadStoreOffset::Immediate(12),
                width: AccessWidth::Word,
            }
        );
        // LDR r2, [r3, #20] = 0x695A
        assert_eq!(
            d.decode_halfword(0x695A, 0),
            Instruction::Ldr {
                rt: 2,
                rn: 3,
                offset: LoadStoreOffset::Immediate(20),
                width: AccessWidth::Word,
            }
        );
        // STRB r4, [r5, #7] = 0x71EC
        assert_eq!(
            d.decode_halfword(0x71EC, 0),
            Instruction::Str {
                rt: 4,
                rn: 5,
                offset: LoadStoreOffset::Immediate(7),
                width: AccessWidth::Byte,
            }
        );
        // LDRB r6, [r7, #3] = 0x78FE
        assert_eq!(
            d.decode_halfword(0x78FE, 0),
            Instruction::Ldr {
                rt: 6,
                rn: 7,
                offset: LoadStoreOffset::Immediate(3),
                width: AccessWidth::Byte,
            }
        );
        // STRH r0, [r1, #6] = 0x80C8
        assert_eq!(
            d.decode_halfword(0x80C8, 0),
            Instruction::Str {
                rt: 0,
                rn: 1,
                offset: LoadStoreOffset::Immediate(6),
                width: AccessWidth::HalfWord,
            }
        );
        // LDRH r2, [r3, #10] = 0x895A
        assert_eq!(
            d.decode_halfword(0x895A, 0),
            Instruction::Ldr {
                rt: 2,
                rn: 3,
                offset: LoadStoreOffset::Immediate(10),
                width: AccessWidth::HalfWord,
            }
        );
    }

    /// 16-bit LDR/STR 寄存器偏移解码（真实编码：bit10 = B）
    #[test]
    fn decode_16bit_ldr_str_reg() {
        let mut d = Decoder::new();
        // STR r0, [r1, r2] = 0x5088
        assert_eq!(
            d.decode_halfword(0x5088, 0),
            Instruction::Str {
                rt: 0,
                rn: 1,
                offset: LoadStoreOffset::Register(2),
                width: AccessWidth::Word,
            }
        );
        // STRB r0, [r1, r2] = 0x5488
        assert_eq!(
            d.decode_halfword(0x5488, 0),
            Instruction::Str {
                rt: 0,
                rn: 1,
                offset: LoadStoreOffset::Register(2),
                width: AccessWidth::Byte,
            }
        );
        // LDR r3, [r4, r5] = 0x5963
        assert_eq!(
            d.decode_halfword(0x5963, 0),
            Instruction::Ldr {
                rt: 3,
                rn: 4,
                offset: LoadStoreOffset::Register(5),
                width: AccessWidth::Word,
            }
        );
        // LDRB r3, [r4, r5] = 0x5D63
        assert_eq!(
            d.decode_halfword(0x5D63, 0),
            Instruction::Ldr {
                rt: 3,
                rn: 4,
                offset: LoadStoreOffset::Register(5),
                width: AccessWidth::Byte,
            }
        );
    }

    /// 16-bit SP 相对 STR/LDR + STMIA/LDMIA + B<cond> 解码
    #[test]
    fn decode_16bit_sp_relative_multi_branch() {
        let mut d = Decoder::new();
        // STR r0, [sp, #4] = 0x9001
        assert_eq!(
            d.decode_halfword(0x9001, 0),
            Instruction::Str {
                rt: 0,
                rn: 13,
                offset: LoadStoreOffset::Immediate(4),
                width: AccessWidth::Word,
            }
        );
        // LDR r0, [sp, #4] = 0x9801
        assert_eq!(
            d.decode_halfword(0x9801, 0),
            Instruction::Ldr {
                rt: 0,
                rn: 13,
                offset: LoadStoreOffset::Immediate(4),
                width: AccessWidth::Word,
            }
        );
        // STMIA r0!, {r1, r2} = 0xC006
        assert_eq!(
            d.decode_halfword(0xC006, 0),
            Instruction::Stm {
                rn: 0,
                regs: 0b110,
                writeback: true,
                descending: false,
            }
        );
        // LDMIA r0!, {r1, r2} = 0xC806
        assert_eq!(
            d.decode_halfword(0xC806, 0),
            Instruction::Ldm {
                rn: 0,
                regs: 0b110,
                writeback: true,
                descending: false,
            }
        );
        // BEQ：0xD006 → 条件分支，目标 = pc+4+6*2
        assert_eq!(
            d.decode_halfword(0xD006, 0x1000),
            Instruction::Branch {
                cond: Some(Cond::Eq),
                target: 0x1000 + 4 + 12,
            }
        );
        // BNE：0xD106
        assert_eq!(
            d.decode_halfword(0xD106, 0x1000),
            Instruction::Branch {
                cond: Some(Cond::Ne),
                target: 0x1000 + 4 + 12,
            }
        );
    }

    // ============ P2-补：MRS/MSR 解码（编码经 arm-none-eabi-as 实测） ============

    /// MRS 解码：APSR/IPSR/EPSR/PRIMASK/BASEPRI/FAULTMASK/CONTROL/MSP/PSP
    #[test]
    fn decode_mrs_variants() {
        let mut d = Decoder::new();
        // MRS r0, APSR = 0xF3EF 8000
        assert_eq!(
            d.decode_word(0xF3EF_8000, 0),
            Instruction::MsrMrs {
                rt: 0,
                reg: SpecialReg::Apsr,
                read: true,
            }
        );
        // MRS r0, IPSR = 0xF3EF 8005
        assert_eq!(
            d.decode_word(0xF3EF_8005, 0),
            Instruction::MsrMrs {
                rt: 0,
                reg: SpecialReg::Ipsr,
                read: true,
            }
        );
        // MRS r0, EPSR = 0xF3EF 8006
        assert_eq!(
            d.decode_word(0xF3EF_8006, 0),
            Instruction::MsrMrs {
                rt: 0,
                reg: SpecialReg::Epsr,
                read: true,
            }
        );
        // MRS r0, PRIMASK = 0xF3EF 8010
        assert_eq!(
            d.decode_word(0xF3EF_8010, 0),
            Instruction::MsrMrs {
                rt: 0,
                reg: SpecialReg::Primask,
                read: true,
            }
        );
        // MRS r3, CONTROL = 0xF3EF 8314（Rd = r3）
        assert_eq!(
            d.decode_word(0xF3EF_8314, 0),
            Instruction::MsrMrs {
                rt: 3,
                reg: SpecialReg::Control,
                read: true,
            }
        );
        // MRS r0, MSP = 0xF3EF 8008
        assert_eq!(
            d.decode_word(0xF3EF_8008, 0),
            Instruction::MsrMrs {
                rt: 0,
                reg: SpecialReg::Msp,
                read: true,
            }
        );
        // MRS r1, FAULTMASK = 0xF3EF 8113
        assert_eq!(
            d.decode_word(0xF3EF_8113, 0),
            Instruction::MsrMrs {
                rt: 1,
                reg: SpecialReg::Faultmask,
                read: true,
            }
        );
    }

    /// MSR 解码：寄存器形式（APSR/PRIMASK/BASEPRI/BASEPRI_MAX/FAULTMASK/CONTROL/MSP/PSP）
    #[test]
    fn decode_msr_variants() {
        let mut d = Decoder::new();
        // MSR APSR_nzcvq, r1 = 0xF381 8800
        assert_eq!(
            d.decode_word(0xF381_8800, 0),
            Instruction::MsrMrs {
                rt: 1,
                reg: SpecialReg::Apsr,
                read: false,
            }
        );
        // MSR APSR_nzcvqg, r1 = 0xF381 8C00（mask bit10 → GE）
        assert_eq!(
            d.decode_word(0xF381_8C00, 0),
            Instruction::MsrMrs {
                rt: 1,
                reg: SpecialReg::ApsrGe,
                read: false,
            }
        );
        // MSR PRIMASK, r2 = 0xF382 8810
        assert_eq!(
            d.decode_word(0xF382_8810, 0),
            Instruction::MsrMrs {
                rt: 2,
                reg: SpecialReg::Primask,
                read: false,
            }
        );
        // MSR BASEPRI_MAX, r4 = 0xF384 8812
        assert_eq!(
            d.decode_word(0xF384_8812, 0),
            Instruction::MsrMrs {
                rt: 4,
                reg: SpecialReg::BasepriMax,
                read: false,
            }
        );
        // MSR CONTROL, r6 = 0xF386 8814
        assert_eq!(
            d.decode_word(0xF386_8814, 0),
            Instruction::MsrMrs {
                rt: 6,
                reg: SpecialReg::Control,
                read: false,
            }
        );
        // MSR MSP, r7 = 0xF387 8808
        assert_eq!(
            d.decode_word(0xF387_8808, 0),
            Instruction::MsrMrs {
                rt: 7,
                reg: SpecialReg::Msp,
                read: false,
            }
        );
        // MSR PSP, r8 = 0xF388 8809
        assert_eq!(
            d.decode_word(0xF388_8809, 0),
            Instruction::MsrMrs {
                rt: 8,
                reg: SpecialReg::Psp,
                read: false,
            }
        );
    }

    /// 未知 SYSm → 诚实 Unimplemented（MRS 0x12 BASEPRI_MAX 读为 UNPREDICTABLE；MSR 未知字段）
    #[test]
    fn decode_mrs_msr_unknown_sysm() {
        let mut d = Decoder::new();
        // MRS r0, #0x99（未定义 SYSm）
        assert_eq!(
            d.decode_word(0xF3EF_8099, 0),
            Instruction::Unimplemented { bits: 0xF3EF_8099 }
        );
        // MRS BASEPRI_MAX（0x12）：读为 UNPREDICTABLE → Unimplemented
        assert_eq!(
            d.decode_word(0xF3EF_8012, 0),
            Instruction::Unimplemented { bits: 0xF3EF_8012 }
        );
        // MSR 未知 SYSm（0x20）
        assert_eq!(
            d.decode_word(0xF382_8820, 0),
            Instruction::Unimplemented { bits: 0xF382_8820 }
        );
    }

    // ============ P4-补：VLDM/VSTM + VCVT 定点解码（编码经 arm-none-eabi-as 实测） ============

    /// VLDM/VSTM 解码：SP/DP、IA/DB、回写、S16+（D 位）
    #[test]
    fn decode_vldm_vstm_variants() {
        let mut d = Decoder::new();
        // VSTMIA r0, {s0-s3} = 0xEC80 0A04
        assert_eq!(
            d.decode_word(0xEC80_0A04, 0),
            Instruction::FpLoadStoreMulti {
                vd: 0,
                rn: 0,
                count: 4,
                load: false,
                double: false,
                decrement: false,
                writeback: false,
            }
        );
        // VLDMIA r0, {s0-s3} = 0xEC90 0A04
        assert_eq!(
            d.decode_word(0xEC90_0A04, 0),
            Instruction::FpLoadStoreMulti {
                vd: 0,
                rn: 0,
                count: 4,
                load: true,
                double: false,
                decrement: false,
                writeback: false,
            }
        );
        // VSTMDB r0!, {s0-s3} = 0xED20 0A04（回写 + 先减）
        assert_eq!(
            d.decode_word(0xED20_0A04, 0),
            Instruction::FpLoadStoreMulti {
                vd: 0,
                rn: 0,
                count: 4,
                load: false,
                double: false,
                decrement: true,
                writeback: true,
            }
        );
        // VLDMIA r1!, {s16-s19} = 0xECB1 8A04（D=0, Vd=8 → S16；回写）
        assert_eq!(
            d.decode_word(0xECB1_8A04, 0),
            Instruction::FpLoadStoreMulti {
                vd: 16,
                rn: 1,
                count: 4,
                load: true,
                double: false,
                decrement: false,
                writeback: true,
            }
        );
        // VLDMIA r0, {s1-s2} = 0xECC0 0A02（D=1, Vd=0 → S1；无回写）
        assert_eq!(
            d.decode_word(0xECC0_0A02, 0),
            Instruction::FpLoadStoreMulti {
                vd: 1,
                rn: 0,
                count: 2,
                load: false,
                double: false,
                decrement: false,
                writeback: false,
            }
        );
        // VSTMIA r0, {d0-d1} = 0xEC80 0B04（DP：count = 4/2 = 2）
        assert_eq!(
            d.decode_word(0xEC80_0B04, 0),
            Instruction::FpLoadStoreMulti {
                vd: 0,
                rn: 0,
                count: 2,
                load: false,
                double: true,
                decrement: false,
                writeback: false,
            }
        );
        // VSTMDB r0!, {d0-d1} = 0xED20 0B04
        assert_eq!(
            d.decode_word(0xED20_0B04, 0),
            Instruction::FpLoadStoreMulti {
                vd: 0,
                rn: 0,
                count: 2,
                load: false,
                double: true,
                decrement: true,
                writeback: true,
            }
        );
    }

    /// VCVT 定点解码：S16/U16/S32/U32 双向、fbits 计算
    #[test]
    fn decode_vcvt_fixed_variants() {
        let mut d = Decoder::new();
        // VCVT.S16.F32 s0, s0, #8 = 0xEEBE 0A44
        assert_eq!(
            d.decode_word(0xEEBE_0A44, 0),
            Instruction::FpCvtFixed {
                vd: 0,
                fbits: 8,
                width: 16,
                signed: true,
                to_float: false,
            }
        );
        // VCVT.U16.F32 s0, s0, #8 = 0xEEBF 0A44
        assert_eq!(
            d.decode_word(0xEEBF_0A44, 0),
            Instruction::FpCvtFixed {
                vd: 0,
                fbits: 8,
                width: 16,
                signed: false,
                to_float: false,
            }
        );
        // VCVT.S32.F32 s4, s4, #16 = 0xEEBE 2AC8（Vd=S4）
        assert_eq!(
            d.decode_word(0xEEBE_2AC8, 0),
            Instruction::FpCvtFixed {
                vd: 4,
                fbits: 16,
                width: 32,
                signed: true,
                to_float: false,
            }
        );
        // VCVT.U32.F32 s0, s0, #16 = 0xEEBF 0AC8
        assert_eq!(
            d.decode_word(0xEEBF_0AC8, 0),
            Instruction::FpCvtFixed {
                vd: 0,
                fbits: 16,
                width: 32,
                signed: false,
                to_float: false,
            }
        );
        // VCVT.F32.S16 s0, s0, #8 = 0xEEBA 0A44（定点→浮点）
        assert_eq!(
            d.decode_word(0xEEBA_0A44, 0),
            Instruction::FpCvtFixed {
                vd: 0,
                fbits: 8,
                width: 16,
                signed: true,
                to_float: true,
            }
        );
        // VCVT.F32.U32 s3, s3, #16 = 0xEEFB 1AC8（U32→F32，Vd=S3）
        assert_eq!(
            d.decode_word(0xEEFB_1AC8, 0),
            Instruction::FpCvtFixed {
                vd: 3,
                fbits: 16,
                width: 32,
                signed: false,
                to_float: true,
            }
        );
    }
// 包 D 引擎解码补全的单元测试
//
// 所有编码均以 arm-none-eabi-as 汇编 + objdump 反汇编（或真实固件字节）实测为准：
// - 0x1A9B = subs r3, r3, r2；0x1C5A = adds r2, r3, #1（ADD/SUB 寄存器/立即数形式）
// - 0x46BD = mov sp, r7；0x4770 = bx lr（高寄存器 MOV / BX）
// - 0x480C = ldr r0, [pc, #48]（LDR literal）
// - 0xB580 = push {r7, lr}；0xBD80 = pop {r7, pc}（PUSH/POP）
// - 0xAF00 = add r7, sp, #0；0xB002 = add sp, #8；0xB083 = sub sp, #12
// - 0xB140 = cbz r0, +16；0xB939 = cbnz r1, +14（CBZ/CBNZ 位布局）
// - 0xBF00 = nop；0xBF30 = wfi（提示指令）
// - 0xF000 0xF80F @0x86E → bl 0x890；0xF7FF 0xFF93 @0x54A → bl 0x474（BL）
// - 0xF44F 0x32E1 = mov.w r2, #115200；0xF003 0x0301 = and.w r3, r3, #1；
//   0xF1B3 0x3FA5 = cmp.w r3, #0xA5A5A5A5（修改立即数）

/// ADD/SUB：寄存器形式（bit10=0）与立即数形式（bit10=1）
#[test]
fn decode_add_sub_reg_and_imm_forms() {
    let mut d = Decoder::new();
    // subs r3, r3, r2 = 0x1A9B
    assert_eq!(
        d.decode_halfword(0x1A9B, 0),
        Instruction::Sub {
            rd: 3,
            rn: 3,
            rm: Some(2),
            imm: None,
            flags: true,
        }
    );
    // adds r2, r3, #1 = 0x1C5A
    assert_eq!(
        d.decode_halfword(0x1C5A, 0),
        Instruction::Add {
            rd: 2,
            rn: 3,
            rm: None,
            imm: Some(1),
            flags: true,
        }
    );
    // subs r2, r3, #1 = 0x1E5A
    assert_eq!(
        d.decode_halfword(0x1E5A, 0),
        Instruction::Sub {
            rd: 2,
            rn: 3,
            rm: None,
            imm: Some(1),
            flags: true,
        }
    );
    // subs r2, r2, r1 = 0x1A52
    assert_eq!(
        d.decode_halfword(0x1A52, 0),
        Instruction::Sub {
            rd: 2,
            rn: 2,
            rm: Some(1),
            imm: None,
            flags: true,
        }
    );
}

/// 高寄存器 MOV：0x46BD = mov sp, r7；0x4618 = mov r0, r3；0x4603 = mov r3, r0
#[test]
fn decode_high_register_mov() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_halfword(0x46BD, 0),
        Instruction::Mov {
            rd: 13,
            rm: 7,
            imm: None,
            flags: false,
        }
    );
    assert_eq!(
        d.decode_halfword(0x4618, 0),
        Instruction::Mov {
            rd: 0,
            rm: 3,
            imm: None,
            flags: false,
        }
    );
    assert_eq!(
        d.decode_halfword(0x4603, 0),
        Instruction::Mov {
            rd: 3,
            rm: 0,
            imm: None,
            flags: false,
        }
    );
}

/// BX/BLX：0x4770 = bx lr（bit7=0 → BX）；0x4780 = blx r0（bit7=1 → BLX）
#[test]
fn decode_bx_blx() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_halfword(0x4770, 0),
        Instruction::BranchExchange { rm: 14 }
    );
    assert_eq!(
        d.decode_halfword(0x4708, 0),
        Instruction::BranchExchange { rm: 1 }
    );
    assert_eq!(
        d.decode_halfword(0x4780, 0),
        Instruction::BranchLinkExchange { rm: 0 }
    );
}

/// LDR literal：0x480C = ldr r0, [pc, #48]
#[test]
fn decode_ldr_literal() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_halfword(0x480C, 0x844),
        Instruction::LdrLiteral { rt: 0, imm: 48 }
    );
    // 0x4B06 = ldr r3, [pc, #24]
    assert_eq!(
        d.decode_halfword(0x4B06, 0x404),
        Instruction::LdrLiteral { rt: 3, imm: 24 }
    );
}

/// PUSH/POP：0xB580 = push {r7, lr}；0xB480 = push {r7}；0xBD80 = pop {r7, pc}；0xBC80 = pop {r7}
#[test]
fn decode_push_pop() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_halfword(0xB580, 0),
        Instruction::Push {
            regs: 0x80,
            lr: true,
        }
    );
    assert_eq!(
        d.decode_halfword(0xB480, 0),
        Instruction::Push {
            regs: 0x80,
            lr: false,
        }
    );
    assert_eq!(
        d.decode_halfword(0xBD80, 0),
        Instruction::Pop {
            regs: 0x80,
            pc: true,
        }
    );
    assert_eq!(
        d.decode_halfword(0xBC80, 0),
        Instruction::Pop {
            regs: 0x80,
            pc: false,
        }
    );
    // 0xB510 = push {r4, lr}
    assert_eq!(
        d.decode_halfword(0xB510, 0),
        Instruction::Push {
            regs: 0x10,
            lr: true,
        }
    );
}

/// SP 立即数加减：0xAF00 = add r7, sp, #0；0xB002 = add sp, #8；0xB083 = sub sp, #12
#[test]
fn decode_sp_imm() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_halfword(0xAF00, 0),
        Instruction::Add {
            rd: 7,
            rn: 13,
            rm: None,
            imm: Some(0),
            flags: false,
        }
    );
    assert_eq!(
        d.decode_halfword(0xB002, 0),
        Instruction::Add {
            rd: 13,
            rn: 13,
            rm: None,
            imm: Some(8),
            flags: false,
        }
    );
    assert_eq!(
        d.decode_halfword(0xB083, 0),
        Instruction::Sub {
            rd: 13,
            rn: 13,
            rm: None,
            imm: Some(12),
            flags: false,
        }
    );
}

/// CBZ/CBNZ：0xB140 = cbz r0, +16；0xB939 = cbnz r1, +14（i=bit9，imm5=bits[7:3]，Rn=bits[2:0]）
#[test]
fn decode_cbz_cbnz() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_halfword(0xB140, 0),
        Instruction::CompareBranch {
            rn: 0,
            target: 0x14,
            zero: true,
        }
    );
    assert_eq!(
        d.decode_halfword(0xB939, 0x2),
        Instruction::CompareBranch {
            rn: 1,
            target: 0x14,
            zero: false,
        }
    );
}

/// 提示指令：0xBF00 = nop；0xBF30 = wfi（建模为 NOP）
#[test]
fn decode_hints() {
    let mut d = Decoder::new();
    assert_eq!(d.decode_halfword(0xBF00, 0), Instruction::Nop);
    assert_eq!(d.decode_halfword(0xBF30, 0), Instruction::Nop);
    // IT 指令（bits[3:0] != 0）→ It 解码：0xBF08 = it eq（cond=0000，mask=1000）
    assert_eq!(
        d.decode_halfword(0xBF08, 0),
        Instruction::It {
            cond: Cond::Eq,
            mask: 0x8,
        }
    );
    // BKPT #imm8（0xBEAB → imm8=0xAB）
    assert_eq!(
        d.decode_halfword(0xBEAB, 0),
        Instruction::Breakpoint { imm8: 0xAB }
    );
}

/// 分支符号扩展：0xD1F9 @0x45A → bne.n 0x450（向后）；0xE7FE → b.n 自身
#[test]
fn decode_backward_branches() {
    let mut d = Decoder::new();
    // bne.n -14：0x45A + 4 + (-7*2) = 0x450
    assert_eq!(
        d.decode_halfword(0xD1F9, 0x45A),
        Instruction::Branch {
            cond: Some(Cond::Ne),
            target: 0x450,
        }
    );
    // b.n -2：0xE7FE → pc+4-4 = pc
    assert_eq!(
        d.decode_halfword(0xE7FE, 0x762),
        Instruction::Branch {
            cond: None,
            target: 0x762,
        }
    );
}

/// BL：0xF000 0xF80F @0x86E → 0x890；0xF7FF 0xFF93 @0x54A → 0x474（负偏移）
#[test]
fn decode_bl_targets() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_word(0xF000_F80F, 0x86E),
        Instruction::BranchLink { target: 0x890 }
    );
    assert_eq!(
        d.decode_word(0xF7FF_FF93, 0x54A),
        Instruction::BranchLink { target: 0x474 }
    );
    // BLX（低半字 11110 开头）→ 诚实 Unimplemented
    assert!(matches!(
        d.decode_word(0xF000_F000, 0),
        Instruction::Unimplemented { .. }
    ));
}

/// Thumb-2 修改立即数展开（与 GNU objdump 实测一致）
#[test]
fn thumb_expand_imm_verified() {
    // mov.w r2, #115200 = f44f 32e1 → imm12 = 0xBE1
    assert_eq!(Decoder::thumb_expand_imm(0xBE1), 115200);
    // mov.w r0, #0xA5A5A5A5 = f04f 30a5 → imm12 = 0x3A5
    assert_eq!(Decoder::thumb_expand_imm(0x3A5), 0xA5A5_A5A5);
    // mov.w r0, #0xFF00FF00 = f04f 20ff → imm12 = 0x2FF
    assert_eq!(Decoder::thumb_expand_imm(0x2FF), 0xFF00_FF00);
    // mov.w r0, #1 = f04f 0001 → imm12 = 0x001
    assert_eq!(Decoder::thumb_expand_imm(0x001), 1);
    assert_eq!(Decoder::thumb_expand_imm(0x000), 0);
}

/// 32-bit 数据处理（修改立即数）：MOV.W / AND.W / CMP.W
#[test]
fn decode_data_proc_imm() {
    let mut d = Decoder::new();
    // mov.w r2, #115200（0xF44F 0x32E1）
    assert_eq!(
        d.decode_word(0xF44F_32E1, 0),
        Instruction::Mov {
            rd: 2,
            rm: 0,
            imm: Some(115200),
            flags: false,
        }
    );
    // and.w r3, r3, #1（0xF003 0x0301）
    assert_eq!(
        d.decode_word(0xF003_0301, 0),
        Instruction::And {
            rd: 3,
            rn: 3,
            rm: None,
            imm: Some(1),
            flags: false,
        }
    );
    // cmp.w r3, #0xA5A5A5A5（0xF1B3 0x3FA5）
    assert_eq!(
        d.decode_word(0xF1B3_3FA5, 0),
        Instruction::Cmp {
            rn: 3,
            rm: None,
            imm: Some(0xA5A5_A5A5),
        }
    );
    // P0-1 回归：tst.w r3, #1（0xF013 0x0F01）→ Tst（Rd=1111，不写 PC）
    assert_eq!(
        d.decode_word(0xF013_0F01, 0),
        Instruction::Tst {
            rn: 3,
            rm: None,
            imm: Some(1),
        }
    );
    // P0-1 回归：teq.w r3, #2（0xF093 0x0F02，arm-none-eabi-as 实测）→ Teq（Rd=1111）
    assert_eq!(
        d.decode_word(0xF093_0F02, 0),
        Instruction::Teq {
            rn: 3,
            rm: None,
            imm: Some(2),
        }
    );
    // P0-1 回归：cmn.w r3, #3（0xF113 0x0F03）→ Cmn（Rd=1111）
    assert_eq!(
        d.decode_word(0xF113_0F03, 0),
        Instruction::Cmn {
            rn: 3,
            rm: None,
            imm: Some(3),
        }
    );
    // P0-1 回归：sub.w r3, r3, #4 正常形式不受影响（0xF1A3 0x0304，汇编实测）→ Sub
    assert_eq!(
        d.decode_word(0xF1A3_0304, 0),
        Instruction::Sub {
            rd: 3,
            rn: 3,
            rm: None,
            imm: Some(4),
            flags: false,
        }
    );
}

/// 16-bit 寄存器数据处理组（0x4000-0x43FF）完整解码——十六种操作映射
/// （编码与 arm-none-eabi-as 实测一致：0x401A=ANDS r2,r3 … 0x43DA=MVNS r2,r3）
#[test]
fn decode_data_proc_reg_16ops() {
    let mut d = Decoder::new();
    // 0000 ANDS r2, r3（0x401A）
    assert_eq!(
        d.decode_halfword(0x401A, 0),
        Instruction::And {
            rd: 2,
            rn: 2,
            rm: Some(3),
            imm: None,
            flags: true,
        }
    );
    // 0001 EORS r2, r3（0x405A）
    assert_eq!(
        d.decode_halfword(0x405A, 0),
        Instruction::Eor {
            rd: 2,
            rn: 2,
            rm: Some(3),
            imm: None,
            flags: true,
        }
    );
    // 0010 LSLS r2, r5（0x40AA：Rd=bits[2:0]=r2，Rs=bits[5:3]=r5，源=目的=r2）
    assert_eq!(
        d.decode_halfword(0x40AA, 0),
        Instruction::Shift {
            rd: 2,
            rm: 2,
            kind: ShiftKind::Lsl,
            amount: ShiftAmount::Register(5),
            flags: true,
        }
    );
    // 0011 LSRS r2, r5（0x40EA）
    assert_eq!(
        d.decode_halfword(0x40EA, 0),
        Instruction::Shift {
            rd: 2,
            rm: 2,
            kind: ShiftKind::Lsr,
            amount: ShiftAmount::Register(5),
            flags: true,
        }
    );
    // 0100 ASRS r2, r5（0x412A）
    assert_eq!(
        d.decode_halfword(0x412A, 0),
        Instruction::Shift {
            rd: 2,
            rm: 2,
            kind: ShiftKind::Asr,
            amount: ShiftAmount::Register(5),
            flags: true,
        }
    );
    // 0101 ADCS r2, r3（0x415A）
    assert_eq!(
        d.decode_halfword(0x415A, 0),
        Instruction::Adc {
            rd: 2,
            rn: 2,
            rm: 3,
            flags: true,
        }
    );
    // 0110 SBCS r2, r3（0x419A）
    assert_eq!(
        d.decode_halfword(0x419A, 0),
        Instruction::Sbc {
            rd: 2,
            rn: 2,
            rm: 3,
            flags: true,
        }
    );
    // 0111 RORS r2, r5（0x41EA）
    assert_eq!(
        d.decode_halfword(0x41EA, 0),
        Instruction::Shift {
            rd: 2,
            rm: 2,
            kind: ShiftKind::Ror,
            amount: ShiftAmount::Register(5),
            flags: true,
        }
    );
    // 1000 TST r2, r3（0x421A）
    assert_eq!(
        d.decode_halfword(0x421A, 0),
        Instruction::Tst {
            rn: 2,
            rm: Some(3),
            imm: None,
        }
    );
    // 1001 NEGS r2, r3（0x425A：Rd=bits[2:0]=r2，源 Rn=bits[5:3]=r3）
    assert_eq!(
        d.decode_halfword(0x425A, 0),
        Instruction::Neg {
            rd: 2,
            rn: 3,
            flags: true,
        }
    );
    // 1010 CMP r2, r3（0x429A）
    assert_eq!(
        d.decode_halfword(0x429A, 0),
        Instruction::Cmp {
            rn: 2,
            rm: Some(3),
            imm: None,
        }
    );
    // 1011 CMN r2, r3（0x42DA）
    assert_eq!(
        d.decode_halfword(0x42DA, 0),
        Instruction::Cmn {
            rn: 2,
            rm: Some(3),
            imm: None,
        }
    );
    // 1100 ORRS r2, r3（0x431A）
    assert_eq!(
        d.decode_halfword(0x431A, 0),
        Instruction::Orr {
            rd: 2,
            rn: 2,
            rm: Some(3),
            imm: None,
            flags: true,
        }
    );
    // 1101 MULS r2, r3（0x435A：ARMv7E-M 不更新 flags）
    assert_eq!(
        d.decode_halfword(0x435A, 0),
        Instruction::Mul {
            rd: 2,
            rn: 2,
            rm: 3,
            flags: false,
        }
    );
    // 1110 BICS r2, r3（0x439A）
    assert_eq!(
        d.decode_halfword(0x439A, 0),
        Instruction::Bic {
            rd: 2,
            rn: 2,
            rm: Some(3),
            imm: None,
            flags: true,
        }
    );
    // 1111 MVNS r2, r3（0x43DA）
    assert_eq!(
        d.decode_halfword(0x43DA, 0),
        Instruction::Mvn {
            rd: 2,
            rm: 3,
            flags: true,
        }
    );
}

/// B1: 16 位立即数移位形式 decode（编码经 arm-none-eabi-as 实测）
/// ARMv7-M：LSR/ASR 立即数 imm5=0 表示 #32（移满），LSL #0 为不移位。
#[test]
fn decode_16bit_shift_imm() {
    let mut d = Decoder::new();
    // LSRS r3, r2, #32（imm5=0 → 必须转换为 Immediate(32)，否则静默返回原值）
    assert_eq!(
        d.decode_halfword(0x0813, 0),
        Instruction::Shift {
            rd: 3,
            rm: 2,
            kind: ShiftKind::Lsr,
            amount: ShiftAmount::Immediate(32),
            flags: true,
        }
    );
    // ASRS r3, r2, #32（imm5=0 → Immediate(32)）
    assert_eq!(
        d.decode_halfword(0x1013, 0),
        Instruction::Shift {
            rd: 3,
            rm: 2,
            kind: ShiftKind::Asr,
            amount: ShiftAmount::Immediate(32),
            flags: true,
        }
    );
    // LSLS r3, r2, #0（0x0013：汇编器同义为 movs r3, r2）→ 不移位，imm5 原样传入
    assert_eq!(
        d.decode_halfword(0x0013, 0),
        Instruction::Shift {
            rd: 3,
            rm: 2,
            kind: ShiftKind::Lsl,
            amount: ShiftAmount::Immediate(0),
            flags: true,
        }
    );
    // 非零立即数不受影响：LSRS r3, r2, #31 = 0x0FD3
    assert_eq!(
        d.decode_halfword(0x0FD3, 0),
        Instruction::Shift {
            rd: 3,
            rm: 2,
            kind: ShiftKind::Lsr,
            amount: ShiftAmount::Immediate(31),
            flags: true,
        }
    );
    // ASRS r3, r2, #31 = 0x17D3
    assert_eq!(
        d.decode_halfword(0x17D3, 0),
        Instruction::Shift {
            rd: 3,
            rm: 2,
            kind: ShiftKind::Asr,
            amount: ShiftAmount::Immediate(31),
            flags: true,
        }
    );
}

/// B2: 32 位 MOVS.W / MOV.W 立即数 + MOVS.W r15 非法路径 decode（编码经 as 实测）
/// op4=0x2 且 Rn=1111 恒为 MOV（与 S 无关）；Rd=1111 且 S=1 为 UNPREDICTABLE。
#[test]
fn decode_32bit_movs_imm() {
    let mut d = Decoder::new();
    // MOVS.W r0, #0x8000（0xF45F 4000，S=1/Rn=1111）→ Mov 置标志，禁止走 Orr 读 PC
    assert_eq!(
        d.decode_word(0xF45F_4000, 0),
        Instruction::Mov {
            rd: 0,
            rm: 0,
            imm: Some(0x8000),
            flags: true,
        }
    );
    // MOV.W r0, #0x8000（0xF44F 4000，S=0）→ Mov 不置标志
    assert_eq!(
        d.decode_word(0xF44F_4000, 0),
        Instruction::Mov {
            rd: 0,
            rm: 0,
            imm: Some(0x8000),
            flags: false,
        }
    );
    // MOVS.W r15, #0x8000（0xF45F 4F80，Rd=1111 且 S=1，ARM 语义 UNPREDICTABLE）
    // → 诚实 Unimplemented，不静默写 PC
    assert!(matches!(
        d.decode_word(0xF45F_4F80, 0),
        Instruction::Unimplemented { .. }
    ));
    // 回归：正常 ORRS.W r3, r2, #1（0xF052 0301）仍走 Orr 置标志
    assert_eq!(
        d.decode_word(0xF052_0301, 0),
        Instruction::Orr {
            rd: 3,
            rn: 2,
            rm: None,
            imm: Some(1),
            flags: true,
        }
    );
    // 回归：ORR.W r3, r2, #1（0xF042 0301，S=0）
    assert_eq!(
        d.decode_word(0xF042_0301, 0),
        Instruction::Orr {
            rd: 3,
            rn: 2,
            rm: None,
            imm: Some(1),
            flags: false,
        }
    );
}

// ============ P1：FreeRTOS 前置指令解码（FRT-INS，编码 as 实测） ============

/// CPSIE/CPSID 解码（FRT-INS-01）：cpsie i=0xB662/cpsid i=0xB672/cpsie f=0xB661/cpsid f=0xB671
#[test]
fn decode_cpsie_cpsid() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_halfword(0xB662, 0),
        Instruction::Cps {
            disable: false,
            i: true,
            f: false,
        }
    );
    assert_eq!(
        d.decode_halfword(0xB672, 0),
        Instruction::Cps {
            disable: true,
            i: true,
            f: false,
        }
    );
    assert_eq!(
        d.decode_halfword(0xB661, 0),
        Instruction::Cps {
            disable: false,
            i: false,
            f: true,
        }
    );
    assert_eq!(
        d.decode_halfword(0xB671, 0),
        Instruction::Cps {
            disable: true,
            i: false,
            f: true,
        }
    );
}

/// DSB/ISB/DMB 解码（FRT-INS-02）：dsb=0xF3BF 8F4F / isb=0xF3BF 8F6F / dmb=0xF3BF 8F5F
#[test]
fn decode_barriers() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_word(0xF3BF_8F4F, 0),
        Instruction::Barrier {
            kind: BarrierKind::Dsb
        }
    );
    assert_eq!(
        d.decode_word(0xF3BF_8F5F, 0),
        Instruction::Barrier {
            kind: BarrierKind::Dmb
        }
    );
    assert_eq!(
        d.decode_word(0xF3BF_8F6F, 0),
        Instruction::Barrier {
            kind: BarrierKind::Isb
        }
    );
}

/// 32 位 LDM/STM 解码（FRT-INS-03）：IA/DB + 回写 + r14/PC 组合
#[test]
fn decode_ldm_stm_32bit() {
    let mut d = Decoder::new();
    // stmia.w r0!, {r4-r11, r14} = 0xE8A0 4FF0：IA + 回写
    assert_eq!(
        d.decode_word(0xE8A0_4FF0, 0),
        Instruction::Stm {
            rn: 0,
            regs: 0b0100_1111_1111_0000,
            writeback: true,
            descending: false,
        }
    );
    // ldmia.w r0!, {r4-r11, r14} = 0xE8B0 4FF0
    assert_eq!(
        d.decode_word(0xE8B0_4FF0, 0),
        Instruction::Ldm {
            rn: 0,
            regs: 0b0100_1111_1111_0000,
            writeback: true,
            descending: false,
        }
    );
    // stmdb r0!, {r4-r11, r14} = 0xE920 4FF0：DB + 回写
    assert_eq!(
        d.decode_word(0xE920_4FF0, 0),
        Instruction::Stm {
            rn: 0,
            regs: 0b0100_1111_1111_0000,
            writeback: true,
            descending: true,
        }
    );
    // ldmdb r0!, {r4-r11, r14} = 0xE930 4FF0
    assert_eq!(
        d.decode_word(0xE930_4FF0, 0),
        Instruction::Ldm {
            rn: 0,
            regs: 0b0100_1111_1111_0000,
            writeback: true,
            descending: true,
        }
    );
    // stmia.w r0, {r4, r5}（无回写）= 0xE880 0030
    assert_eq!(
        d.decode_word(0xE880_0030, 0),
        Instruction::Stm {
            rn: 0,
            regs: 0b11_0000,
            writeback: false,
            descending: false,
        }
    );
    // ldmia.w sp!, {r4-r11, pc} = 0xE8BD 8FF0（POP 32 位：含 r15）
    assert_eq!(
        d.decode_word(0xE8BD_8FF0, 0),
        Instruction::Ldm {
            rn: 13,
            regs: 0b1000_1111_1111_0000,
            writeback: true,
            descending: false,
        }
    );
    // stmdb r1!, {r0, r3} = 0xE921 0009
    assert_eq!(
        d.decode_word(0xE921_0009, 0),
        Instruction::Stm {
            rn: 1,
            regs: 0b1001,
            writeback: true,
            descending: true,
        }
    );
}

/// CLZ/REV/RBIT 解码（FRT-INS-04/05）：clz r0,r1=0xFAB1 F081；rbit r0,r1=0xFA91 F0A1
#[test]
fn decode_clz_rev_rbit() {
    let mut d = Decoder::new();
    assert_eq!(
        d.decode_word(0xFAB1_F081, 0),
        Instruction::Clz { rd: 0, rm: 1 }
    );
    assert_eq!(
        d.decode_word(0xFA91_F0A1, 0),
        Instruction::Rbit { rd: 0, rm: 1 }
    );
    assert_eq!(
        d.decode_halfword(0xBA08, 0),
        Instruction::Rev {
            rd: 0,
            rm: 1,
            kind: RevKind::Rev
        }
    );
    assert_eq!(
        d.decode_halfword(0xBA48, 0),
        Instruction::Rev {
            rd: 0,
            rm: 1,
            kind: RevKind::Rev16
        }
    );
    assert_eq!(
        d.decode_halfword(0xBAC8, 0),
        Instruction::Rev {
            rd: 0,
            rm: 1,
            kind: RevKind::RevSh
        }
    );
}

/// UBFX/SBFX/BFI/BFC 解码（FRT-INS-05 SHOULD）
#[test]
fn decode_bitfield_ops() {
    let mut d = Decoder::new();
    // ubfx r0, r1, #3, #5 = 0xF3C1 00C4
    assert_eq!(
        d.decode_word(0xF3C1_00C4, 0),
        Instruction::BitField {
            rd: 0,
            rn: 1,
            lsb: 3,
            width: 5,
            kind: BitFieldKind::Ubfx,
        }
    );
    // sbfx r0, r1, #3, #5 = 0xF341 00C4
    assert_eq!(
        d.decode_word(0xF341_00C4, 0),
        Instruction::BitField {
            rd: 0,
            rn: 1,
            lsb: 3,
            width: 5,
            kind: BitFieldKind::Sbfx,
        }
    );
    // bfi r0, r1, #3, #5 = 0xF361 00C7（msb=7 → lsb=3, width=5）
    assert_eq!(
        d.decode_word(0xF361_00C7, 0),
        Instruction::BitField {
            rd: 0,
            rn: 1,
            lsb: 3,
            width: 5,
            kind: BitFieldKind::Bfi,
        }
    );
    // bfc r0, #3, #5 = 0xF36F 00C7（Rn=1111 → Bfc）
    assert_eq!(
        d.decode_word(0xF36F_00C7, 0),
        Instruction::BitField {
            rd: 0,
            rn: 0xF,
            lsb: 3,
            width: 5,
            kind: BitFieldKind::Bfc,
        }
    );
}

/// LDREX/STREX 家族解码（FRT-INS-05 SHOULD，单核语义）
#[test]
fn decode_ldrex_strex() {
    let mut d = Decoder::new();
    // ldrex r0, [r1] = 0xE851 0F00（imm=0×4）
    assert_eq!(
        d.decode_word(0xE851_0F00, 0),
        Instruction::Ldrex {
            rt: 0,
            rn: 1,
            imm: 0,
            width: AccessWidth::Word,
        }
    );
    // ldrex r1, [r0, #4] = 0xE850 1F01（imm=1×4）
    assert_eq!(
        d.decode_word(0xE850_1F01, 0),
        Instruction::Ldrex {
            rt: 1,
            rn: 0,
            imm: 4,
            width: AccessWidth::Word,
        }
    );
    // strex r0, r1, [r2] = 0xE842 1000
    assert_eq!(
        d.decode_word(0xE842_1000, 0),
        Instruction::Strex {
            rd: 0,
            rt: 1,
            rn: 2,
            imm: 0,
            width: AccessWidth::Word,
        }
    );
    // strex r2, r3, [r0, #4] = 0xE840 3201
    assert_eq!(
        d.decode_word(0xE840_3201, 0),
        Instruction::Strex {
            rd: 2,
            rt: 3,
            rn: 0,
            imm: 4,
            width: AccessWidth::Word,
        }
    );
    // ldrexb r4, [r5] = 0xE8D5 4F4F
    assert_eq!(
        d.decode_word(0xE8D5_4F4F, 0),
        Instruction::Ldrex {
            rt: 4,
            rn: 5,
            imm: 0,
            width: AccessWidth::Byte,
        }
    );
    // ldrexh r9, [r10] = 0xE8DA 9F5F
    assert_eq!(
        d.decode_word(0xE8DA_9F5F, 0),
        Instruction::Ldrex {
            rt: 9,
            rn: 10,
            imm: 0,
            width: AccessWidth::HalfWord,
        }
    );
    // strexb r6, r7, [r8] = 0xE8C8 7F46（Rd=r6 在低 4 位）
    assert_eq!(
        d.decode_word(0xE8C8_7F46, 0),
        Instruction::Strex {
            rd: 6,
            rt: 7,
            rn: 8,
            imm: 0,
            width: AccessWidth::Byte,
        }
    );
    // strexh r11, r12, [r0] = 0xE8C0 CF5B
    assert_eq!(
        d.decode_word(0xE8C0_CF5B, 0),
        Instruction::Strex {
            rd: 11,
            rt: 12,
            rn: 0,
            imm: 0,
            width: AccessWidth::HalfWord,
        }
    );
}
}
