//! FPU — Cortex-M4F 浮点单元（单精度为主 + 双精度骨架）
//!
//! Phase 4 实现：寄存器文件 S0-S31/D0-D15 别名、FPSCR 状态位、VFP 立即数展开、
//! 浮点运算语义（含异常标志 IOC/DZC/OFC/UFC/IXC）。
//!
//! 说明：
//! - 双精度为骨架级支持（硬件 M4F 仅 VFPv4-SP），Dn 与 S(2n):S(2n+1) 别名。
//! - 舍入模式：RN（默认）用原生 f32/f64 运算；RZ/RP/RM 通过 f64 中间量近似
//!   （双精度定向舍入为骨架近似，见 `round_f64_per_mode` 注释）。
//! - IXC（不精确）检测：单精度用 f64 精确中间量比对；双精度用 u128 尾数分解
//!   判断结果是否可无舍入表示（加法/减法/乘法精确可判，除法用整除性近似）。

/// FPU 寄存器文件（S0-S31 单精度 / D0-D15 双精度别名）
#[derive(Debug, Default, Clone)]
pub struct FpuRegisters {
    /// S0-S31 单精度寄存器（按位存储 u32）
    pub s: [u32; 32],
    /// FPSCR 浮点状态与控制寄存器
    pub fpscr: u32,
    /// FPCCR 浮点上下文控制（惰性压栈）
    pub fpccr: u32,
}

impl FpuRegisters {
    pub fn new() -> Self {
        Self::default()
    }

    /// 读取单精度寄存器 S[n]
    pub fn read_s(&self, n: usize) -> u32 {
        self.s[n & 0x1F]
    }

    /// 写入单精度寄存器 S[n]
    pub fn write_s(&mut self, n: usize, val: u32) {
        self.s[n & 0x1F] = val;
    }

    /// 读取双精度寄存器 D[n]（两个 S 寄存器组合）
    pub fn read_d(&self, n: usize) -> u64 {
        let idx = (n & 0xF) * 2;
        (self.s[idx] as u64) | ((self.s[idx + 1] as u64) << 32)
    }

    /// 写入双精度寄存器 D[n]
    pub fn write_d(&mut self, n: usize, val: u64) {
        let idx = (n & 0xF) * 2;
        self.s[idx] = val as u32;
        self.s[idx + 1] = (val >> 32) as u32;
    }

    /// 读取 FPSCR 标志位（N/Z/C/V）
    pub fn fpscr_flags(&self) -> (bool, bool, bool, bool) {
        (
            self.fpscr & (1 << 31) != 0, // N
            self.fpscr & (1 << 30) != 0, // Z
            self.fpscr & (1 << 29) != 0, // C
            self.fpscr & (1 << 28) != 0, // V
        )
    }

    /// 写入 N/Z/C/V 标志（VCMP 结果）
    pub fn set_nzcv(&mut self, n: bool, z: bool, c: bool, v: bool) {
        let mask = 0xF << 28;
        let val = ((n as u32) << 31) | ((z as u32) << 30) | ((c as u32) << 29) | ((v as u32) << 28);
        self.fpscr = (self.fpscr & !mask) | val;
    }

    /// 置位累积异常标志（IOC/DZC/OFC/UFC/IXC/IDC，均为粘性置位）
    pub fn set_cumulative(
        &mut self,
        ioc: bool,
        dzc: bool,
        ofc: bool,
        ufc: bool,
        ixc: bool,
        idc: bool,
    ) {
        let mut flags = 0u32;
        if ioc {
            flags |= 1 << 0;
        }
        if dzc {
            flags |= 1 << 1;
        }
        if ofc {
            flags |= 1 << 2;
        }
        if ufc {
            flags |= 1 << 3;
        }
        if ixc {
            flags |= 1 << 4;
        }
        if idc {
            flags |= 1 << 7;
        }
        self.fpscr |= flags;
    }

    /// 置位 QC（累积饱和标志，粘性）
    pub fn set_qc(&mut self) {
        self.fpscr |= 1 << 27;
    }

    /// 读取 QC 标志
    pub fn qc(&self) -> bool {
        self.fpscr & (1 << 27) != 0
    }

    /// 当前舍入模式（FPSCR[23:22]）
    pub fn rounding_mode(&self) -> FpRounding {
        match (self.fpscr >> 22) & 0x3 {
            0 => FpRounding::Nearest,
            1 => FpRounding::PlusInf,
            2 => FpRounding::MinusInf,
            _ => FpRounding::Zero,
        }
    }

    /// 刷新到零模式（FPSCR[24] FZ）
    pub fn flush_to_zero(&self) -> bool {
        self.fpscr & (1 << 24) != 0
    }

    /// 复位 FPU 状态
    pub fn reset(&mut self) {
        self.s = [0; 32];
        self.fpscr = 0;
    }
}

/// FPSCR 舍入模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FpRounding {
    /// 就近舍入（偶数，默认）
    Nearest,
    /// 向 +∞ 舍入
    PlusInf,
    /// 向 −∞ 舍入
    MinusInf,
    /// 向零舍入
    Zero,
}

/// VFP 立即数展开（ARM ARM VFPExpandImm，位模式匹配 QEMU vfp_expand_imm）
///
/// imm8 布局：bit7 = 符号，bit6 = 指数基选择（1 → 0x7C/0x3FE 基，0 → 0x80/0x400 基），
/// bit5:4 = 指数低 2 位，bit3:0 = 尾数高 4 位。
pub fn vfp_expand_imm(imm8: u8, double: bool) -> u64 {
    let sign = (imm8 & 0x80) as u64;
    let exp_base: u64 = if imm8 & 0x40 != 0 {
        if double {
            0x3FE
        } else {
            0x7C
        }
    } else {
        if double {
            0x400
        } else {
            0x80
        }
    };
    let exp = exp_base | (((imm8 >> 5) & 1) as u64) << 1 | (((imm8 >> 4) & 1) as u64);
    if double {
        (sign << 56) | (exp << 52) | (((imm8 & 0xF) as u64) << 48)
    } else {
        (sign << 24) | (exp << 23) | (((imm8 & 0xF) as u64) << 19)
    }
}

/// 单精度 f32 运算标志（供 exec 汇总到 FPSCR）
#[derive(Debug, Clone, Copy, Default)]
pub struct FpOpFlags {
    pub ioc: bool,
    pub dzc: bool,
    pub ofc: bool,
    pub ufc: bool,
    pub ixc: bool,
    pub idc: bool,
    /// 饱和（VCVT 范围溢出 → FPSCR.QC）
    pub qc: bool,
}

impl FpOpFlags {
    /// 判断输入是否含次正规数（输入非正规 → IDC）
    fn probe_inputs_f32(&mut self, a: u32, b: u32) {
        self.idc = is_denormal_f32(a) || is_denormal_f32(b);
    }
    fn probe_inputs_f64(&mut self, a: u64, b: u64) {
        self.idc = is_denormal_f64(a) || is_denormal_f64(b);
    }
}

/// 判断 f32 位模式是否为次正规数（指数域全 0 且尾数非 0）
pub fn is_denormal_f32(bits: u32) -> bool {
    bits & 0x7F80_0000 == 0 && bits & 0x007F_FFFF != 0
}

/// 判断 f64 位模式是否为次正规数
pub fn is_denormal_f64(bits: u64) -> bool {
    bits & 0x7FF0_0000_0000_0000 == 0 && bits & 0x000F_FFFF_FFFF_FFFF != 0
}

/// 判断 f32 是否为 NaN（安静或信号）
pub fn is_nan_f32(bits: u32) -> bool {
    bits & 0x7F80_0000 == 0x7F80_0000 && bits & 0x007F_FFFF != 0
}

/// 判断 f64 是否为 NaN
pub fn is_nan_f64(bits: u64) -> bool {
    bits & 0x7FF0_0000_0000_0000 == 0x7FF0_0000_0000_0000 && bits & 0x000F_FFFF_FFFF_FFFF != 0
}

/// 安静化 NaN（置尾数最高位）
pub fn quiet_nan(bits: u32) -> u32 {
    bits | 0x0040_0000
}

/// 判断 f32 是否为信号 NaN（尾数最高位为 0）
pub fn is_signaling_nan_f32(bits: u32) -> bool {
    is_nan_f32(bits) && bits & 0x0040_0000 == 0
}

/// 判断 f64 是否为信号 NaN
pub fn is_signaling_nan_f64(bits: u64) -> bool {
    is_nan_f64(bits) && bits & 0x0008_0000_0000_0000 == 0
}

/// 安静化 NaN（f64）
fn quiet_nan_f64(bits: u64) -> u64 {
    bits | 0x0008_0000_0000_0000
}

/// 默认 NaN（VSQRT 负数、DN=1 时使用）
pub const DEFAULT_NAN_F32: u32 = 0x7FC0_0000;
pub const DEFAULT_NAN_F64: u64 = 0x7FF8_0000_0000_0000;

/// f32 加法（含 FPSCR 舍入模式与异常标志）
pub fn f32_add(fpu: &FpuRegisters, a: u32, b: u32) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f32(a, b);
    if is_nan_f32(a) || is_nan_f32(b) {
        flags.ioc = true;
        let nan = if is_nan_f32(a) { a } else { b };
        return (quiet_nan(nan), flags);
    }
    let ra = f32::from_bits(a);
    let rb = f32::from_bits(b);
    let res = f32_binop_rounded(fpu, |x, y| x + y, |x, y| x + y, ra, rb, &mut flags);
    (res.to_bits(), flags)
}

/// f32 减法
pub fn f32_sub(fpu: &FpuRegisters, a: u32, b: u32) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f32(a, b);
    if is_nan_f32(a) || is_nan_f32(b) {
        flags.ioc = true;
        let nan = if is_nan_f32(a) { a } else { b };
        return (quiet_nan(nan), flags);
    }
    let ra = f32::from_bits(a);
    let rb = f32::from_bits(b);
    let res = f32_binop_rounded(fpu, |x, y| x - y, |x, y| x - y, ra, rb, &mut flags);
    (res.to_bits(), flags)
}

/// f32 乘法
pub fn f32_mul(fpu: &FpuRegisters, a: u32, b: u32) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f32(a, b);
    if is_nan_f32(a) || is_nan_f32(b) {
        flags.ioc = true;
        let nan = if is_nan_f32(a) { a } else { b };
        return (quiet_nan(nan), flags);
    }
    let ra = f32::from_bits(a);
    let rb = f32::from_bits(b);
    let res = f32_binop_rounded(fpu, |x, y| x * y, |x, y| x * y, ra, rb, &mut flags);
    (res.to_bits(), flags)
}

/// f32 除法（除零 → DZC）
pub fn f32_div(fpu: &FpuRegisters, a: u32, b: u32) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f32(a, b);
    if is_nan_f32(a) || is_nan_f32(b) {
        flags.ioc = true;
        let nan = if is_nan_f32(a) { a } else { b };
        return (quiet_nan(nan), flags);
    }
    let ra = f32::from_bits(a);
    let rb = f32::from_bits(b);
    if rb == 0.0 && ra != 0.0 {
        flags.dzc = true;
        let res = if ra > 0.0 {
            f32::INFINITY
        } else {
            f32::NEG_INFINITY
        };
        // 除零结果按 ARM 语义为 ±Inf（被除数为 0/0 时由 NaN 分支处理）
        return (res.to_bits(), flags);
    }
    let res = f32_binop_rounded(fpu, |x, y| x / y, |x, y| x / y, ra, rb, &mut flags);
    (res.to_bits(), flags)
}

/// 按 FPSCR 舍入模式计算 f32 二元运算
///
/// - RN（默认）：原生 f32 运算（正确舍入）。
/// - RZ/RP/RM：经 f64 精确中间量再定向舍入（常规指数范围内 f32 二元运算
///   在 f64 中可精确表示，无双重舍入；极端指数差场景为骨架近似）。
fn f32_binop_rounded<FN, FW>(
    fpu: &FpuRegisters,
    op_narrow: FN,
    op_wide: FW,
    a: f32,
    b: f32,
    flags: &mut FpOpFlags,
) -> f32
where
    FN: Fn(f32, f32) -> f32,
    FW: Fn(f64, f64) -> f64,
{
    let wide = op_wide(a as f64, b as f64);
    let res = if fpu.rounding_mode() == FpRounding::Nearest {
        op_narrow(a, b)
    } else {
        round_f64_per_mode_f32(fpu.rounding_mode(), wide)
    };
    f32_flags_common(a, b, res, wide, flags);
    if fpu.flush_to_zero() && is_denormal_f32(res.to_bits()) {
        return 0.0;
    }
    res
}

/// f32 运算通用标志（溢出/次正规/不精确）
///
/// 不精确判定：f32 结果与 f64 中间量比对。常规指数范围内 f64 对 f32 二元运算
/// 精确，判定可靠；除法 f64 商本身可能已舍入，属骨架近似（正常测试值可靠）。
fn f32_flags_common(a: f32, b: f32, res: f32, wide: f64, flags: &mut FpOpFlags) {
    if res.is_infinite() && !a.is_infinite() && !b.is_infinite() {
        flags.ofc = true;
        flags.ixc = true;
    }
    if is_denormal_f32(res.to_bits()) {
        flags.ufc = true;
        flags.ixc = true;
    }
    if (res as f64) != wide {
        flags.ixc = true;
    }
}

/// 从 f64 按舍入模式取 f32（next_up/next_down 逐位调整）
fn round_f64_per_mode_f32(mode: FpRounding, v: f64) -> f32 {
    let r = v as f32; // 先 RN
    match mode {
        FpRounding::Nearest => r,
        FpRounding::Zero => {
            if v >= 0.0 {
                if r > v as f32 {
                    next_down_f32(r)
                } else {
                    r
                }
            } else if r < v as f32 {
                next_up_f32(r)
            } else {
                r
            }
        }
        FpRounding::PlusInf => {
            if r < v as f32 {
                next_up_f32(r)
            } else {
                r
            }
        }
        FpRounding::MinusInf => {
            if r > v as f32 {
                next_down_f32(r)
            } else {
                r
            }
        }
    }
}

/// 下一个（向 +∞ 方向）可表示的 f32
fn next_up_f32(f: f32) -> f32 {
    if f.is_nan() || f == f32::INFINITY {
        return f;
    }
    if f == 0.0 {
        return f32::from_bits(1); // +最小次正规
    }
    if f.is_sign_negative() {
        f32::from_bits(f.to_bits() - 1)
    } else {
        f32::from_bits(f.to_bits() + 1)
    }
}

/// 下一个（向 −∞ 方向）可表示的 f32
fn next_down_f32(f: f32) -> f32 {
    if f.is_nan() || f == f32::NEG_INFINITY {
        return f;
    }
    if f == 0.0 {
        return f32::from_bits(0x8000_0001); // −最小次正规
    }
    if f.is_sign_negative() {
        f32::from_bits(f.to_bits() + 1)
    } else {
        f32::from_bits(f.to_bits() - 1)
    }
}

/// f32 乘加（VMLA/VMLS 等，融合单舍入）
pub fn f32_mul_add(
    fpu: &FpuRegisters,
    a: u32,
    b: u32,
    c: u32,
    neg_prod: bool,
    neg_acc: bool,
) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f32(a, b);
    flags.probe_inputs_f32(c, 0);
    let ra = f32::from_bits(a);
    let rb = f32::from_bits(b);
    let rc = f32::from_bits(c);
    if is_nan_f32(a) || is_nan_f32(b) || is_nan_f32(c) {
        flags.ioc = true;
        let nan = if is_nan_f32(a) {
            a
        } else if is_nan_f32(b) {
            b
        } else {
            c
        };
        return (quiet_nan(nan), flags);
    }
    // 融合乘加：Rust mul_add 即 fused（单舍入）
    let (pa, pb) = if neg_prod { (-ra, rb) } else { (ra, rb) };
    let res = pa.mul_add(pb, rc);
    let res = if neg_acc { -res } else { res };
    // 溢出/次正规/不精确：mul_add 的精确结果可通过 f64 对比近似（f32 融合乘加在常规范围）
    if res.is_infinite() && !ra.is_infinite() && !rb.is_infinite() && !rc.is_infinite() {
        flags.ofc = true;
        flags.ixc = true;
    }
    if is_denormal_f32(res.to_bits()) {
        flags.ufc = true;
        flags.ixc = true;
    }
    if fpu.flush_to_zero() && is_denormal_f32(res.to_bits()) {
        return (0.0f32.to_bits(), flags);
    }
    (res.to_bits(), flags)
}

// ==================== 双精度（骨架） ====================

/// f64 加法
pub fn f64_add(fpu: &FpuRegisters, a: u64, b: u64) -> (u64, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f64(a, b);
    if is_nan_f64(a) || is_nan_f64(b) {
        flags.ioc = true;
        let nan = if is_nan_f64(a) { a } else { b };
        return (quiet_nan_f64(nan), flags);
    }
    let ra = f64::from_bits(a);
    let rb = f64::from_bits(b);
    let res = ra + rb;
    let mode = fpu.rounding_mode();
    if mode == FpRounding::Nearest {
        f64_common_flags(ra, rb, res, &mut flags, BinOp::Add);
        (res.to_bits(), flags)
    } else {
        let rel = f64_add_relation(ra, rb, res);
        let final_res = round_f64_dir(mode, res, rel);
        f64_dir_flags(ra, rb, final_res, rel, &mut flags);
        (final_res.to_bits(), flags)
    }
}

/// f64 减法
pub fn f64_sub(fpu: &FpuRegisters, a: u64, b: u64) -> (u64, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f64(a, b);
    if is_nan_f64(a) || is_nan_f64(b) {
        flags.ioc = true;
        let nan = if is_nan_f64(a) { a } else { b };
        return (quiet_nan_f64(nan), flags);
    }
    let ra = f64::from_bits(a);
    let rb = f64::from_bits(b);
    let res = ra - rb;
    let mode = fpu.rounding_mode();
    if mode == FpRounding::Nearest {
        f64_common_flags(ra, rb, res, &mut flags, BinOp::Sub);
        (res.to_bits(), flags)
    } else {
        let rel = f64_add_relation(ra, -rb, res);
        let final_res = round_f64_dir(mode, res, rel);
        f64_dir_flags(ra, rb, final_res, rel, &mut flags);
        (final_res.to_bits(), flags)
    }
}

/// f64 乘法
pub fn f64_mul(fpu: &FpuRegisters, a: u64, b: u64) -> (u64, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f64(a, b);
    if is_nan_f64(a) || is_nan_f64(b) {
        flags.ioc = true;
        let nan = if is_nan_f64(a) { a } else { b };
        return (quiet_nan_f64(nan), flags);
    }
    let ra = f64::from_bits(a);
    let rb = f64::from_bits(b);
    let res = ra * rb;
    let mode = fpu.rounding_mode();
    if mode == FpRounding::Nearest {
        f64_common_flags(ra, rb, res, &mut flags, BinOp::Mul);
        (res.to_bits(), flags)
    } else {
        let rel = f64_mul_relation(ra, rb, res);
        let final_res = round_f64_dir(mode, res, rel);
        f64_dir_flags(ra, rb, final_res, rel, &mut flags);
        (final_res.to_bits(), flags)
    }
}

/// f64 除法
pub fn f64_div(fpu: &FpuRegisters, a: u64, b: u64) -> (u64, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f64(a, b);
    if is_nan_f64(a) || is_nan_f64(b) {
        flags.ioc = true;
        let nan = if is_nan_f64(a) { a } else { b };
        return (quiet_nan_f64(nan), flags);
    }
    let ra = f64::from_bits(a);
    let rb = f64::from_bits(b);
    if rb == 0.0 && ra != 0.0 {
        flags.dzc = true;
        let res = if ra > 0.0 {
            f64::INFINITY
        } else {
            f64::NEG_INFINITY
        };
        return (res.to_bits(), flags);
    }
    let res = ra / rb;
    let mode = fpu.rounding_mode();
    if mode == FpRounding::Nearest {
        f64_common_flags(ra, rb, res, &mut flags, BinOp::Div);
        (res.to_bits(), flags)
    } else {
        let rel = f64_div_relation(ra, rb, res);
        let final_res = round_f64_dir(mode, res, rel);
        f64_dir_flags(ra, rb, final_res, rel, &mut flags);
        (final_res.to_bits(), flags)
    }
}

/// f64 乘加（融合单舍入）
pub fn f64_mul_add(
    _fpu: &FpuRegisters,
    a: u64,
    b: u64,
    c: u64,
    neg_prod: bool,
    neg_acc: bool,
) -> (u64, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    flags.probe_inputs_f64(a, b);
    flags.probe_inputs_f64(c, 0);
    let ra = f64::from_bits(a);
    let rb = f64::from_bits(b);
    let rc = f64::from_bits(c);
    if is_nan_f64(a) || is_nan_f64(b) || is_nan_f64(c) {
        flags.ioc = true;
        let nan = if is_nan_f64(a) {
            a
        } else if is_nan_f64(b) {
            b
        } else {
            c
        };
        return (quiet_nan_f64(nan), flags);
    }
    let (pa, pb) = if neg_prod { (-ra, rb) } else { (ra, rb) };
    let res = pa.mul_add(pb, rc);
    let res = if neg_acc { -res } else { res };
    if res.is_infinite() && !ra.is_infinite() && !rb.is_infinite() && !rc.is_infinite() {
        flags.ofc = true;
        flags.ixc = true;
    }
    if is_denormal_f64(res.to_bits()) {
        flags.ufc = true;
        flags.ixc = true;
    }
    (res.to_bits(), flags)
}

#[derive(Clone, Copy)]
enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
}

/// 汇总 f64 二元运算通用标志（含精确性判定）
fn f64_common_flags(a: f64, b: f64, res: f64, flags: &mut FpOpFlags, op: BinOp) {
    if res.is_infinite() && !a.is_infinite() && !b.is_infinite() {
        flags.ofc = true;
        flags.ixc = true;
    }
    if is_denormal_f64(res.to_bits()) {
        flags.ufc = true;
        flags.ixc = true;
    }
    // 精确性判定：u128 尾数分解（加法/减法/乘法精确可判；除法用整除性近似）
    let exact = match op {
        BinOp::Add => f64_add_exact(a, b),
        BinOp::Sub => f64_add_exact(a, -b),
        BinOp::Mul => f64_mul_exact(a, b),
        BinOp::Div => f64_div_exact(a, b),
    };
    if !exact {
        flags.ixc = true;
    }
}

/// 分解 f64 为 (尾数, 无偏指数)；零与次正规按 ARM 语义简化处理
fn f64_decompose(v: f64) -> (u64, i32) {
    let bits = v.to_bits();
    let exp = ((bits >> 52) & 0x7FF) as i32;
    let frac = bits & 0xF_FFFF_FFFF_FFFF;
    if exp == 0 {
        // 次正规/零：尾数无隐含位
        (frac, -1022)
    } else {
        (frac | (1 << 52), exp - 1023)
    }
}

/// f64 加法/减法精确性：结果可无舍入表示（u128 尾数算术）
fn f64_add_exact(a: f64, b: f64) -> bool {
    if a == 0.0 || b == 0.0 {
        return true;
    }
    if a.is_infinite() || b.is_infinite() {
        return true; // 无穷结果由 OFC 处理，不算舍入
    }
    let (ma, ea) = f64_decompose(a);
    let (mb, eb) = f64_decompose(b);
    // 统一到较大指数
    let (big_m, big_e, small_m, small_e) = if ea >= eb {
        (ma, ea, mb, eb)
    } else {
        (mb, eb, ma, ea)
    };
    let diff = (big_e - small_e) as u32;
    if diff > 52 {
        // 小操作数完全落在目标 LSB 之下 → 结果需要舍入（除非被吸收为 0）
        return false;
    }
    // 尾数符号：以 f64 符号分离处理
    let sign_a = a.is_sign_negative();
    let sign_b = b.is_sign_negative();
    // 对齐：小尾数右移 diff，移出位记 sticky
    let shifted = if diff >= 64 {
        (0u128, small_m != 0)
    } else {
        let sm = small_m as u128;
        (sm >> diff, (sm & ((1u128 << diff) - 1)) != 0)
    };
    if shifted.1 {
        return false; // 移出非零位 → 必然舍入
    }
    let (mut m_big, mut m_small) = (big_m as u128, shifted.0);
    if sign_a != (ea >= eb) {
        m_big = m_big.wrapping_neg();
    }
    if sign_b != (ea < eb) {
        m_small = m_small.wrapping_neg();
    }
    let sum = m_big.wrapping_add(m_small);
    // 归一化后有效位数 ≤ 53 且无被移出的非零位 → 精确
    sig_bits_u128(sum) <= 53
}

/// f64 乘法精确性：尾数乘积的有效位数 ≤ 53
fn f64_mul_exact(a: f64, b: f64) -> bool {
    if a == 0.0 || b == 0.0 {
        return true;
    }
    if a.is_infinite() || b.is_infinite() {
        return true;
    }
    let (ma, _ea) = f64_decompose(a);
    let (mb, _eb) = f64_decompose(b);
    let p = (ma as u128) * (mb as u128);
    if p == 0 {
        return true;
    }
    sig_bits_u128(p) <= 53
}

/// f64 除法精确性近似：约分后分母为 2 的幂且商有效位数 ≤ 53
fn f64_div_exact(a: f64, b: f64) -> bool {
    if a == 0.0 {
        return true;
    }
    if b.is_infinite() || a.is_infinite() {
        return true;
    }
    let (ma, ea) = f64_decompose(a);
    let (mb, eb) = f64_decompose(b);
    let g = gcd_u64(ma, mb);
    let m1 = ma / g;
    let m2 = mb / g;
    if !m2.is_power_of_two() {
        return false; // 分母含奇因子 → 二进制无限循环 → 不精确
    }
    let shift = m2.trailing_zeros();
    // 商 = m1 × 2^(ea - eb - shift)：需归一化后有效位数 ≤ 53
    let sig = sig_bits_u64(m1);
    if sig > 53 {
        return false;
    }
    // 指数范围检查（正规结果可表示性；次正规除法近似认为不精确）
    let exp = ea - eb - shift as i32;
    (-1021..=1024).contains(&exp)
}

/// 有效位数（bit_length − 尾随零），0 返回 0
fn sig_bits_u128(x: u128) -> usize {
    if x == 0 {
        0
    } else {
        (128 - x.leading_zeros() - x.trailing_zeros()) as usize
    }
}

/// 有效位数（u64）
fn sig_bits_u64(x: u64) -> usize {
    if x == 0 {
        0
    } else {
        (64 - x.leading_zeros() - x.trailing_zeros()) as usize
    }
}

fn gcd_u64(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

// ==================== f64 定向舍入（P5-补：RZ/RP/RM 精确化） ====================
//
// 策略：以原生 f64 运算得到正确舍入的 RN 结果 r，再按「精确结果与 r 的大小关系」
// 用 next_up/next_down 调整 1 ulp 得到 RZ/RP/RM 结果。
// - 加/减/乘：精确结果用 u128/i128 尾数算术精确求出，关系判定精确（含 sticky 尾）。
// - 除：RN 结果正确舍入；关系用融合乘加余数 r*b-a 的符号近似判定（常规值可靠，
//   极端指数差场景有理论误差，见 f64_div_relation 注释）。

use std::cmp::Ordering;

/// 下一个可表示的 f64（向 +∞）
fn next_up_f64(f: f64) -> f64 {
    if f.is_nan() || f == f64::INFINITY {
        return f;
    }
    if f == 0.0 {
        return f64::from_bits(1); // +最小次正规
    }
    if f.is_sign_negative() {
        f64::from_bits(f.to_bits() - 1)
    } else {
        f64::from_bits(f.to_bits() + 1)
    }
}

/// 下一个可表示的 f64（向 −∞）
fn next_down_f64(f: f64) -> f64 {
    if f.is_nan() || f == f64::NEG_INFINITY {
        return f;
    }
    if f == 0.0 {
        return f64::from_bits(0x8000_0000_0000_0001); // −最小次正规
    }
    if f.is_sign_negative() {
        f64::from_bits(f.to_bits() + 1)
    } else {
        f64::from_bits(f.to_bits() - 1)
    }
}

/// 按舍入模式对 RN 结果 r 做定向调整（rel = 精确结果与 r 的关系）
fn round_f64_dir(mode: FpRounding, r: f64, rel: Ordering) -> f64 {
    match mode {
        FpRounding::Nearest => r,
        FpRounding::Zero => match rel {
            Ordering::Equal => r,
            Ordering::Greater => {
                if r > 0.0 {
                    r
                } else {
                    next_up_f64(r)
                }
            }
            Ordering::Less => {
                if r > 0.0 {
                    next_down_f64(r)
                } else {
                    r
                }
            }
        },
        FpRounding::PlusInf => match rel {
            Ordering::Equal => r,
            Ordering::Greater => next_up_f64(r),
            Ordering::Less => r,
        },
        FpRounding::MinusInf => match rel {
            Ordering::Equal => r,
            Ordering::Greater => r,
            Ordering::Less => next_down_f64(r),
        },
    }
}

/// 定向舍入路径的通用标志（OFC/UFC/IXC）
fn f64_dir_flags(a: f64, b: f64, res: f64, rel: Ordering, flags: &mut FpOpFlags) {
    if res.is_infinite() && !a.is_infinite() && !b.is_infinite() {
        flags.ofc = true;
        flags.ixc = true;
    }
    if is_denormal_f64(res.to_bits()) {
        flags.ufc = true;
        flags.ixc = true;
    }
    if rel != Ordering::Equal {
        flags.ixc = true;
    }
}

/// 精确加法结果：返回 (有符号尾数 i128, 指数, 尾整数 u64, 尾指数, 尾为负)
/// 值 = sig × 2^(exp-52) ± tail_int × 2^(tail_exp-52)，|tail_int × 2^(tail_exp-exp)| < 1 帧 LSB
fn f64_exact_add(a: f64, b: f64) -> (i128, i32, u64, i32, bool) {
    let sa = a.is_sign_negative();
    let sb = b.is_sign_negative();
    let (ma, ea) = f64_decompose(a);
    let (mb, eb) = f64_decompose(b);
    let (big_m, big_e, small_m, small_e, big_neg, small_neg) = if ea >= eb {
        (ma, ea, mb, eb, sa, sb)
    } else {
        (mb, eb, ma, ea, sb, sa)
    };
    let diff = (big_e - small_e) as u32;
    let (shifted, tail_int) = if diff >= 64 {
        (0i128, small_m)
    } else {
        let sm = small_m as i128;
        (sm >> diff, small_m & ((1u64 << diff) - 1))
    };
    let mut sig = big_m as i128;
    if big_neg {
        sig = -sig;
    }
    let mut s = shifted;
    if small_neg {
        s = -s;
    }
    (sig + s, big_e, tail_int, small_e, small_neg)
}

/// 精确结果 (sig, exp, tail_int, tail_exp, tail_neg) 与 RN 结果 r 的比较。
///
/// 将三者统一到公共指数帧（min(exp, rexp)）做有符号 i128 全精度比较；
/// 尾数右移丢失的位以 sticky 破平。RN 语义下指数差 ≤ 1，60 位移位余量充足，
/// 超出时（非法数据防御）结果可能不准（见 f64_div_relation 的近似注记）。
fn cmp_exact_vs_f64(
    sig: i128,
    exp: i32,
    tail_int: u64,
    tail_exp: i32,
    tail_neg: bool,
    r: f64,
) -> Ordering {
    if r == 0.0 {
        if sig != 0 || tail_int != 0 {
            let neg = sig < 0 || (sig == 0 && tail_neg);
            return if neg {
                Ordering::Less
            } else {
                Ordering::Greater
            };
        }
        return Ordering::Equal;
    }
    if r.is_infinite() {
        return if r > 0.0 {
            Ordering::Less
        } else {
            Ordering::Greater
        };
    }
    let (rsig, rexp) = f64_decompose(r);
    let e_min = exp.min(rexp);
    // sig / rsig 左移到公共帧（带符号；r 的符号并入 rsig）
    let lhs = (sig as i128) << (exp - e_min).min(60);
    let r_signed = if r.is_sign_negative() {
        -(rsig as i128)
    } else {
        rsig as i128
    };
    let rhs = r_signed << (rexp - e_min).min(60);
    // tail 对齐：tail_exp ≤ exp；可能需右移（丢失位 → sticky）
    let (tail_v, tail_sticky) = if tail_exp >= e_min {
        ((tail_int as i128) << (tail_exp - e_min).min(60), false)
    } else {
        let sh = (e_min - tail_exp).min(64);
        if sh >= 64 {
            (0i128, tail_int != 0)
        } else {
            (
                (tail_int as i128) >> sh,
                (tail_int & ((1u64 << sh) - 1)) != 0,
            )
        }
    };
    let tail_signed = if tail_neg { -tail_v } else { tail_v };
    let diff = lhs + tail_signed - rhs;
    match diff.cmp(&0) {
        Ordering::Equal => {
            // 帧内整数相等：由 sticky 破平
            if tail_sticky {
                if tail_neg {
                    Ordering::Less
                } else {
                    Ordering::Greater
                }
            } else {
                Ordering::Equal
            }
        }
        o => o,
    }
}

/// 加法/减法：精确结果与 RN 结果 r 的关系
fn f64_add_relation(a: f64, b: f64, r: f64) -> Ordering {
    if a == 0.0 || b == 0.0 {
        // 精确结果 = 另一操作数
        let x = if a == 0.0 { b } else { a };
        return if x < r {
            Ordering::Less
        } else if x > r {
            Ordering::Greater
        } else {
            Ordering::Equal
        };
    }
    if r.is_infinite() {
        // 精确有限 → 与 ±Inf 的关系
        return if r > 0.0 {
            Ordering::Less
        } else {
            Ordering::Greater
        };
    }
    let (sig, exp, tail_int, tail_exp, tail_neg) = f64_exact_add(a, b);
    cmp_exact_vs_f64(sig, exp, tail_int, tail_exp, tail_neg, r)
}

/// 乘法：精确乘积（u128 尾数，无尾数）与 RN 结果 r 的关系
fn f64_mul_relation(a: f64, b: f64, r: f64) -> Ordering {
    if a == 0.0 || b == 0.0 {
        // 精确结果 = ±0
        return if r == 0.0 {
            Ordering::Equal
        } else if r > 0.0 {
            Ordering::Less
        } else {
            Ordering::Greater
        };
    }
    if r.is_infinite() {
        return if r > 0.0 {
            Ordering::Less
        } else {
            Ordering::Greater
        };
    }
    let (ma, ea) = f64_decompose(a);
    let (mb, eb) = f64_decompose(b);
    let mut sig = (ma as i128) * (mb as i128);
    if a.is_sign_negative() ^ b.is_sign_negative() {
        sig = -sig;
    }
    cmp_exact_vs_f64(sig, ea + eb, 0, ea + eb, false, r)
}

/// 除法：关系近似判定（余数符号法）。
///
/// rem = fma(r, b, -a) = r×b − a（单舍入）。rem 的符号给出 r 相对 a/b 的偏向：
/// r×b > a ⟺ r > a/b（b > 0 时）。常规指数范围内可靠；极端指数差场景（次正规/超大
/// 指数）fma 自身舍入可能引入误差，属近似（与 f32 除法的既有近似同级别）。
fn f64_div_relation(a: f64, b: f64, r: f64) -> Ordering {
    if r == 0.0 {
        if a == 0.0 {
            return Ordering::Equal; // 0/b = 0
        }
        return if (a > 0.0) == (b > 0.0) {
            Ordering::Greater
        } else {
            Ordering::Less
        };
    }
    if r.is_infinite() {
        return if r > 0.0 {
            Ordering::Less
        } else {
            Ordering::Greater
        };
    }
    let rem = r.mul_add(b, -a); // r×b − a（融合单舍入）
    if rem == 0.0 {
        Ordering::Equal
    } else if (rem > 0.0) == (b > 0.0) {
        Ordering::Less // r > a/b → 精确 < r
    } else {
        Ordering::Greater
    }
}

// ==================== VCVT 转换（整数 ↔ 浮点） ====================

/// 整数 → f32（按 FPSCR 舍入模式；整数在 f64 中精确，无双重舍入）
pub fn cvt_int_to_f32(fpu: &FpuRegisters, x: i64) -> f32 {
    round_f64_per_mode_f32(fpu.rounding_mode(), x as f64)
}

/// 整数 → f64（i32/u32 在 f64 中精确）
pub fn cvt_int_to_f64(x: i64) -> f64 {
    x as f64
}

/// f32 → 有符号/无符号 32 位整数
///
/// `round_nearest`：true = VCVTR（就近舍入），false = VCVT（朝零舍入）。
/// 语义：NaN → 0 + IOC；越界 → 饱和 + QC；舍入发生 → IXC。
pub fn cvt_f32_to_int(
    _fpu: &FpuRegisters,
    bits: u32,
    signed: bool,
    round_nearest: bool,
) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    let f = f32::from_bits(bits);
    if f.is_nan() {
        flags.ioc = true;
        return (0, flags);
    }
    let fv = f as f64;
    let rounded = if round_nearest {
        f.round_ties_even() as f64
    } else {
        f.trunc() as f64
    };
    if signed {
        if fv > 2147483647.0 || (f.is_infinite() && f > 0.0) {
            flags.qc = true;
            return (0x7FFF_FFFF, flags);
        }
        if fv < -2147483648.0 || (f.is_infinite() && f < 0.0) {
            flags.qc = true;
            return (0x8000_0000, flags);
        }
        if rounded != fv {
            flags.ixc = true;
        }
        (rounded as i64 as u32, flags)
    } else {
        if fv > 4294967295.0 || (f.is_infinite() && f > 0.0) {
            flags.qc = true;
            return (0xFFFF_FFFF, flags);
        }
        if fv < 0.0 {
            flags.qc = true;
            return (0, flags);
        }
        if rounded != fv {
            flags.ixc = true;
        }
        (rounded as u64 as u32, flags)
    }
}

/// f64 → 有符号/无符号 32 位整数
pub fn cvt_f64_to_int(
    _fpu: &FpuRegisters,
    bits: u64,
    signed: bool,
    round_nearest: bool,
) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    let f = f64::from_bits(bits);
    if f.is_nan() {
        flags.ioc = true;
        return (0, flags);
    }
    let fv = f;
    let rounded = if round_nearest {
        f.round_ties_even()
    } else {
        f.trunc()
    };
    if signed {
        if fv > 2147483647.0 || (f.is_infinite() && f > 0.0) {
            flags.qc = true;
            return (0x7FFF_FFFF, flags);
        }
        if fv < -2147483648.0 || (f.is_infinite() && f < 0.0) {
            flags.qc = true;
            return (0x8000_0000, flags);
        }
        if rounded != fv {
            flags.ixc = true;
        }
        (rounded as i64 as u32, flags)
    } else {
        if fv > 4294967295.0 || (f.is_infinite() && f > 0.0) {
            flags.qc = true;
            return (0xFFFF_FFFF, flags);
        }
        if fv < 0.0 {
            flags.qc = true;
            return (0, flags);
        }
        if rounded != fv {
            flags.ixc = true;
        }
        (rounded as u64 as u32, flags)
    }
}

/// f64 → f32（按 FPSCR 舍入模式；NaN 传播无异常）
pub fn cvt_f64_to_f32(fpu: &FpuRegisters, bits: u64) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    let f = f64::from_bits(bits);
    if f.is_nan() {
        // NaN 传播（安静化），无异常标志
        let q = quiet_nan_f64(bits);
        return ((q >> 32) as u32 | 0x0040_0000, flags);
    }
    if is_denormal_f64(bits) {
        flags.idc = true;
    }
    let wide = f;
    let res = round_f64_per_mode_f32(fpu.rounding_mode(), wide);
    if res.is_infinite() && !f.is_infinite() {
        flags.ofc = true;
        flags.ixc = true;
    }
    if is_denormal_f32(res.to_bits()) {
        flags.ufc = true;
        flags.ixc = true;
    }
    if (res as f64) != wide {
        flags.ixc = true;
    }
    (res.to_bits(), flags)
}

// ==================== VCVT 定点转换（P4-补） ====================

/// f32 → 定点（VCVT.S16.F32 等）：按 FPSCR 舍入模式取整(Sm × 2^fbits)，
/// 饱和到目标宽度（超出范围 → FPSCR.QC）。NaN → 0 + IOC；舍入发生 → IXC。
/// 注意：×2^fbits 为 2 的幂缩放，f64 中间量精确。
pub fn cvt_f32_to_fixed(
    fpu: &FpuRegisters,
    bits: u32,
    fbits: u8,
    signed: bool,
    width: u8,
) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    let f = f32::from_bits(bits);
    if f.is_nan() {
        flags.ioc = true;
        return (0, flags);
    }
    let scaled = (f as f64) * ((1u64 << fbits) as f64);
    let rounded = match fpu.rounding_mode() {
        FpRounding::Nearest => scaled.round_ties_even(),
        FpRounding::Zero => scaled.trunc(),
        FpRounding::PlusInf => scaled.ceil(),
        FpRounding::MinusInf => scaled.floor(),
    };
    let (min, max) = if signed {
        (-(1i64 << (width - 1)), (1i64 << (width - 1)) - 1)
    } else {
        (0, (1i64 << width) - 1)
    };
    let (val, saturated) = if f.is_infinite() || rounded > max as f64 {
        (max, true)
    } else if rounded < min as f64 {
        (min, true)
    } else {
        (rounded as i64, false)
    };
    if saturated {
        flags.qc = true;
    }
    if rounded != scaled {
        flags.ixc = true;
    }
    (val as u32, flags)
}

/// 定点 → f32（VCVT.F32.S16 等）：符号/零扩展值 ÷ 2^fbits（2 的幂，f64 精确），
/// 按 FPSCR 舍入模式取 f32；不精确 → IXC。
pub fn cvt_fixed_to_f32(
    fpu: &FpuRegisters,
    bits: u32,
    fbits: u8,
    signed: bool,
    width: u8,
) -> (u32, FpOpFlags) {
    let mut flags = FpOpFlags::default();
    let raw = bits & if width == 16 { 0xFFFF } else { 0xFFFF_FFFF };
    let val = if signed {
        if width == 16 {
            (raw as u16 as i16) as f64
        } else {
            (raw as i32) as f64
        }
    } else {
        raw as f64
    };
    let exact = val / ((1u64 << fbits) as f64);
    let r = round_f64_per_mode_f32(fpu.rounding_mode(), exact);
    if (r as f64) != exact {
        flags.ixc = true;
    }
    (r.to_bits(), flags)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// VFP 立即数展开 golden（与汇编器输出一致）
    #[test]
    fn vfp_expand_imm_golden() {
        // 1.0 / 0.5 / 2.0 / 3.0 / 1.5 / 0.75 / -1.0 / -2.5
        assert_eq!(vfp_expand_imm(0x70, false), 0x3F80_0000);
        assert_eq!(vfp_expand_imm(0x60, false), 0x3F00_0000);
        assert_eq!(vfp_expand_imm(0x00, false), 0x4000_0000);
        assert_eq!(vfp_expand_imm(0x08, false), 0x4040_0000);
        assert_eq!(vfp_expand_imm(0x78, false), 0x3FC0_0000);
        assert_eq!(vfp_expand_imm(0x68, false), 0x3F40_0000);
        assert_eq!(vfp_expand_imm(0xF0, false), 0xBF80_0000);
        assert_eq!(vfp_expand_imm(0x84, false), 0xC020_0000);
        // 双精度 1.0 / -2.5
        assert_eq!(vfp_expand_imm(0x70, true), 0x3FF0_0000_0000_0000);
        assert_eq!(vfp_expand_imm(0x84, true), 0xC004_0000_0000_0000);
    }

    #[test]
    fn fpu_register_file_alias() {
        let mut fpu = FpuRegisters::new();
        fpu.write_d(3, 0x0123_4567_89AB_CDEF);
        assert_eq!(fpu.read_s(6), 0x89AB_CDEF); // D3 = S6:S7
        assert_eq!(fpu.read_s(7), 0x0123_4567);
        assert_eq!(fpu.read_d(3), 0x0123_4567_89AB_CDEF);
        fpu.write_s(6, 0xFFFF_FFFF);
        assert_eq!(fpu.read_d(3) & 0xFFFF_FFFF, 0xFFFF_FFFF);
    }

    #[test]
    fn f32_add_flags() {
        let fpu = FpuRegisters::new();
        // 1.0 + 2.0 = 3.0（精确，无标志）
        let (res, flags) = f32_add(&fpu, 0x3F80_0000, 0x4000_0000);
        assert_eq!(res, 0x4040_0000);
        assert!(!flags.ioc && !flags.ixc && !flags.ofc && !flags.ufc && !flags.dzc);
        // NaN 输入 → IOC
        let (res, flags) = f32_add(&fpu, 0x7FC0_0000, 0x3F80_0000);
        assert!(is_nan_f32(res));
        assert!(flags.ioc);
        // 溢出：MAX + MAX → Inf + OFC + IXC
        let (res, flags) = f32_add(&fpu, 0x7F7F_FFFF, 0x7F7F_FFFF);
        assert!(res == f32::INFINITY.to_bits());
        assert!(flags.ofc && flags.ixc);
    }

    #[test]
    fn f32_div_zero() {
        let fpu = FpuRegisters::new();
        let (res, flags) = f32_div(&fpu, 0x3F80_0000, 0x0000_0000);
        assert_eq!(res, f32::INFINITY.to_bits());
        assert!(flags.dzc);
    }

    #[test]
    fn f64_exactness_helpers() {
        // 1.0 + 2.0 = 3.0 精确
        assert!(f64_add_exact(1.0, 2.0));
        // 1.0 + 2^-53 不精确
        assert!(!f64_add_exact(1.0, 2f64.powi(-53)));
        // 1.5 × 2.0 = 3.0 精确
        assert!(f64_mul_exact(1.5, 2.0));
        // 0.1 × 0.2 不精确
        assert!(!f64_mul_exact(0.1, 0.2));
        // 1.0 / 4.0 精确
        assert!(f64_div_exact(1.0, 4.0));
        // 1.0 / 3.0 不精确
        assert!(!f64_div_exact(1.0, 3.0));
    }

    // ============ P5-补：f64 定向舍入 + VSQRT IXC + CPACR ============

    /// f64 定向舍入：RP/RM/RZ（1.0 + 0.1，精确结果略小于 RN 值）
    #[test]
    fn f64_directed_rounding_add() {
        let mut fpu = FpuRegisters::new();
        // RN：1.0 + 0.1 → 0x3FF1_9999_9999_999A（1.1000000000000000888，精确 sig 分数 .625 向上取整）
        let rn = f64_add(&fpu, 1.0f64.to_bits(), 0.1f64.to_bits()).0;
        assert_eq!(rn, 0x3FF1_9999_9999_999A);
        // 精确 < RN（rel = Less）
        assert_eq!(
            f64_add_relation(1.0, 0.1, f64::from_bits(rn)),
            Ordering::Less
        );
        // RP（向 +∞）：精确 < RN → 保持 RN
        fpu.fpscr = 1 << 22;
        let (rp, flags) = f64_add(&fpu, 1.0f64.to_bits(), 0.1f64.to_bits());
        assert_eq!(rp, 0x3FF1_9999_9999_999A);
        assert!(flags.ixc);
        // RM（向 −∞）：精确 < RN → next_down
        fpu.fpscr = 2 << 22;
        let (rm, _) = f64_add(&fpu, 1.0f64.to_bits(), 0.1f64.to_bits());
        assert_eq!(rm, 0x3FF1_9999_9999_9999);
        // RZ（向零）：正数同 RM
        fpu.fpscr = 3 << 22;
        let (rz, _) = f64_add(&fpu, 1.0f64.to_bits(), 0.1f64.to_bits());
        assert_eq!(rz, 0x3FF1_9999_9999_9999);
    }

    /// f64 定向舍入：乘法（0.1 × 0.2，不精确）与除法（1/3）
    #[test]
    fn f64_directed_rounding_mul_div() {
        let mut fpu = FpuRegisters::new();
        let rn = f64_mul(&fpu, 0.1f64.to_bits(), 0.2f64.to_bits()).0;
        let rel = f64_mul_relation(0.1, 0.2, f64::from_bits(rn));
        // RP：rel = Greater → next_up（位模式 +1）；否则保持 RN
        fpu.fpscr = 1 << 22;
        let rp = f64_mul(&fpu, 0.1f64.to_bits(), 0.2f64.to_bits()).0;
        let expected_rp = if rel == Ordering::Greater {
            rn + 1
        } else {
            rn
        };
        assert_eq!(rp, expected_rp);
        // RZ：正数 → 结果 ≤ RN
        fpu.fpscr = 3 << 22;
        let rz = f64_mul(&fpu, 0.1f64.to_bits(), 0.2f64.to_bits()).0;
        assert!(rz <= rn);
        // 除法 1/3：RN = 0x3FD5_5555_5555_5555，精确 1/3 > RN → RP = +1
        let rn3 = f64_div(&fpu, 1.0f64.to_bits(), 3.0f64.to_bits()).0;
        assert_eq!(rn3, 0x3FD5_5555_5555_5555);
        fpu.fpscr = 1 << 22;
        let rp3 = f64_div(&fpu, 1.0f64.to_bits(), 3.0f64.to_bits()).0;
        assert_eq!(rp3, 0x3FD5_5555_5555_5556);
        fpu.fpscr = 2 << 22;
        let rm3 = f64_div(&fpu, 1.0f64.to_bits(), 3.0f64.to_bits()).0;
        assert_eq!(rm3, 0x3FD5_5555_5555_5555);
    }

    /// 精确结果路径：可精确表示的加法在任意舍入模式下不变（rel = Equal）
    #[test]
    fn f64_directed_rounding_exact_case() {
        let mut fpu = FpuRegisters::new();
        // 1.5 + 2.25 = 3.75 精确
        for mode in [0u32, 1, 2, 3] {
            fpu.fpscr = mode << 22;
            let (r, flags) = f64_add(&fpu, 1.5f64.to_bits(), 2.25f64.to_bits());
            assert_eq!(r, 3.75f64.to_bits());
            assert!(!flags.ixc);
        }
        // 1.0 / 4.0 精确（除法余数为 0 → rel = Equal）
        fpu.fpscr = 1 << 22;
        let (r, flags) = f64_div(&fpu, 1.0f64.to_bits(), 4.0f64.to_bits());
        assert_eq!(r, 0.25f64.to_bits());
        assert!(!flags.ixc);
    }

    /// f64 加/减/乘的精确关系判定（与 RN 结果的方向）
    #[test]
    fn f64_relation_helpers() {
        // 1.0 + 0.1：精确 < RN（sig 分数 .625 向上取整）
        let rn = (1.0f64 + 0.1f64).to_bits();
        assert_eq!(
            f64_add_relation(1.0, 0.1, f64::from_bits(rn)),
            Ordering::Less
        );
        // 1.0 + 2.0 = 3.0 精确
        assert_eq!(f64_add_relation(1.0, 2.0, 3.0), Ordering::Equal);
        // 0.1 × 0.2 不精确
        assert_ne!(f64_mul_relation(0.1, 0.2, 0.1 * 0.2), Ordering::Equal);
        // 减法：1.0 - 0.1 不精确（精确值 0x...6666 舍入到 0x...6668 = r，精确 < r）
        assert_eq!(f64_add_relation(1.0, -0.1, 1.0 - 0.1), Ordering::Less);
        // 精确减法：1.5 - 0.25 = 1.25 可表示
        assert_eq!(f64_add_relation(1.5, -0.25, 1.25), Ordering::Equal);
    }

    #[test]
    fn f64_add_flags() {
        let fpu = FpuRegisters::new();
        let (res, flags) = f64_add(&fpu, 0x3FF0_0000_0000_0000, 0x4000_0000_0000_0000);
        assert_eq!(res, 0x4008_0000_0000_0000); // 1.0 + 2.0 = 3.0
        assert!(!flags.ioc && !flags.ixc && !flags.ofc && !flags.ufc && !flags.dzc);
        // 1.0 + 2^-53 → 不精确
        let (_, flags) = f64_add(&fpu, 1.0f64.to_bits(), (2f64.powi(-53)).to_bits());
        assert!(flags.ixc);
    }

    #[test]
    fn rounding_mode_helpers() {
        let mut fpu = FpuRegisters::new();
        assert_eq!(fpu.rounding_mode(), FpRounding::Nearest);
        fpu.fpscr |= 3 << 22;
        assert_eq!(fpu.rounding_mode(), FpRounding::Zero);
    }

    // ==================== 指令级 golden 测试（GIVEN/WHEN/THEN） ====================
    // 编码均由 arm-none-eabi-as -mcpu=cortex-m4/-m7 汇编验证

    use crate::engine::exec::ExecOutcome;
    use crate::engine::test_util::Harness;

    fn f32(v: f32) -> u32 {
        v.to_bits()
    }

    #[test]
    fn golden_vmov_reg() {
        // GIVEN: S1 = 1.5
        let mut h = Harness::new();
        h.cpu.fpu.write_s(1, f32(1.5));
        // WHEN: VMOV.F32 S0, S1（0xEEB0 0A60）
        assert_eq!(h.exec_word(0xEEB0_0A60), ExecOutcome::Continue);
        // THEN: S0 = 1.5
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.5));
        // 高寄存器：VMOV.F32 S16, S17（0xEEB0 8A68）
        h.cpu.fpu.write_s(17, f32(-2.25));
        assert_eq!(h.exec_word(0xEEB0_8A68), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(16), f32(-2.25));
    }

    #[test]
    fn golden_vmov_core_roundtrip() {
        // GIVEN: R1 = 0x3F80_0000（1.0 的位模式）
        let mut h = Harness::new();
        h.cpu.regs[1] = f32(1.0);
        // WHEN: VMOV S0, R1（0xEE00 1A10）
        assert_eq!(h.exec_word(0xEE00_1A10), ExecOutcome::Continue);
        // THEN: S0 = 1.0
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.0));
        // WHEN: VMOV R2, S0（0xEE10 2A10）
        assert_eq!(h.exec_word(0xEE10_2A10), ExecOutcome::Continue);
        // THEN: R2 = 1.0 位模式
        assert_eq!(h.cpu.regs[2], f32(1.0));
        // 高寄存器：VMOV R3, S16（0xEE18 3A10）
        h.cpu.fpu.write_s(16, f32(3.5));
        assert_eq!(h.exec_word(0xEE18_3A10), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[3], f32(3.5));
    }

    #[test]
    fn golden_vmov_imm() {
        // GIVEN: 空
        let mut h = Harness::new();
        // WHEN: VMOV.F32 S0, #1.0（0xEEB7 0A00）
        assert_eq!(h.exec_word(0xEEB7_0A00), ExecOutcome::Continue);
        // THEN: S0 = 1.0
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.0));
        // WHEN: VMOV.F32 S1, #-2.5（0xEEF8 0A04：imm8=0x84，Vd=S1）
        assert_eq!(h.exec_word(0xEEF8_0A04), ExecOutcome::Continue);
        // THEN: S1 = -2.5
        assert_eq!(h.cpu.fpu.read_s(1), f32(-2.5));
        // 高寄存器：VMOV.F32 S16, #1.0（0xEEB7 8A00）
        assert_eq!(h.exec_word(0xEEB7_8A00), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(16), f32(1.0));
    }

    #[test]
    fn golden_vadd_f32() {
        // GIVEN: S1 = 1.0，S2 = 2.0
        let mut h = Harness::new();
        h.cpu.fpu.write_s(1, f32(1.0));
        h.cpu.fpu.write_s(2, f32(2.0));
        // WHEN: VADD.F32 S0, S1, S2（0xEE30 0A81）
        assert_eq!(h.exec_word(0xEE30_0A81), ExecOutcome::Continue);
        // THEN: S0 = 3.0，无异常标志
        assert_eq!(h.cpu.fpu.read_s(0), f32(3.0));
        assert_eq!(h.cpu.fpu.fpscr & 0xFF, 0);
    }

    #[test]
    fn golden_vsub_vmul_vdiv_f32() {
        let mut h = Harness::new();
        h.cpu.fpu.write_s(1, f32(1.5));
        h.cpu.fpu.write_s(2, f32(2.0));
        // WHEN: VSUB.F32 S0, S1, S2（0xEE30 0AC1）→ 1.5 − 2.0 = −0.5
        assert_eq!(h.exec_word(0xEE30_0AC1), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(-0.5));
        // WHEN: VMUL.F32 S0, S1, S2（0xEE20 0A81）→ 1.5 × 2.0 = 3.0
        assert_eq!(h.exec_word(0xEE20_0A81), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(3.0));
        // WHEN: VDIV.F32 S0, S1, S2（0xEE80 0A81）→ 1.5 / 2.0 = 0.75
        assert_eq!(h.exec_word(0xEE80_0A81), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(0.75));
    }

    #[test]
    fn golden_vdiv_by_zero_sets_dzc() {
        // GIVEN: S1 = 1.0，S2 = 0.0
        let mut h = Harness::new();
        h.cpu.fpu.write_s(1, f32(1.0));
        h.cpu.fpu.write_s(2, f32(0.0));
        // WHEN: VDIV.F32 S0, S1, S2
        assert_eq!(h.exec_word(0xEE80_0A81), ExecOutcome::Continue);
        // THEN: S0 = +Inf，FPSCR.DZC（bit1）置位
        assert_eq!(h.cpu.fpu.read_s(0), f32::INFINITY.to_bits());
        assert_ne!(h.cpu.fpu.fpscr & (1 << 1), 0);
    }

    #[test]
    fn golden_vmla_fused() {
        // GIVEN: S0 = 10.0，S1 = 1.5，S2 = 2.0
        let mut h = Harness::new();
        h.cpu.fpu.write_s(0, f32(10.0));
        h.cpu.fpu.write_s(1, f32(1.5));
        h.cpu.fpu.write_s(2, f32(2.0));
        // WHEN: VMLA.F32 S0, S1, S2（0xEE00 0A81）→ 10 + 1.5×2 = 13
        assert_eq!(h.exec_word(0xEE00_0A81), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(13.0));
        // WHEN: VMLS.F32 S0, S1, S2（0xEE00 0AC1）→ 13 − 3 = 10
        assert_eq!(h.exec_word(0xEE00_0AC1), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(10.0));
    }

    #[test]
    fn golden_vnmla_vnmls_vnmul() {
        let mut h = Harness::new();
        h.cpu.fpu.write_s(0, f32(4.0));
        h.cpu.fpu.write_s(1, f32(1.5));
        h.cpu.fpu.write_s(2, f32(2.0));
        // WHEN: VNMLA.F32 S0, S1, S2（0xEE10 0AC1）→ −(4 + 3) = −7
        assert_eq!(h.exec_word(0xEE10_0AC1), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(-7.0));
        // WHEN: VNMLS.F32 S0, S1, S2（0xEE10 0A81）→ −(S0 − 3) = −(−7 − 3) = 10
        assert_eq!(h.exec_word(0xEE10_0A81), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(10.0));
        // WHEN: VNMUL.F32 S0, S1, S2（0xEE20 0AC1）→ −(1.5 × 2) = −3
        assert_eq!(h.exec_word(0xEE20_0AC1), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(-3.0));
    }

    #[test]
    fn golden_vabs_vneg_vsqrt() {
        let mut h = Harness::new();
        h.cpu.fpu.write_s(1, f32(-4.0));
        // WHEN: VABS.F32 S0, S1（0xEEB0 0AE0）
        assert_eq!(h.exec_word(0xEEB0_0AE0), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(4.0));
        // WHEN: VNEG.F32 S0, S1（0xEEB1 0A60）
        assert_eq!(h.exec_word(0xEEB1_0A60), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(4.0)); // −(−4) = 4
                                                   // WHEN: VSQRT.F32 S0, S1（0xEEB1 0AE0，S1 = +4.0）→ sqrt(4) = 2
        h.cpu.fpu.write_s(1, f32(4.0));
        assert_eq!(h.exec_word(0xEEB1_0AE0), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(2.0));
        // 负数开方 → 默认 NaN + IOC（bit0）
        h.cpu.fpu.write_s(1, f32(-1.0));
        assert_eq!(h.exec_word(0xEEB1_0AE0), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), DEFAULT_NAN_F32);
        assert_ne!(h.cpu.fpu.fpscr & 1, 0);
    }

    #[test]
    fn golden_vcmp_flags() {
        let mut h = Harness::new();
        // GIVEN: S0 = 1.0，S1 = 2.0
        h.cpu.fpu.write_s(0, f32(1.0));
        h.cpu.fpu.write_s(1, f32(2.0));
        // WHEN: VCMP.F32 S0, S1（0xEEB4 0A60）
        assert_eq!(h.exec_word(0xEEB4_0A60), ExecOutcome::Continue);
        // THEN: S0 < S1 → N=1，Z=C=V=0
        assert_eq!(h.cpu.fpu.fpscr & (0xF << 28), 1 << 31);
        // WHEN: VCMP.F32 S0, S0 → 相等 → Z=1, C=1
        assert_eq!(h.exec_word(0xEEB4_0A60), ExecOutcome::Continue); // S1 仍为 2.0？改 S0=S1 重比
        h.cpu.fpu.write_s(1, f32(1.0));
        assert_eq!(h.exec_word(0xEEB4_0A60), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.fpscr & (0xF << 28), (0b0110) << 28); // Z=1, C=1
                                                                   // WHEN: VCMP.F32 S0, #0.0（0xEEB5 0A40）→ 1.0 > 0.0 → C=1
        assert_eq!(h.exec_word(0xEEB5_0A40), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.fpscr & (0xF << 28), (0b0010) << 28); // C=1
    }

    #[test]
    fn golden_vcmpe_nan_sets_ioc() {
        // GIVEN: S0 = NaN，S1 = 1.0
        let mut h = Harness::new();
        h.cpu.fpu.write_s(0, 0x7FC0_0000);
        h.cpu.fpu.write_s(1, f32(1.0));
        // WHEN: VCMPE.F32 S0, S1（0xEEB4 0AE0）
        assert_eq!(h.exec_word(0xEEB4_0AE0), ExecOutcome::Continue);
        // THEN: 无序 → C=1, V=1；IOC（bit0）置位
        assert_eq!(h.cpu.fpu.fpscr & (0xF << 28), (0b0011) << 28);
        assert_ne!(h.cpu.fpu.fpscr & 1, 0);
    }

    #[test]
    fn golden_vcvt_s32_f32() {
        let mut h = Harness::new();
        // GIVEN: S1 = 1.9
        h.cpu.fpu.write_s(1, f32(1.9));
        // WHEN: VCVT.S32.F32 S0, S1（0xEEBD 0AE0，朝零舍入）
        assert_eq!(h.exec_word(0xEEBD_0AE0), ExecOutcome::Continue);
        // THEN: S0 = 1（位模式），IXC 置位（舍入发生）
        assert_eq!(h.cpu.fpu.read_s(0), 1);
        assert_ne!(h.cpu.fpu.fpscr & (1 << 4), 0);
        // WHEN: VCVTR.S32.F32 S0, S1（0xEEBD 0A60，就近舍入）→ 1.9 → 2
        assert_eq!(h.exec_word(0xEEBD_0A60), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 2);
        // WHEN: VCVT.F32.S32 S0, S1（0xEEB8 0AE0：S1 持有整数 5）
        h.cpu.fpu.write_s(1, 5);
        assert_eq!(h.exec_word(0xEEB8_0AE0), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(5.0));
    }

    #[test]
    fn golden_vcvt_u32_saturation() {
        // GIVEN: S1 = -1.0
        let mut h = Harness::new();
        h.cpu.fpu.write_s(1, f32(-1.0));
        // WHEN: VCVT.U32.F32 S0, S1（0xEEBC 0AE0）
        assert_eq!(h.exec_word(0xEEBC_0AE0), ExecOutcome::Continue);
        // THEN: S0 = 0，QC（bit27）置位
        assert_eq!(h.cpu.fpu.read_s(0), 0);
        assert!(h.cpu.fpu.qc());
        // 大正数 → 饱和 0xFFFF_FFFF
        h.cpu.fpu.write_s(1, f32(5e9));
        assert_eq!(h.exec_word(0xEEBC_0AE0), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 0xFFFF_FFFF);
    }

    #[test]
    fn golden_vcvt_nan_sets_ioc() {
        // GIVEN: S1 = NaN
        let mut h = Harness::new();
        h.cpu.fpu.write_s(1, 0x7FC0_0000);
        // WHEN: VCVT.S32.F32 S0, S1
        assert_eq!(h.exec_word(0xEEBD_0AE0), ExecOutcome::Continue);
        // THEN: S0 = 0，IOC 置位
        assert_eq!(h.cpu.fpu.read_s(0), 0);
        assert_ne!(h.cpu.fpu.fpscr & 1, 0);
    }

    #[test]
    fn golden_vcvt_f32_f64() {
        let mut h = Harness::new();
        // GIVEN: D1 = 1.5
        h.cpu.fpu.write_d(1, 1.5f64.to_bits());
        // WHEN: VCVT.F32.F64 S0, D1（0xEEB7 0BC1）
        assert_eq!(h.exec_word(0xEEB7_0BC1), ExecOutcome::Continue);
        // THEN: S0 = 1.5
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.5));
        // WHEN: VCVT.F64.F32 D2, S3（0xEEB7 2AE1）
        h.cpu.fpu.write_s(3, f32(2.25));
        assert_eq!(h.exec_word(0xEEB7_2AE1), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(2), 2.25f64.to_bits());
    }

    #[test]
    fn golden_f64_arith() {
        let mut h = Harness::new();
        // GIVEN: D1 = 1.5，D2 = 2.0
        h.cpu.fpu.write_d(1, 1.5f64.to_bits());
        h.cpu.fpu.write_d(2, 2.0f64.to_bits());
        // WHEN: VADD.F64 D0, D1, D2（0xEE31 0B02）→ 3.5
        assert_eq!(h.exec_word(0xEE31_0B02), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(0), 3.5f64.to_bits());
        // WHEN: VMUL.F64 D0, D1, D2（0xEE21 0B02）→ 3.0
        assert_eq!(h.exec_word(0xEE21_0B02), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(0), 3.0f64.to_bits());
        // WHEN: VSUB.F64 D0, D1, D2（0xEE31 0B42）→ −0.5
        assert_eq!(h.exec_word(0xEE31_0B42), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(0), (-0.5f64).to_bits());
        // WHEN: VDIV.F64 D0, D1, D2（0xEE81 0B02）→ 0.75
        assert_eq!(h.exec_word(0xEE81_0B02), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(0), 0.75f64.to_bits());
    }

    #[test]
    fn golden_f64_mla_vmov_cmp() {
        let mut h = Harness::new();
        h.cpu.fpu.write_d(0, 10.0f64.to_bits());
        h.cpu.fpu.write_d(1, 1.5f64.to_bits());
        h.cpu.fpu.write_d(2, 2.0f64.to_bits());
        // WHEN: VMLA.F64 D0, D1, D2（0xEE01 0B02）→ 10 + 3 = 13
        assert_eq!(h.exec_word(0xEE01_0B02), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(0), 13.0f64.to_bits());
        // WHEN: VMOV.F64 D3, D0（0xEEB0 3B40）
        assert_eq!(h.exec_word(0xEEB0_3B40), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(3), 13.0f64.to_bits());
        // WHEN: VCMP.F64 D0, D2（0xEEB4 0B41）→ 13 > 2 → C=1
        assert_eq!(h.exec_word(0xEEB4_0B41), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.fpscr & (0xF << 28), (0b0010) << 28);
    }

    #[test]
    fn golden_vldr_vstr_roundtrip() {
        // GIVEN: R1 = 0x2000_0000（SRAM），S1 = 1.25
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x2000_0000;
        h.cpu.fpu.write_s(1, f32(1.25));
        // WHEN: VSTR S1, [R1, #4]（0xEDC1 0A01：S1 → bit22=1）
        assert_eq!(h.exec_word(0xEDC1_0A01), ExecOutcome::Continue);
        // THEN: 内存 [0x2000_0004] = 1.25 位模式
        assert_eq!(h.mem.read_u32(0x2000_0004).unwrap(), f32(1.25));
        // WHEN: VLDR S0, [R1, #4]（0xED91 0A01）
        assert_eq!(h.exec_word(0xED91_0A01), ExecOutcome::Continue);
        // THEN: S0 = 1.25
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.25));
        // 负偏移：VLDR S16, [R2, #-8]（0xED12 8A02）
        h.cpu.regs[2] = 0x2000_000C;
        h.mem.write_u32(0x2000_0004, f32(-3.5)).unwrap();
        assert_eq!(h.exec_word(0xED12_8A02), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(16), f32(-3.5));
    }

    #[test]
    fn golden_vldr_double() {
        // GIVEN: R1 = 0x2000_0000
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x2000_0000;
        h.cpu.fpu.write_d(0, 2.5f64.to_bits());
        // WHEN: VSTR D0, [R1, #8]（0xED81 0B02）
        assert_eq!(h.exec_word(0xED81_0B02), ExecOutcome::Continue);
        // THEN: 内存低字 = D0 低 32 位，高字 = 高 32 位
        assert_eq!(
            h.mem.read_u32(0x2000_0008).unwrap(),
            2.5f64.to_bits() as u32
        );
        assert_eq!(
            h.mem.read_u32(0x2000_000C).unwrap(),
            (2.5f64.to_bits() >> 32) as u32
        );
        // WHEN: VLDR D2, [R1, #8]（0xED91 2B02）
        assert_eq!(h.exec_word(0xED91_2B02), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(2), 2.5f64.to_bits());
    }

    #[test]
    fn golden_vmov_core_double() {
        // GIVEN: R1 = 0x89AB_CDEF，R2 = 0x0123_4567
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x89AB_CDEF;
        h.cpu.regs[2] = 0x0123_4567;
        // WHEN: VMOV D0, R1, R2（0xEC42 1B10）
        assert_eq!(h.exec_word(0xEC42_1B10), ExecOutcome::Continue);
        // THEN: D0 = 0x0123_4567_89AB_CDEF
        assert_eq!(h.cpu.fpu.read_d(0), 0x0123_4567_89AB_CDEF);
        // WHEN: VMOV R3, R4, D0（0xEC54 3B10：Rt2=R4）
        assert_eq!(h.exec_word(0xEC54_3B10), ExecOutcome::Continue);
        // THEN: R3 = 低 32 位，R4 = 高 32 位
        assert_eq!(h.cpu.regs[3], 0x89AB_CDEF);
        assert_eq!(h.cpu.regs[4], 0x0123_4567);
    }

    #[test]
    fn golden_vldr_unaligned_faults() {
        // GIVEN: R1 = 0x2000_0002（非 4 字节对齐）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x2000_0002;
        // WHEN: VLDR S0, [R1]（0xED91 0A00）
        let outcome = h.exec_word(0xED91_0A00);
        // THEN: UnalignedAccess 故障，S0 不变
        assert_eq!(
            outcome,
            ExecOutcome::Fault {
                reason: crate::engine::FaultReason::UnalignedAccess {
                    address: 0x2000_0002
                }
            }
        );
    }

    // ============ P4-补：VLDM/VSTM + VCVT 定点 golden（编码经 arm-none-eabi-as 实测） ============

    /// VSTMIA/VLDMIA 单精度多寄存器往返
    #[test]
    fn golden_vstm_vldm_roundtrip() {
        // GIVEN: R0 = 0x2000_0000，S0-S3 = 1.0/2.0/3.0/4.0
        let mut h = Harness::new();
        h.cpu.regs[0] = 0x2000_0000;
        for (i, v) in [1.0f32, 2.0, 3.0, 4.0].iter().enumerate() {
            h.cpu.fpu.write_s(i, v.to_bits());
        }
        // WHEN: VSTMIA r0, {s0-s3}（0xEC80 0A04）
        assert_eq!(h.exec_word(0xEC80_0A04), ExecOutcome::Continue);
        // THEN: 内存依次为 1.0/2.0/3.0/4.0
        for (i, v) in [1.0f32, 2.0, 3.0, 4.0].iter().enumerate() {
            assert_eq!(h.mem.read_u32(0x2000_0000 + i as u32 * 4).unwrap(), v.to_bits());
        }
        // WHEN: VLDMIA r0, {s0-s3}（0xEC90 0A04）
        h.cpu.fpu.write_s(0, 0);
        h.cpu.fpu.write_s(1, 0);
        h.cpu.fpu.write_s(2, 0);
        h.cpu.fpu.write_s(3, 0);
        h.cpu.regs[0] = 0x2000_0000;
        assert_eq!(h.exec_word(0xEC90_0A04), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.0));
        assert_eq!(h.cpu.fpu.read_s(1), f32(2.0));
        assert_eq!(h.cpu.fpu.read_s(2), f32(3.0));
        assert_eq!(h.cpu.fpu.read_s(3), f32(4.0));
    }

    /// VLDMIA 回写 + VSTMDB 先减后存
    #[test]
    fn golden_vldm_writeback_vstmdb() {
        let mut h = Harness::new();
        // VSTMDB r0!, {s0-s3}：R0 = 0x2000_0010 → 数据写入 0x2000_0000..0x2000_000C，R0 回写 0x2000_0000
        h.cpu.regs[0] = 0x2000_0010;
        for (i, v) in [1.0f32, 2.0, 3.0, 4.0].iter().enumerate() {
            h.cpu.fpu.write_s(i, v.to_bits());
        }
        assert_eq!(h.exec_word(0xED20_0A04), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x2000_0000);
        assert_eq!(h.mem.read_u32(0x2000_0000).unwrap(), f32(1.0));
        assert_eq!(h.mem.read_u32(0x2000_000C).unwrap(), f32(4.0));
        // VLDMIA r0!, {s0-s3}：R0 = 0x2000_0000 → 加载后回写 0x2000_0010
        h.cpu.fpu.write_s(0, 0);
        h.cpu.regs[0] = 0x2000_0000;
        assert_eq!(h.exec_word(0xECB0_0A04), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0x2000_0010);
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.0));
        assert_eq!(h.cpu.fpu.read_s(3), f32(4.0));
    }

    /// VLDM/VSTM 双精度 + 高 S 寄存器（S16-S19，D 位）
    #[test]
    fn golden_vldm_double_and_s16() {
        let mut h = Harness::new();
        // 双精度：VSTMIA r0, {d0-d1}（0xEC80 0B04）
        h.cpu.regs[0] = 0x2000_0000;
        h.cpu.fpu.write_d(0, 2.5f64.to_bits());
        h.cpu.fpu.write_d(1, (-1.25f64).to_bits());
        assert_eq!(h.exec_word(0xEC80_0B04), ExecOutcome::Continue);
        assert_eq!(h.mem.read_u32(0x2000_0000).unwrap(), 2.5f64.to_bits() as u32);
        assert_eq!(h.mem.read_u32(0x2000_0008).unwrap(), (-1.25f64).to_bits() as u32);
        // VLDMIA r0, {d0-d1}（0xEC90 0B04）
        h.cpu.fpu.write_d(0, 0);
        h.cpu.fpu.write_d(1, 0);
        assert_eq!(h.exec_word(0xEC90_0B04), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_d(0), 2.5f64.to_bits());
        assert_eq!(h.cpu.fpu.read_d(1), (-1.25f64).to_bits());
        // S16-S19：VLDMIA r1!, {s16-s19}（0xECB1 8A04）
        for (i, v) in [1.5f32, -2.5, 3.5, -4.5].iter().enumerate() {
            h.mem.write_u32(0x2000_0000 + i as u32 * 4, v.to_bits()).unwrap();
        }
        h.cpu.regs[1] = 0x2000_0000;
        assert_eq!(h.exec_word(0xECB1_8A04), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(16), f32(1.5));
        assert_eq!(h.cpu.fpu.read_s(17), f32(-2.5));
        assert_eq!(h.cpu.fpu.read_s(18), f32(3.5));
        assert_eq!(h.cpu.fpu.read_s(19), f32(-4.5));
        assert_eq!(h.cpu.regs[1], 0x2000_0010);
    }

    /// VCVT.S16.F32：定点换算 + 饱和 → QC
    #[test]
    fn golden_vcvt_s16_f32_fixed() {
        let mut h = Harness::new();
        // GIVEN: S0 = 1.5（Q15 定点 fbits=8 → 1.5×256 = 384）
        h.cpu.fpu.write_s(0, f32(1.5));
        // WHEN: VCVT.S16.F32 S0, S0, #8（0xEEBE 0A44）
        assert_eq!(h.exec_word(0xEEBE_0A44), ExecOutcome::Continue);
        // THEN: S0 = 384（位模式），QC 不置位
        assert_eq!(h.cpu.fpu.read_s(0), 384);
        assert!(!h.cpu.fpu.qc());
        // 饱和：S0 = 1000.0 → 1000×256 = 256000 > 32767 → 饱和 32767 + QC
        h.cpu.fpu.write_s(0, f32(1000.0));
        assert_eq!(h.exec_word(0xEEBE_0A44), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0) as i32, 32767);
        assert!(h.cpu.fpu.qc());
    }

    /// VCVT.F32.S16：定点 → 浮点
    #[test]
    fn golden_vcvt_f32_s16_fixed() {
        let mut h = Harness::new();
        // GIVEN: S0 = 384（Q15：384 / 256 = 1.5）
        h.cpu.fpu.write_s(0, 384);
        // WHEN: VCVT.F32.S16 S0, S0, #8（0xEEBA 0A44）
        assert_eq!(h.exec_word(0xEEBA_0A44), ExecOutcome::Continue);
        // THEN: S0 = 1.5
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.5));
        // 负值：-0.5 → -128 / 256 = -0.5
        h.cpu.fpu.write_s(0, 0xFFFF_FF80u32); // -128 符号扩展
        assert_eq!(h.exec_word(0xEEBA_0A44), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(-0.5));
    }

    /// VCVT.S32.F32 #16 与 VCVT.F32.S32 #16（Q16.16）
    #[test]
    fn golden_vcvt_s32_f32_q16() {
        let mut h = Harness::new();
        // 1.5 × 65536 = 98304
        h.cpu.fpu.write_s(0, f32(1.5));
        // WHEN: VCVT.S32.F32 S0, S0, #16（0xEEBE 0AC8）
        assert_eq!(h.exec_word(0xEEBE_0AC8), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 98304);
        // WHEN: VCVT.F32.S32 S0, S0, #16（0xEEBA 0AC8）→ 98304 / 65536 = 1.5
        assert_eq!(h.exec_word(0xEEBA_0AC8), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), f32(1.5));
        // 饱和：S0 = 1e10 → 超 2^31 → 饱和 0x7FFF_FFFF + QC
        h.cpu.fpu.write_s(0, f32(1e10));
        assert_eq!(h.exec_word(0xEEBE_0AC8), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 0x7FFF_FFFF);
        assert!(h.cpu.fpu.qc());
        // U32：负值 → 饱和 0 + QC
        h.cpu.fpu.write_s(0, f32(-1.0));
        assert_eq!(h.exec_word(0xEEBF_0AC8), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 0);
        assert!(h.cpu.fpu.qc());
    }

    /// VCVT 定点 NaN → IOC；舍入 → IXC
    #[test]
    fn golden_vcvt_fixed_flags() {
        let mut h = Harness::new();
        // NaN → 0 + IOC
        h.cpu.fpu.write_s(0, 0x7FC0_0000);
        assert_eq!(h.exec_word(0xEEBE_0A44), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 0);
        assert_ne!(h.cpu.fpu.fpscr & 1, 0);
        // 0.1 × 256 = 25.6 → 舍入 26，IXC 置位
        h.cpu.fpu.write_s(0, f32(0.1));
        assert_eq!(h.exec_word(0xEEBE_0A44), ExecOutcome::Continue);
        assert_eq!(h.cpu.fpu.read_s(0), 26);
        assert_ne!(h.cpu.fpu.fpscr & (1 << 4), 0);
    }
}
