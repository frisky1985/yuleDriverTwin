//! DSP 扩展 — Cortex-M4F 饱和运算与 SIMD 指令
//!
//! Phase 3 实现：SSAT/USAT/QADD/QSUB/QDADD/QDSUB/SADD16 等半字 SIMD/
//! SMUAD/SMLAD 系列乘加。Phase 1 提供饱和运算基础工具函数。
//!
//! 纯函数层：Q 标志/GE 标志的写入由 exec 层负责（需要 CpuState）。

/// 有符号饱和到 n 位（1-32）
pub fn ssat(val: i32, bit_pos: u32) -> i32 {
    let bit_pos = bit_pos.clamp(1, 32);
    if bit_pos == 32 {
        return val;
    }
    let max = (1i32 << (bit_pos - 1)) - 1;
    let min = -(1i32 << (bit_pos - 1));
    val.clamp(min, max)
}

/// 无符号饱和到 n 位（0-32）
pub fn usat(val: i32, bit_pos: u32) -> u32 {
    let bit_pos = bit_pos.clamp(0, 31);
    let max = if bit_pos == 31 {
        u32::MAX
    } else {
        (1u32 << (bit_pos + 1)) - 1
    };
    if val < 0 {
        0
    } else {
        (val as u32).min(max)
    }
}

/// 饱和加法（QADD）
pub fn qadd(a: i32, b: i32) -> i32 {
    a.saturating_add(b)
}

/// 饱和减法（QSUB）
pub fn qsub(a: i32, b: i32) -> i32 {
    a.saturating_sub(b)
}

/// 双半字 SIMD 加法（SADD16）：[15:0] 与 [31:16] 独立相加（无符号半字）
pub fn sadd16(a: u32, b: u32) -> u32 {
    let lo = (a & 0xFFFF).wrapping_add(b & 0xFFFF) & 0xFFFF;
    let hi = ((a >> 16) & 0xFFFF).wrapping_add((b >> 16) & 0xFFFF) & 0xFFFF;
    lo | (hi << 16)
}

/// 双半字 SIMD 减法（SSUB16）
pub fn ssub16(a: u32, b: u32) -> u32 {
    let lo = (a & 0xFFFF).wrapping_sub(b & 0xFFFF) & 0xFFFF;
    let hi = ((a >> 16) & 0xFFFF).wrapping_sub((b >> 16) & 0xFFFF) & 0xFFFF;
    lo | (hi << 16)
}

/// QADD（带饱和标志）：返回 (结果, 是否饱和)
pub fn qadd_q(a: i32, b: i32) -> (i32, bool) {
    let (r, sat) = a.overflowing_add(b);
    if sat {
        (if a > 0 { i32::MAX } else { i32::MIN }, true)
    } else {
        (r, false)
    }
}

/// QSUB（带饱和标志）
pub fn qsub_q(a: i32, b: i32) -> (i32, bool) {
    let (r, sat) = a.overflowing_sub(b);
    if sat {
        (if a >= 0 && b < 0 { i32::MAX } else { i32::MIN }, true)
    } else {
        (r, false)
    }
}

/// 有符号加倍（饱和）：SAT(2×b)
fn qdbl(b: i32) -> (i32, bool) {
    if b > 0x3FFF_FFFF {
        (i32::MAX, true)
    } else if b < -0x4000_0000 {
        (i32::MIN, true)
    } else {
        (b * 2, false)
    }
}

/// QDADD（带饱和标志）：Rd = SAT(Rm + SAT(2×Rn))
pub fn qdadd_q(a: i32, b: i32) -> (i32, bool) {
    let (db, s1) = qdbl(b);
    let (r, s2) = qadd_q(a, db);
    (r, s1 || s2)
}

/// QDSUB（带饱和标志）：Rd = SAT(Rm − SAT(2×Rn))
pub fn qdsub_q(a: i32, b: i32) -> (i32, bool) {
    let (db, s1) = qdbl(b);
    let (r, s2) = qsub_q(a, db);
    (r, s1 || s2)
}

/// 半字 SIMD 运算：返回 (结果, GE[1:0])
///
/// 有符号：GE = 对应半字结果 ≥ 0；无符号：加法 GE = 进位，减法 GE = 无借位。
pub fn simd16(
    a: u32,
    b: u32,
    kind: crate::engine::decode::Simd16Kind,
    unsigned: bool,
) -> (u32, u8) {
    let al = a as u16 as i32;
    let ah = (a >> 16) as u16 as i32;
    let bl = b as u16 as i32;
    let bh = (b >> 16) as u16 as i32;
    // 每半字的运算类型：true = 加法（半字交叉按 ARM 语义）
    let (lo_add, hi_add, lo, hi) = match kind {
        crate::engine::decode::Simd16Kind::Add16 => (true, true, al + bl, ah + bh),
        crate::engine::decode::Simd16Kind::Asx => (true, false, al + bh, ah - bl),
        crate::engine::decode::Simd16Kind::Sax => (false, true, al - bh, ah + bl),
        crate::engine::decode::Simd16Kind::Sub16 => (false, false, al - bl, ah - bh),
    };
    let (res_lo, res_hi) = ((lo as u16) as u32, (hi as u16) as u32);
    let ge = if unsigned {
        // 加法半字 → 进位；减法半字 → 无借位
        let g_lo = if lo_add {
            ((al as u32)
                + if kind == crate::engine::decode::Simd16Kind::Asx {
                    bh as u32
                } else {
                    bl as u32
                })
                > 0xFFFF
        } else {
            al >= if kind == crate::engine::decode::Simd16Kind::Sax {
                bh
            } else {
                bl
            }
        };
        let g_hi = if hi_add {
            ((ah as u32)
                + if kind == crate::engine::decode::Simd16Kind::Sax {
                    bl as u32
                } else {
                    bh as u32
                })
                > 0xFFFF
        } else {
            ah >= if kind == crate::engine::decode::Simd16Kind::Asx {
                bl
            } else {
                bh
            }
        };
        (g_lo as u8) | ((g_hi as u8) << 1)
    } else {
        ((res_lo as i16 >= 0) as u8) | (((res_hi as i16 >= 0) as u8) << 1)
    };
    (res_lo | (res_hi << 16), ge)
}

/// 8-bit SIMD：每字节独立加/减（可减半），返回 (结果, GE[3:0])
///
/// - 非减半：有符号 GE = 字节结果 ≥ 0；无符号加 GE = 进位；无符号减 GE = 无借位
/// - 减半（SHADD8/SHSUB8/UHADD8/UHSUB8）：结果 = (a ± b) >> 1（扩展宽度运算后移 1），
///   GE[3:0] 清零（与 QEMU target/arm/helper.c 实现一致）
pub fn simd8(a: u32, b: u32, unsigned: bool, halving: bool, sub: bool) -> (u32, u8) {
    let mut res = 0u32;
    let mut ge = 0u8;
    for n in 0..4 {
        let shift = n * 8;
        let av = (a >> shift) & 0xFF;
        let bv = (b >> shift) & 0xFF;
        // 字节结果（减半用扩展宽度求 (a±b) 后移 1，避免环绕丢位）
        let byte: u32 = if halving {
            // QEMU op_addsub.h 语义：SHADD8/SHSUB8 用 int32 算术右移，
            // UHADD8/UHSUB8 用 uint32 逻辑右移（UHSUB8 回绕减，无 +0x100）
            let sum: u32 = if sub {
                if unsigned {
                    av.wrapping_sub(bv)
                } else {
                    ((av as i8 as i32) - (bv as i8 as i32)) as u32
                }
            } else if unsigned {
                av.wrapping_add(bv)
            } else {
                ((av as i8 as i32) + (bv as i8 as i32)) as u32
            };
            if unsigned {
                (sum >> 1) & 0xFF
            } else {
                ((sum as i32) >> 1) as u32 & 0xFF
            }
        } else if sub {
            if unsigned {
                av.wrapping_sub(bv) & 0xFF
            } else {
                ((av as i8 as i16) - (bv as i8 as i16)) as u32 & 0xFF
            }
        } else if unsigned {
            av.wrapping_add(bv) & 0xFF
        } else {
            ((av as i8 as i16) + (bv as i8 as i16)) as u32 & 0xFF
        };
        res |= byte << shift;
        if !halving {
            let flag = if sub {
                if unsigned {
                    av >= bv // 无借位
                } else {
                    (av as i8 as i16) - (bv as i8 as i16) >= 0
                }
            } else if unsigned {
                (av as u16) + (bv as u16) > 0xFF // 进位
            } else {
                (av as i8 as i16) + (bv as i8 as i16) >= 0
            };
            if flag {
                ge |= 1 << n;
            }
        }
    }
    (res, ge)
}

/// 取 Rm 的双半字（可选交换：SMUADX 等交换 Rm 的 lo/hi）
/// 返回 (低半字有符号扩展, 高半字有符号扩展)
pub fn dual_half_operands(rm: u32, swap: bool) -> (i32, i32) {
    let lo = (rm & 0xFFFF) as u16 as i32;
    let hi = ((rm >> 16) & 0xFFFF) as u16 as i32;
    if swap {
        (hi, lo)
    } else {
        (lo, hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ssat_bounds() {
        assert_eq!(ssat(200, 8), 127); // 超出 → 饱和到 127
        assert_eq!(ssat(-200, 8), -128); // 超出 → 饱和到 -128
        assert_eq!(ssat(100, 8), 100); // 范围内 → 原值
        assert_eq!(ssat(-100, 8), -100);
    }

    #[test]
    fn usat_bounds() {
        assert_eq!(usat(-1, 8), 0); // 负数 → 0
        assert_eq!(usat(600, 8), 511); // 超出 9 位无符号 → 饱和 511 (ARM USAT #8 = 9 位)
        assert_eq!(usat(300, 8), 300); // 9 位范围内 → 原值
        assert_eq!(usat(100, 8), 100);
    }

    #[test]
    fn qadd_saturates() {
        assert_eq!(qadd(i32::MAX, 1), i32::MAX);
        assert_eq!(qadd(i32::MIN, -1), i32::MIN);
    }

    #[test]
    fn simd_halfword() {
        assert_eq!(sadd16(0x0001_0002, 0x0001_0002), 0x0002_0004);
        assert_eq!(ssub16(0x0005_0005, 0x0002_0001), 0x0003_0004);
    }

    #[test]
    fn q_saturating_helpers() {
        // QADD 饱和 + 标志
        assert_eq!(qadd_q(i32::MAX, 1), (i32::MAX, true));
        assert_eq!(qadd_q(5, 3), (8, false));
        // QSUB 饱和
        assert_eq!(qsub_q(i32::MIN, 1), (i32::MIN, true));
        assert_eq!(qsub_q(5, 3), (2, false));
        // QDADD: SAT(SAT(2×Rn) + Rm)
        assert_eq!(qdadd_q(0x7FFF_FFFF, 0x4000_0000), (0x7FFF_FFFF, true));
        assert_eq!(qdadd_q(100, 50), (200, false));
        // QDSUB: SAT(Rm − SAT(2×Rn))
        assert_eq!(qdsub_q(0, 0x4000_0000), (i32::MIN + 1, true));
        assert_eq!(qdsub_q(-100, 50), (-200, false));
    }

    #[test]
    fn simd16_ge_semantics() {
        use crate::engine::decode::Simd16Kind;
        // 有符号 SADD16: GE = 半字结果 ≥ 0
        let (r, ge) = simd16(0x0001_0002, 0x0003_0004, Simd16Kind::Add16, false);
        assert_eq!(r, 0x0004_0006);
        assert_eq!(ge, 0b11);
        let (r, ge) = simd16(0x8000_0000, 0x0000_0000, Simd16Kind::Add16, false);
        assert_eq!(r, 0x8000_0000); // 高半字 = -32768，低半字 = 0
        assert_eq!(ge, 0b01); // GE[0]=1（低半字 0 ≥ 0），GE[1]=0
                              // 无符号 UADD16: GE = 进位
        let (r, ge) = simd16(0xFFFF_0001, 0x0001_0001, Simd16Kind::Add16, true);
        assert_eq!(r, 0x0000_0002); // 低半字 1+1=2，高半字 0xFFFF+1=0 进位
        assert_eq!(ge, 0b10); // 高半字进位
                              // SASX: 低半字 = Rn.lo − Rm.hi，高半字 = Rn.hi + Rm.lo
        let (r, ge) = simd16(0x0002_0005, 0x0001_0003, Simd16Kind::Sax, false);
        // 低半字: Rn.lo(0x0005) − Rm.hi(0x0001) = 4；高半字: Rn.hi(0x0002) + Rm.lo(0x0003) = 5
        assert_eq!(r, 0x0005_0004);
        assert_eq!(ge, 0b11);
    }

    #[test]
    fn simd8_ge_semantics() {
        // 有符号 SADD8：GE = 每字节结果 ≥ 0
        let (r, ge) = simd8(0x01_02_03_04, 0x01_02_03_04, false, false, false);
        assert_eq!(r, 0x02_04_06_08);
        assert_eq!(ge, 0b1111);
        // 有符号负数：高字节 -1 + 0 = -1 → GE 清位
        let (r, ge) = simd8(0xFF_00_00_00, 0x00_00_00_00, false, false, false);
        assert_eq!(r, 0xFF_00_00_00);
        assert_eq!(ge, 0b0111);
        // 无符号 UADD8：GE = 进位
        let (r, ge) = simd8(0xFF_01_01_01, 0x01_01_01_01, true, false, false);
        assert_eq!(r, 0x00_02_02_02);
        assert_eq!(ge, 0b1000);
        // 无符号 USUB8：GE = 无借位
        let (r, ge) = simd8(0x05_05_05_05, 0x02_08_01_0F, true, false, true);
        assert_eq!(r, 0x03_FD_04_F6);
        assert_eq!(ge, 0b1010); // 字节 1,3 无借位；字节 0,2 借位（0x0F/0x08 > 0x05）
        // 有符号 SSUB8：GE = 结果 ≥ 0
        let (r, ge) = simd8(0x05_05_00_05, 0x02_01_01_01, false, false, true);
        assert_eq!(r, 0x0304_FF04); // 字节 1：0x00-0x01 = -1 → 0xFF
        assert_eq!(ge, 0b1101);
    }

    #[test]
    fn simd8_halving_semantics() {
        // SHADD8(1, 3) = 2；SHADD8(-1, -1) = -1（算术移位）
        let (r, ge) = simd8(0x01_01_01_01, 0x03_03_03_03, false, true, false);
        assert_eq!(r, 0x02_02_02_02);
        assert_eq!(ge, 0); // 减半不更新 GE（QEMU 语义：清零）
        let (r, _) = simd8(0xFF_FF_FF_FF, 0xFF_FF_FF_FF, false, true, false);
        assert_eq!(r, 0xFF_FF_FF_FF); // (-2)>>1 = -1
        // UHADD8(0xFF, 0x01) = 0x80（无符号扩展相加）
        let (r, ge) = simd8(0xFF_00_00_00, 0x01_00_00_00, true, true, false);
        assert_eq!(r, 0x80_00_00_00);
        assert_eq!(ge, 0);
        // SHSUB8(1, 3) = -1（算术移位）；UHSUB8(0x05, 0x03) = (5-3)>>1 = 1
        let (r, _) = simd8(0x01_00_00_00, 0x03_00_00_00, false, true, true);
        assert_eq!(r, 0xFF_00_00_00); // (-2)>>1 = -1
        let (r, _) = simd8(0x05_00_00_00, 0x03_00_00_00, true, true, true);
        assert_eq!(r, 0x01_00_00_00);
        // UHSUB8(0x01, 0x03) = 回绕减后逻辑右移 → 0xFF（QEMU 语义，无 +0x100）
        let (r, _) = simd8(0x01_00_00_00, 0x03_00_00_00, true, true, true);
        assert_eq!(r, 0xFF_00_00_00);
    }

    // ==================== 指令级 golden 测试（GIVEN/WHEN/THEN） ====================
    // 编码均由 arm-none-eabi-as -mcpu=cortex-m4 汇编验证

    use crate::engine::exec::ExecOutcome;
    use crate::engine::test_util::Harness;

    #[test]
    fn golden_ssat_saturates_and_sets_q() {
        // GIVEN: R1 = 200 超出 8 位有符号范围，Q 标志初始为 0
        let mut h = Harness::new();
        h.cpu.regs[1] = 200;
        // WHEN: SSAT R0, #8, R1（0xF301 0007）
        assert_eq!(h.exec_word(0xF301_0007), ExecOutcome::Continue);
        // THEN: R0 = 127（饱和），Q 置位
        assert_eq!(h.cpu.regs[0], 127);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_ssat_no_saturation_keeps_q_clear() {
        // GIVEN: R1 = 100 在范围内
        let mut h = Harness::new();
        h.cpu.regs[1] = 100;
        // WHEN: SSAT R0, #8, R1
        assert_eq!(h.exec_word(0xF301_0007), ExecOutcome::Continue);
        // THEN: R0 = 100，Q 不置位
        assert_eq!(h.cpu.regs[0], 100);
        assert!(!h.q_flag());
    }

    #[test]
    fn golden_ssat_shift_lsl() {
        // GIVEN: R1 = 0x08
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x08;
        // WHEN: SSAT R0, #8, R1, LSL #4（0xF301 1007）→ 移位后 0x80 = 128 → 饱和 127
        assert_eq!(h.exec_word(0xF301_1007), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 127);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_ssat_shift_asr() {
        // GIVEN: R1 = 0x8000_0000（-2^31）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x8000_0000;
        // WHEN: SSAT R0, #8, R1, ASR #8（0xF321 1007）→ ASR 后 = -2^23 → 饱和 -128
        assert_eq!(h.exec_word(0xF321_1007), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 0xFFFF_FF80u32 as i32 as u32); // -128
        assert_eq!(h.cpu.regs[0] as i32, -128);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_usat_saturates() {
        // GIVEN: R1 = 600（超出 9 位无符号范围）
        let mut h = Harness::new();
        h.cpu.regs[1] = 600;
        // WHEN: USAT R0, #8, R1（0xF381 0008）
        assert_eq!(h.exec_word(0xF381_0008), ExecOutcome::Continue);
        // THEN: R0 = 511，Q 置位
        assert_eq!(h.cpu.regs[0], 511);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_usat_negative_clamps_to_zero() {
        // GIVEN: R1 = -1（0xFFFF_FFFF）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0xFFFF_FFFF;
        // WHEN: USAT R0, #8, R1
        assert_eq!(h.exec_word(0xF381_0008), ExecOutcome::Continue);
        // THEN: R0 = 0，Q 置位
        assert_eq!(h.cpu.regs[0], 0);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_qadd_saturates() {
        // GIVEN: R1 = 0x7FFF_FFFF，R2 = 1
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x7FFF_FFFF;
        h.cpu.regs[2] = 1;
        // WHEN: QADD R0, R1, R2（0xFA82 F081）
        assert_eq!(h.exec_word(0xFA82_F081), ExecOutcome::Continue);
        // THEN: R0 = 0x7FFF_FFFF（饱和），Q 置位
        assert_eq!(h.cpu.regs[0], 0x7FFF_FFFF);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_qadd_normal() {
        // GIVEN: R1 = 100，R2 = 50
        let mut h = Harness::new();
        h.cpu.regs[1] = 100;
        h.cpu.regs[2] = 50;
        // WHEN: QADD R3, R4, R5（0xFA85 F384：R3=Rd, R4=Rm, R5=Rn）
        h.cpu.regs[4] = 100;
        h.cpu.regs[5] = 50;
        assert_eq!(h.exec_word(0xFA85_F384), ExecOutcome::Continue);
        // THEN: R3 = 150，Q 不置位
        assert_eq!(h.cpu.regs[3], 150);
        assert!(!h.q_flag());
    }

    #[test]
    fn golden_qdadd_doubling_saturates() {
        // GIVEN: R1 = 0x7FFF_FFFF（Rm），R2 = 0x4000_0000（Rn，加倍即饱和）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x7FFF_FFFF;
        h.cpu.regs[2] = 0x4000_0000;
        // WHEN: QDADD R0, R1, R2（0xFA82 F091）
        assert_eq!(h.exec_word(0xFA82_F091), ExecOutcome::Continue);
        // THEN: R0 = 0x7FFF_FFFF，Q 置位
        assert_eq!(h.cpu.regs[0], 0x7FFF_FFFF);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_qdsub() {
        // GIVEN: R1 = 0（Rm），R2 = 0x4000_0000（Rn，2×Rn = 0x8000_0000）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0;
        h.cpu.regs[2] = 0x4000_0000;
        // WHEN: QDSUB R0, R1, R2（0xFA82 F0B1）
        assert_eq!(h.exec_word(0xFA82_F0B1), ExecOutcome::Continue);
        // THEN: 加倍 2×0x4000_0000 饱和为 0x7FFF_FFFF，R0 = 0 − 0x7FFF_FFFF = 0x8000_0001，Q 置位
        assert_eq!(h.cpu.regs[0], 0x8000_0001);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_qsub_saturates() {
        // GIVEN: R1 = 0x8000_0000（-2^31），R2 = 1
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x8000_0000;
        h.cpu.regs[2] = 1;
        // WHEN: QSUB R0, R1, R2（0xFA82 F0A1）
        assert_eq!(h.exec_word(0xFA82_F0A1), ExecOutcome::Continue);
        // THEN: R0 = 0x8000_0000（饱和），Q 置位
        assert_eq!(h.cpu.regs[0], 0x8000_0000);
        assert!(h.q_flag());
    }

    #[test]
    fn golden_sadd16_ge_flags() {
        // GIVEN: R1 = 0x0001_0002，R2 = 0x0003_0004
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0001_0002;
        h.cpu.regs[2] = 0x0003_0004;
        // WHEN: SADD16 R0, R1, R2（0xFA91 F002）
        assert_eq!(h.exec_word(0xFA91_F002), ExecOutcome::Continue);
        // THEN: R0 = 0x0004_0006，GE[1:0] = 11
        assert_eq!(h.cpu.regs[0], 0x0004_0006);
        assert_eq!(h.ge_flags(), 0b11);
    }

    #[test]
    fn golden_sadd16_negative_ge_clear() {
        // GIVEN: R1 = 0x8000_0000（低半字 -32768，高半字 0）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x8000_0000;
        // WHEN: SADD16 R0, R1, R2（R2 = 0）
        assert_eq!(h.exec_word(0xFA91_F002), ExecOutcome::Continue);
        // THEN: R0 = 0x8000_0000，GE = 01（低半字 0 ≥ 0，高半字 -32768 < 0）
        assert_eq!(h.cpu.regs[0], 0x8000_0000);
        assert_eq!(h.ge_flags(), 0b01);
    }

    #[test]
    fn golden_uadd16_carry_ge() {
        // GIVEN: R1 = 0xFFFF_0001，R2 = 0x0001_0001
        let mut h = Harness::new();
        h.cpu.regs[1] = 0xFFFF_0001;
        h.cpu.regs[2] = 0x0001_0001;
        // WHEN: UADD16 R0, R1, R2（0xFA91 F042）
        assert_eq!(h.exec_word(0xFA91_F042), ExecOutcome::Continue);
        // THEN: 低半字 1+1=2 无进位 → GE[0]=0；高半字 0xFFFF+1 进位 → GE[1]=1
        assert_eq!(h.cpu.regs[0], 0x0000_0002);
        assert_eq!(h.ge_flags(), 0b10);
    }

    #[test]
    fn golden_usub16_borrow_ge() {
        // GIVEN: R1 = 0x0005_0005，R2 = 0x0002_0001
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0005_0005;
        h.cpu.regs[2] = 0x0002_0001;
        // WHEN: USUB16 R0, R1, R2（0xFAD1 F042）
        assert_eq!(h.exec_word(0xFAD1_F042), ExecOutcome::Continue);
        // THEN: 两半字均无借位 → GE = 11
        assert_eq!(h.cpu.regs[0], 0x0003_0004);
        assert_eq!(h.ge_flags(), 0b11);
    }

    #[test]
    fn golden_sasx_cross_halfword() {
        // GIVEN: R1 = 0x0002_0005（hi=2, lo=5），R2 = 0x0001_0003（hi=1, lo=3）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0005;
        h.cpu.regs[2] = 0x0001_0003;
        // WHEN: SASX R0, R1, R2（0xFAA1 F002）→ lo = 5+1=6, hi = 2-3=-1
        assert_eq!(h.exec_word(0xFAA1_F002), ExecOutcome::Continue);
        // THEN: R0 = 0xFFFF_0006，GE = 01（lo=6≥0，hi=-1<0）
        assert_eq!(h.cpu.regs[0], 0xFFFF_0006);
        assert_eq!(h.ge_flags(), 0b01);
    }

    #[test]
    fn golden_ssax_cross_halfword() {
        // GIVEN: R1 = 0x0002_0005，R2 = 0x0001_0003
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0005;
        h.cpu.regs[2] = 0x0001_0003;
        // WHEN: SSAX R0, R1, R2（0xFAE1 F002）→ lo = 5-1=4, hi = 2+3=5
        assert_eq!(h.exec_word(0xFAE1_F002), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 0x0005_0004);
        assert_eq!(h.ge_flags(), 0b11);
    }

    #[test]
    fn golden_ssub16() {
        // GIVEN: R1 = 0x0005_0005，R2 = 0x0002_0001
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0005_0005;
        h.cpu.regs[2] = 0x0002_0001;
        // WHEN: SSUB16 R0, R1, R2（0xFAD1 F002）
        assert_eq!(h.exec_word(0xFAD1_F002), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 0x0003_0004);
        assert_eq!(h.ge_flags(), 0b11);
    }

    #[test]
    fn golden_smuad() {
        // GIVEN: R1 = 0x0002_0003（hi=2, lo=3），R2 = 0x0004_0005（hi=4, lo=5）
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0003;
        h.cpu.regs[2] = 0x0004_0005;
        // WHEN: SMUAD R0, R1, R2（0xFB21 F002）→ 3×5 + 2×4 = 23
        assert_eq!(h.exec_word(0xFB21_F002), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 23);
    }

    #[test]
    fn golden_smuadx_swaps_rm() {
        // GIVEN: R1 = 0x0002_0003，R2 = 0x0004_0005
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0003;
        h.cpu.regs[2] = 0x0004_0005;
        // WHEN: SMUADX R0, R1, R2（0xFB21 F012）→ 3×4 + 2×5 = 22
        assert_eq!(h.exec_word(0xFB21_F012), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 22);
    }

    #[test]
    fn golden_smusd() {
        // GIVEN: R1 = 0x0002_0003，R2 = 0x0004_0005
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0003;
        h.cpu.regs[2] = 0x0004_0005;
        // WHEN: SMUSD R0, R1, R2（0xFB41 F002）→ 3×5 − 2×4 = 7
        assert_eq!(h.exec_word(0xFB41_F002), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 7);
    }

    #[test]
    fn golden_smusdx() {
        // GIVEN: R1 = 0x0002_0003，R2 = 0x0004_0005
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0003;
        h.cpu.regs[2] = 0x0004_0005;
        // WHEN: SMUSDX R0, R1, R2（0xFB41 F012）→ 3×4 − 2×5 = 2
        assert_eq!(h.exec_word(0xFB41_F012), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 2);
    }

    #[test]
    fn golden_smlad_accumulate() {
        // GIVEN: R1 = 0x0002_0003，R2 = 0x0004_0005，R3 = 100
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0003;
        h.cpu.regs[2] = 0x0004_0005;
        h.cpu.regs[3] = 100;
        // WHEN: SMLAD R0, R1, R2, R3（0xFB21 3002）→ 100 + 23 = 123
        assert_eq!(h.exec_word(0xFB21_3002), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 123);
    }

    #[test]
    fn golden_smlsd_subtract_accumulate() {
        // GIVEN: R1 = 0x0002_0003，R2 = 0x0004_0005，R3 = 100
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0002_0003;
        h.cpu.regs[2] = 0x0004_0005;
        h.cpu.regs[3] = 100;
        // WHEN: SMLSD R0, R1, R2, R3（0xFB41 3002）→ 100 + (15 − 8) = 107
        assert_eq!(h.exec_word(0xFB41_3002), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 107);
    }

    #[test]
    fn golden_smlald_64bit_accumulate() {
        // GIVEN: R0:R1 = 0，R2 = 0x0002_0003，R3 = 0x0004_0005
        let mut h = Harness::new();
        h.cpu.regs[2] = 0x0002_0003;
        h.cpu.regs[3] = 0x0004_0005;
        // WHEN: SMLALD R0, R1, R2, R3（0xFBC2 01C3：RdLo=R0, RdHi=R1）→ 0 + 23
        assert_eq!(h.exec_word(0xFBC2_01C3), ExecOutcome::Continue);
        // THEN: R0 = 23，R1 = 0
        assert_eq!(h.cpu.regs[0], 23);
        assert_eq!(h.cpu.regs[1], 0);
    }

    #[test]
    fn golden_smlald_carry_into_high() {
        // GIVEN: RdLo:RdHi = 0xFFFF_FFFF:0，R2 = 0x0002_0003，R3 = 0x0004_0005
        let mut h = Harness::new();
        h.cpu.regs[0] = 0xFFFF_FFFF;
        h.cpu.regs[1] = 0;
        h.cpu.regs[2] = 0x0002_0003;
        h.cpu.regs[3] = 0x0004_0005;
        // WHEN: SMLALD R0, R1, R2, R3 → 0xFFFF_FFFF + 23 = 0x1_0000_0016
        assert_eq!(h.exec_word(0xFBC2_01C3), ExecOutcome::Continue);
        // THEN: R0 = 0x16，R1 = 1
        assert_eq!(h.cpu.regs[0], 0x16);
        assert_eq!(h.cpu.regs[1], 1);
    }

    #[test]
    fn golden_smlsld_subtract_long() {
        // GIVEN: R0:R1 = 100，R2 = 0x0002_0003，R3 = 0x0004_0005
        let mut h = Harness::new();
        h.cpu.regs[0] = 100;
        h.cpu.regs[1] = 0;
        h.cpu.regs[2] = 0x0002_0003;
        h.cpu.regs[3] = 0x0004_0005;
        // WHEN: SMLSLD R0, R1, R2, R3（0xFBD2 01C3）→ 100 + (15 − 8) = 107
        assert_eq!(h.exec_word(0xFBD2_01C3), ExecOutcome::Continue);
        // THEN
        assert_eq!(h.cpu.regs[0], 107);
        assert_eq!(h.cpu.regs[1], 0);
    }

    #[test]
    fn golden_mla_mls() {
        // GIVEN: R1 = 6，R2 = 7，R3 = 100
        let mut h = Harness::new();
        h.cpu.regs[1] = 6;
        h.cpu.regs[2] = 7;
        h.cpu.regs[3] = 100;
        // WHEN: MLA R0, R1, R2, R3（0xFB01 3002）→ 100 + 42 = 142
        assert_eq!(h.exec_word(0xFB01_3002), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 142);
        // WHEN: MLS R0, R1, R2, R3（0xFB01 3012）→ 100 − 42 = 58
        assert_eq!(h.exec_word(0xFB01_3012), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 58);
    }

    #[test]
    fn golden_pkhbt_pkhtb() {
        // GIVEN: R1 = 0xAAAA_BBBB，R2 = 0xCCCC_DDDD
        let mut h = Harness::new();
        h.cpu.regs[1] = 0xAAAA_BBBB;
        h.cpu.regs[2] = 0xCCCC_DDDD;
        // WHEN: PKHBT R0, R1, R2, LSL #4（0xEAC1 1002）
        assert_eq!(h.exec_word(0xEAC1_1002), ExecOutcome::Continue);
        // THEN: R0[15:0] = 0xBBBB，R0[31:16] = (0xCCCC_DDDD << 4)[31:16] = 0xCCCD
        assert_eq!(h.cpu.regs[0], 0xCCCD_BBBB);
        // WHEN: PKHTB R0, R1, R2, ASR #4（0xEAC1 1022）
        assert_eq!(h.exec_word(0xEAC1_1022), ExecOutcome::Continue);
        // THEN: R0[31:16] = 0xAAAA，R0[15:0] = (0xCCCC_DDDD >> 4)[15:0] = 0xCDDD
        assert_eq!(h.cpu.regs[0], 0xAAAA_CDDD);
    }

    #[test]
    fn golden_pkhbt_no_shift() {
        // GIVEN: R1 = 0x1111_2222，R2 = 0x3333_4444
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x1111_2222;
        h.cpu.regs[2] = 0x3333_4444;
        // WHEN: PKHBT R0, R1, R2（0xEAC1 0002）
        assert_eq!(h.exec_word(0xEAC1_0002), ExecOutcome::Continue);
        // THEN: R0 = 0x3333_2222
        assert_eq!(h.cpu.regs[0], 0x3333_2222);
    }

    // ============ P3-补：8-bit SIMD golden（编码经 arm-none-eabi-as 实测） ============

    #[test]
    fn golden_sadd8_ge() {
        // GIVEN: R1 = 0x01_02_03_04，R2 = 0x01_02_03_04
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0102_0304;
        h.cpu.regs[2] = 0x0102_0304;
        // WHEN: SADD8 R0, R1, R2（0xFA81 F002）
        assert_eq!(h.exec_word(0xFA81_F002), ExecOutcome::Continue);
        // THEN: R0 = 0x02_04_06_08，GE[3:0] = 1111
        assert_eq!(h.cpu.regs[0], 0x0204_0608);
        assert_eq!(h.ge_flags(), 0b1111);
    }

    #[test]
    fn golden_sadd8_negative_ge_clear() {
        // GIVEN: R1 = 0xFF_00_00_00（高字节 -1），R2 = 0
        let mut h = Harness::new();
        h.cpu.regs[1] = 0xFF00_0000;
        // WHEN: SADD8 R0, R1, R2（0xFA81 F002）
        assert_eq!(h.exec_word(0xFA81_F002), ExecOutcome::Continue);
        // THEN: 高字节结果 -1 < 0 → GE[3] = 0，其余 ≥ 0 → GE = 0111
        assert_eq!(h.cpu.regs[0], 0xFF00_0000);
        assert_eq!(h.ge_flags(), 0b0111);
    }

    #[test]
    fn golden_uadd8_carry_ge() {
        // GIVEN: R1 = 0xFF_01_01_01，R2 = 0x01_01_01_01
        let mut h = Harness::new();
        h.cpu.regs[1] = 0xFF01_0101;
        h.cpu.regs[2] = 0x0101_0101;
        // WHEN: UADD8 R0, R1, R2（0xFA81 F042）
        assert_eq!(h.exec_word(0xFA81_F042), ExecOutcome::Continue);
        // THEN: 高字节进位 → GE = 1000
        assert_eq!(h.cpu.regs[0], 0x0002_0202);
        assert_eq!(h.ge_flags(), 0b1000);
    }

    #[test]
    fn golden_usub8_borrow_ge() {
        // GIVEN: R1 = 0x05_05_05_05，R2 = 0x02_08_01_0F
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0505_0505;
        h.cpu.regs[2] = 0x0208_010F;
        // WHEN: USUB8 R0, R1, R2（0xFAC1 F042）
        assert_eq!(h.exec_word(0xFAC1_F042), ExecOutcome::Continue);
        // THEN: R0 = 0x03_FD_04_F6；GE = 1010（字节 0,2 无借位）
        assert_eq!(h.cpu.regs[0], 0x03FD_04F6);
        assert_eq!(h.ge_flags(), 0b1010);
    }

    #[test]
    fn golden_ssub8_signed_ge() {
        // GIVEN: R1 = 0x0505_0005（字节 1 = 0x00），R2 = 0x0201_0101
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0505_0005;
        h.cpu.regs[2] = 0x0201_0101;
        // WHEN: SSUB8 R0, R1, R2（0xFAC1 F002）
        assert_eq!(h.exec_word(0xFAC1_F002), ExecOutcome::Continue);
        // THEN: 字节 1 结果 -1 → GE[1] = 0 → GE = 1101
        assert_eq!(h.cpu.regs[0], 0x0304_FF04);
        assert_eq!(h.ge_flags(), 0b1101);
    }

    #[test]
    fn golden_shadd8_halving_ge_zero() {
        // GIVEN: R1 = 0x01_01_01_01，R2 = 0x03_03_03_03，GE 预置 0xF
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0101_0101;
        h.cpu.regs[2] = 0x0303_0303;
        h.cpu.xpsr |= 0xF << 16;
        // WHEN: SHADD8 R0, R1, R2（0xFA81 F022）
        assert_eq!(h.exec_word(0xFA81_F022), ExecOutcome::Continue);
        // THEN: R0 = 0x02_02_02_02；减半指令 GE 清零
        assert_eq!(h.cpu.regs[0], 0x0202_0202);
        assert_eq!(h.ge_flags(), 0);
    }

    #[test]
    fn golden_uhadd8_wide_sum() {
        // GIVEN: R1 = 0xFF_00_00_00，R2 = 0x01_00_00_00
        let mut h = Harness::new();
        h.cpu.regs[1] = 0xFF00_0000;
        h.cpu.regs[2] = 0x0100_0000;
        // WHEN: UHADD8 R0, R1, R2（0xFA81 F062）
        assert_eq!(h.exec_word(0xFA81_F062), ExecOutcome::Continue);
        // THEN: (255+1)>>1 = 128 = 0x80
        assert_eq!(h.cpu.regs[0], 0x8000_0000);
        assert_eq!(h.ge_flags(), 0);
    }

    #[test]
    fn golden_shsub8_uhsub8() {
        // GIVEN: R1 = 0x01_00_00_00，R2 = 0x03_00_00_00
        let mut h = Harness::new();
        h.cpu.regs[1] = 0x0100_0000;
        h.cpu.regs[2] = 0x0300_0000;
        // WHEN: SHSUB8 R0, R1, R2（0xFAC1 F022）→ (-2)>>1 = -1
        assert_eq!(h.exec_word(0xFAC1_F022), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFF00_0000);
        // WHEN: UHSUB8 R0, R1, R2（0xFAC1 F062）→ 回绕减后逻辑右移 → 0xFF
        assert_eq!(h.exec_word(0xFAC1_F062), ExecOutcome::Continue);
        assert_eq!(h.cpu.regs[0], 0xFF00_0000);
        assert_eq!(h.ge_flags(), 0);
    }
}
