//! DSP 扩展 — Cortex-M4F 饱和运算与 SIMD 指令
//!
//! Phase 3 实现：SSAT/USAT/QADD/QSUB/SADD16/SMUAD 等。
//! Phase 1 提供饱和运算基础工具函数。

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
    let max = if bit_pos == 31 { u32::MAX } else { (1u32 << (bit_pos + 1)) - 1 };
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ssat_bounds() {
        assert_eq!(ssat(200, 8), 127);   // 超出 → 饱和到 127
        assert_eq!(ssat(-200, 8), -128); // 超出 → 饱和到 -128
        assert_eq!(ssat(100, 8), 100);   // 范围内 → 原值
        assert_eq!(ssat(-100, 8), -100);
    }

    #[test]
    fn usat_bounds() {
        assert_eq!(usat(-1, 8), 0);      // 负数 → 0
        assert_eq!(usat(600, 8), 511);   // 超出 9 位无符号 → 饱和 511 (ARM USAT #8 = 9 位)
        assert_eq!(usat(300, 8), 300);   // 9 位范围内 → 原值
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
}
