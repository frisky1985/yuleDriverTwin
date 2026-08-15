#!/usr/bin/env bash
# =============================================================================
# 构建 dtwin A9 E2E 压力固件（Cortex-M4F / ARMv7E-M）
#
# 产物（输出目录默认为 fixtures/build，仓库 gitignore 已忽略 *.o/*.elf）：
#   e2e_driver_stress.elf    — 可执行固件（QEMU + dtwin 共用）
#   e2e_driver_stress.elf.dat— ELF 重命名副本（规避仓库 *.elf 忽略规则，
#                              dtwin 集成测试 include_bytes! 使用）
#
# 用法：
#   scripts/build_driver_stress.sh [OUT_DIR]
#     OUT_DIR 默认 crates/dtwin-chip/tests/fixtures/build
#
# 说明：
#   - -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard（S32K312 = M4F）
#   - -nostartfiles/-nostdlib/-ffreestanding：裸机无 libc
#   - 允许 movw/movt（顺带覆盖 A5 修复路径）；常数其余走字面量池
#   - 不写 CPACR（dtwin 引擎默认 FPU 已使能，0xE000ED88 不在其内存模型内）
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
OUT_DIR="${1:-$FIXTURES/build}"

PREFIX="${PREFIX:-arm-none-eabi}"
CC="$PREFIX-gcc"

mkdir -p "$OUT_DIR"

echo "== 编译 e2e_driver_stress（cortex-m4, fpv4-sp-d16, hard-float, -O0）=="
"$CC" \
  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -O0 -g3 \
  -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable \
  -ffreestanding -fdata-sections -ffunction-sections \
  -nostartfiles -nodefaultlibs -nostdlib \
  -Wl,--gc-sections \
  -T "$FIXTURES/link_e2e.ld" \
  -o "$OUT_DIR/e2e_driver_stress.elf" \
  "$FIXTURES/startup_e2e.S" "$FIXTURES/e2e_driver_stress.c"

# .elf → .elf.dat（git 忽略 *.elf，集成测试 include_bytes! 引用 .dat）
cp "$OUT_DIR/e2e_driver_stress.elf" "$OUT_DIR/e2e_driver_stress.elf.dat"
# 提交到 fixtures 根目录供 dtwin 集成测试 include_bytes! 使用
cp "$OUT_DIR/e2e_driver_stress.elf.dat" "$FIXTURES/e2e_driver_stress.elf.dat"

echo "== 产物 =="
"$PREFIX-size" "$OUT_DIR/e2e_driver_stress.elf"
ls -la "$OUT_DIR/e2e_driver_stress.elf" "$OUT_DIR/e2e_driver_stress.elf.dat"
