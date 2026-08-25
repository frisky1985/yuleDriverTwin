#!/usr/bin/env bash
# =============================================================================
# 构建 FreeRTOS V11.1.0 最小演示固件（Cortex-M4F / ARM_CM4F port）
#
# 产物（默认 crates/dtwin-chip/tests/fixtures/build/）：
#   freertos_demo.elf     — QEMU + dtwin 共用固件
#   freertos_demo.elf.dat — ELF 重命名副本（git 忽略 *.elf，集成测试 include_bytes!）
#
# 用法：scripts/build_freertos_demo.sh [OUT_DIR]
#
# 内核/移植（全部 vendor 入库，离线可复现）：
#   fixtures/freertos/kernel/        — FreeRTOS V11.1.0 内核（tasks/queue/list/heap_4）
#   fixtures/freertos/port/          — ARM_CM4F port.c + portmacro.h（V11.1.0，MIT）
#   fixtures/freertos/FreeRTOSConfig.h / startup_freertos.S / link_freertos.ld
#   fixtures/freertos/main_freertos.c
#
# 构建参数（FRT-FW-05）：-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16
# -mfloat-abi=hard -O2 -ffreestanding -nostdlib -Wall -Werror
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
FRT="$FIXTURES/freertos"
OUT_DIR="${1:-$FIXTURES/build}"

PREFIX="${PREFIX:-arm-none-eabi}"
CC="$PREFIX-gcc"

mkdir -p "$OUT_DIR"

echo "== 编译 freertos_demo（cortex-m4, fpv4-sp-d16, hard-float, -O2）=="
"$CC" \
  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -O2 -g3 \
  -Wall -Werror \
  -ffreestanding -fdata-sections -ffunction-sections \
  -nostartfiles -nodefaultlibs -nostdlib \
  -Wl,--gc-sections \
  -DFREERTOS_CONFIG_H_FILE="FreeRTOSConfig.h" \
  -I "$FRT" \
  -I "$FRT/kernel/include" \
  -I "$FRT/port" \
  -I "$FRT/include" \
  -T "$FRT/link_freertos.ld" \
  -o "$OUT_DIR/freertos_demo.elf" \
  "$FRT/kernel/src/tasks.c" \
  "$FRT/kernel/src/queue.c" \
  "$FRT/kernel/src/list.c" \
  "$FRT/kernel/portable/heap_4.c" \
  "$FRT/port/port.c" \
  "$FRT/startup_freertos.S" \
  "$FRT/main_freertos.c" \
  "$FRT/libc_stubs.c"

# .elf → .elf.dat（git 忽略 *.elf，集成测试 include_bytes! 引用 .dat）
cp "$OUT_DIR/freertos_demo.elf" "$OUT_DIR/freertos_demo.elf.dat"
cp "$OUT_DIR/freertos_demo.elf.dat" "$FIXTURES/freertos_demo.elf.dat"

echo "== 产物 =="
"$PREFIX-size" "$OUT_DIR/freertos_demo.elf"
ls -la "$OUT_DIR/freertos_demo.elf" "$OUT_DIR/freertos_demo.elf.dat"
