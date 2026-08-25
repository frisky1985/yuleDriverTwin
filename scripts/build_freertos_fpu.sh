#!/usr/bin/env bash
# =============================================================================
# 构建 FreeRTOS FPU 场景 B 变体固件（FRT-AC-07）
#
# 产物（默认 crates/dtwin-chip/tests/fixtures/build/）：
#   freertos_fpu.elf     — 浮点任务（VADD/VMUL/VDIV/VCVT 累计）+ 纯整数任务
#   freertos_fpu.elf.dat — ELF 重命名副本（git 忽略 *.elf，测试 include_bytes!）
#
# 用法：scripts/build_freertos_fpu.sh [OUT_DIR]
#
# 源码：fixtures/freertos/main_freertos_fpu.c——vFpuTask(pri2) 的 float 局部
# 跨 vTaskDelay 存活（hard-float AAPCS → callee-saved s16-s31），每次睡眠/唤醒
# 都经 FPU 扩展帧（EXC_RETURN=ED）+ s16-s31 保存/恢复；vIntTask(pri1) 纯整数
# 走 FD 变体，双向切换覆盖两条路径。
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
FRT="$FIXTURES/freertos"
OUT_DIR="${1:-$FIXTURES/build}"

PREFIX="${PREFIX:-arm-none-eabi}"
CC="$PREFIX-gcc"

mkdir -p "$OUT_DIR"

echo "== 编译 freertos_fpu（cortex-m4, fpv4-sp-d16, hard-float, -O2）=="
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
  -o "$OUT_DIR/freertos_fpu.elf" \
  "$FRT/kernel/src/tasks.c" \
  "$FRT/kernel/src/queue.c" \
  "$FRT/kernel/src/list.c" \
  "$FRT/kernel/portable/heap_4.c" \
  "$FRT/port/port.c" \
  "$FRT/startup_freertos.S" \
  "$FRT/main_freertos_fpu.c" \
  "$FRT/libc_stubs.c"

cp "$OUT_DIR/freertos_fpu.elf" "$OUT_DIR/freertos_fpu.elf.dat"
cp "$OUT_DIR/freertos_fpu.elf.dat" "$FIXTURES/freertos_fpu.elf.dat"

echo "== 产物 =="
"$PREFIX-size" "$OUT_DIR/freertos_fpu.elf"
ls -la "$OUT_DIR/freertos_fpu.elf" "$OUT_DIR/freertos_fpu.elf.dat"
