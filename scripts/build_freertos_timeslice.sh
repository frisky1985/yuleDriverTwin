#!/usr/bin/env bash
# =============================================================================
# 构建 FreeRTOS 时间片轮转变体固件（FRT-AC-02）
#
# 产物（默认 crates/dtwin-chip/tests/fixtures/build/）：
#   freertos_timeslice.elf        — configUSE_TIME_SLICING=1（验收对象）
#   freertos_timeslice_noslice.elf— configUSE_TIME_SLICING=0（对照：同固件仅
#                                   此一处配置差异 → 交替退化为 A 全量先跑完）
#   *.elf.dat                     — ELF 重命名副本（git 忽略 *.elf，测试 include_bytes!）
#
# 用法：scripts/build_freertos_timeslice.sh [OUT_DIR]
#
# 源码：fixtures/freertos/main_freertos_timeslice.c（2 个同优先级 pri2 任务，
# 忙循环打印 [TS] A/B 各 N=40 次，不阻塞；忙等 SysTick COUNTFLAG——时间片
# 轮转为唯一抢占机制；打印受临界区保护，行不交错）。
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
FRT="$FIXTURES/freertos"
OUT_DIR="${1:-$FIXTURES/build}"

PREFIX="${PREFIX:-arm-none-eabi}"
CC="$PREFIX-gcc"

mkdir -p "$OUT_DIR"

COMMON=(
  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  -O2 -g3
  -Wall -Werror
  -ffreestanding -fdata-sections -ffunction-sections
  -nostartfiles -nodefaultlibs -nostdlib
  -Wl,--gc-sections
  -DFREERTOS_CONFIG_H_FILE="FreeRTOSConfig.h"
  -I "$FRT"
  -I "$FRT/kernel/include"
  -I "$FRT/port"
  -I "$FRT/include"
  -T "$FRT/link_freertos.ld"
  "$FRT/kernel/src/tasks.c"
  "$FRT/kernel/src/queue.c"
  "$FRT/kernel/src/list.c"
  "$FRT/kernel/portable/heap_4.c"
  "$FRT/port/port.c"
  "$FRT/startup_freertos.S"
  "$FRT/libc_stubs.c"
)

echo "== 编译 freertos_timeslice（configUSE_TIME_SLICING=1）=="
"$CC" "${COMMON[@]}" "$FRT/main_freertos_timeslice.c" \
  -o "$OUT_DIR/freertos_timeslice.elf"

echo "== 编译 freertos_timeslice_noslice（-DconfigUSE_TIME_SLICING=0 对照）=="
"$CC" "${COMMON[@]}" -DconfigUSE_TIME_SLICING=0 \
  "$FRT/main_freertos_timeslice.c" \
  -o "$OUT_DIR/freertos_timeslice_noslice.elf"

for f in freertos_timeslice freertos_timeslice_noslice; do
  cp "$OUT_DIR/$f.elf" "$OUT_DIR/$f.elf.dat"
  cp "$OUT_DIR/$f.elf.dat" "$FIXTURES/$f.elf.dat"
done

echo "== 产物 =="
"$PREFIX-size" "$OUT_DIR/freertos_timeslice.elf" "$OUT_DIR/freertos_timeslice_noslice.elf"
ls -la "$OUT_DIR"/freertos_timeslice*.elf "$OUT_DIR"/freertos_timeslice*.elf.dat
