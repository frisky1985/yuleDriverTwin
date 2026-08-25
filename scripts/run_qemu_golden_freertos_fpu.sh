#!/usr/bin/env bash
# =============================================================================
# 生成 freertos_fpu 变体固件的 QEMU 黄金输出（FRT-AC-07）
#
#   qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel <elf>
#
# FPU 变体：浮点任务（VADD/VFMA/VCVT 累计，delay 1）+ 纯整数任务（delay 2），
# 输出 [FPU]/[INT] 行，序列 tick 计数驱动（与主固件同模式，跨模拟器可复现）。
# RUNTIME_SEC 默认 1.2s（~50 行 [FPU]，够覆盖多次 FPU 上下文切换；dtwin 侧
# 换算 max_instructions ≈ (行数+8)×25000）。
# 产物复制到 fixtures/freertos_fpu_golden_output.txt（提交用）。
#
# 用法：scripts/run_qemu_golden_freertos_fpu.sh [ELF] [OUT] [RUNTIME_SEC]
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
ELF="${1:-$FIXTURES/build/freertos_fpu.elf}"
OUT="${2:-/tmp/freertos_fpu_qemu_golden.txt}"
RUNTIME_SEC="${3:-1.2}"

if [ ! -f "$ELF" ]; then
  echo "错误: 固件不存在: $ELF（先运行 scripts/build_freertos_fpu.sh）" >&2
  exit 2
fi

echo "== QEMU golden run: $ELF（${RUNTIME_SEC}s）=="
qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel "$ELF" > "$OUT" 2>&1 &
QPID=$!
sleep "$RUNTIME_SEC"
kill -TERM "$QPID" 2>/dev/null || true
sleep 1

grep -v '^qemu-system-arm:' "$OUT" | grep -v '^$' | tr -d '\r' > "$OUT.tmp" && mv "$OUT.tmp" "$OUT"

FPU=$(grep -c '\[FPU\]' "$OUT" || true)
INT=$(grep -c '\[INT\]' "$OUT" || true)
FAIL=$(grep -c '\[FAIL\]' "$OUT" || true)
echo "== golden: [FPU]=$FPU [INT]=$INT [FAIL]=$FAIL 总行=$(wc -l < "$OUT") =="
[ "$FAIL" -eq 0 ] || { echo "输出含 [FAIL] 行" >&2; exit 1; }
[ "$FPU" -ge 30 ] || { echo "运行窗口不足（FPU=$FPU，需 ≥30 覆盖多次切换）" >&2; exit 1; }

cp "$OUT" "$FIXTURES/freertos_fpu_golden_output.txt"
echo "== 黄金输出已提交 → $FIXTURES/freertos_fpu_golden_output.txt =="
