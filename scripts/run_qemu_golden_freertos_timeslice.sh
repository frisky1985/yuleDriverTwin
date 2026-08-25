#!/usr/bin/env bash
# =============================================================================
# 生成 freertos_timeslice 变体固件的 QEMU 黄金输出（FRT-AC-02）
#
#   qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel <elf>
#
# 时间片变体 82 行（[PASS] + [TS] B 0..39 + [TS] A 0..39 严格交替）在
# QEMU 宿主时钟节奏下约需 2.5s；RUNTIME_SEC 默认 3s 留足余量。
# 产物复制到 fixtures/freertos_timeslice_golden_output.txt（提交用）。
#
# 用法：scripts/run_qemu_golden_freertos_timeslice.sh [ELF] [OUT] [RUNTIME_SEC]
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
ELF="${1:-$FIXTURES/build/freertos_timeslice.elf}"
OUT="${2:-/tmp/freertos_timeslice_qemu_golden.txt}"
RUNTIME_SEC="${3:-3}"

if [ ! -f "$ELF" ]; then
  echo "错误: 固件不存在: $ELF（先运行 scripts/build_freertos_timeslice.sh）" >&2
  exit 2
fi

echo "== QEMU golden run: $ELF（${RUNTIME_SEC}s）=="
qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel "$ELF" > "$OUT" 2>&1 &
QPID=$!
sleep "$RUNTIME_SEC"
kill -TERM "$QPID" 2>/dev/null || true
sleep 1

grep -v '^qemu-system-arm:' "$OUT" | grep -v '^$' | tr -d '\r' > "$OUT.tmp" && mv "$OUT.tmp" "$OUT"

A=$(grep -c '\[TS\] A' "$OUT" || true)
B=$(grep -c '\[TS\] B' "$OUT" || true)
FAIL=$(grep -c '\[FAIL\]' "$OUT" || true)
echo "== golden: [TS] A=$A [TS] B=$B [FAIL]=$FAIL 总行=$(wc -l < "$OUT") =="
[ "$FAIL" -eq 0 ] || { echo "输出含 [FAIL] 行" >&2; exit 1; }
# 时间片变体每任务固定 40 行（TS_ITERATIONS）；窗口不足则捕获失败
[ "$A" -ge 40 ] && [ "$B" -ge 40 ] || { echo "运行窗口不足（A=$A B=$B，需各 ≥40）" >&2; exit 1; }

cp "$OUT" "$FIXTURES/freertos_timeslice_golden_output.txt"
echo "== 黄金输出已提交 → $FIXTURES/freertos_timeslice_golden_output.txt =="
