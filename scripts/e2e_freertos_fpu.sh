#!/usr/bin/env bash
# =============================================================================
# E2E 验证：FPU 场景 B 变体固件（FRT-AC-07）
#
# 用法:
#   scripts/e2e_freertos_fpu.sh [ELF] [GOLDEN]
#     ELF    固件路径（默认 fixtures/build/freertos_fpu.elf）
#     GOLDEN QEMU 黄金输出（默认 fixtures/freertos_fpu_golden_output.txt）
#
# 断言（标准不弱化）：
#   1. dtwin 输出与 QEMU 黄金输出归一化逐行一致（前缀 0 差异）——浮点累计
#      （VADD/VFMA/VCVT 工作负载，跨多次 PendSV 上下文切换存活）两侧逐位一致
#   2. 引擎 FPU 扩展帧统计 > 0：fpu_frames（异常入口压 S0-S15+FPSCR、
#      EXC_RETURN=FPU 变体）与 fpu_exc_returns（FPU 变体返回）被真实触发
#   3. 无 [FAIL] 行、引擎 faults=0
# =============================================================================
set -euo pipefail

if ! command -v cargo >/dev/null 2>&1 && [ -x "$HOME/.cargo/bin/cargo" ]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
ELF="${1:-$FIXTURES/build/freertos_fpu.elf}"
GOLDEN="${2:-$FIXTURES/freertos_fpu_golden_output.txt}"
OUT="$(mktemp /tmp/dtwin_fpu_out.XXXXXX)"

if [ ! -f "$ELF" ]; then
  echo "== 固件不存在，自动构建 =="
  "$REPO_DIR/scripts/build_freertos_fpu.sh"
fi
[ -f "$GOLDEN" ] || { echo "错误: 黄金输出不存在: $GOLDEN（先运行 scripts/run_qemu_golden_freertos_fpu.sh）" >&2; exit 2; }

# 黄金 tick 数 ≈ [FPU] 行数（浮点任务 delay(1) 每 tick 唤醒一次）；+8 tick 余量
GOLDEN_TICKS=$(grep -c '^\[FPU\]' "$GOLDEN" || true)
MAX_INSTR=$(((GOLDEN_TICKS + 8) * 25000))
echo "== golden ticks≈$GOLDEN_TICKS → dtwin max_instructions=$MAX_INSTR =="

echo "== dtwin run $ELF (chip=S32K312, uart=0x40004000) =="
(cd "$REPO_DIR" && cargo build --quiet)
"$REPO_DIR/target/debug/dtwin" run "$ELF" --chip S32K312 --uart-base 0x40004000 \
  --max-instructions "$MAX_INSTR" >"$OUT" 2>&1 || true

tr -d '\r' < "$OUT" | grep -v '^\[run\]' | grep -v '^$' > "$OUT.norm" || true
tr -d '\r' < "$GOLDEN" | grep -v '^qemu-system-arm:' | grep -v '^$' > "$OUT.golden" || true

GOLDEN_LINES=$(wc -l < "$OUT.golden")
DTWIN_LINES=$(wc -l < "$OUT.norm")
echo "== golden=$GOLDEN_LINES 行 / dtwin=$DTWIN_LINES 行（取前缀 $GOLDEN_LINES 对比）=="
[ "$DTWIN_LINES" -ge "$GOLDEN_LINES" ] || { echo "dtwin 行数不足（$DTWIN_LINES < $GOLDEN_LINES）" >&2; exit 1; }

# ---- 断言 1：逐行一致（前缀）----
if ! head -n "$GOLDEN_LINES" "$OUT.norm" | diff -u "$OUT.golden" - >/tmp/dtwin_fpu_diff.txt; then
  echo "== 存在差异（见 /tmp/dtwin_fpu_diff.txt）==" >&2
  head -20 /tmp/dtwin_fpu_diff.txt >&2
  exit 1
fi

# ---- 断言 2：FPU 扩展帧统计（EXC_RETURN FPU 变体真实触发）----
STATS=$(grep '^\[run\] 结果' "$OUT" | head -1 || true)
FPU_FRAMES=$(echo "$STATS" | grep -o 'fpu_frames=[0-9]*' | cut -d= -f2 || true)
FPU_RETURNS=$(echo "$STATS" | grep -o 'fpu_exc_returns=[0-9]*' | cut -d= -f2 || true)
[ "${FPU_FRAMES:-0}" -gt 0 ] || { echo "断言失败: fpu_frames=$FPU_FRAMES（FPU 扩展帧未被触发）" >&2; exit 1; }
[ "${FPU_RETURNS:-0}" -gt 0 ] || { echo "断言失败: fpu_exc_returns=$FPU_RETURNS（FPU 变体返回未被触发）" >&2; exit 1; }
echo "== FPU 扩展帧统计: fpu_frames=$FPU_FRAMES fpu_exc_returns=$FPU_RETURNS =="

# ---- 断言 3：无失败行 / 无 fault ----
if grep -qE '^\[FAIL\]' "$OUT.norm"; then echo "== 输出含 [FAIL] 行 ==" >&2; exit 1; fi
if grep -qE 'faults=[1-9]' "$OUT"; then echo "== 引擎产生 fault ==" >&2; exit 1; fi

echo "== FRT-AC-07 通过：FPU 累计与 QEMU 黄金逐行一致（${GOLDEN_LINES} 行）；FPU 变体上下文切换真实触发 =="
rm -f "$OUT" "$OUT.norm" "$OUT.golden"
exit 0
