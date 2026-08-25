#!/usr/bin/env bash
# =============================================================================
# E2E 验证：时间片轮转变体固件（FRT-AC-02）
#
# 用法:
#   scripts/e2e_freertos_timeslice.sh [ELF] [GOLDEN]
#     ELF    固件路径（默认 fixtures/build/freertos_timeslice.elf）
#     GOLDEN QEMU 黄金输出（默认 fixtures/freertos_timeslice_golden_output.txt）
#
# 断言（标准不弱化）：
#   1. dtwin 输出与 QEMU 黄金输出归一化逐行一致（前缀 0 差异）
#   2. [TS] 两任务严格交替（B,A,B,A…；每任务恰 40 行）——时间片旋转触发证据
#   3. 时间片开关对照：noslice 固件（configUSE_TIME_SLICING=0，同源码仅此
#      一处配置差异）输出恒为 [PASS] + [TS] B 0，无任何 [TS] A 行——
#      证明交替由 configUSE_TIME_SLICING 的时间片旋转引起而非其他机制
#   4. 无 [FAIL] 行、引擎 faults=0
#
# 忙循环确定性：轮次忙等为内存旋转（volatile 读），不依赖 SysTick 寄存器
# 可观测性（COUNTFLAG 相位在 QEMU/dtwin 间的差异不进入判定），双跑可复现。
# =============================================================================
set -euo pipefail

if ! command -v cargo >/dev/null 2>&1 && [ -x "$HOME/.cargo/bin/cargo" ]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
ELF="${1:-$FIXTURES/build/freertos_timeslice.elf}"
GOLDEN="${2:-$FIXTURES/freertos_timeslice_golden_output.txt}"
NOSLICE="${3:-$FIXTURES/build/freertos_timeslice_noslice.elf}"
OUT="$(mktemp /tmp/dtwin_ts_out.XXXXXX)"
OUT2="$(mktemp /tmp/dtwin_ts_ns_out.XXXXXX)"

if [ ! -f "$ELF" ]; then
  echo "== 固件不存在，自动构建 =="
  "$REPO_DIR/scripts/build_freertos_timeslice.sh"
fi
[ -f "$GOLDEN" ] || { echo "错误: 黄金输出不存在: $GOLDEN（先运行 scripts/run_qemu_golden_freertos_timeslice.sh）" >&2; exit 2; }
[ -f "$NOSLICE" ] || { echo "错误: noslice 对照固件不存在: $NOSLICE（先运行 scripts/build_freertos_timeslice.sh）" >&2; exit 2; }

# 时间片变体 82 行（2×40+2）在 ~82 ticks 内完成；+8 tick 余量
MAX_INSTR=$(((40 * 2 + 8) * 25000))
echo "== dtwin run $ELF (max_instructions=$MAX_INSTR) =="
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
if ! head -n "$GOLDEN_LINES" "$OUT.norm" | diff -u "$OUT.golden" - >/tmp/dtwin_ts_diff.txt; then
  echo "== 存在差异（见 /tmp/dtwin_ts_diff.txt）==" >&2
  head -20 /tmp/dtwin_ts_diff.txt >&2
  exit 1
fi

# ---- 断言 2：严格交替 B,A,B,A…（每任务恰 40 行）----
A_COUNT=$(grep -c '^\[TS\] A' "$OUT.norm" || true)
B_COUNT=$(grep -c '^\[TS\] B' "$OUT.norm" || true)
[ "$A_COUNT" -eq 40 ] || { echo "断言失败: [TS] A 行数=$A_COUNT ≠ 40" >&2; exit 1; }
[ "$B_COUNT" -eq 40 ] || { echo "断言失败: [TS] B 行数=$B_COUNT ≠ 40" >&2; exit 1; }
# 严格交替：第 2k 行恒为 B、第 2k+1 行恒为 A（awk ERE，规避 BSD grep 分组量词怪癖）
grep '^\[TS\]' "$OUT.norm" | awk '
  NR % 2 == 1 { if ($2 != "B") bad++ }
  NR % 2 == 0 { if ($2 != "A") bad++ }
  END { exit (bad == 0 && NR == 80) ? 0 : 1 }
' || { echo "断言失败: [TS] 序列非严格 B,A 交替（共 $(grep -c '^\[TS\]' "$OUT.norm") 行）" >&2; exit 1; }

# ---- 断言 3：时间片开关对照（noslice）----
"$REPO_DIR/target/debug/dtwin" run "$NOSLICE" --chip S32K312 --uart-base 0x40004000 \
  --max-instructions "$MAX_INSTR" >"$OUT2" 2>&1 || true
tr -d '\r' < "$OUT2" | grep -v '^\[run\]' | grep -v '^$' > "$OUT2.norm" || true
NS_A=$(grep -c '^\[TS\] A' "$OUT2.norm" || true)
NS_B=$(grep -c '^\[TS\] B' "$OUT2.norm" || true)
[ "$NS_A" -eq 0 ] || { echo "断言失败: noslice 对照出现 [TS] A 行（无时间片时第二任务不应运行）" >&2; exit 1; }
[ "$NS_B" -eq 1 ] || { echo "断言失败: noslice 对照 [TS] B 行数=$NS_B ≠ 1（应仅首任务首行）" >&2; exit 1; }
grep -qF '[TS] B 0' "$OUT2.norm" || { echo "断言失败: noslice 对照缺失 [TS] B 0" >&2; exit 1; }

# ---- 断言 4：无失败行 / 无 fault ----
if grep -qE '^\[FAIL\]' "$OUT.norm"; then echo "== 输出含 [FAIL] 行 ==" >&2; exit 1; fi
if grep -qE 'faults=[1-9]' "$OUT"; then echo "== 引擎产生 fault ==" >&2; exit 1; fi

echo "== FRT-AC-02 通过：交替序列与 QEMU 黄金逐行一致（${GOLDEN_LINES} 行）；noslice 对照无交替 =="
rm -f "$OUT" "$OUT.norm" "$OUT.golden" "$OUT2" "$OUT2.norm"
exit 0
