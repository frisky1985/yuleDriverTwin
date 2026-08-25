#!/usr/bin/env bash
# =============================================================================
# E2E 验证：dtwin run FreeRTOS 镜像 vs QEMU 黄金输出（FRT-FW-07）
#
# 用法:
#   scripts/e2e_freertos.sh [ELF] [GOLDEN]
#     ELF    固件路径（默认 crates/dtwin-chip/tests/fixtures/build/freertos_demo.elf）
#     GOLDEN QEMU 黄金输出（默认 crates/dtwin-chip/tests/fixtures/freertos_golden_output.txt）
#
# 流程:
#   1. 固件缺失时自动构建（scripts/build_freertos_demo.sh）
#   2. 从黄金输出推导 tick 数（TS A/B 最大 seq + 1）；dtwin 以 (ticks+4)×25000
#      指令上限运行（覆盖黄金全部 tick + 处理余量，1 指令=1 周期、25000 周期/tick）
#   3. dtwin run <ELF> --chip S32K312 --uart-base 0x40004000
#   4. 归一化 \r → 剔除 [run] 管理行与空行 → 与黄金逐行前缀对比（黄金行数内 0 差异）
#   5. 退出码：全量一致 → 0；否则 1
#
# 注：QEMU mps2-an386 SysTick 由宿主时间驱动（每 tick 指令数不可复现），黄金
# 输出以 tick 计数为序列驱动（FRT-FW-02 工作量约束），dtwin 侧按同一 tick 数
# 运行 → 序列跨模拟器可复现；对比取黄金行数为前缀（dtwin 覆盖 tick 数 ≥ 黄金）。
# =============================================================================
set -euo pipefail

# cargo 不在 PATH 时补默认位置
if ! command -v cargo >/dev/null 2>&1 && [ -x "$HOME/.cargo/bin/cargo" ]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
ELF="${1:-$FIXTURES/build/freertos_demo.elf}"
GOLDEN="${2:-$FIXTURES/freertos_golden_output.txt}"
OUT="$(mktemp /tmp/dtwin_freertos_out.XXXXXX)"

if [ ! -f "$ELF" ]; then
  echo "== 固件不存在，自动构建 =="
  "$REPO_DIR/scripts/build_freertos_demo.sh"
fi
if [ ! -f "$GOLDEN" ]; then
  echo "错误: 黄金输出不存在: $GOLDEN（先运行 scripts/run_qemu_golden_freertos.sh）" >&2
  exit 2
fi

# 黄金 tick 数 = TS A/B 最大 seq + 1（两任务每 tick 各打印一次）；先剔 \r
GOLDEN_TICKS=$(tr -d '\r' < "$GOLDEN" | grep -E '^\[TS\] [AB]' | awk '{print $3}' | sort -n | tail -1)
GOLDEN_TICKS=$((GOLDEN_TICKS + 1))
MAX_INSTR=$(((GOLDEN_TICKS + 4) * 25000))
echo "== golden ticks=$GOLDEN_TICKS → dtwin max_instructions=$MAX_INSTR =="

echo "== dtwin run $ELF (chip=S32K312, uart=0x40004000) =="
(cd "$REPO_DIR" && cargo build --quiet)
"$REPO_DIR/target/debug/dtwin" run "$ELF" --chip S32K312 --uart-base 0x40004000 \
  --max-instructions "$MAX_INSTR" >"$OUT" 2>&1 || true

# 归一化：去全部 \r、剔 [run] 管理行与空行
sed -e 's/\r$//' "$OUT" | grep -v '^\[run\]' | grep -v '^$' > "$OUT.norm" || true
sed -e 's/\r$//' "$GOLDEN" | grep -v '^qemu-system-arm:' | grep -v '^$' > "$OUT.golden" || true
# （固件 print_line 将 \n 展开为 \r\n、前缀另有 \r → 行尾残留 \r，统一剔除）
tr -d '\r' < "$OUT.norm" > "$OUT.norm2" && mv "$OUT.norm2" "$OUT.norm"
tr -d '\r' < "$OUT.golden" > "$OUT.golden2" && mv "$OUT.golden2" "$OUT.golden"

GOLDEN_LINES=$(wc -l < "$OUT.golden")
DTWIN_LINES=$(wc -l < "$OUT.norm")
echo "== golden=$GOLDEN_LINES 行 / dtwin=$DTWIN_LINES 行（取前缀 $GOLDEN_LINES 对比）=="

# 核心检查行命中（缺一行即失败）
MISSING=0
for pat in '[PASS] freertos demo start' '[TASK] HIGH 0' '[TS] A 0' '[TS] B 0' \
           '[TASK] MID 0' '[TASK] LOW 0' '[SVC] 42' '[CRIT] n=1000'; do
  if ! grep -qF -- "$pat" "$OUT.norm"; then
    echo "== 缺失核心检查行: $pat ==" >&2
    MISSING=$((MISSING + 1))
  fi
done
if grep -qE '^\[FAIL\]' "$OUT.norm"; then
  echo "== 输出含 [FAIL] 行（固件失败钩子触发）==" >&2
  MISSING=$((MISSING + 1))
fi

# 逐行前缀对比（黄金行数内 0 差异）
if head -n "$GOLDEN_LINES" "$OUT.norm" | diff -u "$OUT.golden" - >/tmp/dtwin_freertos_diff.txt; then
  if [ "$MISSING" -eq 0 ]; then
    echo "== 全量输出与 QEMU 黄金输出逐行一致（前缀 ${GOLDEN_LINES} 行）=="
    rm -f "$OUT" "$OUT.norm" "$OUT.golden"
    exit 0
  fi
  echo "== 行序列一致但核心检查行缺失（见上）==" >&2
  rm -f "$OUT" "$OUT.norm" "$OUT.golden"
  exit 1
else
  echo "== 存在差异（见 /tmp/dtwin_freertos_diff.txt）==" >&2
  head -30 /tmp/dtwin_freertos_diff.txt >&2
  rm -f "$OUT" "$OUT.norm" "$OUT.golden"
  exit 1
fi
