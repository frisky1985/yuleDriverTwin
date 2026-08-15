#!/usr/bin/env bash
# =============================================================================
# E2E 验证：dtwin run A9 压力固件 vs QEMU 黄金输出
#
# 用法:
#   scripts/e2e_driver_stress.sh [ELF] [GOLDEN]
#     ELF    固件路径（默认 crates/dtwin-chip/tests/fixtures/build/e2e_driver_stress.elf）
#     GOLDEN QEMU 黄金输出（默认 crates/dtwin-chip/tests/fixtures/e2e_golden_output.txt）
#
# 流程:
#   1. 固件缺失时自动构建（scripts/build_driver_stress.sh）
#   2. dtwin run <ELF> --chip S32K312 --uart-base 0x40004000
#   3. 归一化 \r\n → \n、剔除 [run] 管理行与 QEMU 终止提示行
#   4. 与黄金输出 diff；要求 [PASS] all N checks passed 汇总命中
#   5. 退出码：全量一致 → 0；否则 1
# =============================================================================
set -euo pipefail

# cargo 不在 PATH 时补默认位置
if ! command -v cargo >/dev/null 2>&1 && [ -x "$HOME/.cargo/bin/cargo" ]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIXTURES="$REPO_DIR/crates/dtwin-chip/tests/fixtures"
ELF="${1:-$FIXTURES/build/e2e_driver_stress.elf}"
GOLDEN="${2:-$FIXTURES/e2e_golden_output.txt}"
OUT="$(mktemp /tmp/dtwin_a9_out.XXXXXX)"

if [ ! -f "$ELF" ]; then
  echo "== 固件不存在，自动构建 =="
  "$REPO_DIR/scripts/build_driver_stress.sh"
fi
if [ ! -f "$GOLDEN" ]; then
  echo "错误: 黄金输出不存在: $GOLDEN（先运行 scripts/run_qemu_golden.sh）" >&2
  exit 2
fi

echo "== dtwin run $ELF (chip=S32K312, uart=0x40004000) =="
(cd "$REPO_DIR" && cargo build --quiet)
"$REPO_DIR/target/debug/dtwin" run "$ELF" --chip S32K312 --uart-base 0x40004000 \
  --max-instructions 2000000 >"$OUT" 2>&1 || true

# 归一化：去 \r、剔 [run] 管理行、去 QEMU 终止提示行
sed -e 's/\r$//' "$OUT" | grep -v '^\[run\]' | grep -v '^$' > "$OUT.norm" || true
sed -e 's/\r$//' "$GOLDEN" | grep -v '^qemu-system-arm:' | grep -v '^$' > "$OUT.golden" || true

# 汇总行命中检查
SUMMARY=$(grep -E '^\[PASS\] all ' "$OUT.golden" | head -1 || true)
if [ -n "$SUMMARY" ] && grep -qF -- "$SUMMARY" "$OUT.norm"; then
  echo "== 汇总行命中: $SUMMARY =="
else
  echo "== 缺失汇总行（固件未跑完或输出不一致）==" >&2
  grep -E 'FAIL|Fault' "$OUT" | head -5 >&2 || true
fi

if diff -u "$OUT.golden" "$OUT.norm" >/tmp/dtwin_a9_diff.txt; then
  echo "== 全量输出与 QEMU 黄金输出逐行一致 =="
  rm -f "$OUT" "$OUT.norm" "$OUT.golden"
  exit 0
else
  echo "== 存在差异（见 /tmp/dtwin_a9_diff.txt）==" >&2
  head -30 /tmp/dtwin_a9_diff.txt >&2
  rm -f "$OUT" "$OUT.norm" "$OUT.golden"
  exit 1
fi
