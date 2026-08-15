#!/usr/bin/env bash
# =============================================================================
# E2E 打通验证：dtwin run 真实 yuleASR 固件 vs QEMU 黄金输出
#
# 用法:
#   scripts/e2e_yuleasr.sh [ELF] [GOLDEN]
#     ELF    固件路径（默认 ~/.openclaw/workspace/yuleASR/qemu/build/yuleasr_qemu.elf）
#     GOLDEN QEMU 黄金输出（默认 /tmp/qemu_golden_output.txt）
#
# 流程:
#   1. cargo build（release 可选，默认 debug）
#   2. dtwin run <ELF> --chip S32K312 --uart-base 0x40004000
#      （yuleASR QEMU 兼容固件经 qemu_s32k312_compat.h 把 LPUART0 重定向到
#       CMSDK APB UART 0x40004000，故 dtwin 侧 UART 模型须挂在该地址）
#   3. 归一化 \r\n → \n、剔除 [run] 头尾行与 QEMU 终止提示行
#   4. 与黄金输出 diff；统计 [CHECK]/[MCU]/[PORT]/[DIO]/[BSW]/[MEM]/[PASS] 命中数
#   5. 退出码：全量一致 + 核心检查行全命中 → 0；否则 1
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:-$HOME/.openclaw/workspace/yuleASR/qemu/build/yuleasr_qemu.elf}"
GOLDEN="${2:-/tmp/qemu_golden_output.txt}"
OUT="$(mktemp /tmp/dtwin_e2e_out.XXXXXX)"
NORM_OUT="$(mktemp /tmp/dtwin_e2e_norm.XXXXXX)"
NORM_GOLDEN="$(mktemp /tmp/dtwin_e2e_golden.XXXXXX)"

if [ ! -f "$ELF" ]; then
  echo "错误: 固件不存在: $ELF" >&2
  exit 2
fi
if [ ! -f "$GOLDEN" ]; then
  echo "错误: 黄金输出不存在: $GOLDEN" >&2
  exit 2
fi

echo "== dtwin run $ELF (chip=S32K312, uart=0x40004000) =="
(cd "$REPO_DIR" && cargo build --quiet)
"$REPO_DIR/target/debug/dtwin" run "$ELF" --chip S32K312 --uart-base 0x40004000 \
  --max-instructions 2000000 >"$OUT" 2>&1 || true

# 归一化：去 \r、剔 [run] 管理行、去 QEMU 终止提示行
sed -e 's/\r$//' "$OUT" | grep -v '^\[run\]' | grep -v '^$' >"$NORM_OUT" || true
sed -e 's/\r$//' "$GOLDEN" | grep -v '^qemu-system-arm:' | grep -v '^$' >"$NORM_GOLDEN" || true

# 核心检查行命中统计
TOTAL=$(grep -cE '^\[(CHECK|MCU|PORT|DIO|BSW|MEM|PASS)\]' "$NORM_GOLDEN" || true)
HIT=0
MISS=0
while IFS= read -r line; do
  if grep -qF -- "$line" "$NORM_OUT"; then
    HIT=$((HIT + 1))
  else
    echo "缺失: $line" >&2
    MISS=$((MISS + 1))
  fi
done < <(grep -E '^\[(CHECK|MCU|PORT|DIO|BSW|MEM|PASS)\]' "$NORM_GOLDEN" || true)

echo "== 核心检查行: $HIT/$TOTAL 命中 (缺失 $MISS) =="

if diff -u "$NORM_GOLDEN" "$NORM_OUT" >/tmp/dtwin_e2e_diff.txt; then
  echo "== 全量输出与 QEMU 黄金输出逐行一致 =="
  rm -f "$OUT" "$NORM_OUT" "$NORM_GOLDEN"
  exit 0
else
  echo "== 存在差异（见 /tmp/dtwin_e2e_diff.txt）==" >&2
  head -40 /tmp/dtwin_e2e_diff.txt >&2
  rm -f "$OUT" "$NORM_OUT" "$NORM_GOLDEN"
  exit 1
fi
