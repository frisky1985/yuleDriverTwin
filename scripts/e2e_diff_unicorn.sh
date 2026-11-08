#!/usr/bin/env bash
# =============================================================================
# Thumb-2 指令级差分对拍（dtwin vs Unicorn/QEMU 派生参考模型）
#
# 流程：
#   1. cargo build -p dtwin-cli
#   2. 以既有固件为载体启动 `dtwin debug`（GDB RSP 服务器；固件内容不参与执行，
#      指令流由 harness 经 M/G 报文注入 SRAM 与寄存器）
#   3. 运行 scripts/diff_unicorn.py 锁步对拍
#
# 依赖：python3 + `pip install unicorn`（capstone 可选，用于失配反汇编）
#
# 用法：
#   scripts/e2e_diff_unicorn.sh [COUNT] [SEED] [CLASSES] [PORT]
#     COUNT   用例数（默认 500，每用例 6 条同类指令）
#     SEED    随机种子（默认 1，可复现）
#     CLASSES 指令类（默认 all；tierA / tierB / 逗号分隔类名）
#     PORT    GDB 监听端口（默认 33335）
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v cargo >/dev/null 2>&1 && [ -x "$HOME/.cargo/bin/cargo" ]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

COUNT="${1:-500}"
SEED="${2:-1}"
CLASSES="${3:-all}"
# 默认随机高端口，避免残留进程占用导致 bind 冲突
PORT="${4:-$(( (RANDOM % 20000) + 20000 ))}"
HOST="127.0.0.1"
ELF="$REPO_DIR/crates/dtwin-core/tests/fixtures/yuleasr_qemu.elf.dat"
LOG=/tmp/dtwin_diff.log

python3 -c 'import unicorn' 2>/dev/null || {
  echo "错误: 缺少 unicorn（pip install unicorn）" >&2; exit 2; }
[ -f "$ELF" ] || { echo "错误: 载体固件不存在: $ELF" >&2; exit 2; }

echo "== 构建 dtwin CLI =="
(cd "$REPO_DIR" && cargo build --quiet -p dtwin-cli)
DTWIN="$REPO_DIR/target/debug/dtwin"

echo "== 启动 dtwin debug (port=$PORT) =="
"$DTWIN" debug "$ELF" --chip S32K312 --host "$HOST" --port "$PORT" >"$LOG" 2>&1 &
DTWIN_PID=$!
trap 'kill -TERM "$DTWIN_PID" 2>/dev/null || true' EXIT

for _ in $(seq 1 50); do
  if ss -ltnH "sport = :$PORT" 2>/dev/null | grep -q ":$PORT"; then
    break
  fi
  sleep 0.1
done
ss -ltnH "sport = :$PORT" 2>/dev/null | grep -q ":$PORT" || {
  echo "错误: dtwin GDB 服务器未在 $HOST:$PORT 就绪" >&2
  cat "$LOG" >&2
  exit 1
}

set +e
python3 "$REPO_DIR/scripts/diff_unicorn.py" \
  --host "$HOST" --port "$PORT" \
  --count "$COUNT" --seed "$SEED" --classes "$CLASSES"
RC=$?
set -e

if [ "$RC" -eq 0 ]; then
  echo "== 差分对拍通过 =="
else
  echo "== 差分对拍发现失配（rc=$RC），详见上方报告 ==" >&2
fi
exit "$RC"
