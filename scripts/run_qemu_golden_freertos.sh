#!/usr/bin/env bash
# =============================================================================
# 生成 freertos_demo 固件的 QEMU 黄金输出（FRT-FW-06）
#
#   qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel <elf>
#
# 与 run_qemu_golden.sh 不同：无需 CPACR gdb 补丁——FreeRTOS ARM_CM4F port 的
# vPortStartFirstTask 内嵌 vPortEnableVFP（写 CPACR=0x00F00000）自行使能 FPU，
# 与 dtwin 引擎默认 cpacr 状态一致。
#
# 运行时长：QEMU mps2-an386 的 SysTick 由宿主时间驱动（实测每 tick 指令数
# 48~60 万且逐次不同，见 main_freertos.c 注释）→ 以 RUNTIME_SEC（默认 3s）
# 控制运行窗口，产出 ~3000 tick 的输出（FRT-FW-02 任务集每 tick 打印行数固定，
# 输出序列 tick 计数驱动、跨模拟器可复现）。
#
# 用法：scripts/run_qemu_golden_freertos.sh [ELF] [OUT] [RUNTIME_SEC]
#   ELF   固件路径（默认 crates/dtwin-chip/tests/fixtures/build/freertos_demo.elf）
#   OUT   输出文件（默认 /tmp/freertos_qemu_golden.txt）
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:-$REPO_DIR/crates/dtwin-chip/tests/fixtures/build/freertos_demo.elf}"
OUT="${2:-/tmp/freertos_qemu_golden.txt}"
RUNTIME_SEC="${3:-3}"

if [ ! -f "$ELF" ]; then
  echo "错误: 固件不存在: $ELF（先运行 scripts/build_freertos_demo.sh）" >&2
  exit 2
fi

echo "== QEMU golden run: $ELF（${RUNTIME_SEC}s）=="
qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel "$ELF" > "$OUT" 2>&1 &
QPID=$!
sleep "$RUNTIME_SEC"
kill -TERM "$QPID" 2>/dev/null || true
sleep 1

# 剔除 QEMU 终止提示行与空行 → 黄金输出
grep -v '^qemu-system-arm:' "$OUT" | grep -v '^$' > "$OUT.tmp" && mv "$OUT.tmp" "$OUT"

PASS=$(grep -c '\[PASS\]' "$OUT" || true)
TASK=$(grep -c '\[TASK\]' "$OUT" || true)
TS=$(grep -c '\[TS\]' "$OUT" || true)
SVC=$(grep -c '\[SVC\]' "$OUT" || true)
CRIT=$(grep -c '\[CRIT\]' "$OUT" || true)
FAIL=$(grep -c '\[FAIL\]' "$OUT" || true)
echo "== golden: [PASS]=$PASS [TASK]=$TASK [TS]=$TS [SVC]=$SVC [CRIT]=$CRIT [FAIL]=$FAIL 总行=$(wc -l < "$OUT") → $OUT =="
[ "$FAIL" -eq 0 ]
