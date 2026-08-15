#!/usr/bin/env bash
# =============================================================================
# 生成 e2e_driver_stress 固件的 QEMU 黄金输出（MPS2 AN386 / Cortex-M4）
#
# 背景：QEMU M-profile 复位后 CPACR=0（FPU 被门控）。真实硬件上 FPU 使能由
# BSP SystemInit 写 CPACR(0xE000ED88)=0x00F00000 完成；dtwin 引擎默认
# cpacr=0x00F00000（FPU 已使能）。本脚本用最小 gdb-remote 客户端在复位后
# 写 CPACR 再继续——扮演 SystemInit 角色，让同一份固件二进制在 QEMU 侧
# FPU 可用，产出与 dtwin 引擎初始状态一致的黄金输出。
#
# 用法：
#   scripts/run_qemu_golden.sh [ELF] [OUT] [PORT]
#     ELF  固件路径（默认 fixtures/build/e2e_driver_stress.elf）
#     OUT  输出文件（默认 /tmp/e2e_qemu_golden.txt）
#     PORT gdb stub 端口（默认 1234）
#
# 附带产出（可选）：OUT 目录下 e2e_in_asm_full.txt（-d in_asm 全迹）
# =============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:-$REPO_DIR/crates/dtwin-chip/tests/fixtures/build/e2e_driver_stress.elf}"
OUT="${2:-/tmp/e2e_qemu_golden.txt}"
PORT="${3:-1234}"
TRACE="${OUT%.txt}_in_asm_full.txt"

GDBSTUB="$(mktemp /tmp/qemu_cpacr_XXXXXX.py)"
cat > "$GDBSTUB" <<'PYEOF'
import socket, sys, time

def recv_packet(sock, timeout=5.0):
    sock.settimeout(timeout); data = b""
    while True:
        b = sock.recv(1)
        if not b: break
        if b == b"#":
            sock.recv(2); break
        data += b
    return data

def send_packet(sock, payload, timeout=5.0):
    csum = f"{sum(payload) & 0xFF:02x}".encode()
    sock.settimeout(timeout)
    sock.sendall(b"$" + payload + b"#" + csum)
    if sock.recv(1) != b"+":
        raise RuntimeError("NAK")
    return recv_packet(sock, timeout)

port = int(sys.argv[1])
s = socket.create_connection(("127.0.0.1", port), timeout=5)
send_packet(s, b"?")                      # 状态查询
payload = f"M{0xE000ED88:08x},{4:x}:" + (0x00F00000).to_bytes(4, "little").hex()
r = send_packet(s, payload.encode())      # 写 CPACR 使能 FPU（地址必须小写十六进制）
assert r == b"$OK", r
try:
    s.settimeout(1.0)
    s.sendall(b"$c#63"); s.recv(1)        # 继续执行
except Exception:
    pass
s.close()
PYEOF

if [ ! -f "$ELF" ]; then
  echo "错误: 固件不存在: $ELF（先运行 scripts/build_driver_stress.sh）" >&2
  exit 2
fi

echo "== QEMU golden run: $ELF =="
qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel "$ELF" \
  -S -gdb tcp::$PORT -d in_asm -D "$TRACE" > "$OUT" 2>&1 &
QPID=$!
sleep 2
python3 "$GDBSTUB" "$PORT" >/dev/null
sleep 10
kill -TERM "$QPID" 2>/dev/null || true
sleep 1
rm -f "$GDBSTUB"

# 剔除 QEMU 进程终止提示行 → 黄金输出
grep -v '^qemu-system-arm:' "$OUT" > "$OUT.tmp" && mv "$OUT.tmp" "$OUT"

PASS=$(grep -c ' PASS' "$OUT" || true)
FAIL=$(grep -c ' FAIL' "$OUT" || true)
echo "== golden: PASS=$PASS FAIL=$FAIL → $OUT =="
echo "== 指令全迹: $TRACE ($(wc -l < "$TRACE") 行) =="
[ "$FAIL" -eq 0 ]
