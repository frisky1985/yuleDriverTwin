#!/usr/bin/env bash
# =============================================================================
# GDB FPU 支持验证脚本
#
# 验证 dtwin-gdb 暴露的 Cortex-M4F target.xml 与原生 FPU / 系统寄存器支持：
#   - target.xml 包含 m-profile / m-system / vfp 三个特性
#   - FPU 寄存器 D0-D15 / FPSCR（GDB 据此合成 S0-S31）可作为原生寄存器读写
#   - 系统寄存器 xPSR / MSP / PSP / PRIMASK / BASEPRI / FAULTMASK / CONTROL 可访问
#   - `g` 报文长度 = 224 字节（448 hex），PC 在 hex 偏移 120..128
#
# 用法:
#   scripts/verify_gdb_fpu.sh [ELF] [PORT]
#     ELF   固件路径（默认 crates/dtwin-core/tests/fixtures/yuleasr_qemu.elf.dat）
#     PORT  GDB 监听端口（默认 33333）
#
# 执行模式：
#   1. 若本机有 arm-none-eabi-gdb / gdb-multiarch，使用真实 GDB 连接并校验
#      `info registers`、`print $s0/$d0/$fpscr`、`set` 回环。
#   2. 否则自动降级为 Python 原始 RSP 客户端，在协议层校验 target.xml 与
#      单寄存器读写（不依赖 ARM GDB，沙箱内可直接运行）。
# =============================================================================
set -euo pipefail

if ! command -v cargo >/dev/null 2>&1 && [ -x "$HOME/.cargo/bin/cargo" ]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="${1:-$REPO_DIR/crates/dtwin-core/tests/fixtures/yuleasr_qemu.elf.dat}"
PORT="${2:-33333}"
HOST="127.0.0.1"

PASS=0
FAIL=0
log_pass() { echo "  [PASS] $1"; PASS=$((PASS + 1)); }
log_fail() { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); }

[ -f "$ELF" ] || { echo "错误: 固件不存在: $ELF" >&2; exit 2; }

echo "== 构建 dtwin CLI =="
(cd "$REPO_DIR" && cargo build --quiet -p dtwin-cli)
DTWIN="$REPO_DIR/target/debug/dtwin"

echo "== 启动 dtwin debug (port=$PORT) =="
"$DTWIN" debug "$ELF" --chip S32K312 --host "$HOST" --port "$PORT" >/tmp/dtwin_gdb.log 2>&1 &
SERVER_PID=$!
trap 'kill $SERVER_PID 2>/dev/null || true' EXIT

# 等待 GDB 端口就绪。
# 注意：不能用 connect 探测——任何连入都会被 serve() 当作 GDB 会话，
# 立即断开会导致会话提前结束。改用 ss 检查 LISTEN 状态。
READY=0
for i in $(seq 1 50); do
  if ss -ltnH "sport = :$PORT" 2>/dev/null | grep -q ":$PORT"; then
    READY=1
    break
  fi
  sleep 0.1
done
if [ "$READY" -ne 1 ]; then
  echo "错误: dtwin GDB 服务器未在 $HOST:$PORT 就绪" >&2
  cat /tmp/dtwin_gdb.log >&2
  exit 2
fi
echo "  服务器已就绪 (pid=$SERVER_PID)"

# 检测可用的 ARM GDB
GDB_BIN=""
for cand in arm-none-eabi-gdb gdb-multiarch; do
  if command -v "$cand" >/dev/null 2>&1; then
    GDB_BIN="$cand"
    break
  fi
done

if [ -n "$GDB_BIN" ]; then
  echo "== 使用真实 GDB 验证: $GDB_BIN =="
  GDB_OUT="$(mktemp /tmp/gdb_fpu_out.XXXXXX)"

  cat > /tmp/gdb_fpu_cmds.gdb <<GDBEOF
set pagination off
set confirm off
target remote $HOST:$PORT
echo === target-description ===
maint print target-description
echo === info registers (fpu + system) ===
info registers d0 d1 d15 fpscr s0 s1 s31
info registers xpsr msp psp primask basepri faultmask control
echo === fpu read ===
print \$s0
print \$d0
print \$fpscr
echo === fpu write/read roundtrip ===
set \$s0 = 1.5
print \$s0
set \$d0 = 2.25
print \$d0
echo === system regs ===
print \$xpsr
print \$msp
print \$psp
disconnect
quit
GDBEOF

  "$GDB_BIN" -batch -nx -x /tmp/gdb_fpu_cmds.gdb "$ELF" >"$GDB_OUT" 2>&1 || true

  if grep -q 'org.gnu.gdb.arm.vfp' "$GDB_OUT"; then
    log_pass "target.xml 包含 org.gnu.gdb.arm.vfp 特性"
  else
    log_fail "target.xml 缺少 org.gnu.gdb.arm.vfp 特性"
  fi
  if grep -q 'org.gnu.gdb.arm.m-system' "$GDB_OUT"; then
    log_pass "target.xml 包含 org.gnu.gdb.arm.m-system 特性"
  else
    log_fail "target.xml 缺少 org.gnu.gdb.arm.m-system 特性"
  fi

  if grep -qiE 'd0[[:space:]]+0x' "$GDB_OUT"; then
    log_pass "info registers 能显示 d0"
  else
    log_fail "info registers 无法显示 d0"
  fi
  if grep -qiE 'fpscr[[:space:]]+0x' "$GDB_OUT"; then
    log_pass "info registers 能显示 fpscr"
  else
    log_fail "info registers 无法显示 fpscr"
  fi

  if grep -qE '\$s0 = 1\.5' "$GDB_OUT"; then
    log_pass "set \$s0=1.5 后 print \$s0 返回 1.5"
  else
    log_fail "S0 写读回环失败（期望 1.5）"
  fi
  if grep -qE '\$d0 = 2\.25' "$GDB_OUT"; then
    log_pass "set \$d0=2.25 后 print \$d0 返回 2.25"
  else
    log_fail "D0 写读回环失败（期望 2.25）"
  fi

  if grep -qE '\$xpsr = ' "$GDB_OUT"; then
    log_pass "print \$xpsr 可访问"
  else
    log_fail "print \$xpsr 失败"
  fi
  if grep -qE '\$msp = ' "$GDB_OUT"; then
    log_pass "print \$msp 可访问"
  else
    log_fail "print \$msp 失败"
  fi

  echo "---- GDB 输出 ----"
  cat "$GDB_OUT"
  rm -f "$GDB_OUT" /tmp/gdb_fpu_cmds.gdb
else
  echo "== 未检测到 arm-none-eabi-gdb，降级为 Python 原始 RSP 验证 =="
  # Python 在子进程中运行，其 PASS/FAIL 不影响 bash 变量；
  # 捕获输出并解析 "__SUMMARY__ PASS=N FAIL=M" 行汇总到 bash 计数器。
  PY_SCRIPT="$(mktemp /tmp/rsp_verify.XXXXXX.py)"
  cat > "$PY_SCRIPT" <<'PYEOF'
import socket, struct, sys

host, port = sys.argv[1], int(sys.argv[2])
PASS = FAIL = 0
def ok(m):
    global PASS; PASS += 1; print(f"  [PASS] {m}")
def bad(m):
    global FAIL; FAIL += 1; print(f"  [FAIL] {m}")

def enc(payload: bytes) -> bytes:
    return b'$' + payload + b'#' + f'{sum(payload) & 0xFF:02x}'.encode()

def dec_rle(data: bytes) -> bytes:
    out = bytearray(); i = 0
    while i < len(data):
        if data[i:i+1] == b'*' and out and i+1 < len(data):
            out.extend([out[-1]] * ((data[i+1] - 29) & 0xFF)); i += 2
        else:
            out.append(data[i]); i += 1
    return bytes(out)

def recv(s):
    while True:
        b = s.recv(1)
        if not b: return None
        if b == b'$': break
    p = bytearray()
    while True:
        b = s.recv(1)
        if b == b'#': break
        p += b
    s.recv(2); s.sendall(b'+')
    return dec_rle(bytes(p))

s = socket.socket(); s.settimeout(5)
s.connect((host, port))

s.sendall(enc(b'qSupported:xmlRegisters=arm')); recv(s)
s.sendall(enc(b'?')); recv(s)

# 1. target.xml 三个特性
s.sendall(enc(b'qXfer:features:read:target.xml:0,fff'))
xml = recv(s)
for feat in (b'org.gnu.gdb.arm.m-profile', b'org.gnu.gdb.arm.m-system', b'org.gnu.gdb.arm.vfp'):
    if feat in xml: ok(f"target.xml 含 {feat.decode()}")
    else: bad(f"target.xml 缺 {feat.decode()}")

# 2. 关键寄存器编号
for needle in (b'name="xpsr" bitsize="32" regnum="25"',
               b'name="msp" bitsize="32" regnum="26"',
               b'name="d0" bitsize="64" regnum="32"',
               b'name="fpscr" bitsize="32" regnum="48"'):
    if needle in xml: ok(f"寄存器编号正确: {needle.decode()}")
    else: bad(f"寄存器编号缺失: {needle.decode()}")

# 3. g 报文长度 224 字节
s.sendall(enc(b'g'))
regs = recv(s)
if len(regs) == 448: ok("g 报文长度 448 hex（224 字节）")
else: bad(f"g 报文长度异常: {len(regs)}（期望 448）")

# 4. PC 在 hex 偏移 120..128
pc = struct.unpack('<I', bytes.fromhex(regs[120:128].decode()))[0]
if pc & 1 == 0: ok(f"PC 半字对齐: {pc:#010x}")
else: bad(f"PC 未半字对齐: {pc:#010x}")

# 5. 单寄存器读：xpsr(25) / msp(26) / d0(32) / fpscr(48)
s.sendall(enc(b'p19'))  # 0x19 = 25
xpsr = recv(s)
if len(xpsr) == 8: ok(f"读 xpsr(25): {xpsr.decode()}")
else: bad(f"读 xpsr 长度异常: {len(xpsr)}")

s.sendall(enc(b'p1a'))  # 26
msp = recv(s)
if len(msp) == 8: ok(f"读 msp(26): {msp.decode()}")
else: bad(f"读 msp 长度异常: {len(msp)}")

s.sendall(enc(b'p20'))  # 32
d0 = recv(s)
if len(d0) == 16: ok(f"读 d0(32) 返回 8 字节: {d0.decode()}")
else: bad(f"读 d0 长度异常: {len(d0)}（期望 16 hex）")

s.sendall(enc(b'p30'))  # 48
fpscr = recv(s)
if len(fpscr) == 8: ok(f"读 fpscr(48): {fpscr.decode()}")
else: bad(f"读 fpscr 长度异常: {len(fpscr)}")

# 6. 单寄存器写读回环：写 d0(32) = 0x40020000_3f800000 (d=2.25, s0=1.0)
val = struct.pack('<Q', 0x40020000_3f800000).hex().encode()
s.sendall(enc(b'P20=' + val))
ack = recv(s)
if ack == b'OK':
    s.sendall(enc(b'p20'))
    d0_back = recv(s)
    if d0_back == val: ok("d0 写读回环一致")
    else: bad(f"d0 回环不一致: 写 {val.decode()} 读 {d0_back.decode()}")
else:
    bad(f"写 d0 未返回 OK: {ack!r}")

# 7. 写 s0 等价于写 d0 低半字：写 d0 低字 = 0x3fc00000 (1.5)
val2 = struct.pack('<Q', 0x00000000_3fc00000).hex().encode()
s.sendall(enc(b'P20=' + val2))
recv(s)
s.sendall(enc(b'p20'))
d0_back2 = recv(s)
if d0_back2 == val2: ok("写 d0 低半字（对应 s0）回环一致")
else: bad(f"d0 低半字回环不一致: {d0_back2.decode()}")

s.close()
print(f"__SUMMARY__ PASS={PASS} FAIL={FAIL}")
sys.exit(1 if FAIL else 0)
PYEOF
  set +e
  PY_OUT=$(python3 "$PY_SCRIPT" "$HOST" "$PORT")
  PY_RC=$?
  set -e
  rm -f "$PY_SCRIPT"
  echo "$PY_OUT"
  SUMMARY_LINE=$(echo "$PY_OUT" | grep '__SUMMARY__' || true)
  PY_PASS=$(echo "$SUMMARY_LINE" | grep -o 'PASS=[0-9]*' | cut -d= -f2)
  PY_FAIL=$(echo "$SUMMARY_LINE" | grep -o 'FAIL=[0-9]*' | cut -d= -f2)
  PASS=$((PASS + ${PY_PASS:-0}))
  FAIL=$((FAIL + ${PY_FAIL:-0}))
fi

echo
echo "== 汇总: PASS=$PASS FAIL=$FAIL =="
if [ "$FAIL" -gt 0 ]; then
  echo "== 验证失败 =="
  exit 1
fi
echo "== 验证通过 =="
exit 0
