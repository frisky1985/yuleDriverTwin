//! GDB RSP 端到端测试：真实 TCP + 原始 RSP 报文（不依赖本机 GDB 客户端）。

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::thread;
use std::time::Duration;

use dtwin_core::memory::Memory;
use dtwin_core::nvic::Nvic;
use dtwin_core::CpuState;
use dtwin_gdb::{serve_listener, DtwinTarget};

fn build_target() -> DtwinTarget {
    let mut mem = Memory::test_ram();
    // NOP; NOP; NOP; B .
    mem.load_bytes(0, &[0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0xFE, 0xE7])
        .unwrap();

    let mut cpu = CpuState::default();
    cpu.regs[13] = 0x2000_0F00;
    cpu.msp = 0x2000_0F00;
    cpu.regs[15] = 0;
    cpu.xpsr = 1 << 24;

    DtwinTarget::new(cpu, mem, Nvic::new())
}

fn send_packet(stream: &mut TcpStream, payload: &str) {
    let checksum = payload.bytes().fold(0u8, |acc, b| acc.wrapping_add(b));
    write!(stream, "${payload}#{checksum:02x}").unwrap();
    stream.flush().unwrap();
}

fn recv_packet(stream: &mut TcpStream) -> String {
    loop {
        let mut byte = [0u8; 1];
        stream.read_exact(&mut byte).unwrap();
        match byte[0] {
            b'$' => {}
            b'+' | b'-' => continue,
            _ => continue,
        }

        let mut payload = Vec::new();
        loop {
            stream.read_exact(&mut byte).unwrap();
            if byte[0] == b'#' {
                break;
            }
            payload.push(byte[0]);
        }

        let mut checksum = [0u8; 2];
        stream.read_exact(&mut checksum).unwrap();
        stream.write_all(b"+").unwrap();
        stream.flush().unwrap();

        return String::from_utf8_lossy(&decode_rle(&payload)).into_owned();
    }
}

/// GDB RSP run-length encoding：`*` 后跟 n 表示重复前一字符 (n - 29) 次（按 8 位回绕）。
fn decode_rle(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len() * 2);
    let mut i = 0;
    while i < payload.len() {
        if payload[i] == b'*' && !out.is_empty() && i + 1 < payload.len() {
            let count = payload[i + 1].wrapping_sub(29);
            let last = *out.last().unwrap();
            for _ in 0..count {
                out.push(last);
            }
            i += 2;
        } else {
            out.push(payload[i]);
            i += 1;
        }
    }
    out
}

#[test]
fn rsp_breakpoint_continue_and_detach() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();

    let server = thread::spawn(move || {
        let mut target = build_target();
        let reason = serve_listener(&mut target, listener).unwrap();
        (reason, target.stats().instructions)
    });

    let mut stream = TcpStream::connect(addr).unwrap();
    stream
        .set_read_timeout(Some(Duration::from_secs(10)))
        .unwrap();
    stream
        .set_write_timeout(Some(Duration::from_secs(10)))
        .unwrap();

    send_packet(
        &mut stream,
        "qSupported:multiprocess+;swbreak+;hwbreak+;qRelocInsn+;xmlRegisters=arm",
    );
    let caps = recv_packet(&mut stream);
    assert!(
        caps.contains("PacketSize") || caps.contains("swbreak"),
        "qSupported 响应异常: {caps}"
    );

    send_packet(&mut stream, "?");
    let stop = recv_packet(&mut stream);
    assert!(
        stop.starts_with('T') || stop.starts_with('S'),
        "初始停止包异常: {stop}"
    );

    send_packet(&mut stream, "g");
    let regs = recv_packet(&mut stream);
    assert_eq!(
        regs.len(),
        42 * 4 * 2,
        "ArmCoreRegs 寄存器包长度不符: {regs}"
    );

    send_packet(&mut stream, "m20000000,4");
    assert_eq!(recv_packet(&mut stream), "00000000");

    send_packet(&mut stream, "M20000000,4:01020304");
    assert_eq!(recv_packet(&mut stream), "OK");
    send_packet(&mut stream, "m20000000,4");
    assert_eq!(recv_packet(&mut stream), "01020304");

    send_packet(&mut stream, "Z0,4,2");
    assert_eq!(recv_packet(&mut stream), "OK");

    send_packet(&mut stream, "c");
    let stop = recv_packet(&mut stream);
    assert!(stop.contains("05"), "continue 未停在断点: {stop}");

    send_packet(&mut stream, "g");
    let regs = recv_packet(&mut stream);
    let pc_bytes = (0..4)
        .map(|i| u8::from_str_radix(&regs[120 + i * 2..122 + i * 2], 16).unwrap())
        .collect::<Vec<_>>();
    let pc = u32::from_le_bytes([pc_bytes[0], pc_bytes[1], pc_bytes[2], pc_bytes[3]]);
    assert_eq!(pc, 4, "断点命中后 PC 应为 4");

    send_packet(&mut stream, "z0,4,2");
    assert_eq!(recv_packet(&mut stream), "OK");

    send_packet(&mut stream, "D");
    let detach = recv_packet(&mut stream);
    assert!(detach.starts_with("OK"), "detach 响应异常: {detach}");
    drop(stream);

    let (reason, instructions) = server.join().unwrap();
    assert!(
        matches!(reason, gdbstub::stub::DisconnectReason::Disconnect),
        "会话应正常断开: {reason:?}"
    );
    assert_eq!(instructions, 2, "命中断点前应执行 2 条指令");
}
