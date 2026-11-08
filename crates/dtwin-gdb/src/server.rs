//! GDB RSP 服务端：TCP 监听 + gdbstub 阻塞事件循环。

use std::io;
use std::net::TcpListener;

use gdbstub::common::Signal;
use gdbstub::conn::{Connection, ConnectionExt};
use gdbstub::stub::run_blocking::{BlockingEventLoop, Event, WaitForStopReasonError};
use gdbstub::stub::{DisconnectReason, GdbStub, SingleThreadStopReason};
use gdbstub::target::Target;

use crate::target::{DtwinTarget, RunEvent};
use crate::{GdbServerConfig, GdbServerError};

/// 启动 TCP 监听并等待 GDB 连接（阻塞直到会话结束）
pub fn serve(
    target: &mut DtwinTarget,
    config: &GdbServerConfig,
) -> Result<DisconnectReason, GdbServerError> {
    let listener = TcpListener::bind((config.host.as_str(), config.port))?;
    serve_listener(target, listener)
}

/// 使用已创建的监听器启动 GDB 会话
///
/// 便于测试：调用方可以绑定端口 0，再通过 `local_addr()` 获取实际端口。
pub fn serve_listener(
    target: &mut DtwinTarget,
    listener: TcpListener,
) -> Result<DisconnectReason, GdbServerError> {
    let (stream, _peer) = listener.accept()?;
    stream.set_nodelay(true)?;
    let conn: Box<dyn ConnectionExt<Error = io::Error>> = Box::new(stream);
    GdbStub::new(conn)
        .run_blocking::<DtwinEventLoop>(target)
        .map_err(|e| GdbServerError::Stub(e.to_string()))
}

/// gdbstub 阻塞事件循环：在运行目标与读取 GDB 输入之间轮询。
pub enum DtwinEventLoop {}

impl BlockingEventLoop for DtwinEventLoop {
    type Target = DtwinTarget;
    type Connection = Box<dyn ConnectionExt<Error = io::Error>>;
    type StopReason = SingleThreadStopReason<u32>;

    fn wait_for_stop_reason(
        target: &mut Self::Target,
        conn: &mut Self::Connection,
    ) -> Result<
        Event<Self::StopReason>,
        WaitForStopReasonError<
            <Self::Target as Target>::Error,
            <Self::Connection as Connection>::Error,
        >,
    > {
        // 每执行一小块指令检查一次连接；有输入（通常是 Ctrl-C）就返回给协议层。
        let poll_incoming_data = || conn.peek().map(|b| b.is_some()).unwrap_or(true);

        match target.run(poll_incoming_data) {
            RunEvent::IncomingData => {
                let byte = conn.read().map_err(WaitForStopReasonError::Connection)?;
                Ok(Event::IncomingData(byte))
            }
            RunEvent::Stopped(reason) => Ok(Event::TargetStopped(reason)),
        }
    }

    fn on_interrupt(
        _target: &mut Self::Target,
    ) -> Result<Option<Self::StopReason>, <Self::Target as Target>::Error> {
        Ok(Some(SingleThreadStopReason::Signal(Signal::SIGINT)))
    }
}
