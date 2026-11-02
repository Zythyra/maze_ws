"""Read a one-shot start command without blocking the ROS executor.

Launch passes the parent's tty pathname because ROS launch owns child stdin
and its subprocesses may not have a controlling terminal.
"""
import os
import select


class TerminalStartGate:
    def __init__(self, device, notify, service_name='/polyline/set_enabled'):
        self.fd = None
        self.pending = b''
        self.notify = notify
        try:
            self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
            if not os.isatty(self.fd):
                raise OSError('input device is not a terminal')
        except OSError as error:
            self.close()
            notify(f'无法读取启动终端 {device}: {error}。小车保持停止；'
                   f'可在另一个终端调用 {service_name} 服务启用。')
        else:
            notify('等待启动：请在启动 launch 的终端输入 start 并按回车，小车才会开始运行。')

    def poll(self):
        if self.fd is None:
            return False
        try:
            if not select.select([self.fd], [], [], 0)[0]:
                return False
            data = os.read(self.fd, 4096)
            if not data:
                self.close()
                self.notify('启动终端已关闭，小车继续等待启动。')
                return False
        except BlockingIOError:
            return False
        except OSError as error:
            self.close()
            self.notify(f'启动终端读取失败：{error}；小车继续等待启动。')
            return False
        self.pending += data
        while b'\n' in self.pending:
            line, self.pending = self.pending.split(b'\n', 1)
            if line.strip() == b'start':
                self.close()
                return True
            if line.strip():
                self.notify('尚未启动，请输入 start 并按回车。')
        # Discard an oversized invalid line rather than retaining it forever.
        if len(self.pending) > 4096:
            self.pending = b''
        return False

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
