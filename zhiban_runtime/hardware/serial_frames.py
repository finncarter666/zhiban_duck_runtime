"""增量解析 SCS 回复；串口分片和噪声不应消耗完整的反馈等待窗口。"""


class ReplyBuffer:
    """保存尚未收齐的帧，按校验和重新同步，不将损坏帧作为设备回复。"""

    def __init__(self, expected_length=None):
        self.pending = bytearray()
        self.expected_length = expected_length

    def feed(self, chunk):
        self.pending.extend(chunk)
        frames = []
        while True:
            start = self.pending.find(b"\xff\xff")
            if start < 0:
                # 保留可能跨 read() 边界的帧头首字节。
                self.pending[:] = b"\xff" if self.pending[-1:] == b"\xff" else b""
                break
            if start:
                del self.pending[:start]
            if len(self.pending) < 4:
                break
            length = self.pending[3]
            if length < 2 or (
                self.expected_length is not None and length != self.expected_length
            ):
                del self.pending[0]
                continue
            size = length + 4
            if len(self.pending) < size:
                break
            frame = bytes(self.pending[:size])
            if sum(frame[2:]) & 255 != 255:
                del self.pending[0]
                continue
            frames.append(frame)
            del self.pending[:size]
        return frames
