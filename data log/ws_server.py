# -*- coding: utf-8 -*-
"""极简 WebSocket 服务端（RFC 6455，纯标准库，无第三方依赖）
仅支持文本帧、单帧消息（<64KB），客户端必须掩码。够本项目用。"""
import base64
import hashlib
import socket
import struct
import threading

MAGIC = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class WSClient:
    def __init__(self, sock, addr, on_message, on_close):
        self.sock = sock
        self.addr = addr
        self.on_message = on_message
        self.on_close = on_close
        self.alive = True
        self.lock = threading.Lock()

    def handshake(self, headers):
        key = headers.get(b"sec-websocket-key", b"")
        accept = base64.b64encode(hashlib.sha1(key + MAGIC).digest())
        self.sock.sendall(
            b"HTTP/1.1 101 Switching Protocols\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Accept: " + accept + b"\r\n\r\n"
        )

    def _read_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("closed")
            buf += chunk
        return buf

    def recv_loop(self):
        try:
            while self.alive:
                hdr = self._read_exact(2)
                opcode = hdr[0] & 0x0F
                masked = (hdr[1] & 0x80) != 0
                length = hdr[1] & 0x7F
                if length == 126:
                    length = struct.unpack(">H", self._read_exact(2))[0]
                elif length == 127:
                    length = struct.unpack(">Q", self._read_exact(8))[0]
                mask = self._read_exact(4) if masked else None
                payload = self._read_exact(length) if length else b""
                if mask:
                    payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))

                if opcode == 0x8:      # close
                    self._send_frame(b"", 0x8)
                    break
                elif opcode == 0x9:    # ping -> pong
                    self._send_frame(payload, 0xA)
                elif opcode == 0x1:    # text
                    try:
                        self.on_message(payload.decode("utf-8", "replace"))
                    except Exception:
                        # 单条消息出错只丢弃该消息，绝不杀连接（消息处理方自带护栏）
                        import traceback
                        traceback.print_exc()
        except Exception:
            pass
        finally:
            self.alive = False
            try:
                self.sock.close()
            except Exception:
                pass
            if self.on_close:
                self.on_close(self)

    def _send_frame(self, payload, opcode=0x1):
        with self.lock:
            header = bytes([0x80 | opcode])
            n = len(payload)
            if n < 126:
                header += bytes([n])
            elif n < 65536:
                header += bytes([126]) + struct.pack(">H", n)
            else:
                header += bytes([127]) + struct.pack(">Q", n)
            try:
                self.sock.sendall(header + payload)
            except Exception:
                pass

    def send_text(self, text):
        self._send_frame(text.encode("utf-8"), 0x1)


class WSServer:
    def __init__(self, host, port, on_client):
        self.host = host
        self.port = port
        self.on_client = on_client
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(8)

    def serve_forever(self):
        while True:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                # accept 瞬态错误（连接重置等）绝不能终止服务 —— 否则 bot
                # 进程退出、launcher 重启，造成"时好时坏"的 2-3s 高延迟。
                continue
            except Exception:
                continue
            threading.Thread(target=self._handle, args=(conn, addr), daemon=True).start()

    def _handle(self, conn, addr):
        try:
            request = b""
            while b"\r\n\r\n" not in request:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                request += chunk
            head, _, _ = request.partition(b"\r\n\r\n")
            headers = {}
            for line in head.split(b"\r\n")[1:]:
                k, _, v = line.partition(b":")
                headers[k.strip().lower()] = v.strip()
            client = WSClient(conn, addr, None, None)
            client.handshake(headers)
            if self.on_client:
                self.on_client(client)
            threading.Thread(target=client.recv_loop, daemon=True).start()
        except Exception:
            try:
                conn.close()
            except Exception:
                pass
