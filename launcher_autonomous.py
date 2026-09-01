# -*- coding: utf-8 -*-
"""Autonomous TankTrouble launcher.

Original ``launcher.py`` and ``src/ai/ai-bridge.js`` are intentionally kept
unchanged. This launcher:
- starts the continuous navigation service;
- waits until its WebSocket port is actually accepting connections;
- serves a runtime copy of index.html that swaps in ai-bridge-autonomous.js;
- only then opens the browser, removing the first-round AI startup delay.
"""
from __future__ import annotations

import os
import socket
import socketserver
import subprocess
import sys
import time
import traceback
import webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))
os.chdir(ROOT)

import launcher

AI_SERVER = os.path.join(ROOT, 'python', 'navigation_service.py')
ORIGINAL_BRIDGE = '../ai/ai-bridge.js'
AUTONOMOUS_BRIDGE = '../ai/ai-bridge-autonomous.js'


def wait_port(port: int, proc: subprocess.Popen, timeout: float = 15.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError('navigation service exited early with code %s' % proc.returncode)
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=0.15):
                return
        except OSError:
            time.sleep(0.05)
    raise RuntimeError('navigation service did not become ready within %.1fs' % timeout)


class AutonomousGameHandler(launcher.GameHandler):
    """Serve the original game but swap the bridge only in the autonomous mode."""

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/src/tanktrouble/', '/src/tanktrouble/index.html'):
            index_path = os.path.join(ROOT, 'src', 'tanktrouble', 'index.html')
            try:
                with open(index_path, 'r', encoding='utf-8-sig') as f:
                    html = f.read()
                html = html.replace(ORIGINAL_BRIDGE, AUTONOMOUS_BRIDGE)
                blob = html.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
                self.send_header('Content-Length', str(len(blob)))
                self.end_headers()
                self.wfile.write(blob)
                return
            except Exception:
                traceback.print_exc()
        super().do_GET()


class ReusableTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> None:
    cfg = launcher.load_config()
    http_port = launcher.find_free_port()
    ai_port = launcher.find_free_port()
    launcher._AI_PORT = ai_port

    print('Starting autonomous navigation service...', flush=True)
    ai_proc = subprocess.Popen(
        [sys.executable, AI_SERVER, str(ai_port)],
        cwd=ROOT,
        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0),
    )

    httpd = None
    try:
        # Important: do not let gameplay start before numpy/Pillow imports and
        # WebSocket binding are finished. This removes the old 2-second retry delay.
        wait_port(ai_port, ai_proc)
        print('AI ready: ws://127.0.0.1:%d/ai' % ai_port, flush=True)

        httpd = ReusableTCPServer(('127.0.0.1', http_port), AutonomousGameHandler)
        url = 'http://127.0.0.1:%d/src/tanktrouble/' % http_port
        print('=' * 60)
        print('TankTrouble autonomous fire-assist mode is ready')
        print('Game: ' + url)
        print('Player: full control of movement/turning (AI never touches keys)')
        print('AI: probes current muzzle shot (up to 4 mirror bounces), fires on hit')
        print('Multi-round: enabled (respawn automatically starts a new round)')
        print('=' * 60, flush=True)

        if os.environ.get('TANKTROUBLE_NO_BROWSER', '').strip() != '1':
            try:
                webbrowser.open(url)
            except Exception:
                pass
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if httpd is not None:
            try:
                httpd.shutdown()
            except Exception:
                pass
            try:
                httpd.server_close()
            except Exception:
                pass
        try:
            ai_proc.terminate()
            ai_proc.wait(timeout=3)
        except Exception:
            try:
                ai_proc.kill()
            except Exception:
                pass


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        raise
