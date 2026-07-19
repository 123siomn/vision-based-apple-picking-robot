#!/usr/bin/env python3
"""底盘独立 PI 轮速网页工具，不启动相机、深度 SDK 或机械臂控制。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import serial

SERIAL_DIR = Path(__file__).resolve().parents[1] / "serial"
if str(SERIAL_DIR) not in sys.path:
    sys.path.insert(0, str(SERIAL_DIR))

from robot_protocol import build_frame, open_serial, parse_frame, read_line, send_frame


DEFAULT_PORT = os.environ.get("BASE_PORT", "/dev/ttyUSB0")
DEFAULT_BAUDRATE = 115200
DEFAULT_WEB_PORT = 8081


class BaseLink:
    """串行化底盘请求，保证状态轮询与网页按钮不会同时写入串口。"""

    def __init__(self, port: str, baudrate: int) -> None:
        self.port = port
        self.baudrate = baudrate
        self._lock = threading.Lock()
        self._serial: serial.Serial | None = None
        self._sequence = 0
        self._last_status_log_at = 0.0
        self._last_status_mode: tuple[str, str, str, str, str, str] | None = None

    @staticmethod
    def _log(text: str) -> None:
        """在树莓派终端输出带时间的 PI 运行记录。"""

        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    def _log_status(self, params: list[str]) -> None:
        """每秒记录一次速度闭环状态，模式或目标变化时立即记录。"""

        status = parse_status(params)
        mode = (
            status.get("STATE", "--"),
            status.get("PI", "--"),
            status.get("TGTSPD_L", "--"),
            status.get("TGTSPD_R", "--"),
            status.get("PWM_L", "--"),
            status.get("PWM_R", "--"),
        )
        now = time.monotonic()
        if mode == self._last_status_mode and (now - self._last_status_log_at) < 1.0:
            return

        self._last_status_mode = mode
        self._last_status_log_at = now
        self._log(
            "STATUS "
            f"mode={status.get('STATE', '--')} PI={status.get('PI', '--')} "
            f"target L/R={status.get('TGTSPD_L', '--')}/{status.get('TGTSPD_R', '--')} "
            f"speed L/R={status.get('SPD_L', '--')}/{status.get('SPD_R', '--')} "
            f"pwm L/R={status.get('PWM_L', '--')}/{status.get('PWM_R', '--')} "
            f"pi_out L/R={status.get('PIOUT_L', '--')}/{status.get('PIOUT_R', '--')} "
            f"enc L/R={status.get('ENC_L', '--')}/{status.get('ENC_R', '--')}"
        )

    def close(self) -> None:
        """关闭当前串口连接。"""

        with self._lock:
            if self._serial is not None:
                self._serial.close()
                self._serial = None

    def _ensure_open(self) -> serial.Serial:
        """按需打开串口，并清除上一轮遗留的输入数据。"""

        if self._serial is None or not self._serial.is_open:
            self._serial = open_serial(self.port, self.baudrate, timeout=0.2)
            time.sleep(0.08)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        return self._serial

    def _next_sequence(self) -> int:
        """生成 001~999 循环使用的协议序号。"""

        self._sequence = (self._sequence % 999) + 1
        return self._sequence

    def request(self, command: str, params: list[object] | None = None) -> list[str]:
        """发送一帧 BASE 命令并等待同序号 ACK、STATUS 或 ERR 应答。"""

        if params is None:
            params = []

        with self._lock:
            try:
                serial_port = self._ensure_open()
                sequence = self._next_sequence()
                send_frame(serial_port, build_frame("BASE", sequence, command, params))

                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    raw_line = read_line(serial_port)
                    if not raw_line or not raw_line.startswith("$"):
                        continue
                    try:
                        reply = parse_frame(raw_line)
                    except ValueError:
                        continue
                    if reply.target != "BASE" or reply.seq != f"{sequence:03d}":
                        continue
                    if reply.command == "ERR":
                        detail = ",".join(reply.params) if reply.params else "UNKNOWN"
                        raise RuntimeError(f"底盘返回错误：{detail}")
                    if command in ("STATUS", "STATUSDBG"):
                        if reply.command != "STATUS":
                            raise RuntimeError(f"STATUS 应答类型错误：{reply.command}")
                    elif reply.command != "ACK":
                        raise RuntimeError(f"命令应答类型错误：{reply.command}")
                    if command in ("STATUS", "STATUSDBG"):
                        self._log_status(reply.params)
                    else:
                        parameter_text = ",".join(str(item) for item in params) or "无参数"
                        self._log(f"CMD {command} params={parameter_text} ACK")
                    return reply.params

                raise RuntimeError(f"底盘命令无应答：{command}")
            except (serial.SerialException, OSError, ValueError) as exc:
                if self._serial is not None:
                    self._serial.close()
                    self._serial = None
                raise RuntimeError(str(exc)) from exc


def parse_status(params: list[str]) -> dict[str, str]:
    """将 STM32 STATUS 的 KEY=VALUE 参数列表转换为字典。"""

    status: dict[str, str] = {}
    for item in params:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        status[key] = value
    return status


HTML_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>底盘 PI 轮速测试</title>
<style>
  :root { color-scheme: dark; font-family: Arial, "Microsoft YaHei", sans-serif; }
  body { margin: 0; background: #151719; color: #e8ecef; }
  main { width: min(920px, calc(100% - 32px)); margin: 28px auto 44px; }
  h1 { margin: 0 0 6px; font-size: 27px; letter-spacing: 0; }
  .hint { margin: 0 0 20px; color: #aeb8bd; }
  section { border: 1px solid #39444a; padding: 16px; margin-top: 14px; background: #1d2225; }
  h2 { font-size: 18px; margin: 0 0 14px; }
  .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
  label { display: grid; gap: 6px; color: #c9d2d6; font-size: 14px; }
  input { background: #111416; border: 1px solid #5d6b70; color: #fff; min-height: 35px; padding: 0 9px; font-size: 16px; }
  .actions { display: flex; gap: 9px; flex-wrap: wrap; margin-top: 15px; }
  button { border: 1px solid #809198; background: #273237; color: #fff; min-height: 36px; padding: 0 14px; cursor: pointer; font-size: 15px; }
  button.primary { background: #177a5a; border-color: #4db88f; }
  button.danger { background: #842d35; border-color: #d75d63; }
  #message { min-height: 22px; margin: 13px 0 0; color: #f3d46d; white-space: pre-wrap; }
  table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
  th, td { text-align: left; padding: 9px; border-bottom: 1px solid #39444a; }
  th { color: #aeb8bd; font-weight: 400; width: 37%; }
  td { color: #fff; }
  @media (max-width: 560px) { .grid { grid-template-columns: 1fr; } main { width: min(100% - 20px, 920px); } }
</style>
</head>
<body>
<main>
  <h1>底盘 PI 轮速测试</h1>
  <p class="hint">独立于相机、机械臂和循迹。当前空载实测 SPD 约为 0 到 10；PI 使用基础 PWM 加小幅修正。</p>

  <section>
    <h2>开环参考</h2>
    <div class="grid">
      <label>左轮 PWM（-99 到 99）<input id="openLeft" type="number" value="40" min="-99" max="99"></label>
      <label>右轮 PWM（-99 到 99）<input id="openRight" type="number" value="40" min="-99" max="99"></label>
    </div>
    <div class="actions"><button onclick="openPwm()">输出开环 PWM</button><button class="danger" onclick="stopBase()">停止底盘</button></div>
  </section>

  <section>
    <h2>PI 目标速度</h2>
    <div class="grid">
      <label>左轮目标 SPD（-5 到 5）<input id="targetLeft" type="number" value="3" min="-5" max="5" step="0.1"></label>
      <label>右轮目标 SPD（-5 到 5）<input id="targetRight" type="number" value="3" min="-5" max="5" step="0.1"></label>
    </div>
    <div class="actions">
      <button class="primary" onclick="startPi()">开启 PI 并运行</button>
      <button onclick="updateTarget()">更新目标</button>
      <button class="danger" onclick="stopBase()">停止并退出 PI</button>
    </div>
  </section>

  <section>
    <h2>实时状态</h2>
    <table><tbody id="status"><tr><th>连接</th><td>等待首次查询</td></tr></tbody></table>
    <div id="message"></div>
  </section>
</main>
<script>
const message = document.getElementById('message');
const rows = [['STATE','状态'],['PI','PI 模式'],['TGTSPD_L','左轮目标 SPD'],['TGTSPD_R','右轮目标 SPD'],['SPD_L','左轮实测 SPD'],['SPD_R','右轮实测 SPD'],['PIOUT_L','左轮 PI 修正 PWM'],['PIOUT_R','右轮 PI 修正 PWM'],['PWM_L','左轮实际 PWM'],['PWM_R','右轮实际 PWM'],['ENC_L','左轮编码器'],['ENC_R','右轮编码器'],['VBAT','底盘电压'],['LINE','红外状态'],['ACT','固定动作'],['LINEACT','循迹修正']];
function number(id) { return Number(document.getElementById(id).value); }
async function request(path, payload = null) {
  const options = payload === null ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)};
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.error || '请求失败');
  return data;
}
function show(text) { message.textContent = text; }
async function openPwm() { try { await request('/api/openpwm',{left:number('openLeft'),right:number('openRight')}); show('已输出开环 PWM，请观察 SPD。'); } catch(e) { show(e.message); } }
async function startPi() { try { await request('/api/pi/start',{left:number('targetLeft'),right:number('targetRight')}); show('PI 已启动。'); } catch(e) { show(e.message); } }
async function updateTarget() { try { await request('/api/pi/target',{left:number('targetLeft'),right:number('targetRight')}); show('PI 目标已更新。'); } catch(e) { show(e.message); } }
async function stopBase() { try { await request('/api/stop',{}); show('底盘已停止，PI 已退出。'); } catch(e) { show(e.message); } }
async function refresh() {
  try {
    const data = await request('/api/status');
    document.getElementById('status').innerHTML = rows.map(([key,label]) => `<tr><th>${label}</th><td>${data.status[key] ?? '--'}</td></tr>`).join('');
  } catch(e) { document.getElementById('status').innerHTML = `<tr><th>连接</th><td>${e.message}</td></tr>`; }
}
setInterval(refresh, 250); refresh();
</script>
</body></html>"""


class PiWebHandler(BaseHTTPRequestHandler):
    """PI 测试网页的 HTTP 接口。"""

    link: BaseLink

    def log_message(self, _format: str, *_args: object) -> None:
        """关闭默认访问日志，避免状态轮询刷屏。"""

    def _send_json(self, status: HTTPStatus, data: dict[str, object]) -> None:
        """发送 JSON 响应。"""

        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, object]:
        """读取并校验 POST JSON 请求体。"""

        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        value = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("请求体必须是 JSON 对象")
        return value

    @staticmethod
    def _number(data: dict[str, object], key: str, minimum: float, maximum: float) -> float:
        """读取并限制网页传入的一个数值字段。"""

        value = float(data[key])
        if not minimum <= value <= maximum:
            raise ValueError(f"{key} 必须在 {minimum} 到 {maximum} 之间")
        return value

    def do_GET(self) -> None:  # noqa: N802
        """处理网页和实时状态请求。"""

        path = urlparse(self.path).path
        if path == "/":
            body = HTML_PAGE.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/status":
            try:
                self._send_json(HTTPStatus.OK, {"ok": True, "status": parse_status(self.link.request("STATUSDBG"))})
            except (RuntimeError, ValueError) as exc:
                self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "error": str(exc)})
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "未找到接口"})

    def do_POST(self) -> None:  # noqa: N802
        """处理开环、PI 和停车按钮请求。"""

        path = urlparse(self.path).path
        try:
            data = self._read_json()
            if path == "/api/openpwm":
                left = round(self._number(data, "left", -99, 99))
                right = round(self._number(data, "right", -99, 99))
                self.link.request("OPENPWM", [left, right])
            elif path == "/api/pi/start":
                left = self._number(data, "left", -5, 5)
                right = self._number(data, "right", -5, 5)
                self.link.request("PIMODE", ["ON"])
                self.link.request("PITARGET", [f"{left:.2f}", f"{right:.2f}"])
            elif path == "/api/pi/target":
                left = self._number(data, "left", -5, 5)
                right = self._number(data, "right", -5, 5)
                self.link.request("PITARGET", [f"{left:.2f}", f"{right:.2f}"])
            elif path == "/api/stop":
                self.link.request("STOP")
            else:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "未找到接口"})
                return
            self._send_json(HTTPStatus.OK, {"ok": True})
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})


def main() -> int:
    """启动独立 PI 测试网页服务。"""

    parser = argparse.ArgumentParser(description="底盘 PI 轮速网页测试")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"底盘串口，默认 {DEFAULT_PORT}")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="底盘串口波特率")
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT, help="网页服务端口")
    args = parser.parse_args()

    PiWebHandler.link = BaseLink(args.port, args.baudrate)
    server = ThreadingHTTPServer(("0.0.0.0", args.web_port), PiWebHandler)
    print(f"BASE={args.port} baud={args.baudrate}")
    print(f"Open browser: http://树莓派IP:{args.web_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("server stopped")
    finally:
        server.server_close()
        PiWebHandler.link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
