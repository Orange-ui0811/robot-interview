"""Stateless JSON-RPC over HTTP matching the Phanthy Motus plugin envelope."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from threading import Event, Thread
from urllib.error import URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

from .driver import MODEL_URI, ToolNotFound, Unavailable

LOGGER = logging.getLogger(__name__)
PROTOCOLS = ("2024-11-05", "2025-03-26", "2025-06-18")
MODEL_RESOURCE = {"uri": MODEL_URI, "name": "Go1 URDF", "mimeType": "application/xml"}
MAX_BODY = 64 * 1024


def strict_json(data):
    def reject_constant(value):
        raise ValueError(f"invalid JSON number: {value}")
    return json.loads(data, parse_constant=reject_constant)


def rpc_error(rid, code, message):
    return {"jsonrpc": "2.0", "id": rid, "error": {"code": code, "message": message}}


def dispatch_rpc(driver, rpc):
    if not isinstance(rpc, dict):
        return rpc_error(None, -32600, "Invalid Request")
    rid = rpc.get("id")
    method = rpc.get("method")
    if (rpc.get("jsonrpc") != "2.0" or not isinstance(method, str)
            or type(rid) not in (str, int, type(None))):
        return rpc_error(None, -32600, "Invalid Request")
    if "id" not in rpc:
        return None
    params = rpc.get("params", {})
    if not isinstance(params, dict):
        return rpc_error(rid, -32602, "Invalid params")

    try:
        if method == "initialize":
            requested = params.get("protocolVersion")
            version = requested if requested in PROTOCOLS else PROTOCOLS[0]
            result = {"protocolVersion": version, "capabilities": {"tools": {}, "resources": {}},
                      "serverInfo": {"name": "go1-mock-driver", "version": "0.1.0"}}
        elif method == "ping":
            result = {}
        elif method == "tools/list":
            result = {"tools": driver.get_tools()}
        elif method == "tools/call":
            name = params.get("name")
            arguments = params.get("arguments", {})
            if not isinstance(name, str) or not isinstance(arguments, dict):
                return rpc_error(rid, -32602, "name must be a string and arguments an object")
            try:
                data = driver.dispatch(name, arguments)
                failed = isinstance(data, dict) and data.get("ok") is False
            except ToolNotFound:
                return rpc_error(rid, -32602, f"Unknown tool: {name}")
            except (ValueError, Unavailable) as error:
                data = {"ok": False, "simulated": True, "error": str(error)}
                failed = True
            result = {"content": [{"type": "text", "text": json.dumps(data, allow_nan=False)}],
                      "isError": failed}
        elif method == "resources/list":
            result = {"resources": [MODEL_RESOURCE]}
        elif method == "resources/read":
            if params.get("uri") != MODEL_URI:
                return rpc_error(rid, -32002, "Resource not found")
            result = {"contents": [{"uri": MODEL_URI, "mimeType": "application/xml", "text": driver.model.urdf}]}
        else:
            return rpc_error(rid, -32601, "Method not found")
    except Exception:
        LOGGER.exception("MCP dispatch failed")
        return rpc_error(rid, -32603, "Internal error")
    return {"jsonrpc": "2.0", "id": rid, "result": result}


def handler_for(driver):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            self.request.settimeout(3.0)
            super().setup()

        def log_message(self, _format, *_args):
            pass

        def reply(self, status, body=None):
            data = b"" if body is None else json.dumps(body, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Length", str(len(data)))
            if data:
                self.send_header("Content-Type", "application/json")
            self.end_headers()
            if data:
                self.wfile.write(data)

        def do_GET(self):
            self.reply(405 if self.path == "/mcp" else 404)

        def do_POST(self):
            if self.path != "/mcp":
                return self.reply(404)
            host = self.headers.get("Host", "").lower().partition(":")[0]
            origin = self.headers.get("Origin")
            try:
                origin_host = urlparse(origin).hostname if origin else host
            except ValueError:
                return self.reply(403)
            if host not in ("localhost", "127.0.0.1") or origin_host not in ("localhost", "127.0.0.1"):
                return self.reply(403)
            protocol = self.headers.get("Mcp-Protocol-Version")
            if protocol is not None and protocol not in PROTOCOLS:
                return self.reply(400, rpc_error(None, -32600, "Unsupported protocol version"))
            if self.headers.get_content_type() != "application/json":
                return self.reply(415)
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= MAX_BODY:
                    return self.reply(413 if size > MAX_BODY else 400)
                rpc = strict_json(self.rfile.read(size))
            except (ValueError, TimeoutError, RecursionError):
                return self.reply(400, rpc_error(None, -32700, "Parse error"))
            result = dispatch_rpc(driver, rpc)
            self.reply(202 if result is None else 200, result)

    return Handler


class Registration:
    def __init__(self, config, core_url, advertised_url):
        self._config = config
        self._url = core_url.rstrip("/") + "/api/mcp"
        self._payload = json.dumps({
            "id": config.driver_id, "name": config.name, "url": advertised_url,
            "transport": "http", "category": "driver",
        }).encode("utf-8")
        self._halt = Event()
        self._thread = Thread(target=self._loop, name="go1-mcp-registration")

    def start(self):
        self._thread.start()

    def stop(self):
        self._halt.set()
        self._thread.join()

    def _loop(self):
        registered = False
        while not self._halt.is_set():
            try:
                request = Request(self._url, self._payload, {"Content-Type": "application/json"})
                with urlopen(request, timeout=2):
                    if not registered:
                        LOGGER.info("Registered Go1 mock driver with Agent Core")
                registered = True
                delay = self._config.register_refresh_s
            except (URLError, ValueError, TimeoutError, OSError) as error:
                if registered or not self._halt.is_set():
                    LOGGER.debug("Agent Core registration unavailable: %s", type(error).__name__)
                registered = False
                delay = 5.0
            self._halt.wait(delay)


class Server:
    def __init__(self, driver, config, core_url="", advertise_host="127.0.0.1"):
        if advertise_host not in ("127.0.0.1", "localhost"):
            raise ValueError("advertise_host must be localhost or 127.0.0.1")
        if core_url:
            parsed = urlparse(core_url)
            if parsed.scheme not in ("http", "https") or not parsed.hostname:
                raise ValueError("Agent Core URL must be an HTTP or HTTPS URL")
        self.driver = driver
        self._http = ThreadingHTTPServer((config.host, config.port), handler_for(driver))
        self._http.daemon_threads = False
        self.url = f"http://{advertise_host}:{self._http.server_port}/mcp"
        self._thread = Thread(target=self._http.serve_forever, kwargs={"poll_interval": 0.05},
                              name="go1-mcp-http")
        self._registration = Registration(config, core_url, self.url) if core_url else None
        self._running = False

    def start(self):
        try:
            self.driver.start()
        except BaseException:
            self._http.server_close()
            raise
        self._thread.start()
        self._running = True
        if self._registration:
            self._registration.start()
        LOGGER.info("Go1 MCP listening at %s", self.url)

    def stop(self):
        if not self._running:
            return
        self._running = False
        if self._registration:
            self._registration.stop()
        self._http.shutdown()
        self._thread.join()
        self._http.server_close()
        self.driver.stop()
