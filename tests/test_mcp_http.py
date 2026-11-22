from dataclasses import replace
from email.message import Message
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import socket
from threading import Event, Thread, enumerate as threads
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest

from go1_mcp.driver import Config, Driver, Unavailable, moving_joint_names, skeleton_sample
from go1_mcp.http_server import Server, handler_for

LOCAL = build_opener(ProxyHandler({}))


class FakeRosTransport:
    def __init__(self):
        self.streams = set()
        self.commands = []
        self.sample_ready = True

    def set_stream(self, name, enabled):
        self.streams.add(name) if enabled else self.streams.discard(name)

    def move(self, *args):
        self.commands.append(args)
        return "aabb"

    def stop(self):
        self.commands.append("stop")

    def read(self, name):
        if not self.sample_ready:
            raise Unavailable("no fresh ROS sample")
        return {"kind": name, "simulated": True}


def post(url, payload, **headers):
    data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
    if callable(url):
        return url(data, headers)
    with LOCAL.open(Request(url, data=data, headers={"Content-Type": "application/json", **headers}),
                    timeout=4) as response:
        return response.status, response.read()


def rpc(url, method, params=None):
    payload = {"jsonrpc": "2.0", "id": 123, "method": method}
    if params is not None:
        payload["params"] = params
    status, data = post(url, payload)
    assert status == 200
    return json.loads(data)


def tool(url, name, **arguments):
    return rpc(url, "tools/call", {"name": name, "arguments": arguments})["result"]


def memory_transport(driver):
    """Run the real handler with HTTP wire bytes over in-memory file objects."""
    base = handler_for(driver)

    def request(data, headers):
        class Handler(base):
            def setup(self):
                self.rfile = BytesIO(wire_request)
                self.wfile = BytesIO()

            def finish(self):
                pass

        fields = {
            "Host": "127.0.0.1", "Content-Type": "application/json",
            "Content-Length": str(len(data)), **headers,
        }
        head = "".join(f"{k}: {v}\r\n" for k, v in fields.items())
        wire_request = ("POST /mcp HTTP/1.1\r\n" + head + "\r\n").encode() + data
        handler = Handler(None, ("127.0.0.1", 9999), None)
        head, body = handler.wfile.getvalue().split(b"\r\n\r\n", 1)
        status = int(head.split(b" ", 2)[1])
        if status >= 400:
            raise HTTPError("http://127.0.0.1/mcp", status, "HTTP error", Message(), BytesIO(body))
        return status, body

    return request


@pytest.fixture
def local_socket():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        try:
            with socket.create_connection(listener.getsockname(), timeout=1):
                pass
        except PermissionError as error:
            pytest.skip(f"local TCP connection denied by execution environment: {error}")


@pytest.fixture(params=("memory", "socket"))
def endpoint(urdf_path, request):
    backend = FakeRosTransport()
    config = Config(port=0)
    driver = Driver(backend, config, urdf_path.read_text(encoding="utf-8"))
    if request.param == "memory":
        driver.start()
        try:
            yield memory_transport(driver), backend
        finally:
            driver.stop()
            assert not backend.streams
        return
    request.getfixturevalue("local_socket")
    server = Server(driver, config)
    server.start()
    try:
        yield server.url, backend
    finally:
        server.stop()
        assert not backend.streams


def test_http_initialize_tools_resources_and_notifications(endpoint):
    url, _ = endpoint
    assert rpc(url, "initialize", {"protocolVersion": "2024-11-05"})["result"]["protocolVersion"] == "2024-11-05"
    assert post(url, {"jsonrpc": "2.0", "method": "notifications/initialized"}) == (202, b"")
    tools = {t["name"]: t for t in rpc(url, "tools/list")["result"]["tools"]}
    assert {k: v["type"] for k, v in tools.items()} == {
        "loco": "actuator", "battery": "sensor", "imu": "sensor", "joints": "sensor",
        "loco_state": "sensor", "model": "resource",
    }
    assert all(t["inputSchema"]["type"] == "object" for t in tools.values())
    assert tools["joints"]["topic_out"] == [{"topic": "/go1_mock/state/joints", "format": "sensor/skeleton"}]
    assert "x-action-params" in tools["loco"]["inputSchema"]
    uri = rpc(url, "resources/list")["result"]["resources"][0]["uri"]
    data = json.loads(tool(url, "model")["content"][0]["text"])
    assert rpc(url, "resources/read", {"uri": uri})["result"]["contents"][0]["text"] == data["urdf"]


def test_http_loco_and_sensor_use_transport(endpoint):
    url, backend = endpoint
    result = tool(url, "loco", action="move", vx=0.2, vy=-0.1, vyaw=90, duration=3)
    assert not result["isError"]
    assert json.loads(result["content"][0]["text"])["goal_id"] == "aabb"
    assert backend.commands[0] == pytest.approx((0.2, -0.1, 1.5707963267948966, 3))
    assert not tool(url, "loco", action="stop")["isError"]
    assert backend.commands[1] == "stop"
    for name in ("loco_state", "joints", "battery", "imu"):
        assert json.loads(tool(url, name, action="read")["content"][0]["text"])["data"]["kind"] == name
    tool(url, "imu", action="stop")
    assert "imu" not in backend.streams
    tool(url, "imu", action="start")
    assert "imu" in backend.streams
    backend.sample_ready = False
    assert json.loads(tool(url, "imu", action="info")["content"][0]["text"])["state"] == "unavailable"
    assert tool(url, "imu", action="read")["isError"]


def test_protocol_and_command_errors(endpoint):
    url, backend = endpoint
    assert rpc(url, "tools/call", {"name": "missing"})["error"]["code"] == -32602
    assert rpc(url, "missing")["error"]["code"] == -32601
    assert rpc(url, "tools/call", {"name": "loco", "arguments": []})["error"]["code"] == -32602
    for arguments in ({"vx": True}, {"duration": -1}, {"duration": 31}, {"vx": 1e100}):
        assert tool(url, "loco", action="move", **arguments)["isError"]
    assert not backend.commands
    with pytest.raises(HTTPError) as error:
        post(url, b'{"jsonrpc":"2.0", "id":1, "method":"tools/call", "params":{"vx": NaN}}')
    assert error.value.code == 400 and json.loads(error.value.read())["error"]["code"] == -32700
    with pytest.raises(HTTPError) as error:
        post(url, {}, Origin="https://elsewhere.invalid")
    assert error.value.code == 403


def test_skeleton_reorders_by_name_and_marks_unavailable_fields(urdf_path):
    urdf = urdf_path.read_text(encoding="utf-8")
    order = moving_joint_names(urdf)
    data = skeleton_sample(order[::-1], list(range(12))[::-1], (), order, 10, [1.0, 0.0, 0.0, 0.0])
    assert data["format"] == "sensor/skeleton" and data["simulated"]
    assert [(j["idx"], j["name"], j["q_rad"]) for j in data["joints"]] == list(
        zip(range(12), order, range(12))
    )
    assert all(j["tau"] is None and j["dq"] is None for j in data["joints"])
    with pytest.raises(ValueError, match="exactly once"):
        skeleton_sample((order[0],) * 12, [0.0] * 12, (), order, 10, None)


@pytest.mark.parametrize("transport", ("mock", "socket"))
def test_registration_refresh_and_thread_shutdown(urdf_path, transport, request, monkeypatch):
    seen = []
    received = Event()

    class FakeCoreHandler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            seen.append((self.path, json.loads(self.rfile.read(int(self.headers["Content-Length"])))))
            if len(seen) >= 2:
                received.set()
            self.send_response(200)
            self.end_headers()

    before = set(threads())
    receiver = None
    if transport == "socket":
        request.getfixturevalue("local_socket")
        receiver = ThreadingHTTPServer(("127.0.0.1", 0), FakeCoreHandler)
        receiver_thread = Thread(target=receiver.serve_forever, kwargs={"poll_interval": 0.05})
        receiver_thread.start()
        core_url = f"http://127.0.0.1:{receiver.server_port}"
    else:
        from contextlib import nullcontext
        from urllib.parse import urlparse

        def mocked_core(req, timeout):
            seen.append((urlparse(req.full_url).path, json.loads(req.data)))
            if len(seen) >= 2:
                received.set()
            return nullcontext()

        monkeypatch.setattr("go1_mcp.http_server.urlopen", mocked_core)
        core_url = "http://127.0.0.1:29999"

    backend = FakeRosTransport()
    config = replace(Config(port=0), register_refresh_s=0.05)
    server = Server(Driver(backend, config, urdf_path.read_text(encoding="utf-8")), config,
                    core_url)
    server.start()
    try:
        assert received.wait(3)
        assert seen[0] == ("/api/mcp", {
            "id": "go1-mock-driver", "name": "Go1 Kinematic Simulator", "url": server.url,
            "transport": "http", "category": "driver",
        })
    finally:
        server.stop()
        if receiver is not None:
            receiver.shutdown()
            receiver_thread.join()
            receiver.server_close()
    assert set(threads()) == before
