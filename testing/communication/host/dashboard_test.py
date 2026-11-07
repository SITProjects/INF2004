#!/usr/bin/env python3
"""Drive tools/dashboard/car_dashboard.py with a fake car over real MQTT.

The fake car connects, subscribes to <topic>/cmd and answers as the firmware
does on <topic>/console. No hardware or browser needed.
"""
import json
import pathlib
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

REPO = pathlib.Path(__file__).resolve().parents[3]
TOPIC = "mtk3smp-pico/heartbeat"
MQTT_PORT = 18841
HTTP_PORT = 18080
failures = 0


def check(name, ok):
    global failures
    print(("PASS " if ok else "FAIL ") + name)
    failures += 0 if ok else 1


def length_bytes(count):
    out = bytearray()
    while True:
        byte = count % 128
        count //= 128
        out.append(byte | (0x80 if count else 0))
        if not count:
            return bytes(out)


class FakeCar:
    def __init__(self, stopped=True):
        self.sock = socket.create_connection(("127.0.0.1", MQTT_PORT))
        self.sock.settimeout(0.3)
        self.stopped = stopped
        self.received = []
        self.buffer = b""
        name = b"fake"
        connect = (b"\x00\x04MQTT\x04\x02\x00\x3c"
                   + len(name).to_bytes(2, "big") + name)
        self.sock.sendall(b"\x10" + length_bytes(len(connect)) + connect)
        assert self.sock.recv(4)[0] == 0x20
        topic = f"{TOPIC}/cmd".encode()
        body = b"\x00\x01" + len(topic).to_bytes(2, "big") + topic + b"\x00"
        self.sock.sendall(b"\x82" + length_bytes(len(body)) + body)
        self.console(f"remote: ready {self.state()}")

    def state(self):
        return "stopped" if self.stopped else "running"

    def console(self, text):
        topic = f"{TOPIC}/console".encode()
        body = (len(topic).to_bytes(2, "big") + topic
                + f"1000 {text}\n".encode())
        self.sock.sendall(b"\x30" + length_bytes(len(body)) + body)

    def telemetry(self, text):
        topic = f"{TOPIC}/telemetry".encode()
        body = len(topic).to_bytes(2, "big") + topic + text.encode()
        self.sock.sendall(b"\x30" + length_bytes(len(body)) + body)

    def pump(self):
        """Read what the broker sent, answering as the firmware would."""
        try:
            while True:
                data = self.sock.recv(4096)
                if not data:
                    break
                self.buffer += data
        except socket.timeout:
            pass
        while len(self.buffer) >= 2:
            kind, length = self.buffer[0] >> 4, self.buffer[1]
            if len(self.buffer) < 2 + length:
                break
            body = self.buffer[2:2 + length]
            self.buffer = self.buffer[2 + length:]
            if kind != 3:
                continue
            topic_length = int.from_bytes(body[:2], "big")
            topic = body[2:2 + topic_length].decode()
            message = body[2 + topic_length:].decode()
            self.received.append((topic, message))
            if message in ("start", "stop"):
                self.stopped = message == "stop"
            self.console(f"remote: ok {message}, state {self.state()}")

    def close(self):
        self.sock.close()


def get_state():
    with urllib.request.urlopen(f"http://127.0.0.1:{HTTP_PORT}/state") as reply:
        return json.load(reply)


def post(path, body, header=True):
    request = urllib.request.Request(
        f"http://127.0.0.1:{HTTP_PORT}{path}", json.dumps(body).encode(),
        {"Content-Type": "application/json",
         **({"X-Car-Dashboard": "1"} if header else {})})
    try:
        with urllib.request.urlopen(request) as reply:
            return reply.status, json.load(reply)
    except urllib.error.HTTPError as error:
        return error.code, json.load(error)


def settle(car):
    time.sleep(0.2)
    car.pump()
    time.sleep(0.1)


def main():
    with tempfile.TemporaryDirectory() as folder:
        folder = pathlib.Path(folder)
        server = subprocess.Popen(
            [sys.executable, REPO / "tools/dashboard/car_dashboard.py",
             folder / "log.txt", "--mqtt-port", str(MQTT_PORT),
             "--http-port", str(HTTP_PORT), "--map-file", folder / "map.json"],
            stdout=subprocess.DEVNULL)
        try:
            time.sleep(0.7)
            run(folder)
        finally:
            server.terminate()
            server.wait()
    print(f"{failures} failure(s)")
    return 1 if failures else 0


def run(folder):
    status, _ = post("/command", {"command": "start"})
    check("D01 with no car, Start is refused", status == 409)
    status, reply = post("/map", {"map": [{"barcode": "f", "action": "LEFT"}]})
    check("D02 with no car, actions are saved for later",
          status == 200 and not reply["sent"]
          and reply["map"][0]["barcode"] == "F")

    car = FakeCar()
    settle(car)
    messages = [m for _, m in car.received]
    check("D03 on connecting the car gets the actions, and no Start",
          messages == ["clear", "map F LEFT"]
          and all(t == f"{TOPIC}/cmd" for t, _ in car.received))
    check("D04 the page shows the car connected and stopped",
          get_state()["car_connected"] and get_state()["run"] == "stopped")

    post("/command", {"command": "start"})
    settle(car)
    check("D05 Start reaches the car and the page shows it running",
          ("start" in [m for _, m in car.received])
          and get_state()["run"] == "running")
    post("/command", {"command": "stop"})
    settle(car)
    check("D06 Stop reaches the car and the page shows it stopped",
          car.stopped and get_state()["run"] == "stopped")

    post("/command", {"command": "reverse"})
    settle(car)
    check("D19 Reverse reaches the car and it stays stopped",
          ("reverse" in [m for _, m in car.received])
          and get_state()["run"] == "stopped")

    # A real pass from 2026-11-06: one quiet readout inside it, then stray.
    for kept in (2, 4, 6, 0, 3, 2, 0, 0, 0, 1, 0, 0, 0):
        car.console(f"barcode: B=1 raw=9 kept={kept}")
    settle(car)
    passes = [e for e in get_state()["events"] if "barcode pass" in e]
    check("D20 a pass shows its edges and time once, stray edges do not",
          len(passes) == 1 and "17 edges in 2.5 s" in passes[0])
    car.console("barcode: B=1 raw=9 kept=4")
    settle(car)
    check("D21 a pass still going is not shown yet",
          len([e for e in get_state()["events"]
               if "barcode pass" in e]) == 1)
    for kept in (0, 0, 0):
        car.console(f"barcode: B=0 raw=0 kept={kept}")

    car.console("barcode: F dir=fwd elements=29 narrow=3 us turn=LEFT")
    car.console("line JUNCTION LEFT t=1 h=2 d=3")
    settle(car)
    state = get_state()
    check("D07 the page shows the barcode, its action and the junction",
          state["barcode"] == "F" and state["barcode_action"] == "LEFT"
          and state["junction"] == "LEFT")
    check("D08 the car does not get its own console back",
          all(t.endswith("/cmd") for t, _ in car.received))

    car.received.clear()
    post("/map", {"map": [{"barcode": "F", "action": "UTURN"},
                          {"barcode": "A", "action": "RIGHT"}]})
    settle(car)
    check("D09 a change clears and resends every action",
          [m for _, m in car.received]
          == ["clear", "map F UTURN", "map A RIGHT"])

    too_many = [{"barcode": chr(65 + i), "action": "LEFT"} for i in range(9)]
    bad = [post("/map", {"map": rows})[0] for rows in (
        [{"barcode": "TOOLONG", "action": "LEFT"}],
        [{"barcode": "F G", "action": "LEFT"}],
        [{"barcode": "F", "action": "SIDEWAYS"}],
        [{"barcode": "F", "action": "LEFT"}, {"barcode": "F", "action": "RIGHT"}],
        too_many)]
    check("D10 bad actions are refused", bad == [400] * 5)
    check("D11 unknown commands are refused",
          post("/command", {"command": "explode"})[0] == 400)
    check("D12 a request without the header is refused",
          post("/command", {"command": "stop"}, header=False)[0] == 403)
    saved = json.loads((folder / "map.json").read_text())
    check("D13 a refused change leaves the saved actions alone",
          [r["action"] for r in saved] == ["UTURN", "RIGHT"])

    line = ("CAR v=80 l=70 r=90 d=1234 h=-5 m=5 os=2 ow=208 od=159 av=IDLE "
            "p=12 ro=-3 tr=15 hu=1 co=2")
    car.console(line)
    settle(car)
    state = get_state()
    check("D16 a status line reaches the page as numbers and names",
          state["car"]["v"] == 80 and state["car"]["h"] == -5
          and state["car"]["av"] == "IDLE" and state["car_age_s"] < 2
          and state["last_obstacle"] == {"width_mm": 208, "range_mm": 159})
    car.console(line.replace("ow=208", "ow=0").replace("od=159", "od=0"))
    car.console("CAR v=1 cut")
    settle(car)
    state = get_state()
    check("D17 the last width stays after the path clears, and a cut line "
          "changes nothing",
          state["last_obstacle"]["width_mm"] == 208
          and state["car"]["ow"] == 0 and state["car"]["v"] == 80)
    check("D18 status lines stay out of the events",
          not any("CAR" in event for event in state["events"]))

    # The console stopped after a stall on 2026-11-07: telemetry still came.
    car.telemetry("seq=716 captured_ms=689470 barcode A rev")
    settle(car)
    state = get_state()
    check("D22 with no console, telemetry shows a read and its mapped action",
          state["barcode"] == "A" and state["barcode_action"] == "RIGHT"
          and len([e for e in state["events"] if "barcode: A" in e]) == 1)
    car.console("barcode: A dir=rev elements=29 narrow=3 us turn=LEFT")
    settle(car)
    state = get_state()
    check("D23 the console's report of the same read adds no second event, "
          "and its action stands",
          state["barcode_action"] == "LEFT"
          and len([e for e in state["events"] if "barcode: A" in e]) == 1)
    car.telemetry("seq=717 captured_ms=689470 barcode Q fwd")
    settle(car)
    state = get_state()
    check("D24 a barcode not listed goes straight on",
          state["barcode"] == "Q" and state["barcode_action"] == "STRAIGHT")
    car.console("barcode: Q dir=fwd elements=29 narrow=3 us turn=STRAIGHT")
    car.console("barcode: FAIL bad width ratio")
    settle(car)
    check("D25 a failed read still shows as an event",
          any("barcode: FAIL" in e for e in get_state()["events"]))

    car.close()
    time.sleep(0.3)
    state = get_state()
    car.console("terrain: calibration VALID tilt 7.3 deg "
                "reading 10 20 9800 mm/s2")
    car.console("terrain: hump LEVEL height 10 mm")
    car.console("terrain: hump ASCENDING")
    settle(car)
    events = get_state()["events"]
    check("D26 calibration and each finished hump show as events, not the "
          "steps of a hump",
          any("terrain: calibration VALID" in e for e in events)
          and any("hump LEVEL height 10" in e for e in events)
          and not any("hump ASCENDING" in e for e in events))
    check("D14 a lost car shows as offline with its state unknown",
          not state["car_connected"] and state["run"] == "unknown")
    car = FakeCar()
    settle(car)
    check("D15 a reset car stays stopped and gets the actions again",
          car.stopped and [m for _, m in car.received]
          == ["clear", "map F UTURN", "map A RIGHT"])
    car.close()


if __name__ == "__main__":
    sys.exit(main())
