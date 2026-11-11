#!/usr/bin/env python3
"""Broker, log and web dashboard for the car.

Usage: car_dashboard.py <log file> [--mqtt-port 1884] [--http-port 8080]

The car connects to this script as its MQTT broker. Everything it publishes
is appended to the log, console batches split into "console <uptime ms> <text>"
lines. The page at http://localhost:8080 starts and stops the car and sets what
each barcode does at a junction: messages on <topic>/cmd, see
app_program/communication/remote_command.h.

Barcode actions are saved beside this script and sent again each time the car
connects, because the car forgets them on reset. Start is never sent by itself,
so a car that resets stays still. The page is served on this computer only.
Lab use: MQTT 3.1.1 subset, no authentication, no retained messages, no QoS 2.
"""
import argparse
import collections
import json
import pathlib
import re
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = pathlib.Path(__file__).resolve().parent
PAGE = HERE / "index.html"
map_file = HERE / "barcode_map.json"
ACTIONS = ("STRAIGHT", "LEFT", "RIGHT", "UTURN")
MAP_SIZE = 8  # REMOTE_MAP_SIZE
# Packed layouts: app_program/robot_types.h. Names keep the firmware's order.
AVOID_STATES = ("IDLE", "STOPPING", "WAIT_RESULT", "WAIT_CLEAR", "RELEASING",
                "PREPARE", "SEND", "RUN_LEG", "VERIFY", "BLOCKED_WAIT",
                "RETRY_WAIT", "REPLAN", "FAULT")
LINE_MODES = ("TRACK", "SEARCH_L", "SEARCH_R", "STOP")
LINE_TURNS = ("STRAIGHT", "LEFT", "RIGHT", "UTURN")
BARCODE_SYMBOLS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-.$/+%"
TELEMETRY_WORDS = 3
STATUS_WORDS = 6
TICK_MS = 10
TYPE_OBSTACLE, TYPE_AVOID, TYPE_LINE, TYPE_BARCODE = 1, 2, 3, 4

BARCODE_TEXT = re.compile(r"^[0-9A-Z.$/+%-]{1,3}$")  # REMOTE_TEXT_MAX, no space
SHOWN_EVENT = re.compile(
    r"remote: |barcode: FAIL|line (JUNCTION|SEARCH|STOP|BARCODE|HUMP)"
    r"|obstacle: (stopped|started|car lifted|avoid |width )|mqtt: "
    r"|terrain: (LIFTED|OVERTURNED|calibration|hump LEVEL)")

EDGE_READOUT = re.compile(r"barcode: B=\d raw=\d+ kept=(\d+)")
READOUT_S = 0.5  # The car prints edge counts this often.
PASS_END_READOUTS = 3  # This many in a row with no edges end a pass.
SCAN_REPEAT_S = 3.0  # Console and telemetry each report a read within this.
PASS_MIN_EDGES = 5  # Fewer is a stray mark, not a barcode.
FULL_BARCODE_EDGES = 30  # "*X*" has 29 elements between 30 edges.

lock = threading.Lock()
subscribers = []  # (connection, topic filter)
pass_edges = []  # Edges per readout in this pass, quiet readouts included.
state = {
    "car_connected": False,
    "run": "unknown",
    "barcode": None,
    "barcode_action": None,
    "junction": None,
    "events": collections.deque(maxlen=40),
    "map": [],
    "car": None,  # The latest status record, as numbers and names.
    "car_age_s": None,
    "last_obstacle": None,  # Width and range of the last obstacle measured.
}
car_seen = 0.0  # time.monotonic() of the latest status record.
last_scan = {"barcode": None, "at": 0.0}  # The latest read, for note_scan().
car = {"conn": None, "topic": None}
log_path = "car.log"


def log(line, quiet=False):
    stamp = time.strftime("%H:%M:%S")
    with lock, open(log_path, "a") as out:
        out.write(f"{stamp} {line}\n")
    if not quiet:
        print(f"{stamp} {line}", flush=True)


def field(word, shift, bits, signed=False):
    """One field of a packed word, as robot_field_get() reads it."""
    value = (word >> shift) & ((1 << bits) - 1)
    if signed and value >= (1 << (bits - 1)):
        value -= 1 << bits
    return value


def decode_status(payload):
    """A /status record as named numbers, or None if it is cut short."""
    if len(payload) != STATUS_WORDS * 4:
        return None
    words = struct.unpack(f"<{STATUS_WORDS}I", payload)
    avoid = field(words[1], 28, 4)
    return {
        "v": field(words[1], 0, 11, True),
        "l": field(words[1], 11, 11, True),
        "r": field(words[2], 0, 11, True),
        "d": field(words[0], 0, 24, True),
        "h": field(words[3], 0, 16, True),
        "m": field(words[1], 22, 3),
        "os": field(words[1], 25, 3),
        "ow": field(words[2], 11, 9),
        "od": field(words[2], 20, 12),
        "av": AVOID_STATES[avoid] if avoid < len(AVOID_STATES) else "?",
        "p": field(words[3], 16, 12, True),
        "ro": field(words[4], 0, 12, True),
        "tr": field(words[4], 12, 11, True),
        "hu": field(words[0], 24, 8),
        "lf": field(words[5], 0, 8),
        "rq": field(words[4], 23, 8),
        "rt": field(words[5], 8, 8),
        "cs": field(words[3], 28, 3),
    }


def decode_barcode(first, second):
    """Barcode text from a record's two payload words."""
    text = ""
    for word in (first, second):
        for place in range(5):
            code = field(word, place * 6, 6)
            if code == 0 or code > len(BARCODE_SYMBOLS):
                return text
            text += BARCODE_SYMBOLS[code - 1]
    return text


def decode_telemetry(payload):
    """A /telemetry record as the text line it stands for, or None."""
    if len(payload) != TELEMETRY_WORDS * 4:
        return None
    head, first, second = struct.unpack(f"<{TELEMETRY_WORDS}I", payload)
    kind = field(head, 0, 4)
    stamp = f"seq={field(head, 4, 8)} captured_ms={field(head, 12, 20) * TICK_MS}"
    if kind == TYPE_OBSTACLE:
        return (f"{stamp} obstacle flags={field(first, 0, 8)} "
                f"edges={field(first, 8, 8)} width_mm={field(first, 16, 9)}")
    if kind == TYPE_AVOID:
        before = field(first, 0, 4)
        after = field(first, 4, 4)
        if max(before, after) >= len(AVOID_STATES):
            return None
        return f"{stamp} avoid {AVOID_STATES[before]}>{AVOID_STATES[after]}"
    if kind == TYPE_LINE:
        junction = field(first, 2, 1)
        name = (f"JUNCTION {LINE_TURNS[field(first, 3, 2)]}" if junction
                else LINE_MODES[field(first, 0, 2)])
        return (f"{stamp} line {name} t={field(second, 0, 16, True)} "
                f"h={field(second, 16, 16, True)} "
                f"d={field(first, 5, 20, True)}")
    if kind == TYPE_BARCODE:
        direction = "rev" if field(first, 30, 1) else "fwd"
        return f"{stamp} barcode {decode_barcode(first, second)} {direction}"
    return None


def note_car(values):
    """Keep one status record: see app_program/communication/car_status.c."""
    global car_seen
    with lock:
        state["car"] = values
        car_seen = time.monotonic()
        if values["ow"] > 0:
            state["last_obstacle"] = {"width_mm": values["ow"],
                                      "range_mm": values["od"]}


def note_edges(kept):
    """Add one edge readout; a finished pass goes in the events. Hold lock."""
    if kept or pass_edges:
        pass_edges.append(kept)
    if (len(pass_edges) < PASS_END_READOUTS
            or any(pass_edges[-PASS_END_READOUTS:])):
        return
    edges = sum(pass_edges)
    seconds = (len(pass_edges) - PASS_END_READOUTS - 1) * READOUT_S
    pass_edges.clear()
    if edges >= PASS_MIN_EDGES:
        state["events"].appendleft(
            f"{time.strftime('%H:%M:%S')} barcode pass: {edges} edges in "
            f"{seconds:.1f} s (a full read needs {FULL_BARCODE_EDGES})")


def note_scan(barcode, action, message):
    """Show a read once: the console and telemetry both report it. Holds lock."""
    now = time.monotonic()
    state["barcode"], state["barcode_action"] = barcode, action
    if (barcode != last_scan["barcode"]
            or now - last_scan["at"] > SCAN_REPEAT_S):
        state["events"].appendleft(f"{time.strftime('%H:%M:%S')} {message}")
    last_scan.update(barcode=barcode, at=now)


def note_telemetry(payload):
    """Take reads from telemetry too: it carries on if the console stops."""
    match = re.search(r" barcode (\S+) (fwd|rev)$", payload)
    if match:
        barcode, direction = match.groups()
        with lock:
            action = next((row["action"] for row in state["map"]
                           if row["barcode"] == barcode), "STRAIGHT")
            note_scan(barcode, action, f"barcode: {barcode} dir={direction}")


def note_console(text):
    """Update the dashboard state from one console line."""
    message = text.split(" ", 1)[-1]
    with lock:
        match = re.search(r"remote: (?:ready|.*state) (stopped|running)", message)
        if match:
            state["run"] = match.group(1)
        if "obstacle: car lifted" in message:
            state["run"] = "stopped"  # Lifted or overturned: Start resumes.
        match = re.search(r"barcode: (\S+) dir=\w+ .*turn=(\w+)", message)
        if match:
            note_scan(*match.groups(), message)
        match = EDGE_READOUT.search(message)
        if match:
            note_edges(int(match.group(1)))
        match = re.search(r"line JUNCTION (\w+)", message)
        if match:
            state["junction"] = match.group(1)
        if SHOWN_EVENT.search(message):
            state["events"].appendleft(f"{time.strftime('%H:%M:%S')} {message}")


def read_exact(conn, count):
    data = b""
    while len(data) < count:
        chunk = conn.recv(count - len(data))
        if not chunk:
            raise ConnectionError
        data += chunk
    return data


def read_packet(conn):
    header = read_exact(conn, 1)[0]
    length, scale = 0, 1
    while True:
        byte = read_exact(conn, 1)[0]
        length += (byte & 0x7F) * scale
        scale *= 128
        if not byte & 0x80:
            break
    return header, read_exact(conn, length) if length else b""


def encode_length(count):
    out = bytearray()
    while True:
        byte = count % 128
        count //= 128
        out.append(byte | (0x80 if count else 0))
        if not count:
            return bytes(out)


def publish_packet(topic, payload):
    name = topic.encode()
    body = len(name).to_bytes(2, "big") + name + (payload if isinstance(payload, bytes) else payload.encode())
    return bytes([0x30]) + encode_length(len(body)) + body


def topic_matches(topic_filter, topic):
    if topic_filter in ("#", topic):
        return True
    return topic_filter.endswith("/#") and topic.startswith(topic_filter[:-1])


def send_to_car(*messages):
    """Send messages in order; False if the car is not connected."""
    with lock:
        conn, topic = car["conn"], car["topic"]
        if conn is None:
            return False
        try:
            for message in messages:
                conn.sendall(publish_packet(topic, message))
        except OSError:
            return False
    for message in messages:
        log(f"dashboard -> car: {message}")
    return True


def map_messages():
    with lock:
        rows = list(state["map"])
    return ["clear"] + [f"map {r['barcode']} {r['action']}" for r in rows]


def handle_publish(conn, header, body):
    qos = (header >> 1) & 3
    topic_length = int.from_bytes(body[0:2], "big")
    topic = body[2:2 + topic_length].decode(errors="replace")
    position = 2 + topic_length
    packet_id = None
    if qos:
        packet_id = body[position:position + 2]
        position += 2
    payload = body[position:]
    if topic.endswith("/console"):
        for text in payload.decode(errors="replace").splitlines():
            log(f"console {text}")
            note_console(text)
    elif topic.endswith("/status"):
        values = decode_status(payload)
        if values is not None:  # A cut record: the next is half a second away.
            pairs = " ".join(f"{key}={value}" for key, value in values.items())
            log(f"status CAR {pairs}", quiet=True)
            note_car(values)
    elif topic.endswith("/telemetry"):
        text = decode_telemetry(payload)
        if text is not None:
            log(f"{topic} {text}")
            note_telemetry(text)
    else:
        log(f"{topic} {payload.decode(errors='replace')}")
    if qos == 1:
        conn.sendall(b"\x40\x02" + packet_id)
    packet = publish_packet(topic, payload)
    with lock:
        for entry in list(subscribers):
            if not topic_matches(entry[1], topic):
                continue
            try:
                entry[0].sendall(packet)
            except OSError:
                subscribers.remove(entry)


def handle_subscribe(conn, body):
    packet_id = body[0:2]
    position = 2
    filters = []
    while position < len(body):
        length = int.from_bytes(body[position:position + 2], "big")
        filters.append(body[position + 2:position + 2 + length].decode())
        position += 2 + length + 1
    conn.sendall(bytes([0x90]) + encode_length(2 + len(filters)) + packet_id
                 + b"\x00" * len(filters))
    is_car = False
    with lock:
        for topic_filter in filters:
            subscribers.append((conn, topic_filter))
            if topic_filter.endswith("/cmd"):
                car["conn"], car["topic"] = conn, topic_filter
                state["car_connected"] = True
                is_car = True
    if is_car:
        send_to_car(*map_messages())


def handle_client(conn, address):
    log(f"connect from {address[0]}:{address[1]}")
    try:
        while True:
            header, body = read_packet(conn)
            kind = header >> 4
            if kind == 1:  # CONNECT
                conn.sendall(b"\x20\x02\x00\x00")
            elif kind == 3:  # PUBLISH
                handle_publish(conn, header, body)
            elif kind == 8:  # SUBSCRIBE
                handle_subscribe(conn, body)
            elif kind == 12:  # PINGREQ
                conn.sendall(b"\xd0\x00")
            elif kind == 14:  # DISCONNECT
                break
    except (ConnectionError, OSError):
        pass
    finally:
        with lock:
            subscribers[:] = [s for s in subscribers if s[0] is not conn]
            if car["conn"] is conn:
                car["conn"] = None
                state["car_connected"] = False
                state["run"] = "unknown"
        conn.close()
        log(f"disconnect {address[0]}:{address[1]}")


def serve_mqtt(port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(8)
    log(f"listening on 0.0.0.0:{port}")
    while True:
        conn, address = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=handle_client, args=(conn, address),
                         daemon=True).start()


def clean_map(rows):
    """Return validated rows, or raise ValueError saying what is wrong."""
    if len(rows) > MAP_SIZE:
        raise ValueError(f"The car holds at most {MAP_SIZE} barcodes")
    cleaned, seen = [], set()
    for row in rows:
        barcode = str(row.get("barcode", "")).strip().upper()
        action = str(row.get("action", ""))
        if not BARCODE_TEXT.match(barcode):
            raise ValueError(f"Barcode {barcode!r}: use 1 to 3 Code 39 "
                             "characters")
        if action not in ACTIONS:
            raise ValueError(f"Unknown action {action!r}")
        if barcode in seen:
            raise ValueError(f"Barcode {barcode} is listed twice")
        seen.add(barcode)
        cleaned.append({"barcode": barcode, "action": action})
    return cleaned


class Page(BaseHTTPRequestHandler):
    def reply(self, code, body, kind="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def reply_json(self, code, value):
        self.reply(code, json.dumps(value).encode())

    def do_GET(self):
        if self.path == "/":
            self.reply(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/state":
            with lock:
                snapshot = dict(state, events=list(state["events"]))
                if car_seen:
                    snapshot["car_age_s"] = time.monotonic() - car_seen
            self.reply_json(200, snapshot)
        else:
            self.reply_json(404, {"error": "Not found"})

    def do_POST(self):
        # Another site's page cannot set this header without asking first.
        if self.headers.get("X-Car-Dashboard") != "1":
            self.reply_json(403, {"error": "Missing header"})
            return
        length = int(self.headers.get("Content-Length", 0))
        try:
            request = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            self.reply_json(400, {"error": "Not JSON"})
            return
        if self.path == "/command":
            command = request.get("command")
            if command not in ("start", "stop"):
                self.reply_json(400, {"error": "Unknown command"})
            elif send_to_car(command):
                self.reply_json(200, {"sent": True})
            else:
                self.reply_json(409, {"error": "The car is not connected"})
        elif self.path == "/map":
            try:
                rows = clean_map(request.get("map", []))
            except ValueError as error:
                self.reply_json(400, {"error": str(error)})
                return
            with lock:
                state["map"] = rows
            map_file.write_text(json.dumps(rows, indent=2) + "\n")
            self.reply_json(200, {"sent": send_to_car(*map_messages()),
                                  "map": rows})
        else:
            self.reply_json(404, {"error": "Not found"})

    def log_message(self, *args):
        pass  # The page polls: keep the terminal for the car.


def main():
    global log_path, map_file
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log")
    parser.add_argument("--mqtt-port", type=int, default=1884)
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--map-file", type=pathlib.Path, default=map_file)
    args = parser.parse_args()
    log_path = args.log
    map_file = args.map_file
    if map_file.exists():
        state["map"] = clean_map(json.loads(map_file.read_text()))
    threading.Thread(target=serve_mqtt, args=(args.mqtt_port,),
                     daemon=True).start()
    page = ThreadingHTTPServer(("127.0.0.1", args.http_port), Page)
    print(f"dashboard: http://localhost:{args.http_port}", flush=True)
    page.serve_forever()


if __name__ == "__main__":
    main()
