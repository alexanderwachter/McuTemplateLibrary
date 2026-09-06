#!/usr/bin/env python3
# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""Live and replay viewer for fsm state machines.

Loads the Graphviz DOT graphs written by fsm::writeDot (one per machine),
follows the trace lines of fsm::tracing observers from a log stream, and
serves a page that colors the current state, the edge just taken, and the
last N steps with fading colors. Every received line is saved verbatim,
and a saved log can be replayed step by step.

Trace line grammar (anything before "fsm[" is ignored, e.g. a Zephyr
timestamp and module prefix):
    fsm[<machine>] -> <state>                  initial state
    fsm[<machine>] <from> -(<event>)-> <to>    transition
<machine> is the short name of the machine's transition table, matched
against the "// table: <name>" comment of the DOT graphs (then the
digraph name, then the file stem). <to> is "internal_target" for an
internal transition (the machine stays in <from>), <from> is "any_state"
for a wildcard transition fired through the machine's shared body.

Typical runs (details in README.md):
  a board on its console, graphs from the build directory - the defaults:
    fsmview.py                       (= fsmview.py build --serial /dev/ttyACM0@115200)
  another port, explicit graphs:
    fsmview.py graphs/ --serial /dev/ttyUSB0@921600
  a host program:
    ./TrafficLightExample | fsmview.py traffic_light.dot --stdin
  a saved log:
    fsmview.py graph.dot --replay fsmview-20260906-120000.log
Then open the printed link (http://localhost:8420/ by default).

Usage:
  fsmview.py [GRAPH...] [--tcp HOST:PORT | --listen PORT | --serial DEV[@BAUD] | --stdin | --replay FILE]
             [--save FILE] [--http PORT] [--history N] [--map MACHINE=GRAPH]... [--dot PATH]
"""

import argparse
import json
import os
import queue
import re
import shutil
import socket
import subprocess
import sys
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ANY_STATE = "any_state"
INTERNAL_TARGET = "internal_target"

# Defaults of a board on the bench: its console, and the graphs the
# sample's "dot" build target writes
DEFAULT_DEVICE = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_GRAPHS = "build"

LINE_RE = re.compile(
    r"fsm\[(?P<machine>[^\]]+)\]\s+"
    r"(?:(?P<from>\S+)\s+-\((?P<event>[^)]*)\)->\s+(?P<to>\S+)|->\s+(?P<initial>\S+))"
)
# Zephyr log prefix [HH:MM:SS.mmm,uuu]; the traffic light example prints [  1234ms]
ZEPHYR_TS_RE = re.compile(r"\[(\d+):(\d\d):(\d\d)\.(\d{3}),(\d{3})\]")
MILLIS_TS_RE = re.compile(r"\[\s*(\d+)ms\]")
# terminal colors of a log backend (Zephyr wraps each line in them)
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

DOT_NAME_RE = re.compile(r'digraph\s+"?([^"{\s]+)"?\s*\{')
DOT_TABLE_RE = re.compile(r"^\s*//\s*table:\s*(\S+)", re.MULTILINE)
# one statement per line; labels may contain brackets ("[guard]"), so the
# attribute block runs to the line's closing "];"
DOT_NODE_RE = re.compile(r'^\s*"([^"]+)"\s*(?:\[.*\])?\s*;\s*$', re.MULTILINE)
DOT_EDGE_RE = re.compile(r'^\s*"([^"]+)"\s*->\s*"([^"]+)"\s*\[(.*)\]\s*;\s*$', re.MULTILINE)
DOT_ATTR_RE = re.compile(r'(\w+)="((?:[^"\\]|\\.)*)"')
DOT_INITIAL_RE = re.compile(r'__initial\s*->\s*"([^"]+)"')


def parse_timestamp(prefix):
    """Seconds encoded in a log prefix, None if there is no known form."""
    match = ZEPHYR_TS_RE.search(prefix)
    if match:
        h, m, s, ms, us = (int(group) for group in match.groups())
        return h * 3600 + m * 60 + s + ms / 1e3 + us / 1e6
    match = MILLIS_TS_RE.search(prefix)
    if match:
        return int(match.group(1)) / 1e3
    return None


def parse_line(line):
    """The trace record of a log line, None for any other line.

    Record fields: machine, from (None for the initial line), event (None
    for the initial line), to, ts (seconds or None), raw.
    """
    line = ANSI_RE.sub("", line)
    match = LINE_RE.search(line)
    if not match:
        return None
    record = {
        "machine": match.group("machine"),
        "from": match.group("from"),
        "event": match.group("event"),
        "to": match.group("to") or match.group("initial"),
        "ts": parse_timestamp(line[: match.start()]),
        "raw": line.rstrip("\r\n"),
    }
    return record


class Graph:
    """One DOT graph as written by fsm::writeDot."""

    def __init__(self, stem, dot):
        self.stem = stem
        self.dot = dot
        self.svg = None
        name = DOT_NAME_RE.search(dot)
        self.name = name.group(1) if name else stem
        table = DOT_TABLE_RE.search(dot)
        self.table = table.group(1) if table else None
        self.states = [node for node in DOT_NODE_RE.findall(dot) if node != "__initial"]
        initial = DOT_INITIAL_RE.search(dot)
        self.initial = initial.group(1) if initial else None
        self.edges = []
        for source, target, attributes in DOT_EDGE_RE.findall(dot):
            attrs = dict(DOT_ATTR_RE.findall(attributes))
            edge_id = attrs.get("id")
            if edge_id is None:
                continue
            internal = "(internal)" in attrs.get("label", "")
            to = INTERNAL_TARGET if internal else target
            event = self._event_of(edge_id, source, to)
            self.edges.append({"id": edge_id, "from": source, "event": event, "to": to})
        self.events = sorted({edge["event"] for edge in self.edges})

    @staticmethod
    def _event_of(edge_id, source, to):
        # "<from>__<event>__<to>__<index>": names may contain "__", so
        # strip the known ends instead of splitting
        rest = edge_id.rsplit("__", 1)[0]
        if rest.startswith(source + "__"):
            rest = rest[len(source) + 2 :]
        if rest.endswith("__" + to):
            rest = rest[: -(len(to) + 2)]
        return rest

    def edge_ids(self, source, event, to):
        prefix = f"{source}__{event}__{to}__"
        return [edge["id"] for edge in self.edges if edge["id"].startswith(prefix)]

    def as_json(self):
        return {
            "stem": self.stem,
            "name": self.name,
            "table": self.table,
            "initial": self.initial,
            "states": self.states,
            "events": self.events,
            "edges": self.edges,
            "svg": self.svg,
            "dot": self.dot,
        }


def load_graphs(paths):
    files = []
    for path in paths:
        path = Path(path)
        if path.is_dir():
            files.extend(sorted(path.glob("*.dot")))
        else:
            files.append(path)
    if not files:
        sys.exit("fsmview: no .dot files found")
    return [Graph(file.stem, file.read_text(encoding="utf-8")) for file in files]


def render_graphs(graphs, dot_binary):
    for graph in graphs:
        try:
            result = subprocess.run(
                [dot_binary, "-Tsvg"], input=graph.dot, capture_output=True, text=True, check=True
            )
            graph.svg = result.stdout
        except (OSError, subprocess.CalledProcessError) as error:
            print(f"fsmview: dot failed for {graph.stem}: {error}", file=sys.stderr)


class Trace:
    """Steps in arrival order, the tracked state per machine, and the
    subscribers of the live stream."""

    def __init__(self, graphs, mapping):
        self.graphs = graphs
        self.mapping = mapping  # machine id -> graph stem (--map)
        self.steps = []
        self.state = {}  # machine id -> tracked state name
        self.subscribers = set()
        self.ended = False
        self.failed = False  # the source broke before its first line
        self.lock = threading.Lock()
        self.warnings = []
        self._graph_of = {}
        self._check_collisions()

    def _check_collisions(self):
        claims = {}
        for graph in self.graphs:
            if graph.table:
                claims.setdefault(graph.table, []).append(graph.stem)
        for table, stems in claims.items():
            if len(stems) > 1 and table not in self.mapping:
                self.warnings.append(
                    f"table id '{table}' is claimed by {', '.join(stems)}: "
                    f"using {stems[0]}, pick one with --map {table}=<graph>"
                )

    def graph_for(self, machine):
        if machine in self._graph_of:
            return self._graph_of[machine]
        found = None
        wanted = self.mapping.get(machine)
        for key in (lambda g: g.stem == wanted, lambda g: g.table == machine,
                    lambda g: g.name == machine, lambda g: g.stem == machine):
            found = next((graph for graph in self.graphs if key(graph)), None)
            if found:
                break
        self._graph_of[machine] = found
        return found

    def resolve(self, record):
        """The step of a trace record: the tracked state after it, the
        edges it took, and what did not add up."""
        machine = record["machine"]
        graph = self.graph_for(machine)
        warnings = []
        if graph is None:
            warnings.append(f"no graph for machine '{machine}'")
        tracked = self.state.get(machine)
        if tracked is None and graph is not None:
            tracked = graph.initial

        edges = []
        if record["from"] is None:
            prev, state = None, record["to"]
        else:
            source, event, to = record["from"], record["event"], record["to"]
            prev = tracked if source == ANY_STATE else source
            if source != ANY_STATE and tracked is not None and source != tracked:
                warnings.append(f"'{machine}' was in '{tracked}', log says '{source}'")
            state = prev if to == INTERNAL_TARGET else to
            if graph is not None:
                edges = graph.edge_ids(source, event, to)
                if not edges and source != ANY_STATE:
                    edges = graph.edge_ids(ANY_STATE, event, to)  # per-source fallback path
                if not edges:
                    warnings.append(f"no edge '{source} -({event})-> {to}' in {graph.stem}")
                if event not in graph.events:
                    warnings.append(f"unknown event '{event}' for {graph.stem}")
        if graph is not None:
            for name in (prev, state):
                if name is not None and name not in graph.states:
                    warnings.append(f"unknown state '{name}' for {graph.stem}")
        self.state[machine] = state
        return {
            "machine": machine,
            "graph": graph.stem if graph else None,
            "from": record["from"],
            "event": record["event"],
            "to": record["to"],
            "prev": prev,
            "state": state,
            "edges": edges,
            "ts": record["ts"],
            "raw": record["raw"],
            "warnings": warnings,
        }

    def add_line(self, line):
        record = parse_line(line)
        if record is None:
            return None
        with self.lock:
            step = self.resolve(record)
            step["i"] = len(self.steps)
            self.steps.append(step)
            for subscriber in list(self.subscribers):
                subscriber.put(step)
        return step

    def end(self):
        with self.lock:
            self.ended = True
            for subscriber in list(self.subscribers):
                subscriber.put(None)

    def subscribe(self, start):
        subscriber = queue.Queue()
        with self.lock:
            backlog = self.steps[start:]
            ended = self.ended
            self.subscribers.add(subscriber)
        return subscriber, backlog, ended

    def unsubscribe(self, subscriber):
        with self.lock:
            self.subscribers.discard(subscriber)


# --- log sources: each yields decoded lines --------------------------------

def read_stream(stream):
    for raw in stream:
        yield raw.decode("utf-8", errors="replace")


def source_tcp(target):
    host, _, port = target.rpartition(":")
    with socket.create_connection((host or "localhost", int(port))) as conn:
        print(f"fsmview: connected to {host}:{port}", file=sys.stderr)
        yield from read_stream(conn.makefile("rb"))


def source_listen(port):
    with socket.create_server(("", int(port))) as server:
        print(f"fsmview: waiting for a log pusher on port {port}", file=sys.stderr)
        conn, peer = server.accept()
        print(f"fsmview: log pusher connected from {peer[0]}", file=sys.stderr)
        with conn:
            yield from read_stream(conn.makefile("rb"))


def source_serial(spec):
    import serial  # pyserial; its absence is reported before the server starts
    device, _, baud = spec.partition("@")
    baud = int(baud or DEFAULT_BAUD)
    with serial.Serial(device, baud, timeout=None) as port:
        print(f"fsmview: reading {device} at {baud} baud", file=sys.stderr)
        while True:
            raw = port.readline()
            if not raw:
                return
            yield raw.decode("utf-8", errors="replace")


def source_stdin():
    yield from read_stream(sys.stdin.buffer)


def source_file(path):
    with open(path, "rb") as file:
        yield from read_stream(file)


def pump(source, trace, save, server):
    """Feeds the trace from the source until it ends. A source that
    fails before its first line takes the server down with it: nothing
    to show, and the error must not hide behind a running page."""
    received = False
    try:
        for line in source:
            received = True
            if save is not None:
                save.write(line if line.endswith("\n") else line + "\n")
                save.flush()
            trace.add_line(line)
        print("fsmview: log source ended", file=sys.stderr)
    except (OSError, ValueError) as error:  # pyserial's SerialException is an OSError
        print(f"fsmview: source failed: {error}", file=sys.stderr)
        if isinstance(error, PermissionError) or "Permission denied" in str(error):
            print("fsmview: a serial port usually needs the dialout group "
                  "(sudo usermod -aG dialout $USER, then log in again)", file=sys.stderr)
        if not received:
            trace.failed = True
            server.shutdown()
    finally:
        trace.end()


# --- HTTP: the page, the graphs, the steps, and the live stream ------------

class Handler(BaseHTTPRequestHandler):
    trace = None
    config = None
    page = None

    def log_message(self, *_):
        pass

    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        if url.path == "/":
            self.reply(200, "text/html; charset=utf-8", self.page)
        elif url.path == "/state":
            with self.trace.lock:
                state = dict(self.config, ended=self.trace.ended, count=len(self.trace.steps),
                             warnings=self.trace.warnings,
                             graphs=[graph.as_json() for graph in self.trace.graphs])
            self.reply_json(state)
        elif url.path == "/events":
            start = int(query.get("from", ["0"])[0])
            with self.trace.lock:
                self.reply_json({"steps": self.trace.steps[start:], "ended": self.trace.ended})
        elif url.path == "/stream":
            self.stream(query)
        else:
            self.reply(404, "text/plain", "not found")

    def reply(self, status, content_type, body):
        data = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def reply_json(self, payload):
        self.reply(200, "application/json", json.dumps(payload))

    def stream(self, query):
        # Server-sent events; the browser reconnects with Last-Event-ID,
        # so the backlog starts where the client left off
        last = self.headers.get("Last-Event-ID")
        start = int(last) + 1 if last else int(query.get("from", ["0"])[0])
        subscriber, backlog, ended = self.trace.subscribe(start)
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            for step in backlog:
                self.send_event(step)
            if ended:
                self.wfile.write(b"event: end\ndata: {}\n\n")
                return
            while True:
                try:
                    step = subscriber.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue
                if step is None:
                    self.wfile.write(b"event: end\ndata: {}\n\n")
                    self.wfile.flush()
                    return
                self.send_event(step)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.trace.unsubscribe(subscriber)

    def send_event(self, step):
        self.wfile.write(f"id: {step['i']}\ndata: {json.dumps(step)}\n\n".encode("utf-8"))
        self.wfile.flush()


def build_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser(
            description="Live and replay viewer for fsm state machines",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=__doc__.split("Usage:")[0],
        )
    parser.add_argument("graphs", nargs="*", metavar="GRAPH",
                        help=".dot file written by fsm::writeDot, or a directory of them "
                             f"(default: the .dot files in {DEFAULT_GRAPHS}/)")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--tcp", metavar="HOST:PORT", help="connect to a raw log stream")
    source.add_argument("--listen", metavar="PORT", help="accept one log pusher")
    source.add_argument("--serial", metavar="DEV[@BAUD]",
                        help=f"read a serial port directly (the default source: "
                             f"{DEFAULT_DEVICE} at {DEFAULT_BAUD} baud)")
    source.add_argument("--stdin", action="store_true", help="read the log from stdin")
    source.add_argument("--replay", metavar="FILE", help="replay a saved log")
    parser.add_argument("--save", metavar="FILE",
                        help="where to save the live log (default fsmview-<timestamp>.log)")
    parser.add_argument("--http", type=int, default=8420, metavar="PORT",
                        help="port of the viewer page (default 8420)")
    parser.add_argument("--history", type=int, default=8, metavar="N",
                        help="steps shown in fading colors (default 8)")
    parser.add_argument("--map", action="append", default=[], metavar="MACHINE=GRAPH",
                        help="graph (file stem) for a machine id when several graphs claim it")
    parser.add_argument("--dot", metavar="PATH", default=shutil.which("dot") or os.environ.get("DOT"),
                        help="Graphviz dot binary (default: from PATH or $DOT); without it "
                             "the page renders the graphs with viz.js from jsDelivr")
    return parser


def run(args):
    if not args.graphs:
        if not any(Path(DEFAULT_GRAPHS).glob("*.dot")):
            sys.exit(f"fsmview: no GRAPH given and no .dot files in {DEFAULT_GRAPHS}/ "
                     f"(write them with \"west build -t dot\" or tools/dotgen)")
        args.graphs = [DEFAULT_GRAPHS]
    if not (args.tcp or args.listen or args.serial or args.stdin or args.replay):
        args.serial = DEFAULT_DEVICE
    if args.serial:
        try:
            import serial  # noqa: F401
        except ImportError:
            sys.exit("fsmview: reading a serial port needs pyserial (pip install pyserial); "
                     "or feed the log through --stdin, --tcp or --listen")
    mapping = {}
    for entry in args.map:
        machine, _, stem = entry.partition("=")
        if not stem:
            sys.exit(f"fsmview: --map expects MACHINE=GRAPH, got '{entry}'")
        mapping[machine] = stem

    graphs = load_graphs(args.graphs)
    if args.dot:
        print(f"fsmview: rendering with {args.dot}", file=sys.stderr)
        render_graphs(graphs, args.dot)
    else:
        print("fsmview: no dot binary, the page renders with viz.js (needs internet)",
              file=sys.stderr)
    trace = Trace(graphs, mapping)
    for warning in trace.warnings:
        print(f"fsmview: {warning}", file=sys.stderr)

    save = None
    if args.replay:
        source, mode, description = source_file(args.replay), "replay", args.replay
    else:
        path = args.save or datetime.now().strftime("fsmview-%Y%m%d-%H%M%S.log")
        save = open(path, "a", encoding="utf-8")
        print(f"fsmview: saving the log to {path}", file=sys.stderr)
        mode = "live"
        if args.tcp:
            source, description = source_tcp(args.tcp), f"tcp {args.tcp}"
        elif args.listen:
            source, description = source_listen(args.listen), f"listen {args.listen}"
        elif args.serial:
            source, description = source_serial(args.serial), f"serial {args.serial}"
        else:
            source, description = source_stdin(), "stdin"

    Handler.trace = trace
    Handler.config = {"history": args.history, "mode": mode, "source": description}
    Handler.page = (Path(__file__).with_name("index.html")).read_text(encoding="utf-8")
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    except OSError as error:
        sys.exit(f"fsmview: cannot serve on port {args.http}: {error.strerror} "
                 f"(another viewer running? pick one with --http)")
    server.daemon_threads = True
    threading.Thread(target=pump, args=(source, trace, save, server), daemon=True).start()
    print(f"fsmview: running - open http://localhost:{args.http}/ in your browser",
          file=sys.stderr)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if save is not None:
            save.close()
    if trace.failed:
        sys.exit(1)


def main(argv=None):
    run(build_parser().parse_args(argv))


try:
    from west.commands import WestCommand
except ImportError:  # standalone use without west
    WestCommand = None

if WestCommand is not None:

    class LiveView(WestCommand):
        def __init__(self):
            super().__init__(
                "fsm_liveview",
                "watch or replay fsm state machines in the browser",
                __doc__.split("Usage:")[0],
                requires_workspace=False,
            )

        def do_add_parser(self, parser_adder):
            parser = parser_adder.add_parser(
                self.name, help=self.help, description=self.description,
                formatter_class=argparse.RawDescriptionHelpFormatter)
            return build_parser(parser)

        def do_run(self, args, unknown_args):
            run(args)


if __name__ == "__main__":
    main()
