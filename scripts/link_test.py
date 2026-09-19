#!/usr/bin/env python3
"""Will the Pi's 300 ms watchdog survive THIS network? Measure it, don't guess.

    .venv/bin/python scripts/link_test.py retriever-pi.local:7777 --seconds 120

The watchdog has only ever run over localhost. Real WiFi -- and a hall with a
thousand people on it -- has latency spikes of hundreds of milliseconds. If a
spike outlasts the watchdog, the Pi stops the robot mid-demo: correct behaviour,
terrible demo. This holds a live link and reports whether that will happen.

Commands ZERO velocity the whole time, so it is safe against the real robot with
motors attached. Stdlib only.

Measures:
  round trip   act seq N sent -> first state echoing seq >= N received
  state gaps   time between consecutive state messages (the Pi streams ~50 Hz)
  trips        how often the Pi reported watchdog_tripped mid-run
"""

from __future__ import annotations

import argparse
import socket
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from retriever.bridge.protocol import Act, Heartbeat, Hello, State, decode, encode

ACT_HZ = 20
HEARTBEAT_S = 0.1


def pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(p / 100 * (len(xs) - 1))))]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="HOST:PORT of the Pi bridge")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--watchdog-ms", type=float, default=300.0,
                    help="the Pi's timeout, for the verdict (default 300)")
    args = ap.parse_args()

    host, _, port = args.target.rpartition(":")
    sock = socket.create_connection((host, int(port)), timeout=5)
    sock.settimeout(1.0)
    wlock = threading.Lock()
    sent: dict[int, float] = {}
    rtts: list[float] = []
    gaps: list[float] = []
    trips = 0
    stop = threading.Event()

    def send(msg) -> None:
        with wlock:
            sock.sendall(encode(msg))

    def writer() -> None:
        seq, next_hb = 0, 0.0
        while not stop.is_set():
            now = time.monotonic()
            seq += 1
            sent[seq] = now
            send(Act(seq=seq, base_vx=0.0, base_wz=0.0))       # zero: safe on hardware
            if now >= next_hb:
                send(Heartbeat())
                next_hb = now + HEARTBEAT_S
            stop.wait(1.0 / ACT_HZ)

    threading.Thread(target=writer, daemon=True).start()

    buf, last_arrival, last_tripped, acked = b"", None, None, 0
    silent_seconds = 0          # whole seconds with no data at all: outages, not jitter
    closed_early = False
    t_end = time.monotonic() + args.seconds
    print(f"  holding a zero-velocity link to {args.target} for {args.seconds:.0f} s ...")
    try:
        while time.monotonic() < t_end:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                silent_seconds += 1
                print("  !! no data for 1 s")
                continue
            if not chunk:
                closed_early = True
                print("  !! the Pi closed the connection")
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    msg = decode(line)
                except Exception:
                    continue
                if isinstance(msg, Hello):
                    continue
                if not isinstance(msg, State):
                    continue
                now = time.monotonic()
                if last_arrival is not None:
                    gaps.append(now - last_arrival)
                last_arrival = now
                if msg.watchdog_tripped and last_tripped is False:
                    trips += 1
                    print(f"  !! watchdog tripped at t+{args.seconds - (t_end - now):.1f} s")
                last_tripped = msg.watchdog_tripped
                for s in range(acked + 1, msg.seq + 1):          # every newly echoed act
                    if s in sent:
                        rtts.append(now - sent.pop(s))
                acked = max(acked, msg.seq)
    finally:
        stop.set()
        sock.close()

    # Gaps are measured BETWEEN messages, so a link that dies near the end of
    # the run would otherwise never show up in them. Count the tail too.
    tail = time.monotonic() - last_arrival if last_arrival is not None else float("inf")

    ms = lambda x: f"{x * 1000:6.1f} ms"
    print(f"\n  state messages  {len(gaps) + 1}")
    print(f"  round trip      p50 {ms(pct(rtts, 50))}   p95 {ms(pct(rtts, 95))}   "
          f"p99 {ms(pct(rtts, 99))}   max {ms(max(rtts) if rtts else float('nan'))}")
    print(f"  state gaps      p50 {ms(pct(gaps, 50))}   p95 {ms(pct(gaps, 95))}   "
          f"p99 {ms(pct(gaps, 99))}   max {ms(max(gaps) if gaps else float('nan'))}")
    print(f"  watchdog trips  {trips}")
    print(f"  outages         {silent_seconds} s with no data at all"
          f"{', connection closed by the Pi' if closed_early else ''}; final silence {tail * 1000:.0f} ms")

    wd = args.watchdog_ms / 1000
    worst = max(max(gaps, default=0), max(rtts, default=0), tail)
    print()
    if silent_seconds or closed_early or tail > wd:
        print("  VERDICT: the link went down during the run -- that is an outage, not")
        print("  jitter, and no watchdog setting fixes it. Run scripts/pi_doctor.sh.")
        return 1
    if trips:
        print(f"  VERDICT: the watchdog tripped {trips}x on this network. Either fix the")
        print(f"  link (5 GHz, dedicated router, closer) or raise the Pi's timeout above")
        print(f"  ~{worst * 1.5 * 1000:.0f} ms -- and know that raising it slows the safety stop.")
    elif worst > wd * 0.6:
        print(f"  VERDICT: no trips, but the worst stall ({worst * 1000:.0f} ms) came within")
        print(f"  {100 * worst / wd:.0f}% of the {args.watchdog_ms:.0f} ms watchdog. Marginal -- run it longer,")
        print(f"  and on the venue network, before trusting it.")
    else:
        print(f"  VERDICT: comfortable. Worst stall {worst * 1000:.0f} ms against a "
              f"{args.watchdog_ms:.0f} ms watchdog.")
    return 1 if trips else 0


if __name__ == "__main__":
    raise SystemExit(main())
