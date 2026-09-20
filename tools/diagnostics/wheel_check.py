#!/usr/bin/env python3
"""DDSM115 wheel check: find the RS485 bus, list the motors, spin them one at a time.

    python3 tools/diagnostics/wheel_check.py scan                    # read-only: IDs, mode, temp, errors
    python3 tools/diagnostics/wheel_check.py spin 1 --yes            # motor 1, 20 rpm robot-forward, 2 s
    python3 tools/diagnostics/wheel_check.py spin 3 -30 --seconds 3 --yes
    python3 tools/diagnostics/wheel_check.py sides --yes             # left/right IDs? + is forward?
    python3 tools/diagnostics/wheel_check.py rev 1 --yes             # one turn: checks counts_per_rev

WHEELS OFF THE GROUND. `scan` is read-only (0x74 queries: nothing moves).
Everything else spins motors and needs --yes. Speeds are capped at 60 rpm
unless --fast (and at 330 rpm, the protocol limit, always). Every motor this
tool drove is BRAKED on exit, on error, on Ctrl-C and on SIGTERM.

+RPM means "robot forward": motors in --flipped-ids (default 1,2, the
robot's mirror-mounted pair, measured) get -RPM on the wire. `sides` checks exactly
the mapping the bridge uses (DDSM115Driver) and prints the flags to pass.

Port: --port, else $DDSM115_PORT, else the one WCH (USB VID 1a86) adapter.
Needs pyserial (hardware/nav_pi/setup.sh installs it on the Pi). Options go
AFTER the subcommand. Stop the bridge first: the port is opened exclusively,
so a second program is refused rather than becoming a second bus master.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any, Callable, TextIO

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from retriever.bridge import ddsm115 as dd  # noqa: E402  (frames: no pyserial needed)
from retriever.bridge.drivers import (  # noqa: E402
    DDSM115_COUNTS_PER_REV,
    DDSM115_MAX_RPM,
    DDSM115_MODES,
    DDSM115_POSITION_SIGN,
    PYSERIAL_MISSING,
    WCH_USB_VID,
    DDSM115Bus,
    DDSM115Driver,
    ddsm115_error_names,
    parse_id_list,
    rpm_to_mps,
)
from retriever.navigation.odometry import unwrap_ticks  # noqa: E402

BANNER = "WHEELS OFF THE GROUND: chassis on a stand, no wheel touching anything."
SAFE_RPM = 60           # without --fast
MAX_SECONDS = 30.0
STEP_S = 0.02           # 50 Hz command rate, like the bridge's state stream

Say = Callable[..., None]


def main(argv: list[str] | None = None, *, open_bus: Callable[..., Any] | None = None,
         sleep: Callable[[float], None] = time.sleep,
         clock: Callable[[], float] = time.monotonic,
         out: TextIO = sys.stdout, ask: Callable[[str], str] = input) -> int:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", help="the USB-RS485 adapter (default: $DDSM115_PORT, else "
                        "the one WCH adapter)")
    common.add_argument("--flipped-ids", default="1,2",
                        help="mirror-mounted motors, which get -rpm (default 1,2, measured)")
    common.add_argument("--counts-per-rev", type=int, default=DDSM115_COUNTS_PER_REV,
                        help=f"counts per wheel turn (default {DDSM115_COUNTS_PER_REV})")
    common.add_argument("--reply-timeout-ms", type=float, default=10.0,
                        help="wait this long for each motor's reply (the bridge's default)")
    common.add_argument("--yes", action="store_true",
                        help="allow spinning: the wheels are off the ground")
    common.add_argument("--fast", action="store_true",
                        help=f"allow over {SAFE_RPM} rpm (never over {DDSM115_MAX_RPM})")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("scan", parents=[common], help="read-only: list the motors")
    p.add_argument("--ids", default="1-10", help="IDs to query (default 1-10)")
    p = sub.add_parser("spin", parents=[common], help="spin ONE motor, then brake")
    p.add_argument("id", type=int)
    p.add_argument("rpm", type=float, nargs="?", default=20.0,
                   help="robot-forward rpm (default 20)")
    p.add_argument("--seconds", type=float, default=2.0)
    p = sub.add_parser("sides", parents=[common],
                       help="spin the left side, then the right, and ask what moved")
    p.add_argument("--left-ids", default="1,2")
    p.add_argument("--right-ids", default="3,4")
    p.add_argument("--rpm", type=float, default=15.0)
    p.add_argument("--seconds", type=float, default=2.0)
    p = sub.add_parser("rev", parents=[common],
                       help="turn one motor counts_per_rev counts: is that one revolution?")
    p.add_argument("id", type=int)
    p.add_argument("--rpm", type=float, default=10.0)
    args = ap.parse_args(argv)

    def say(*a: Any) -> None:
        print(*a, file=out, flush=True)

    try:
        flipped = frozenset(parse_id_list(args.flipped_ids))
    except ValueError as exc:
        say(exc)
        return 2
    if args.cmd != "scan":
        say(BANNER)
        if not args.yes:
            say("Nothing sent. Put the chassis on a stand, then run again with --yes.")
            return 2
        limit = DDSM115_MAX_RPM if args.fast else SAFE_RPM
        if abs(args.rpm) > limit:
            say(f"{args.rpm:g} rpm capped to {math.copysign(limit, args.rpm):+g}"
                + ("" if args.fast else f" (--fast allows up to {DDSM115_MAX_RPM})"))
            args.rpm = math.copysign(limit, args.rpm)
        if hasattr(args, "seconds") and not 0 < args.seconds <= MAX_SECONDS:
            args.seconds = min(max(args.seconds, 0.1), MAX_SECONDS)
            say(f"--seconds limited to {args.seconds:g}")

    try:
        bus = (open_bus or _open_bus)(args.port, args.reply_timeout_ms / 1000.0, say)
    except (ConnectionError, ImportError) as exc:
        say(f"\n{exc}")
        return 1

    touched: set[int] = set()
    previous = None
    try:  # SIGTERM too must reach the brakes below
        previous = signal.signal(signal.SIGTERM, _raise_interrupt)
    except ValueError:  # not the main thread
        pass
    try:
        command = {"scan": cmd_scan, "spin": cmd_spin, "sides": cmd_sides, "rev": cmd_rev}
        return command[args.cmd](bus, args, flipped, touched, say, sleep, clock, ask)
    except KeyboardInterrupt:
        say("\ninterrupted")
        return 130
    except Exception as exc:  # ConnectionError from the driver, SerialException, ...
        say(f"\n{type(exc).__name__}: {exc}")
        return 1
    finally:
        try:
            brake_all(bus, touched, say)
        finally:
            bus.close()
            if previous is not None:
                signal.signal(signal.SIGTERM, previous)


def _open_bus(port: str | None, reply_timeout_s: float, say: Say) -> DDSM115Bus:
    try:
        ports = [p for p in dd.list_ports.comports() if getattr(p, "vid", None) == WCH_USB_VID]
    except ImportError:
        raise ImportError(PYSERIAL_MISSING) from None
    except Exception:
        ports = []
    say("WCH USB serial adapters:" if ports else "WCH USB serial adapters: none found")
    for p in ports:
        say(f"  {p.device}  {p.description or ''}")
    bus = DDSM115Bus.open(port, reply_timeout_s=reply_timeout_s)
    say(f"wheel bus: {bus.port}")
    return bus


def _raise_interrupt(signum: int, frame: Any) -> None:
    raise KeyboardInterrupt


# ---------------------------------------------------------------- helpers


def transact(bus: Any, mid: int, pkt: bytes) -> tuple[dict[str, Any] | None, str]:
    try:
        return bus.transact(mid, pkt)
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"


def position(r: dict[str, Any]) -> int:
    """The u16 position in a 0x64 reply."""
    return (r["b6"] << 8) | r["b7"]


def brake_all(bus: Any, ids: set[int], say: Say) -> None:
    """Brake every motor we drove, each confirmed by its reply. A second
    Ctrl-C while braking must not skip the rest, so nothing escapes here."""
    for mid in sorted(ids):
        why = ""
        for _ in range(3):
            try:
                r, why = bus.transact(mid, dd.drive(mid, 0, brake=True))
            except BaseException as exc:
                r, why = None, f"{type(exc).__name__}: {exc}"
            if r is not None:
                say(f"motor {mid}: braked")
                break
        else:
            say(f"motor {mid}: BRAKE NOT CONFIRMED ({why}). Cut motor power if it still turns.")


def ensure_velocity(bus: Any, ids: list[int], say: Say, sleep: Callable[[float], None]) -> bool:
    """Switch to VELOCITY, then confirm it read-only before any drive frame:
    in POSITION mode a drive frame is an angle target, not a speed."""
    for mid in ids:
        bus.send(dd.set_mode(mid, dd.VELOCITY))
        sleep(0.05)
    ok = True
    for mid in ids:
        r, why = None, ""
        for _ in range(3):
            r, why = transact(bus, mid, dd.query(mid))
            if r is not None:
                break
        if r is None:
            say(f"motor {mid}: no reply ({why}). Run `scan`; check its ID, its power and the "
                "RS485 A/B wiring.")
            ok = False
        elif r["mode"] != dd.VELOCITY:
            say(f"motor {mid} is in {DDSM115_MODES.get(r['mode'], r['mode'])} mode after being "
                "switched to VELOCITY: not spinning it. Is something else on this bus?")
            ok = False
        elif r["err"]:
            say(f"motor {mid} reports {', '.join(ddsm115_error_names(r['err']))}: "
                "spinning anyway, watch it")
    return ok


def ask_choice(ask: Callable[[str], str], prompt: str, choices: str) -> str:
    for _ in range(5):
        answer = ask(prompt).strip().lower()[:1]
        if answer in choices:
            return answer
    return "?"


def csv(ids: Any) -> str:
    return ",".join(str(i) for i in ids) or '""'     # an empty list still pastes into a shell


# ---------------------------------------------------------------- commands


def cmd_scan(bus: Any, args: argparse.Namespace, flipped: frozenset[int], touched: set[int],
             say: Say, sleep: Callable[[float], None], clock: Callable[[], float],
             ask: Callable[[str], str]) -> int:
    ids = parse_id_list(args.ids)
    say(f"\nquerying IDs {csv(ids)} (0x74: read-only, nothing moves)")
    say("  ID  mode       temp   current   rpm  pos(u8)  errors            reply")
    found: list[int] = []
    wrong_mode: list[int] = []
    slowest = 0.0
    for mid in ids:
        t0 = clock()
        r, _ = transact(bus, mid, dd.query(mid))
        dt = clock() - t0
        if r is None:
            continue
        found.append(mid)
        slowest = max(slowest, dt)
        if r["mode"] != dd.VELOCITY:
            wrong_mode.append(mid)
        errors = ", ".join(ddsm115_error_names(r["err"])) or "none"
        say(f"  {mid:>2}  {DDSM115_MODES.get(r['mode'], r['mode']):<9} {r['b6']:>3} C  "
            f"{r['current_A']:+6.2f} A  {r['rpm']:+4d}  {r['b7'] * 360 / 256:5.0f} deg  "
            f"{errors:<16} {dt * 1000:5.1f} ms")
    if not found:
        say("\nnothing answered. Check: motor power (the motors need their own supply), the "
            "RS485 A/B wires (swapped A/B is common), --port, and that nothing else has the "
            "port open.")
        return 1
    say(f"\nfound: {csv(found)}")
    missing = sorted({1, 2, 3, 4} - set(found))
    if missing:
        say(f"the bridge expects IDs 1-4 by default (left 1,2, right 3,4); not answering: "
            f"{csv(missing)}. Pass --left-ids/--right-ids if yours differ.")
    if wrong_mode:
        say(f"not in VELOCITY mode: {csv(wrong_mode)} (the bridge, spin, sides and rev switch "
            "them; velocity is also the power-up default)")
    timeout_ms = bus.reply_timeout_s * 1000
    say(f"slowest reply {slowest * 1000:.1f} ms; the bridge waits {timeout_ms:g} ms per motor"
        + (" -- too close: raise --wheel-reply-timeout-ms" if slowest * 1000 > 0.7 * timeout_ms
           else ""))
    return 0


def cmd_spin(bus: Any, args: argparse.Namespace, flipped: frozenset[int], touched: set[int],
             say: Say, sleep: Callable[[float], None], clock: Callable[[], float],
             ask: Callable[[str], str]) -> int:
    mid = args.id
    if not ensure_velocity(bus, [mid], say, sleep):
        return 1
    touched.add(mid)
    sign = -1 if mid in flipped else 1
    rpm = int(round(args.rpm))
    wire = sign * rpm
    say(f"\nmotor {mid}: {rpm:+d} rpm robot-forward"
        + (f" (mirror-mounted: {wire:+d} on the wire)" if sign < 0 else "")
        + f" for {args.seconds:g} s. Ctrl-C brakes.")
    end = clock() + args.seconds
    next_print = clock()
    misses = 0
    while clock() < end:
        r, why = bus.transact(mid, dd.drive(mid, wire, 0))
        if r is None:
            misses += 1
            if misses >= 10:
                say(f"motor {mid} stopped answering ({why})")
                return 1
        else:
            misses = 0
            if clock() >= next_print:
                say(f"  rpm {sign * r['rpm']:+4d} (robot-forward)  current "
                    f"{sign * r['current_A'] + 0.0:+.2f} A  position {position(r):>5}  errors "
                    f"{', '.join(ddsm115_error_names(r['err'])) or 'none'}")
                next_print = clock() + 0.25
        sleep(STEP_S)
    return 0


def cmd_sides(bus: Any, args: argparse.Namespace, flipped: frozenset[int], touched: set[int],
              say: Say, sleep: Callable[[float], None], clock: Callable[[], float],
              ask: Callable[[str], str]) -> int:
    left, right = parse_id_list(args.left_ids), parse_id_list(args.right_ids)
    cpr = args.counts_per_rev
    # The bridge's own driver: this checks the mapping the robot will actually use.
    drv = DDSM115Driver(bus=bus, left_ids=left, right_ids=right, flipped_ids=flipped,
                        counts_per_rev=cpr, clock=clock, sleep=sleep)
    touched.update(left + right)
    rpm = abs(args.rpm)
    v = rpm_to_mps(rpm, drv.geo.wheel_radius_m)       # the driver turns it back into rpm
    expected = rpm / 60.0 * args.seconds * cpr
    answers: dict[str, tuple[str, str, int, int]] = {}
    for name, ids, cmd in (("left", left, (v, 0.0)), ("right", right, (0.0, v))):
        ask(f"\nNext: the {name.upper()} side, motors {csv(ids)}, should roll the robot "
            f"FORWARD at {rpm:g} rpm for {args.seconds:g} s. Watch the wheels, then press "
            "Enter... ")
        moved_counts = [0, 0]
        last = drv.read_ticks()
        end = clock() + args.seconds
        while clock() < end:
            drv.set_wheels(*cmd)
            now = drv.read_ticks()
            for i in (0, 1):
                moved_counts[i] += unwrap_ticks(now[i], last[i], cpr)
            last = now
            sleep(STEP_S)
        drv.stop()
        sleep(0.3)                                    # let it stop, then count the overshoot
        now = drv.read_ticks()
        for i in (0, 1):
            moved_counts[i] += unwrap_ticks(now[i], last[i], cpr)
        own, other = (moved_counts if name == "left" else moved_counts[::-1])
        say(f"  odometry: left {moved_counts[0]:+d}, right {moved_counts[1]:+d} counts "
            f"(expected about {expected:+.0f} on the {name} side, ~0 on the other)")
        moved = ask_choice(ask, "  Which wheels turned? [l]eft side, [r]ight side, [b]oth "
                                "sides, [n]one: ", "lrbn")
        forward = "n"
        if moved != "n":
            forward = ask_choice(ask, "  Would they drive the robot FORWARD? [y]es / [n]o: ",
                                 "yn")
        answers[name] = (moved, forward, own, other)
    return sides_verdict(answers, left, right, flipped, expected, say)


def sides_verdict(answers: dict[str, tuple[str, str, int, int]], left: tuple[int, ...],
                  right: tuple[int, ...], flipped: frozenset[int], expected: float,
                  say: Say) -> int:
    ok = True
    new_left, new_right, new_flipped = left, right, set(flipped)
    say("\nverdict:")
    phases = (answers["left"][0], answers["right"][0])
    if phases == ("l", "r"):
        say("  left/right IDs: CONFIRMED")
    elif phases == ("r", "l"):
        say("  left/right IDs: SWAPPED (the motors called left are on the right)")
        new_left, new_right, ok = right, left, False
    else:
        say(f"  left/right IDs: unexpected answers {phases}. Each phase should spin exactly "
            "one side. Check the IDs with `scan`, then run `sides` again.")
        ok = False
    for name, ids in (("left", left), ("right", right)):
        moved, forward, own, other = answers[name]
        if moved == "n":
            ok = False
            continue
        if forward == "y":
            say(f"  motors {csv(ids)}: + is robot-forward, CONFIRMED")
        else:
            say(f"  motors {csv(ids)}: BACKWARDS. Flip them.")
            new_flipped ^= set(ids)
            ok = False
        # The driver assumes a motor's position rises when it turns at + raw rpm.
        # Then commanding a side forward always counts up, whatever the mounting.
        if own <= 0:
            say(f"  motors {csv(ids)}: odometry counted {own:+d} while driven forward: their "
                "position runs opposite to their rpm, so the driver's position sign is wrong "
                "for them. Do not drive the robot; tell whoever owns bridge/drivers.py.")
            ok = False
        elif not 0.5 * expected <= own <= 1.5 * expected:
            say(f"  motors {csv(ids)}: odometry counted {own:+d}, expected about "
                f"{expected:.0f}: check --counts-per-rev with `rev`, and that nothing held "
                "the wheel.")
            ok = False
        if abs(other) > 0.1 * expected:
            say(f"  the other side's odometry moved {other:+d} while only {csv(ids)} spun: "
                "the IDs overlap or a wheel was pushed.")
            ok = False
    flags = (f"--left-ids {csv(new_left)} --right-ids {csv(new_right)} "
             f"--wheel-flipped-ids {csv(sorted(new_flipped))}")
    if ok:
        say(f"\nCONFIRMED. Bridge flags for this chassis (scripts/run_bridge.py): {flags}")
        return 0
    say(f"\nNOT CONFIRMED. Suggested bridge flags: {flags}\n"
        "Run `sides` again with the same values (--left-ids, --right-ids, --flipped-ids) "
        "until it says CONFIRMED.")
    return 1


def cmd_rev(bus: Any, args: argparse.Namespace, flipped: frozenset[int], touched: set[int],
            say: Say, sleep: Callable[[float], None], clock: Callable[[], float],
            ask: Callable[[str], str]) -> int:
    mid, cpr = args.id, args.counts_per_rev
    if not ensure_velocity(bus, [mid], say, sleep):
        return 1
    touched.add(mid)
    sign = -1 if mid in flipped else 1
    rpm = max(1, int(round(abs(args.rpm))))
    ask(f"\nPut a tape mark on motor {mid}'s tyre and note where it points. It will turn "
        f"{cpr} counts forward at {rpm} rpm: exactly one revolution if counts_per_rev = {cpr} "
        "is right. Press Enter... ")
    r, why = bus.transact(mid, dd.drive(mid, 0, 0))
    if r is None:
        say(f"motor {mid}: no reply ({why})")
        return 1
    last = position(r)
    highest, total, revs_by_rpm = last, 0, 0.0
    t0 = t_prev = clock()
    deadline = t0 + 1.5 * 60.0 / rpm + 2.0
    wrap_jump = max(64, cpr // 64)       # far more than one 20 ms step at <= 60 rpm
    while total < cpr and clock() < deadline:
        r, why = bus.transact(mid, dd.drive(mid, sign * rpm, 0))
        now = clock()
        if r is not None:
            p = position(r)
            if p >= cpr:
                say(f"\nposition {p} >= --counts-per-rev {cpr}: this motor counts further. "
                    "Run again with --counts-per-rev 65536, and give the bridge the value "
                    "that passes as --wheel-counts-per-rev.")
                return 1
            step = sign * DDSM115_POSITION_SIGN * unwrap_ticks(p, last, cpr)
            if step < -wrap_jump:
                guess = 1 << highest.bit_length()    # encoders count in powers of two
                say(f"\nthe position wrapped after about {highest + 1} counts, not {cpr}: "
                    f"counts_per_rev is probably {guess}. Run again with "
                    f"--counts-per-rev {guess}.")
                return 1
            total += step
            last, highest = p, max(highest, p)
            revs_by_rpm += sign * r["rpm"] / 60.0 * (now - t_prev)
            t_prev = now
        sleep(STEP_S)
    r, why = bus.transact(mid, dd.drive(mid, 0, brake=True))
    sleep(0.3)
    r, why = bus.transact(mid, dd.drive(mid, 0, brake=True))
    if r is not None:
        total += (sign * DDSM115_POSITION_SIGN
                  * unwrap_ticks(position(r), last, cpr))     # the braking overshoot
    elapsed = t_prev - t0
    say(f"\nmoved {total:+d} counts in {elapsed:.1f} s; the motor's rpm feedback adds up to "
        f"{revs_by_rpm:.2f} revolutions")
    if revs_by_rpm > 0.2:
        say(f"  -> about {total / revs_by_rpm:.0f} counts per revolution by rpm feedback "
            "(rough: rpm is reported in whole rpm)")
    if total < -cpr / 16:
        say(f"the position ran BACKWARDS while motor {mid} turned forward: its position runs "
            "opposite to its rpm. Do not drive the robot; tell whoever owns bridge/drivers.py.")
        return 1
    if total < cpr:
        say(f"did not reach {cpr} counts in time: counts_per_rev is probably larger, or the "
            "wheel is blocked.")
    answer = ask_choice(ask, "Is the tape mark back where it started? [y]es / [s]hort of it "
                             "/ [p]ast it: ", "ysp")
    say("While the tape is on: measure the tyre's diameter. The bridge's --wheel-radius is "
        "half of it, in metres.")
    if answer == "y" and total >= cpr:
        say(f"counts_per_rev = {cpr}: CONFIRMED on motor {mid}")
        return 0
    if answer == "s":
        say(f"less than one turn in {cpr} counts: counts_per_rev is LARGER than {cpr}")
    elif answer == "p":
        say(f"more than one turn in {cpr} counts: counts_per_rev is SMALLER than {cpr}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
