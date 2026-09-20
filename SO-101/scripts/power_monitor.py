#!/usr/bin/env python3
"""
Log board health during a battery test.

    python3 power_monitor.py --out battery-test.csv
    python3 power_monitor.py --port /dev/soarm_leader      # also log servo rail

Samples thermals, CPU frequency (a drop means throttling), load, and - if a
servo bus is given - the actual supply voltage the servos see. That last one
is the number that matters: a battery sagging under stall current shows up on
the servo rail long before the board itself misbehaves.

Ctrl-C to stop. Prints a summary with the minimum voltage seen.
"""

import argparse
import csv
import glob
import os
import sys
import time

# Feetech/STS control table - the Hiwonder HX-30HM reuses it wholesale.
ADDR_PRESENT_VOLTAGE = 62  # 1 byte, tenths of a volt (121 -> 12.1 V)


def read_thermals() -> dict[str, float]:
    out = {}
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        try:
            with open(os.path.join(zone, "type")) as f:
                name = f.read().strip()
            with open(os.path.join(zone, "temp")) as f:
                out[name] = int(f.read().strip()) / 1000.0
        except OSError:
            continue
    return out


def read_cpu_mhz() -> list[int]:
    freqs = []
    for path in sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq")):
        try:
            with open(path) as f:
                freqs.append(int(f.read().strip()) // 1000)
        except OSError:
            continue
    return freqs


class ServoBus:
    """Minimal STS-protocol reader. Deliberately not lerobot: this must work
    from system python 3.10, and must not risk triggering the recalibration
    prompt that a full connect() can."""

    def __init__(self, port: str, baud: int = 1_000_000):
        import serial

        self.ser = serial.Serial(port, baud, timeout=0.05)

    def read_byte(self, motor_id: int, addr: int) -> int | None:
        length = 1
        chk = (~(motor_id + 0x04 + 0x02 + addr + length)) & 0xFF
        pkt = bytes([0xFF, 0xFF, motor_id, 0x04, 0x02, addr, length, chk])
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        time.sleep(0.005)
        resp = self.ser.read(7)
        if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == motor_id:
            return resp[5]
        return None

    def voltages(self, ids: range) -> dict[int, float]:
        out = {}
        for i in ids:
            v = self.read_byte(i, ADDR_PRESENT_VOLTAGE)
            if v is not None:
                out[i] = v / 10.0
        return out

    def close(self):
        self.ser.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="battery-test.csv")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--port", help="servo bus, e.g. /dev/soarm_follower - enables voltage logging")
    ap.add_argument("--ids", default="1-6", help="servo id range, default 1-6")
    args = ap.parse_args()

    lo, _, hi = args.ids.partition("-")
    ids = range(int(lo), int(hi or lo) + 1)

    bus = None
    if args.port:
        try:
            bus = ServoBus(args.port)
            print(f"logging servo rail from {args.port} (ids {ids.start}-{ids.stop - 1})")
        except Exception as exc:  # noqa: BLE001
            print(f"could not open {args.port}: {exc}; continuing without voltage", file=sys.stderr)

    zones = sorted(read_thermals())
    ncpu = len(read_cpu_mhz())
    cols = (
        ["t", "load1"]
        + [f"temp_{z}" for z in zones]
        + [f"cpu{i}_mhz" for i in range(ncpu)]
        + (["v_min", "v_mean"] if bus else [])
    )

    v_floor = None
    peak_temp = 0.0
    min_mhz = 10**9
    t0 = time.time()

    with open(args.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        print(f"logging to {args.out}; Ctrl-C to stop\n")
        try:
            while True:
                temps = read_thermals()
                mhz = read_cpu_mhz()
                load1 = os.getloadavg()[0]
                row = [round(time.time() - t0, 1), round(load1, 2)]
                row += [round(temps.get(z, float("nan")), 1) for z in zones]
                row += mhz

                vline = ""
                if bus:
                    volts = bus.voltages(ids)
                    if volts:
                        vmin, vmean = min(volts.values()), sum(volts.values()) / len(volts)
                        v_floor = vmin if v_floor is None else min(v_floor, vmin)
                        row += [vmin, round(vmean, 2)]
                        vline = f"  rail {vmin:.1f}V (min seen {v_floor:.1f}V)"
                    else:
                        row += ["", ""]
                        vline = "  rail: no reply"

                w.writerow(row)
                fh.flush()

                peak_temp = max(peak_temp, max(temps.values(), default=0.0))
                if mhz:
                    min_mhz = min(min_mhz, min(mhz))
                print(
                    f"\r{row[0]:7.1f}s  load {load1:4.2f}  "
                    f"{max(temps.values(), default=0):4.1f}C  "
                    f"{min(mhz) if mhz else 0:5d}MHz{vline}   ",
                    end="",
                    flush=True,
                )
                time.sleep(args.interval)
        except KeyboardInterrupt:
            pass
        finally:
            if bus:
                bus.close()

    print("\n\n--- summary ---")
    print(f"  duration      : {time.time() - t0:.0f}s")
    print(f"  peak temp     : {peak_temp:.1f} C")
    print(f"  lowest CPU    : {min_mhz} MHz  (throttling if well below max)")
    if v_floor is not None:
        print(f"  lowest rail   : {v_floor:.1f} V")
        if v_floor < 10.5:
            print("  WARNING: below ~10.5V the servos are browning out, not the board.")
    print(f"  csv           : {args.out}")


if __name__ == "__main__":
    main()
