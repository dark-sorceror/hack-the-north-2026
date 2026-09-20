"""Waveshare DDSM115 hub motor over RS485 (115200 8N1, 10-byte frames, CRC-8/MAXIM).

As a library:
    from ddsm115 import Bus
    with Bus() as bus:                      # auto-finds the Waveshare adapter; brakes on exit
        bus.speed(1, 50, accel=30)          # rpm, returns feedback dict (or None if no reply)
        print(bus.status(1))                # {'id', 'mode', 'current_A', 'rpm', 'temp_C', 'err'}

From the command line:
    python ddsm115.py                       -> self-check + list motors on the bus (nothing spins)
    python ddsm115.py spin 1:30 2:-30       -> motor 1 at 30 rpm, motor 2 at -30 rpm for 2 s, then brake
    python ddsm115.py setid 2               -> ONLY ONE MOTOR ON THE BUS, then power-cycle it

Port: auto-detected (WCH CH34x chip). Override with Bus(port=...) or env DDSM115_PORT
(Windows e.g. COM5, Pi 5 usually /dev/ttyACM0). On Linux: sudo usermod -aG dialout $USER, then re-login.

VENDORED (retriever): a teammate's file, known to drive the real chassis over a
Waveshare USB-RS485 adapter. Kept as close to theirs as possible so the two can
be diffed. The one behavioural change is the guarded pyserial import below: with pyserial missing,
the pure frame/CRC/parse functions still import (the bridge's tests and its
stdlib-only fake mode need that), and Bus()/find_port() raise ImportError
instead. With pyserial installed, behaviour is identical. The bridge driver,
retriever.bridge.drivers.DDSM115Driver, builds on the frame functions here
and does its own bounded I/O.
"""
import os, struct, sys, threading, time
try:
    import serial
    from serial.tools import list_ports
except ImportError:  # retriever: the only change from the teammate's file (see docstring)
    class _NoPyserial:
        def __getattr__(self, name):
            raise ImportError("pyserial is not installed: on the Pi, nav-pi/setup.sh "
                              "installs it (python3-serial); elsewhere, pip install pyserial")
    serial = list_ports = _NoPyserial()

CURRENT, VELOCITY, POSITION = 1, 2, 3
# Mirror-mounted wheels: + rpm means "robot forward" for every ID. Override per Bus(flipped=...).
# ponytail: flip applies to speed/current; position-mode targets on these IDs are not mirrored.
FLIPPED = {3, 4}


# ---- raw protocol frames (no I/O) ----

def crc8(data):  # CRC-8/MAXIM (reflected poly 0x31 -> 0x8C)
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
    return c


def frame(*b9):
    return bytes(b9) + bytes([crc8(b9)])


def drive(mid, value, accel=0, brake=False):
    # value: rpm (-330..330) | current (-32767..32767 = -8..8A) | position (0..32767 = 0..360deg)
    # accel: 0.1 ms per rpm of speed change (0 = fastest, 255 = gentlest); speed mode only
    hi, lo = struct.pack(">h" if value < 0x8000 else ">H", value)
    return frame(mid, 0x64, hi, lo, 0, 0, accel, 0xFF if brake else 0, 0)


def query(mid):
    return frame(mid, 0x74, 0, 0, 0, 0, 0, 0, 0)


def set_mode(mid, mode):  # no CRC, no reply
    return bytes([mid, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode])


def set_id(new_id):  # no CRC; send 5x, one motor on bus, once per power-up
    return bytes([0xAA, 0x55, 0x53, new_id, 0, 0, 0, 0, 0, 0])


def parse(r):
    """Reply to 0x64 (bytes 6-7 = u16 position) or 0x74 (byte 6 = temp C, byte 7 = u8 position)."""
    if len(r) != 10 or crc8(r[:9]) != r[9]:
        return None
    mid, mode, cur, rpm, b6, b7, err = struct.unpack(">BBhhBBB", r[:9])
    return dict(id=mid, mode=mode, current_A=cur * 8 / 32767, rpm=rpm, b6=b6, b7=b7, err=err)


def find_port():
    if os.environ.get("DDSM115_PORT"):
        return os.environ["DDSM115_PORT"]
    for p in list_ports.comports():
        if p.vid == 0x1A86:  # WCH: CH343 on the Waveshare USB-RS485
            return p.device
    raise RuntimeError("No Waveshare USB-RS485 found; pass Bus(port=...) or set DDSM115_PORT")


# ---- high-level API ----

class Bus:
    """One RS485 bus, many motors. Thread-safe: each request/reply holds a lock."""

    def __init__(self, port=None, flipped=FLIPPED):
        # serial_for_url: same as serial.Serial for real ports, also accepts "loop://" for testing
        self.ser = serial.serial_for_url(port or find_port(), 115200, timeout=0.05)
        self.flipped = set(flipped)
        self.lock = threading.Lock()
        self.used = set()  # motors we drove, braked on close()

    def _xfer(self, pkt):
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(pkt)
            r = parse(self.ser.read(10))
        if r and r["id"] in self.flipped:
            r["current_A"], r["rpm"] = -r["current_A"], -r["rpm"]
        return r

    def mode(self, mid, mode):
        """CURRENT / VELOCITY (default at power-up) / POSITION (motor must be < 10 rpm)."""
        with self.lock:
            self.ser.write(set_mode(mid, mode))
        time.sleep(0.05)

    def drive(self, mid, value, accel=0, brake=False):
        """Raw setpoint in whatever mode the motor is in (see drive() above for units)."""
        self.used.add(mid)
        return self._xfer(drive(mid, -value if mid in self.flipped else value, accel, brake))

    def speed(self, mid, rpm, accel=0):
        return self.drive(mid, max(-330, min(330, int(rpm))), accel)

    def brake(self, mid):
        return self.drive(mid, 0, brake=True)

    def status(self, mid):
        r = self._xfer(query(mid))
        if r:
            r["temp_C"], r["pos_deg"] = r.pop("b6"), r.pop("b7") * 360 / 256
        return r

    def scan(self, ids=range(1, 11)):
        return [m for m in ids if self.status(m)]

    def set_id(self, new_id):
        """ONLY ONE MOTOR ON THE BUS. Takes effect after the motor is power-cycled."""
        for _ in range(5):
            with self.lock:
                self.ser.write(set_id(new_id))
            time.sleep(0.01)

    def close(self):
        for m in self.used:
            self.brake(m)
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def demo():
    # frames straight from the Waveshare wiki
    assert query(1).hex(" ") == "01 74 00 00 00 00 00 00 00 04"
    assert drive(1, 0).hex(" ") == "01 64 00 00 00 00 00 00 00 50"
    assert drive(1, -50).hex(" ") == "01 64 ff ce 00 00 00 00 00 da"
    assert drive(1, 30000).hex(" ") == "01 64 75 30 00 00 00 00 00 a7"
    assert drive(1, 0, brake=True).hex(" ") == "01 64 00 00 00 00 00 ff 00 d1"
    assert frame(0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0).hex(" ") == "c8 64 00 00 00 00 00 00 00 de"
    assert set_id(1).hex(" ")[:12] == "aa 55 53 01 "
    print("self-check ok")


if __name__ == "__main__":
    demo()
    cmd = sys.argv[1:] or ["scan"]
    with Bus() as bus:
        if cmd[0] == "scan":
            for m in range(1, 11):
                print(m, bus.status(m))
        elif cmd[0] == "spin":
            speeds = {int(m): int(r) for m, r in (p.split(":") for p in cmd[1:])}
            for m in speeds:
                bus.mode(m, VELOCITY)
            end = time.time() + 2
            while time.time() < end:  # ~500Hz bus total, shared across motors
                print([bus.speed(m, r) for m, r in speeds.items()])
                time.sleep(0.02)
        elif cmd[0] == "setid":
            bus.set_id(int(cmd[1]))
            print("sent; power-cycle, then scan")
