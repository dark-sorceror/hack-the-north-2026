"""DDSM115Driver, DDSM115Bus, the vendored ddsm115 frames, and scripts/wheel_check.py.
"""

import contextlib
import importlib.util
import io
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from retriever.bridge import ddsm115 as dd  # noqa: E402

HAVE_PYSERIAL = importlib.util.find_spec("serial") is not None


def i16(v):
    return list((int(v) & 0xFFFF).to_bytes(2, "big"))


# ---------------------------------------------------------------- frames


class TestVendoredFrames(unittest.TestCase):
    def test_the_teammates_self_check_passes(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            dd.demo()
        self.assertIn("self-check ok", out.getvalue())

    def test_frames_match_the_wiki_vectors(self):
        self.assertEqual(dd.query(1).hex(" "), "01 74 00 00 00 00 00 00 00 04")
        self.assertEqual(dd.drive(1, 0).hex(" "), "01 64 00 00 00 00 00 00 00 50")
        self.assertEqual(dd.drive(1, -50).hex(" "), "01 64 ff ce 00 00 00 00 00 da")
        self.assertEqual(dd.drive(1, 30000).hex(" "), "01 64 75 30 00 00 00 00 00 a7")
        self.assertEqual(dd.drive(1, 0, brake=True).hex(" "), "01 64 00 00 00 00 00 ff 00 d1")
        self.assertEqual(dd.set_mode(3, dd.VELOCITY).hex(" "), "03 a0 00 00 00 00 00 00 00 02")

    def test_crc_is_crc8_maxim(self):
        self.assertEqual(dd.crc8(b"123456789"), 0xA1)   # the catalogue check value

    def test_parse_signed_fields_and_rejects_bad_frames(self):
        r = dd.parse(dd.frame(4, 2, *i16(-4096), *i16(-57), 0x12, 0x34, 0x08))
        self.assertEqual((r["id"], r["mode"], r["rpm"], r["err"]), (4, 2, -57, 8))
        self.assertAlmostEqual(r["current_A"], -4096 * 8 / 32767)
        self.assertEqual((r["b6"] << 8) | r["b7"], 0x1234)
        good = dd.frame(4, 2, 0, 0, 0, 0, 0, 0, 0)
        self.assertIsNone(dd.parse(good[:9] + bytes([good[9] ^ 1])))   # bad CRC
        self.assertIsNone(dd.parse(good[:9]))                           # short
        self.assertIsNone(dd.parse(b""))

    def test_the_bridge_imports_neither_pyserial_nor_ddsm115(self):
        code = ("import sys; sys.path.insert(0, 'src'); "
                "import retriever.bridge.server, retriever.bridge.drivers; "
                "bad = [m for m in ('serial', 'retriever.bridge.ddsm115') "
                "if m in sys.modules]; "
                "assert not bad, bad; "
                "from retriever.bridge import ddsm115; ddsm115.demo()")
        # -S: no site-packages at all, like a bare python3 on the Pi
        res = subprocess.run([sys.executable, "-S", "-c", code], cwd=ROOT,
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(res.returncode, 0, res.stderr)
        self.assertIn("self-check ok", res.stdout)

    @unittest.skipIf(HAVE_PYSERIAL, "pyserial is installed here")
    def test_without_pyserial_the_bus_says_it_is_missing(self):
        with self.assertRaises(ImportError) as ctx:
            dd.Bus(port="/dev/null")
        self.assertIn("pyserial is not installed", str(ctx.exception))
        with self.assertRaises(ImportError):
            dd.find_port()


if __name__ == "__main__":
    unittest.main()
