"""The Pi deploy / diagnose tooling: syntax, lint, the service's bind, no hangs.

Runs on the laptop, no Pi needed:

    .venv/bin/python -m unittest tests.test_pi_scripts -v

tools/diagnostics/pi_doctor.sh exists for the moment the Pi is NOT reachable, so the
property that matters most is that it cannot hang there: it must reach a
VERDICT inside its time cap against a name nobody answers for.
"""

from __future__ import annotations

import os
import shlex
import shutil
import socket
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCTOR = ROOT / "tools" / "diagnostics" / "pi_doctor.sh"
DEPLOY = ROOT / "tools" / "deploy" / "nav_pi.sh"
SETUP = ROOT / "hardware" / "nav_pi" / "setup.sh"
SERVICE = ROOT / "hardware" / "nav_pi" / "retriever-bridge.service"
SHELL_FILES = sorted({*(ROOT / "tools" / "deploy").glob("*.sh"),
                      *(ROOT / "tools" / "diagnostics").glob("*.sh"),
                      *(ROOT / "hardware" / "nav_pi").glob("*.sh")})
BASH = shutil.which("bash")
UNREACHABLE = "nonexistent-pi.local"


def run(args: list[str], timeout: float, env: dict | None = None) -> subprocess.CompletedProcess:
    full_env = {**os.environ, "NO_COLOR": "1", **(env or {})}
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout,
                          cwd=ROOT, env=full_env, stdin=subprocess.DEVNULL)


def bash_lib(script: Path, code: str, timeout: float = 15) -> subprocess.CompletedProcess:
    """Source a script (functions only) and run CODE with it."""
    return run([BASH, "-c", f". {shlex.quote(str(script))}\n{code}"], timeout)


@unittest.skipUnless(BASH, "no bash")
class TestShellSyntax(unittest.TestCase):
    def test_the_scripts_exist_and_are_executable(self):
        for f in (DOCTOR, DEPLOY):
            self.assertTrue(f.is_file(), f)
            self.assertTrue(os.access(f, os.X_OK), f"{f.name} is not chmod +x")
        self.assertIn(SETUP, SHELL_FILES)

    def test_bash_n(self):
        for f in SHELL_FILES:
            with self.subTest(f.name):
                r = run([BASH, "-n", str(f)], 10)
                self.assertEqual(r.returncode, 0, r.stderr)

    def test_functions_shipped_to_the_pi_parse(self):
        # pi_deploy.sh sends these to the Pi with `declare -f`; what arrives must
        # be valid bash, and must be all of them.
        r = bash_lib(DEPLOY, "declare -f _remote_tests _remote_stop _remote_start")
        self.assertEqual(r.returncode, 0, r.stderr)
        for fn in ("_remote_tests", "_remote_stop", "_remote_start"):
            self.assertIn(f"{fn} ()", r.stdout)
        check = subprocess.run([BASH, "-n"], input=r.stdout, capture_output=True, text=True,
                               timeout=10)
        self.assertEqual(check.returncode, 0, check.stderr)

    @unittest.skipUnless(shutil.which("shellcheck"), "shellcheck not installed")
    def test_shellcheck(self):
        r = run(["shellcheck", "-x", "-S", "warning", *map(str, SHELL_FILES)], 60)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


class TestServiceBind(unittest.TestCase):
    """The unit must bind every address family. On the direct cable to the Mac
    the Pi may have only an IPv6 link-local address, so 0.0.0.0 is unreachable;
    and asyncio sets IPV6_V6ONLY, so "::" alone refuses IPv4 clients. The empty
    host (--host=) is what binds both."""

    def exec_start(self) -> list[str]:
        lines = [ln for ln in SERVICE.read_text().splitlines() if ln.startswith("ExecStart=")]
        self.assertEqual(len(lines), 1, lines)
        return shlex.split(lines[0][len("ExecStart="):])

    def host_arg(self, argv: list[str]) -> str:
        for i, a in enumerate(argv):
            if a.startswith("--host="):
                return a[len("--host="):]
            if a == "--host":
                return argv[i + 1]
        self.fail(f"no --host in ExecStart: {argv}")

    def test_exec_start_binds_all_addresses(self):
        host = self.host_arg(self.exec_start())
        self.assertNotEqual(host, "0.0.0.0", "IPv4 only: unreachable over an IPv6-only link")
        self.assertNotEqual(host, "::", "asyncio makes '::' IPv6 only (IPV6_V6ONLY)")
        self.assertEqual(host, "", "the empty host binds IPv4 and IPv6")

    @unittest.skipUnless(socket.has_ipv6, "no IPv6 on this machine")
    def test_that_bind_really_answers_on_ipv4_and_ipv6(self):
        port = _free_dual_stack_port()
        if port is None:
            self.skipTest("could not bind IPv6 loopback here")
        host = self.host_arg(self.exec_start())
        proc = subprocess.Popen(
            [sys.executable, str(ROOT / "scripts" / "run_bridge.py"), f"--host={host}",
             "--port", str(port), "--driver", "fake"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(_stop, proc)
        for addr in ("127.0.0.1", "::1"):
            with self.subTest(addr):
                line = _first_line(addr, port, deadline=time.monotonic() + 10)
                self.assertIn('"type":"hello"', line, f"no hello from [{addr}]:{port}")


@unittest.skipUnless(BASH, "no bash")
class TestDoctor(unittest.TestCase):
    def test_help(self):
        r = run([BASH, str(DOCTOR), "--help"], 10)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("VERDICT", r.stdout)

    def test_bad_arguments_exit_2(self):
        for args in (["--bogus"], ["--port", "x"], ["--budget", "0"], ["--host"]):
            with self.subTest(args):
                r = run([BASH, str(DOCTOR), *args], 10)
                self.assertEqual(r.returncode, 2, r.stdout + r.stderr)

    def test_unreachable_host_gets_a_verdict_within_the_cap(self):
        # The default budget is 20 s. mDNS blocks forever on a name nobody
        # answers for; this proves nothing in the doctor does.
        t0 = time.monotonic()
        r = run([BASH, str(DOCTOR), "--host", UNREACHABLE], timeout=45)
        took = time.monotonic() - t0
        self.assertLess(took, 20 + 4, r.stdout)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        verdicts = [ln for ln in r.stdout.splitlines() if ln.startswith("VERDICT:")]
        self.assertEqual(len(verdicts), 1, r.stdout)
        self.assertIn("CANNOT USE THE PI", verdicts[0])
        # It says WHY: the verdict repeats the first failed check -- the host
        # lookup, or a local link that is already down before the Pi is tried.
        first_fail = next(ln for ln in r.stdout.splitlines() if ln.lstrip().startswith("[FAIL]"))
        self.assertIn(first_fail.split("[FAIL]", 1)[1].strip(), verdicts[0])
        self.assertIn("Fix:", verdicts[0])

    def test_budget_caps_the_whole_run(self):
        t0 = time.monotonic()
        r = run([BASH, str(DOCTOR), "--host", UNREACHABLE, "--budget", "2"], timeout=30)
        self.assertLess(time.monotonic() - t0, 2 + 4, r.stdout)
        self.assertEqual(r.returncode, 1)
        self.assertRegex(r.stdout, r"(?m)^VERDICT: CANNOT USE THE PI")

    def test_bounded_kills_a_hung_command(self):
        t0 = time.monotonic()
        r = bash_lib(DOCTOR, "bounded 1 sleep 30; echo rc=$?")
        self.assertLess(time.monotonic() - t0, 5)
        self.assertIn("rc=142", r.stdout)                # 128 + SIGALRM

    def test_tcp_probe_tells_open_from_refused(self):
        srv = socket.create_server(("127.0.0.1", 0))
        port = srv.getsockname()[1]

        def serve() -> None:
            conn, _ = srv.accept()
            conn.sendall(b"SSH-2.0-OpenSSH_test\r\n")
            conn.close()

        t = threading.Thread(target=serve, daemon=True)
        t.start()
        r = bash_lib(DOCTOR, f"tcp_probe 127.0.0.1 {port} 2")
        srv.close()
        self.assertEqual(r.stdout.strip(), "OPEN SSH-2.0-OpenSSH_test")
        closed = _closed_port()
        r = bash_lib(DOCTOR, f"tcp_probe 127.0.0.1 {closed} 2; echo rc=$?")
        self.assertEqual(r.stdout.split(), ["REFUSED", "rc=1"])

    def test_link_local_addresses_come_back_scoped(self):
        # dscacheutil prints fe80:19::1 (KAME: scope id inside the address).
        r = bash_lib(DOCTOR, "scoped_v6 fe80:1::abcd; scoped_v6 fe80::1%en9; "
                             "scoped_v6 2001:db8::1")
        kame, scoped, glob = r.stdout.split()
        self.assertTrue(kame.startswith("fe80::abcd"), kame)   # %lo0 on a Mac
        self.assertEqual(scoped, "fe80::1%en9")
        self.assertEqual(glob, "2001:db8::1")

    def test_is_ip(self):
        r = bash_lib(DOCTOR, "for a in 10.0.0.2 fe80::1%en9 hao.local 7777; do "
                             "is_ip $a && echo y || echo n; done")
        self.assertEqual(r.stdout.split(), ["y", "y", "n", "n"])


@unittest.skipUnless(BASH, "no bash")
class TestDeploy(unittest.TestCase):
    def test_help(self):
        r = run([BASH, str(DEPLOY), "--help"], 10)
        self.assertEqual(r.returncode, 0, r.stderr)
        for flag in ("--host", "--skip-tests", "--run", "--link-test", "--stop"):
            self.assertIn(flag, r.stdout)

    def test_bad_arguments_exit_2_without_touching_the_network(self):
        for args in (["--bogus"], ["--link-test", "abc"], ["--stop", "--run"],
                     ["--port", "x"], ["--host"]):
            with self.subTest(args):
                t0 = time.monotonic()
                r = run([BASH, str(DEPLOY), *args], 10)
                self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
                self.assertLess(time.monotonic() - t0, 3)
                self.assertNotIn("preflight", r.stdout)

    def test_unreachable_pi_fails_fast_at_preflight(self):
        t0 = time.monotonic()
        r = run([BASH, str(DEPLOY), "--host", UNREACHABLE, "--run", "--link-test", "5"],
                timeout=30, env={"PI_DOCTOR_BUDGET": "3"})
        self.assertLess(time.monotonic() - t0, 3 + 5)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("VERDICT: CANNOT USE THE PI", r.stdout)
        self.assertIn("FAILED -- preflight", r.stderr)
        self.assertNotIn("==> sync", r.stdout)            # stopped before touching anything


@unittest.skipUnless(BASH, "no bash")
class TestDeployLinkTest(unittest.TestCase):
    """--link-test against bridges on this laptop (no Pi, no ssh: --no-sync)."""

    def deploy_link_test(self, port: int, seconds: int = 2) -> subprocess.CompletedProcess:
        return run([BASH, str(DEPLOY), "--host", "127.0.0.1", "--port", str(port),
                    "--no-preflight", "--no-sync", "--skip-tests",
                    "--link-test", str(seconds)], timeout=seconds + 40)

    def test_against_a_real_local_bridge(self):
        port = _closed_port()
        proc = subprocess.Popen(
            [sys.executable, str(ROOT / "scripts" / "run_bridge.py"), "--host", "127.0.0.1",
             "--port", str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(_stop, proc)
        _first_line("127.0.0.1", port, deadline=time.monotonic() + 10)
        r = self.deploy_link_test(port)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        # Not the wording: a loaded machine legitimately reports "Marginal" when a
        # stall lands near the watchdog, and that is the link's fault, not deploy's.
        self.assertRegex(r.stdout, r"(?m)^  VERDICT: ", r.stdout)
        self.assertIn("watchdog trips  0", r.stdout)

    def test_a_link_that_goes_silent_fails_even_if_link_test_says_comfortable(self):
        # link_test.py times stalls BETWEEN state messages, so a link that dies
        # and never comes back leaves its verdict at "comfortable". The deploy
        # must not believe it.
        srv = socket.create_server(("127.0.0.1", 0))
        port = srv.getsockname()[1]
        held: list[socket.socket] = []

        def serve() -> None:
            while True:
                try:
                    conn, _ = srv.accept()
                except OSError:
                    return
                conn.sendall(b'{"v":1,"type":"hello"}\n')  # then silence
                held.append(conn)

        threading.Thread(target=serve, daemon=True).start()
        self.addCleanup(lambda: [c.close() for c in [srv, *held]])
        r = self.deploy_link_test(port)
        self.assertEqual(r.returncode, 6, r.stdout + r.stderr)
        self.assertIn("no data for 1 s", r.stdout)
        self.assertIn("went completely silent", r.stderr)

    def test_no_bridge_fails_fast_with_the_reason(self):
        t0 = time.monotonic()
        r = self.deploy_link_test(_closed_port())
        self.assertLess(time.monotonic() - t0, 8)
        self.assertEqual(r.returncode, 6, r.stdout + r.stderr)
        self.assertIn("not answering", r.stderr)
        self.assertIn("--run", r.stderr)                 # ... and the fix


# -- helpers ------------------------------------------------------------------


def _free_dual_stack_port() -> int | None:
    """A port free on both the IPv4 and the IPv6 wildcard."""
    for _ in range(20):
        s4 = socket.socket(socket.AF_INET)
        s6 = socket.socket(socket.AF_INET6)
        try:
            s6.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
            s4.bind(("0.0.0.0", 0))
            port = s4.getsockname()[1]
            s6.bind(("::", port))
            return port
        except OSError:
            continue
        finally:
            s4.close()
            s6.close()
    return None


def _closed_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _first_line(addr: str, port: int, deadline: float) -> str:
    last: Exception | None = None
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((addr, port), timeout=2) as s:
                return s.makefile("r").readline()
        except OSError as exc:
            last = exc
            time.sleep(0.1)
    raise AssertionError(f"[{addr}]:{port} never answered: {last}")


def _stop(proc: subprocess.Popen) -> None:
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


if __name__ == "__main__":
    unittest.main()
