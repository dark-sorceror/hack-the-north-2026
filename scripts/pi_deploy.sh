#!/usr/bin/env bash
# pi_deploy.sh -- one command from "code on the laptop" to "bridge running on the Pi".
#
#   scripts/pi_deploy.sh                          # preflight, sync, Pi-side tests
#   scripts/pi_deploy.sh --run                    # ... then (re)start the bridge on the Pi
#   scripts/pi_deploy.sh --run --link-test 30     # ... then hold a zero-velocity link for 30 s
#   scripts/pi_deploy.sh --stop                   # stop the bridge started by --run
#
# Steps, each bounded in time and failing fast with the reason:
#   1. preflight   scripts/pi_doctor.sh: can we reach the Pi at all, and if not, why
#   2. sync        rsync src/ scripts/ pi/ tests/ -> ~/retriever on the Pi
#   3. tests       the Pi-side stdlib test subset, on the Pi's own Python
#   4. --run       restart the bridge in the background, no sudo: nohup + pidfile + log
#                  in ~/retriever/run/, then wait until it answers FROM THE LAPTOP
#   5. --link-test scripts/link_test.py from the laptop for N seconds, prints its verdict
#
# Everything is idempotent: re-running syncs only what changed and restarts the
# bridge cleanly; --stop on a stopped bridge is a no-op.
#
# The bridge binds all addresses, IPv4 and IPv6 (--host= , the empty host):
# over the Mac's direct cable the Pi may have ONLY an IPv6 link-local address,
# and asyncio's "::" is IPv6-only (it sets IPV6_V6ONLY), so neither "0.0.0.0"
# nor "::" alone reaches the Pi on every link. --bind overrides it.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
# shellcheck source=scripts/pi_doctor.sh
. "$HERE/pi_doctor.sh"

REMOTE_DIR=retriever                  # relative to the Pi user's home
TEST_MODULES="tests.test_bridge tests.test_tank_backend tests.test_drivers tests.test_odometry"
SYNC_DIRS="src scripts pi tests"

usage() {
  cat <<'EOF'
usage: scripts/pi_deploy.sh [--host HOST] [--user USER] [--port PORT]
                            [--skip-tests] [--run] [--link-test SECONDS] [--stop]
                            [--bind ADDR] [--no-preflight] [--no-sync] [-- FAKE_PI_ARGS...]

Deploy the bridge to the Raspberry Pi and check it, in one command:
  preflight (pi_doctor.sh) -> rsync src/ scripts/ pi/ tests/ to ~/retriever
  -> Pi-side tests -> [--run: restart the bridge] -> [--link-test N: link check]

  --host HOST        Pi host name or address; USER@HOST works too   (default hao.local)
  --user USER        SSH user on the Pi                             (default hao)
  --port PORT        bridge TCP port                                (default 7777)
  --skip-tests       do not run the Pi-side tests
  --run              (re)start the bridge on the Pi in the background (no sudo);
                     pidfile + log in ~/retriever/run/
  --link-test SECS   hold a zero-velocity link from this laptop for SECS seconds
                     and print scripts/link_test.py's verdict
  --stop             stop the bridge that --run started, and nothing else
  --bind ADDR        address the bridge binds on the Pi (default: empty = all
                     addresses, IPv4 and IPv6; "::" is IPv6 only)
  --no-preflight     skip pi_doctor.sh (it takes a few seconds when all is well)
  --no-sync          do not copy code; use what is already on the Pi (e.g. just
                     re-measure: --no-sync --skip-tests --link-test 30)
  -- ARGS...         passed to scripts/fake_pi.py, e.g. -- --timeout-ms 500

  e.g.  scripts/pi_deploy.sh --run --link-test 30
        scripts/pi_deploy.sh --stop

exit: 0 ok; 1 preflight failed; 2 usage; 3 sync failed; 4 Pi tests failed;
      5 bridge would not start; 6 link test failed or never connected
EOF
}

say() { printf '\n==> %s\n' "$*"; }
die() {
  local code=$1
  shift
  printf '\npi_deploy: FAILED -- %s\n' "$*" >&2
  exit "$code"
}

# --------------------------------------------------- run ON the Pi (bash -s) ---
# These functions are shipped to the Pi verbatim with `declare -f` and run by the
# Pi's bash. They only touch ~/retriever and processes pi_deploy started itself.

_remote_tests() {
  local mods=$1 rc=0
  cd ~/retriever || { echo "no ~/retriever on the Pi"; return 90; }
  mkdir -p run
  if [ -f tests/test_ddsm115.py ]; then mods="$mods tests.test_ddsm115"; fi
  echo "python3 $(python3 -c 'import platform; print(platform.python_version(), platform.machine())' 2>&1): $mods"
  # shellcheck disable=SC2086  # $mods is a word list on purpose
  python3 -m unittest $mods > run/tests.log 2>&1 || rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "---- last 40 lines of ~/retriever/run/tests.log ----"
    tail -n 40 run/tests.log
    echo "----"
  fi
  grep -E '^(Ran [0-9]+ tests? in|OK|FAILED|NO TESTS RAN)' run/tests.log | tail -n 2
  return "$rc"
}

_remote_stop() {
  local port=$1 pid left
  cd ~/retriever 2>/dev/null || { echo "bridge not running (no ~/retriever on the Pi)"; return 0; }
  if [ -f run/bridge.pid ]; then
    pid=$(cat run/bridge.pid 2>/dev/null)
    # Only ever kill what we started: the pid must still be a fake_pi.py.
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null &&
      tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q 'fake_pi\.py'; then
      kill -TERM "$pid"                     # fake_pi.py stops the motors on SIGTERM
      for _ in $(seq 1 25); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
      done
      if kill -0 "$pid" 2>/dev/null; then
        kill -KILL "$pid"
        echo "bridge pid $pid ignored SIGTERM for 5 s: killed"
      else
        echo "bridge pid $pid stopped ($(tail -n 1 run/bridge.log 2>/dev/null | cut -c1-80))"
      fi
    else
      echo "bridge not running (stale pidfile removed)"
    fi
    rm -f run/bridge.pid
  else
    echo "bridge not running (no pidfile)"
  fi
  left=$(ss -ltnH "sport = :$port" 2>/dev/null | awk '{ print $4 }' | tr '\n' ' ')
  if [ -n "$left" ]; then
    echo "note: something pi_deploy did not start still listens on :$port ($left)"
    if systemctl is-active --quiet retriever-bridge 2>/dev/null; then
      echo "      it is the systemd service: sudo systemctl stop retriever-bridge"
    fi
  fi
  return 0
}

_remote_start() {
  local port=$1 hostarg=$2 pid="" holder
  shift 2
  cd ~/retriever || { echo "no ~/retriever on the Pi"; return 90; }
  mkdir -p run
  _remote_stop "$port"
  holder=$(ss -ltnpH "sport = :$port" 2>/dev/null)
  if [ -n "$holder" ]; then
    echo "port $port on the Pi is taken by something pi_deploy did not start:"
    echo "  $holder"
    if systemctl is-active --quiet retriever-bridge 2>/dev/null; then
      echo "  that is the systemd service: it already runs the bridge. For new code:"
      echo "  sudo systemctl restart retriever-bridge   (or stop it to use --run instead)"
    fi
    return 91
  fi
  if [ -f run/bridge.log ]; then mv -f run/bridge.log run/bridge.log.1; fi
  # setsid: own session, so the ssh session ending does not SIGHUP it. The sh
  # writes its own pid then execs python, so the pidfile is python's pid.
  # shellcheck disable=SC2016
  setsid nohup sh -c 'echo $$ > run/bridge.pid; exec "$@"' sh \
    python3 -u scripts/fake_pi.py "$hostarg" --port "$port" "$@" \
    > run/bridge.log 2>&1 < /dev/null &
  for _ in $(seq 1 50); do                  # 10 s
    if [ -z "$pid" ] && [ -s run/bridge.pid ]; then pid=$(cat run/bridge.pid); fi
    if [ -n "$pid" ] && ! kill -0 "$pid" 2>/dev/null; then
      echo "the bridge exited during startup:"
      tail -n 20 run/bridge.log
      rm -f run/bridge.pid
      return 92
    fi
    if [ -n "$pid" ] && [ -n "$(ss -ltnH "sport = :$port" 2>/dev/null)" ]; then
      echo "bridge pid $pid listening on $(ss -ltnH "sport = :$port" | awk '{ print $4 }' | tr '\n' ' ')"
      return 0
    fi
    sleep 0.2
  done
  echo "the bridge did not start listening on :$port within 10 s:"
  tail -n 20 run/bridge.log
  return 93
}

# pi_remote SECS FUNC [ARGS...] -- run one of the _remote_* functions on the Pi.
pi_remote() {
  local secs=$1 fn=$2
  shift 2
  {
    declare -f _remote_tests _remote_stop _remote_start
    printf '%s' "$fn"
    printf ' %q' "$@"
    printf '\n'
  } | bounded "$secs" ssh "${PI_SSH_OPTS[@]}" "$PI_USER@$PI_HOST" bash -s
}

# --------------------------------------------------------------- laptop side ---

# host:port as link_test.py takes it (it splits on the LAST ':',
# so a bare fe80::1%en9:7777 works and must not be bracketed).
hostport() { printf '%s:%s' "$PI_HOST" "$PI_PORT"; }

# wait_bridge SECS -- poll the bridge FROM THE LAPTOP until it says hello.
# Sets BRIDGE_PROBE to the last probe result.
wait_bridge() {
  local deadline=$((SECONDS + $1))
  while :; do
    BRIDGE_PROBE=$(tcp_probe "$PI_HOST" "$PI_PORT" 3)
    case $BRIDGE_PROBE in
      *'"type":"hello"'*) return 0 ;;
      *'another client'*) return 2 ;;
    esac
    [ "$SECONDS" -ge "$deadline" ] && return 1
    sleep 1
  done
}

main() {
  PI_HOST=${PI_HOST:-hao.local} PI_USER=${PI_USER:-hao} PI_PORT=${PI_PORT:-7777}
  local skip_tests=0 run=0 link=0 stop=0 preflight=1 sync=1 bind='' rc
  local -a extra=()
  while [ $# -gt 0 ]; do
    case $1 in
      --host | --user | --port | --link-test | --bind)
        [ $# -ge 2 ] || { echo "pi_deploy: $1 needs a value" >&2; exit 2; }
        case $1 in
          --host) PI_HOST=$2 ;; --user) PI_USER=$2 ;; --port) PI_PORT=$2 ;;
          --link-test) link=$2 ;; --bind) bind=$2 ;;
        esac
        shift 2
        ;;
      --skip-tests) skip_tests=1; shift ;;
      --run) run=1; shift ;;
      --stop) stop=1; shift ;;
      --no-preflight) preflight=0; shift ;;
      --no-sync) sync=0; shift ;;
      -h | --help) usage; exit 0 ;;
      --) shift; extra=("$@"); break ;;
      *) echo "pi_deploy: unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
  done
  case $PI_HOST in *@*) PI_USER=${PI_HOST%%@*} PI_HOST=${PI_HOST#*@} ;; esac
  case $PI_PORT in '' | *[!0-9]*) echo "pi_deploy: bad --port: $PI_PORT" >&2; exit 2 ;; esac
  case $link in *[!0-9]*) echo "pi_deploy: --link-test takes whole seconds, got: $link" >&2; exit 2 ;; esac
  if [ "$stop" = 1 ] && { [ "$run" = 1 ] || [ "$link" != 0 ]; }; then
    echo "pi_deploy: --stop does not combine with --run / --link-test" >&2
    exit 2
  fi

  local hostflag=''
  [ "$PI_USER@$PI_HOST" = hao@hao.local ] || hostflag=" --host $PI_USER@$PI_HOST"
  local py="$REPO/.venv/bin/python"
  [ -x "$py" ] || py=python3
  local t0=$SECONDS

  # 1. preflight --------------------------------------------------------------
  if [ "$preflight" = 1 ]; then
    say "preflight: scripts/pi_doctor.sh --host $PI_USER@$PI_HOST"
    "$HERE/pi_doctor.sh" --host "$PI_USER@$PI_HOST" --port "$PI_PORT"
    rc=$?
    # 3 = SSH fine, bridge not answering: expected before --run.
    if [ "$rc" != 0 ] && [ "$rc" != 3 ]; then
      die 1 "preflight: the Pi is not reachable (see the VERDICT above). Fix that, then re-run."
    fi
  fi

  if [ "$stop" = 1 ]; then
    say "stopping the bridge on $PI_HOST"
    pi_remote 20 _remote_stop "$PI_PORT" || die 5 "could not reach the Pi to stop the bridge (ssh exit $?)"
    return 0
  fi

  # 2. sync ------------------------------------------------------------------
  if [ "$sync" = 0 ]; then
    say "sync: skipped (--no-sync)"
  else
    say "sync: $SYNC_DIRS -> $PI_USER@$PI_HOST:~/$REMOTE_DIR/"
    local dest="$PI_USER@$PI_HOST" rsh="ssh ${PI_SSH_OPTS[*]}"
    case $PI_HOST in
      *:*) # An IPv6 literal: rsync (macOS's openrsync) splits user@host:path at the
        # first ':', and [brackets] do not help. Give it a placeholder host and hand
        # ssh the address as HostName -- with '%' doubled, since ssh percent-expands
        # HostName (fe80::1%en9 -> "unknown key %e").
        dest="$PI_USER@pi"
        rsh="$rsh -o HostName=${PI_HOST//\%/%%}"
        ;;
    esac
    # --delete keeps the Pi's copy exact (a stale module there can mask a missing
    # one here); it only applies inside the synced dirs, never to ~/retriever/run.
    # shellcheck disable=SC2086
    (cd "$REPO" && bounded 120 rsync -az --delete --timeout=20 --stats \
      --exclude=__pycache__ --exclude='*.pyc' --exclude='*.pt' --exclude=weights \
      --exclude=data --exclude=.DS_Store \
      -e "$rsh" $SYNC_DIRS "$dest:$REMOTE_DIR/") > "${TMPDIR:-/tmp}/pi_deploy_rsync.$$" 2>&1
    rc=$?
    grep -iE 'files transferred|number of (regular )?files|total transferred file size|error|failed|denied|timeout' \
      "${TMPDIR:-/tmp}/pi_deploy_rsync.$$" | sed 's/^/    /'
    rm -f "${TMPDIR:-/tmp}/pi_deploy_rsync.$$"
    case $rc in
      0) echo "    synced" ;;
      142) die 3 "sync: rsync did not finish within 120 s (the link dropped mid-copy?). Re-run: it resumes." ;;
      *) die 3 "sync: rsync exit $rc (see above). Check: scripts/pi_doctor.sh --host $PI_USER@$PI_HOST" ;;
    esac
  fi

  # 3. Pi-side tests -----------------------------------------------------------
  if [ "$skip_tests" = 1 ]; then
    say "tests on the Pi: skipped (--skip-tests)"
  else
    say "tests on the Pi (python3 -m unittest, stdlib subset)"
    pi_remote 300 _remote_tests "$TEST_MODULES" | sed 's/^/    /'
    rc=${PIPESTATUS[0]}
    case $rc in
      0) ;;
      142) die 4 "the Pi-side tests did not finish within 300 s (link dropped?). Full log: ~/retriever/run/tests.log on the Pi" ;;
      255) die 4 "lost the ssh connection while running the tests (link dropped?)" ;;
      *) die 4 "Pi-side tests FAILED (exit $rc). Not starting a bridge that fails its tests; --skip-tests overrides. Log: ssh $PI_USER@$PI_HOST cat ~/retriever/run/tests.log" ;;
    esac
  fi

  # 4. --run --------------------------------------------------------------------
  if [ "$run" = 1 ]; then
    say "bridge: (re)starting scripts/fake_pi.py on $PI_HOST:$PI_PORT (bind '${bind:-*}' = ${bind:-all addresses, IPv4 + IPv6})"
    pi_remote 40 _remote_start "$PI_PORT" "--host=$bind" ${extra[@]+"${extra[@]}"} | sed 's/^/    /'
    rc=${PIPESTATUS[0]}
    case $rc in
      0) ;;
      91) die 5 "port $PI_PORT on the Pi is taken (see above)" ;;
      142 | 255) die 5 "lost the ssh connection while starting the bridge (link dropped?)" ;;
      *) die 5 "the bridge did not start (exit $rc, log above; full log: ssh $PI_USER@$PI_HOST cat ~/retriever/run/bridge.log)" ;;
    esac
    wait_bridge 10
    case $? in
      0) echo "    answers from this laptop at $(hostport): ${BRIDGE_PROBE#OPEN }" | cut -c1-110 ;;
      2) echo "    answers from this laptop, but another laptop is connected (busy)" ;;
      *) die 5 "the bridge runs on the Pi but does not answer from this laptop at $(hostport): $BRIDGE_PROBE. Diagnose: scripts/pi_doctor.sh --host $PI_USER@$PI_HOST" ;;
    esac
    echo "    log:   ssh $PI_USER@$PI_HOST tail -f $REMOTE_DIR/run/bridge.log"
    echo "    stop:  scripts/pi_deploy.sh --stop$hostflag"
  fi

  # 5. --link-test ------------------------------------------------------------------
  if [ "$link" != 0 ]; then
    say "link test: $link s of zero-velocity commands to $(hostport)"
    if [ "$run" != 1 ]; then
      wait_bridge 0
      case $? in
        0) ;;
        2) die 6 "the bridge is busy: another laptop is connected to it" ;;
        *) die 6 "the bridge is not answering at $(hostport) ($BRIDGE_PROBE). Start it: scripts/pi_deploy.sh --run --link-test $link" ;;
      esac
    fi
    local out="${TMPDIR:-/tmp}/pi_deploy_link.$$"
    bounded $((link + 30)) "$py" -u "$REPO/scripts/link_test.py" "$(hostport)" --seconds "$link" 2>&1 | tee "$out"
    rc=${PIPESTATUS[0]}
    local tripped=0 silent closed
    grep -q 'VERDICT: the watchdog tripped' "$out" && tripped=1
    # link_test.py measures stalls between state messages, so a link that dies
    # near the end (no state after it) leaves its verdict at "comfortable".
    # Any whole second of silence is > the 300 ms watchdog: that is a failure.
    silent=$(grep -c 'no data for 1 s' "$out" || true)
    closed=$(grep -c 'closed the connection' "$out" || true)
    # link_test.py now reports outages itself (including silence at the very end
    # of the run) with this verdict and exit 1; the checks on rc 0 below stay as
    # a backstop for an older link_test.py.
    local outage=0
    grep -q 'VERDICT: the link went down' "$out" && outage=1
    rm -f "$out"
    local silent_msg="went completely silent for ~${silent} s (the '!! no data' lines)"
    [ "${silent:-0}" -gt 0 ] || silent_msg="went completely silent at the end of the run"
    case $rc in
      0)
        if [ "${closed:-0}" -gt 0 ]; then
          die 6 "link test: the Pi closed the connection mid-test (bridge restarted or crashed? ssh $PI_USER@$PI_HOST tail ~/$REMOTE_DIR/run/bridge.log)"
        elif [ "${silent:-0}" -gt 0 ]; then
          die 6 "link test: the link $silent_msg, far past the 300 ms watchdog -- the verdict above misses a stall at the very end. Diagnose: scripts/pi_doctor.sh$hostflag"
        fi
        ;;
      142) die 6 "link_test.py did not finish within $((link + 30)) s" ;;
      *)
        if [ "${closed:-0}" -gt 0 ]; then
          die 6 "link test: the Pi closed the connection mid-test (bridge restarted or crashed? ssh $PI_USER@$PI_HOST tail ~/$REMOTE_DIR/run/bridge.log)"
        elif [ "$outage" = 1 ] || [ "${silent:-0}" -gt 0 ]; then
          die 6 "link test: the link $silent_msg -- an outage, not jitter; no watchdog setting fixes it. If preflight warned that NetworkManager is waiting for DHCP, that is the cause: eth0 drops every 45 s. Diagnose: scripts/pi_doctor.sh$hostflag"
        elif [ "$tripped" = 1 ]; then
          die 6 "link test: the watchdog tripped on this link (verdict above). If preflight warned that NetworkManager is waiting for DHCP, that is the cause: eth0 drops every 45 s"
        else
          die 6 "link test could not run (exit $rc, see above). Diagnose: scripts/pi_doctor.sh$hostflag"
        fi ;;
    esac
  fi

  say "done in $((SECONDS - t0)) s"
  return 0
}

# Sourcing (the tests do) only defines the functions.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  main "$@"
fi
