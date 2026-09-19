#!/usr/bin/env bash
# One-shot Pi setup. Run ON the Pi, from ~/retriever:
#
#   bash pi/setup.sh                  # bridge with the FAKE wheels, on every boot
#   bash pi/setup.sh --driver real    # the real DDSM115 wheels (any fake_pi.py flags work)
#
# Rehearsed on the Pi 4 so the Pi 5 is a repeat, not an experiment. Idempotent:
# safe to run again after pulling new code.
#
# Works offline: on the direct cable to the laptop the Pi usually has no
# internet (IPv6 link-local only), so the apt step skips itself with a message
# -- the fake-driver bridge is stdlib-only and needs none of those packages.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
echo "retriever at $HERE, user $USER"
# Everything given to this script goes on the bridge's command line, in the unit.
BRIDGE_ARGS="${*:---driver fake}"
case "$BRIDGE_ARGS" in *'|'* | *'&'* | *'\'*)
  echo "setup.sh: bridge arguments may not contain | & or \\" >&2; exit 2 ;; esac
echo "bridge arguments: $BRIDGE_ARGS"

# The bridge itself is stdlib-only. These are for the REAL drivers and GPIO:
# pyserial for the arm and wheel buses, gpiozero + lgpio because RPi.GPIO does
# not work on the Pi 5.
PKGS="python3-serial python3-gpiozero python3-lgpio avahi-daemon"

# DNS + a TCP connect to a mirror, a few seconds at most -- apt-get itself would
# sit through long DNS and connect timeouts before failing.
have_internet() {
  local h
  for h in deb.debian.org archive.raspberrypi.com; do
    if timeout 4 getent hosts "$h" > /dev/null 2>&1 &&
      timeout 4 bash -c "exec 3<>/dev/tcp/$h/80" 2> /dev/null; then
      return 0
    fi
  done
  return 1
}

MISSING=""
for p in $PKGS; do
  st="$(dpkg-query -W -f='${Status}' "$p" 2> /dev/null || true)"
  case "$st" in *"install ok installed"*) ;; *) MISSING="$MISSING $p" ;; esac
done
MISSING="${MISSING# }"
if [ -z "$MISSING" ]; then
  echo "apt: already installed: $PKGS"
elif have_internet; then
  echo "apt: installing $MISSING"
  # shellcheck disable=SC2086  # a word list on purpose
  if ! { sudo timeout 300 apt-get update -qq &&
    sudo timeout 600 apt-get install -y -qq $MISSING; }; then
    echo "apt: FAILED installing $MISSING -- continuing: the fake-driver bridge"
    echo "     needs none of it. Fix and re-run before using --driver real / GPIO."
  fi
else
  echo "apt: no internet -- SKIPPED (missing: $MISSING)."
  echo "     The fake-driver bridge is stdlib-only and needs none of these. The real"
  echo "     drivers / GPIO do: re-run this once the Pi is online (on the Mac: System"
  echo "     Settings > General > Sharing > Internet Sharing, share to the USB LAN)."
fi

# Serial and GPIO access without sudo. Only groups that exist on this image:
# a missing group in the unit's SupplementaryGroups makes systemd refuse to
# start the service at all (status 216/GROUP), which is a bad way to find out.
GROUPS_OK=""
for g in dialout gpio; do
  if getent group "$g" > /dev/null; then
    sudo usermod -aG "$g" "$USER" || true
    GROUPS_OK="$GROUPS_OK $g"
  else
    echo "note: group '$g' does not exist on this image, skipping"
  fi
done
GROUPS_OK="${GROUPS_OK# }"

python3 -c "import sys; assert sys.version_info >= (3, 10), sys.version; print('python', sys.version.split()[0])"
python3 -S -c "import sys; sys.path.insert(0, '$HERE/src'); import retriever.bridge.server; print('bridge imports with stdlib only: ok')"

# A bridge started by scripts/pi_deploy.sh --run holds :7777, and the service
# would crash-loop on "address in use". Stop it: from now on systemd owns it.
PIDFILE="$HERE/run/bridge.pid"
if [ -f "$PIDFILE" ]; then
  pid="$(cat "$PIDFILE" 2> /dev/null || true)"
  cmd="$(tr '\0' ' ' < "/proc/${pid:-0}/cmdline" 2> /dev/null || true)"
  if [ -n "$pid" ] && [[ $cmd == *fake_pi.py* ]]; then
    echo "stopping the bridge pi_deploy.sh --run started (pid $pid): the service takes over"
    kill -TERM "$pid" || true
    for _ in $(seq 1 25); do kill -0 "$pid" 2> /dev/null || break; sleep 0.2; done
  fi
  rm -f "$PIDFILE"
fi

sed -e "s|__USER__|$USER|" -e "s|__HOME__|$HOME|" -e "s|__BRIDGE_ARGS__|$BRIDGE_ARGS|" \
    -e "s|^SupplementaryGroups=.*|SupplementaryGroups=$GROUPS_OK|" \
    "$HERE/pi/retriever-bridge.service" \
  | { if [ -z "$GROUPS_OK" ]; then grep -v '^SupplementaryGroups='; else cat; fi; } \
  | sudo tee /etc/systemd/system/retriever-bridge.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now retriever-bridge
sudo systemctl restart retriever-bridge   # pick up new code on a re-run
sleep 1
systemctl --no-pager --lines=5 status retriever-bridge || true

echo
echo "bridge listening on $(hostname).local:7777 (all addresses, IPv4 + IPv6): $BRIDGE_ARGS"
echo "  logs:     journalctl -u retriever-bridge -f"
echo "  restart:  sudo systemctl restart retriever-bridge   (after every scripts/pi_deploy.sh)"
echo "  from the laptop:"
echo "    scripts/pi_doctor.sh --host $USER@$(hostname).local"
echo "    scripts/pi_deploy.sh --host $USER@$(hostname).local --skip-tests --link-test 120"
echo "  (with the service installed, use it instead of pi_deploy.sh --run: both want :7777)"
