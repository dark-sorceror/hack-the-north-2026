#!/usr/bin/env bash
#
# Set up the Hiwonder LeRobot stack on an RDK S100 (Linux / aarch64).
#
#   bash setup-board.sh
#
# Idempotent: safe to re-run. Override defaults with env vars, e.g.
#   INSTALL_DIR=/opt/soarm bash setup-board.sh
#
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/Hiwonder-official/hiwonder-SoArm-101.git}"
PIN="${PIN:-a24998f7}"
INSTALL_DIR="${INSTALL_DIR:-$HOME/hiwonder-SoArm-101}"
MIN_FREE_GB="${MIN_FREE_GB:-10}"

# 2 hit sites in the follower, 1 in the leader.
EXPECTED_SITES=3
FOLLOWER_REL="src/lerobot/robots/so_follower/so_follower.py"
LEADER_REL="src/lerobot/teleoperators/so_leader/so_leader.py"
BUS_REL="src/lerobot/motors/motors_bus.py"

CAL_DIR="$HOME/.cache/huggingface/lerobot/calibration"
FOLLOWER_CAL="$CAL_DIR/robots/so_follower/my_follower_arm.json"

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[fail]\033[0m %s\n' "$*" >&2; exit 1; }

# --------------------------------------------------------------- preflight --
step "Preflight"
[ "$(uname -s)" = "Linux" ] || die "This targets Linux. Run it on the board, not the laptop."
info "arch:   $(uname -m)"
info "kernel: $(uname -r)"

command -v git >/dev/null 2>&1 || die "missing 'git' - install it: sudo apt install -y git"

# The RDK image ships wget but not curl, and apt can be locked by a stuck
# PackageKit session, so don't hard-require curl. Either fetcher works.
if command -v curl >/dev/null 2>&1; then
  FETCH="curl -LsSf"
elif command -v wget >/dev/null 2>&1; then
  FETCH="wget -qO-"
else
  die "need curl or wget to fetch the uv installer"
fi
info "fetcher: ${FETCH%% *}"

free_gb=$(df -PBG "$HOME" | awk 'NR==2 {gsub(/G/,"",$4); print $4}')
info "free on \$HOME: ${free_gb}G"
[ "$free_gb" -ge "$MIN_FREE_GB" ] \
  || die "need >= ${MIN_FREE_GB}G free (torch is large); only ${free_gb}G available"

# --------------------------------------------------------------------- uv --
step "uv"
if ! command -v uv >/dev/null 2>&1; then
  info "installing uv via ${FETCH%% *}..."
  $FETCH https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi
command -v uv >/dev/null 2>&1 || die "uv not on PATH - add \$HOME/.local/bin to PATH and re-run"
info "uv $(uv --version | awk '{print $2}')"

# ------------------------------------------------------------------ clone --
step "Clone / update repo, pinned at $PIN"
if [ -d "$INSTALL_DIR/.git" ]; then
  info "reusing existing clone at $INSTALL_DIR"
  git -C "$INSTALL_DIR" fetch --all --tags --quiet
else
  git clone "$REPO_URL" "$INSTALL_DIR"
fi
git -C "$INSTALL_DIR" checkout --quiet "$PIN"
info "HEAD is $(git -C "$INSTALL_DIR" rev-parse --short HEAD) (detached, as intended)"

cd "$INSTALL_DIR"
[ -f "$FOLLOWER_REL" ] || die "missing $FOLLOWER_REL - layout changed, patch would silently no-op"
[ -f "$LEADER_REL" ]   || die "missing $LEADER_REL - layout changed, patch would silently no-op"
[ -f "$BUS_REL" ]      || die "missing $BUS_REL - layout changed, patch would silently no-op"

# ---------------------------------------------------------------- uv sync --
step "uv sync (pulls torch - slow on shared wifi)"
# The lock was authored on win32/x86_64. If it carries no linux-aarch64
# resolution this is exactly where it breaks, so fail with a real explanation
# instead of a raw resolver trace.
if ! uv sync; then
  die "uv sync failed.
    If the error mentions platform markers or 'no matching distribution', the
    lockfile has no linux-aarch64 resolution. Re-resolve with:
        cd $INSTALL_DIR && uv lock && uv sync
    That drifts from the pinned lock - note it if behaviour differs later."
fi

# --------------------------------------------------------------- the patch --
step "Apply sync_read retry patch (num_retry=3)"
# Why: the 1 Mbps half-duplex servo bus corrupts ~0.03% of frames (measured: 3
# failures in 10,512 loops over 180s). With num_retry=0 one bad frame raises
# ConnectionError and kills the process; disconnect() then runs disable_torque,
# all six servos go slack at once, and it looks exactly like the arm lost
# power. It is not a power fault - supply was logged at 11.8-12.6V, flat
# through the failure instant. These sites are live during autonomous
# inference, not just teleop.
sed -i 's/self\.bus\.sync_read("Present_Position")/self.bus.sync_read("Present_Position", num_retry=3)/g' \
  "$FOLLOWER_REL" "$LEADER_REL"

# Same bug at connect time: _assert_motors_exist pings each motor once with
# num_retry=0, so one dropped reply fails the whole connect with "Missing motor
# IDs" - the phantom missing-motor symptom. 2% loss on a single servo was
# enough to trigger it. ping() already accepted num_retry; it just never asked.
sed -i 's/model_nb = self\.ping(id_)/model_nb = self.ping(id_, num_retry=3)/' "$BUS_REL"

patched=$(grep -o 'sync_read("Present_Position", num_retry=3)' "$FOLLOWER_REL" "$LEADER_REL" | wc -l)
unpatched=$(grep -o 'sync_read("Present_Position")' "$FOLLOWER_REL" "$LEADER_REL" | wc -l || true)
pinged=$(grep -c 'ping(id_, num_retry=3)' "$BUS_REL" || true)

info "patched:    $patched (expect $EXPECTED_SITES)"
info "unpatched:  $unpatched (expect 0)"
info "ping retry: $pinged (expect 1)"
[ "$patched" -eq "$EXPECTED_SITES" ] && [ "$unpatched" -eq 0 ] && [ "$pinged" -eq 1 ] \
  || die "patch verification failed - do NOT run the arm until this is resolved"

git --no-pager diff --stat

# ----------------------------------------------------------- serial access --
step "Serial port access"
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  info "$USER already in 'dialout'"
else
  info "adding $USER to 'dialout' (needs sudo)"
  sudo usermod -aG dialout "$USER"
  warn "log out and back in (or reboot) before the group takes effect"
fi

info "serial devices present:"
if [ -d /dev/serial/by-id ]; then
  ls -l /dev/serial/by-id/ | sed 's/^/      /'
else
  info "      (none - plug the follower arm in and re-check)"
fi

cat <<'UDEV'

    ttyUSB numbering is NOT stable across reboots, so leader and follower can
    swap. Pin a name using the serial from the listing above, in
    /etc/udev/rules.d/99-soarm.rules:

      SUBSYSTEM=="tty", ATTRS{serial}=="<SERIAL>", SYMLINK+="soarm_follower"

    then reload:

      sudo udevadm control --reload-rules && sudo udevadm trigger

    and point the robot config at /dev/soarm_follower, not /dev/ttyUSB0.
UDEV

# ------------------------------------------------------------- calibration --
step "Calibration"
mkdir -p "$(dirname "$FOLLOWER_CAL")"
if [ -f "$FOLLOWER_CAL" ]; then
  info "follower calibration present: $FOLLOWER_CAL"
else
  board_ip=$(hostname -I 2>/dev/null | awk '{print $1}')
  warn "follower calibration MISSING at $FOLLOWER_CAL"
  cat <<CAL

    Copy it from the recording machine. Do NOT run calibration here.

    The policy emits positions in calibrated space. If the board's calibration
    differs from the machine the data was recorded on, every action maps to a
    different physical pose. Recalibrating would also discard the re-seated
    wrist_flex horn and the gripper range_min 647 -> 860 fix, which brings back
    the overload trip (red LED, '[ServoStatus] Overload error!').

    From the laptop, in PowerShell, one line:

      scp "\$env:USERPROFILE\.cache\huggingface\lerobot\calibration\robots\so_follower\my_follower_arm.json" $USER@$board_ip:"$FOLLOWER_CAL"

CAL
fi

# ----------------------------------------------------------------- summary --
step "Done"
cat <<SUMMARY

    install dir : $INSTALL_DIR
    pinned at   : $(git -C "$INSTALL_DIR" rev-parse --short HEAD)
    run things  : cd $INSTALL_DIR && uv run <cmd>

    Remaining, in order:
      1. copy the follower calibration across, if it was missing above
      2. log out/in if you were just added to 'dialout'
      3. add the udev rule so the port name is stable
      4. smoke test: cd $INSTALL_DIR && uv run python -c "import lerobot; print(lerobot.__file__)"

    Heads up: broadcast_ping is unreliable in this fork. Connect may report
    phantom "Missing motor IDs: 4, 5, 6" while every servo pings fine
    individually. Confirm with individual pings before suspecting hardware.

SUMMARY
