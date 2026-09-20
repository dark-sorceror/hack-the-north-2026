#!/usr/bin/env bash
#
# Install the board's long-running services as systemd units, so a reboot
# restores them with no monitor and no manual SSH.
#
#   bash install-services.sh              # install + enable + start all
#   bash install-services.sh --status     # just report
#
# Units created:
#   rerun-viewer    :9090 web viewer, :9876 gRPC sink for --display_ip
#   lelab           :8000 LeRobot web UI (isolated venv, upstream lerobot)
#   camera-stream   :8765 websocket JPEG, :8766 test viewer page
#
set -euo pipefail

USER_NAME="${SUDO_USER:-$USER}"
HOME_DIR="$(getent passwd "$USER_NAME" | cut -d: -f6)"

ARM_VENV="$HOME_DIR/hiwonder-SoArm-101/.venv"
LELAB_DIR="$HOME_DIR/leLab"
CAM_SCRIPT="$HOME_DIR/camera_ws_server.py"

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
die()  { printf '\033[1;31m[fail]\033[0m %s\n' "$*" >&2; exit 1; }

report() {
  printf '\n%-16s %-10s %-10s %s\n' UNIT ENABLED ACTIVE PORTS
  for u in rerun-viewer lelab camera-stream; do
    printf '%-16s %-10s %-10s ' "$u" \
      "$(systemctl is-enabled "$u" 2>/dev/null || echo -)" \
      "$(systemctl is-active  "$u" 2>/dev/null || echo -)"
    case $u in
      rerun-viewer)  p="9090 9876" ;;
      lelab)         p="8000" ;;
      camera-stream) p="8765 8766" ;;
    esac
    out=""
    for port in $p; do
      ss -tln 2>/dev/null | grep -q ":$port " && out="$out $port:up" || out="$out $port:DOWN"
    done
    echo "$out"
  done
  echo
}

if [ "${1:-}" = "--status" ]; then report; exit 0; fi

step "Preflight"
[ -x "$ARM_VENV/bin/rerun" ] || die "missing $ARM_VENV/bin/rerun - run setup-board.sh first"
[ -x "$LELAB_DIR/.venv/bin/python" ] || info "WARNING: no leLab venv at $LELAB_DIR - its unit will fail until installed"
[ -f "$CAM_SCRIPT" ] || die "missing $CAM_SCRIPT - copy it to the board first"
info "user: $USER_NAME   home: $HOME_DIR"

step "Freeing ports held by hand-started processes"
# Kill by listening port rather than by name: a pattern like "lelab" also
# matches this script's own command line, and pkill -f would kill the shell
# running it. Learned that one the hard way.
for port in 9090 9876 8000 8765 8766; do
  if sudo fuser -k "${port}/tcp" 2>/dev/null; then
    info "freed :$port"
  fi
done
sleep 2

step "Writing units"

sudo tee /etc/systemd/system/rerun-viewer.service >/dev/null <<UNIT
[Unit]
Description=Rerun web viewer and gRPC sink
Documentation=https://rerun.io
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$HOME_DIR
ExecStart=$ARM_VENV/bin/rerun --serve-web --bind 0.0.0.0
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
UNIT

sudo tee /etc/systemd/system/lelab.service >/dev/null <<UNIT
[Unit]
Description=LeLab web UI (isolated venv; upstream lerobot, not the Hiwonder fork)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$LELAB_DIR
# The lelab CLI hardcodes HOST=127.0.0.1, so drive uvicorn directly to bind
# all interfaces - otherwise it is unreachable from the laptop or phone.
ExecStart=$LELAB_DIR/.venv/bin/python -m uvicorn lelab.server:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
UNIT

sudo tee /etc/systemd/system/camera-stream.service >/dev/null <<UNIT
[Unit]
Description=WebSocket camera stream (board -> Pi / phone / laptop)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
SupplementaryGroups=video
WorkingDirectory=$HOME_DIR
ExecStart=/usr/bin/python3 $CAM_SCRIPT
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
UNIT

step "Enabling"
sudo systemctl daemon-reload
for u in rerun-viewer lelab camera-stream; do
  sudo systemctl enable --now "$u" >/dev/null 2>&1 && info "$u enabled + started" || info "$u FAILED - journalctl -u $u"
done
sleep 6

step "Status"
report

cat <<'NOTE'
    IMPORTANT: camera-stream holds /dev/video0 exclusively. LeRobot cannot
    open the same camera while it runs. Before recording episodes:

        sudo systemctl stop camera-stream

    and afterwards:

        sudo systemctl start camera-stream

    Logs for any of them:  journalctl -u <unit> -f
NOTE
