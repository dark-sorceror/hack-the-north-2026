#!/usr/bin/env bash
# One-shot Pi setup. Run ON the Pi, from ~/retriever:   bash pi/setup.sh
#
# Rehearsed on the Pi 4 so the Pi 5 is a repeat, not an experiment. Idempotent:
# safe to run again after pulling new code.
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
echo "retriever at $HERE, user $USER"

# The bridge itself is stdlib-only. These are for the REAL drivers and GPIO:
# pyserial for the arm and wheel buses, gpiozero + lgpio because RPi.GPIO does
# not work on the Pi 5.
sudo apt-get update -qq
sudo apt-get install -y -qq python3-serial python3-gpiozero python3-lgpio avahi-daemon

# Serial and GPIO access without sudo.
sudo usermod -aG dialout,gpio "$USER" || true

python3 -c "import sys; assert sys.version_info >= (3, 9), sys.version; print('python', sys.version.split()[0])"
python3 -S -c "import sys; sys.path.insert(0, '$HERE/src'); import retriever.bridge.server; print('bridge imports with stdlib only: ok')"

sed -e "s|__USER__|$USER|" -e "s|__HOME__|$HOME|" "$HERE/pi/retriever-bridge.service" \
  | sudo tee /etc/systemd/system/retriever-bridge.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now retriever-bridge
sleep 1
systemctl --no-pager --lines=5 status retriever-bridge || true

echo
echo "bridge listening on $(hostname).local:7777"
echo "  logs:     journalctl -u retriever-bridge -f"
echo "  restart:  sudo systemctl restart retriever-bridge"
echo "  from the laptop:  .venv/bin/python scripts/link_test.py $(hostname).local:7777 --seconds 120"
