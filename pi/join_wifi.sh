#!/usr/bin/env bash
# Put the Pi on a Wi-Fi network for good, the robot way. Run ON the Pi:
#
#   sudo bash pi/join_wifi.sh "Hao's iPhone" PASSWORD
#
# The robot runs untethered: the Pi rides on it and the Mac reaches it over
# Wi-Fi, so both must join the same network (a phone hotspot or a travel
# router; venue Wi-Fi usually blocks device-to-device traffic).
#
#   * The name matches whatever apostrophe you type: iPhones name hotspots with
#     a curly one (Hao’s iPhone), which a straight ' never matches otherwise.
#   * Wi-Fi power saving OFF: it parks the radio between packets, and the
#     latency spikes that makes trip the bridge's 300 ms watchdog.
#   * Retries forever: a hotspot switched off and on again is rejoined, instead
#     of NetworkManager giving up after 4 tries.
#   * Saved as the connection "robot-wifi", preferred over other saved networks.
#     Running it again replaces it; `sudo nmcli con delete robot-wifi` removes it.
set -euo pipefail

if [ "$#" -ne 2 ]; then
  echo "usage: sudo bash pi/join_wifi.sh \"NETWORK NAME\" PASSWORD" >&2
  exit 2
fi
if [ "$(id -u)" -ne 0 ]; then
  echo "join_wifi.sh: needs root, run it with sudo" >&2
  exit 2
fi
WANT="$1"
PASS="$2"

norm() { # lower case, every apostrophe-like character -> '
  printf '%s' "$1" | sed -e "s/’/'/g" -e "s/‘/'/g" -e "s/\`/'/g" | tr '[:upper:]' '[:lower:]'
}

nmcli radio wifi on
echo "scanning..."
nmcli dev wifi rescan ifname wlan0 2> /dev/null || true
sleep 4
SSID=""
while IFS= read -r s; do
  [ -n "$s" ] || continue
  if [ "$(norm "$s")" = "$(norm "$WANT")" ]; then
    SSID="$s"
    break
  fi
done < <(nmcli -t -e no -f SSID dev wifi list ifname wlan0 | sort -u)

# Not visible by name: an iPhone hides its hotspot's name once the Personal
# Hotspot screen closes, while still serving whoever is on it. Joining it as a
# hidden network (probing for the name) works either way. Its default name has
# a curly apostrophe, so try that spelling first, then the one typed.
if [ -n "$SSID" ]; then
  CANDIDATES=("$SSID")
else
  echo "\"$WANT\" is not showing its name; trying it as a hidden network"
  CURLY="${WANT//\'/’}"
  CANDIDATES=("$CURLY")
  if [ "$CURLY" != "$WANT" ]; then CANDIDATES+=("$WANT"); fi
fi

SSID=""
for s in "${CANDIDATES[@]}"; do
  nmcli con delete robot-wifi > /dev/null 2>&1 || true
  # hidden yes always: a reboot while the phone hides its name must still join
  if nmcli --wait 30 dev wifi connect "$s" password "$PASS" ifname wlan0 name robot-wifi hidden yes; then
    SSID="$s"
    break
  fi
done
if [ -z "$SSID" ]; then
  nmcli con delete robot-wifi > /dev/null 2>&1 || true
  echo "join_wifi.sh: could not join \"$WANT\". Visible networks:" >&2
  nmcli -t -e no -f SSID dev wifi list ifname wlan0 | sort -u | sed 's/^/    /' >&2
  echo "Check the password, and open Settings > Personal Hotspot on the phone." >&2
  exit 1
fi
nmcli con mod robot-wifi connection.autoconnect yes connection.autoconnect-priority 10 \
  connection.autoconnect-retries 0 802-11-wireless.powersave 2 802-11-wireless.hidden yes

echo
echo "joined \"$SSID\" as robot-wifi (power saving off, retries forever)"
ip -4 -o addr show wlan0 | awk '{print "  wlan0 address: " $4}'
echo "  the Mac must join \"$SSID\" too; then from the Mac: $(hostname).local"
