#!/bin/sh
# Kill tests for the judges' claim: kill any part and the heartbeat stops.
cd "$(dirname "$0")" || exit 1
pin() { echo qnxuser | sudo -S gpio-rp1 get 17 2>/dev/null | sed -n 's/.*level=\([01]\).*/\1/p'; }
samples() { s=""; i=0; while [ $i -lt 10 ]; do s="$s$(pin)"; i=$((i+1)); done; echo "$s"; }

echo "## A: kill -9 the detector"
SOURCE=people.jpg ./run.sh start >/dev/null; sleep 5
echo "pin while SAFE:        $(samples)"
kill -9 "$(cat detector.pid)"; sleep 0.5
echo "pin after kill -9:     $(samples)"
grep -E "OK|STOP" lineguard.log | grep -v lineguard: | tail -2
./run.sh stop >/dev/null; sleep 0.5

echo "## B: kill -9 lineguard itself"
SOURCE=people.jpg ./run.sh start >/dev/null; sleep 5
echo "pin while SAFE:        $(samples)"
kill -9 "$(cat lineguard.pid)"; sleep 0.3
echo "pin after kill -9:     $(samples)   (frozen: the robot sees no edges -> STOP)"
./run.sh stop >/dev/null
echo "cleanup: releasing the pin"; echo -n in > /dev/gpio/17; echo "final: $(echo qnxuser | sudo -S gpio-rp1 get 17 2>/dev/null)"
