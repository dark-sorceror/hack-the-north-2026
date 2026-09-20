#!/bin/sh
# Start, stop or check the QNX safety supervisor on the Pi:
#   lineguard (C, SCHED_FIFO, owns the heartbeat pin) + detector.py (TFLite).
#
#   ./run.sh start      MODEL=... SOURCE=... override the defaults below
#   ./run.sh stop
#   ./run.sh status
#
# PIDs go in *.pid: a shell's background jobs ignore SIGINT, so stop by PID.
cd "$(dirname "$0")" || exit 1
MODEL=${MODEL:-ssd_mobilenet_v1_quant.tflite}
SOURCE=${SOURCE:-approach/*.jpg}
FPS=${FPS:-15}

stop_one() {
    [ -f "$1.pid" ] || return 0
    kill -TERM "$(cat "$1.pid")" 2>/dev/null && echo "stopped $1"
    rm -f "$1.pid"
}

case "$1" in
start)
    "$0" stop >/dev/null
    ./lineguard -v >lineguard.log 2>&1 &
    echo $! >lineguard.pid
    sleep 0.2
    python3 -W ignore detector.py --model "$MODEL" --source "$SOURCE" --loop --fps "$FPS" \
        >detector.log 2>&1 &
    echo $! >detector.pid
    echo "started lineguard ($(cat lineguard.pid)) and detector ($(cat detector.pid)); logs: *.log"
    ;;
stop)
    stop_one detector      # detector first: lineguard then reports STOP on silence
    sleep 0.3
    stop_one lineguard
    ;;
status)
    for p in lineguard detector; do
        if [ -f $p.pid ] && kill -0 "$(cat $p.pid)" 2>/dev/null; then
            echo "$p: running ($(cat $p.pid))"
        else
            echo "$p: not running"
        fi
    done
    echo "--- lineguard.log"; tail -n 5 lineguard.log 2>/dev/null
    echo "--- detector.log"; grep -v '^INFO' detector.log 2>/dev/null | tail -n 3
    ;;
*)
    echo "usage: $0 start|stop|status" >&2
    exit 2
    ;;
esac
