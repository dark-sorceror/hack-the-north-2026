#!/usr/bin/env bash
# Record episodes for the SO-ARM-101 rig.
#   ./record.sh <repo_id> [num_episodes] [--resume]
# e.g.
#   ./record.sh local/pickup_v2 10
#   ./record.sh local/pickup_v2 10 --resume
set -euo pipefail

REPO="${1:-local/pickup_v2}"
N="${2:-10}"
RESUME=""
[ "${3:-}" = "--resume" ] && RESUME="--resume=true"

TASK="${TASK:-pick up the object and hold it up}"
BIRD=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_222E1FAF-video-index0
CLAW=/dev/v4l/by-id/usb-icSpring_icspring_camera_202404160005-video-index0

for d in /dev/soarm_follower /dev/soarm_leader "$BIRD" "$CLAW"; do
  [ -e "$d" ] || { echo "MISSING: $d"; exit 1; }
done
systemctl is-active --quiet camera-stream && {
  echo "stopping camera-stream so it releases the cameras"; sudo systemctl stop camera-stream; sleep 2; }

# resume() refuses to run without an explicit root: with root=None it would
# write into the revision-safe Hub snapshot cache and corrupt it. Pass it
# always, so create and resume behave identically.
ROOT="$HOME/.cache/huggingface/lerobot/$REPO"
echo "recording $N episodes into $REPO ${RESUME:+(resuming)}"
echo "  root: $ROOT"
cd ~/hiwonder-SoArm-101
exec uv run lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/soarm_follower \
  --robot.id=my_follower_arm \
  --robot.disable_torque_on_disconnect=false \
  --robot.cameras="{birdseye: {type: opencv, index_or_path: $BIRD, width: 640, height: 480, fps: 30}, claw: {type: opencv, index_or_path: $CLAW, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=/dev/soarm_leader \
  --teleop.id=my_leader_arm \
  --dataset.repo_id="$REPO" \
  --dataset.root="$ROOT" \
  --dataset.single_task="$TASK" \
  --dataset.num_episodes="$N" \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --display_data=false \
  $RESUME
