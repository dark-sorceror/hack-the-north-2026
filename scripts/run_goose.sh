#!/usr/bin/env bash
# Pick up the goose, then present it to the user.
#   ./run_goose.sh [checkpoint_step]     (default: 010000)
#
# Stop the policy by typing:  q  then Enter.  Do NOT Ctrl-C --
# it orphans the process holding the serial port and skips the
# clean shutdown, which can latch the gripper in overload.
#
# After you stop the policy, the arm KEEPS torque (holding the goose)
# and the handoff pose runs automatically.

set -euo pipefail

STEP="${1:-010000}"
CKPT="/home/sunrise/checkpoints/checkpoints/${STEP}/pretrained_model"
EVAL_DIR="/home/sunrise/eval_test"

BIRDSEYE="/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_222E1FAF-video-index0"
CLAW="/dev/v4l/by-id/usb-icSpring_icspring_camera_202404160005-video-index0"

if [ ! -d "$CKPT" ]; then
  echo "ERROR: checkpoint not found: $CKPT"
  echo "Available:"; ls /home/sunrise/checkpoints/checkpoints/ 2>/dev/null || echo "  (none)"
  exit 1
fi

rm -rf "$EVAL_DIR"

cd /home/sunrise/hiwonder-SoArm-101

echo "=== 1/2: running policy from step $STEP  (type q + Enter when it has the goose) ==="
# disable_torque_on_disconnect=false is essential: without it the arm goes
# limp the moment the policy stops and drops the goose before handoff.
# video=false skips mp4 encoding entirely. This is inference, not data
# collection, so the episode video is pure waste -- and lerobot defaults to
# vcodec=libsvtav1, i.e. AV1 software encoding, which on this 6-core ARM CPU
# stalls for tens of seconds after you press q and delays the handoff.
# If you ever do want the video, use --dataset.vcodec=libx264 instead.
# fourcc=MJPG is required: both cameras share a USB bus and uncompressed
# streams exceed USB 2.0 bandwidth, starving the second camera.
./.venv/bin/lerobot-record \
  --robot.type=so101_follower \
  --robot.id=my_follower_arm \
  --robot.port=/dev/soarm_follower \
  --robot.disable_torque_on_disconnect=false \
  --robot.cameras="{birdseye: {type: opencv, index_or_path: ${BIRDSEYE}, width: 640, height: 480, fps: 30, fourcc: MJPG}, claw: {type: opencv, index_or_path: ${CLAW}, width: 640, height: 480, fps: 30, fourcc: MJPG}}" \
  --policy.path="$CKPT" \
  --dataset.repo_id=local/eval_test \
  --dataset.root="$EVAL_DIR" \
  --dataset.single_task="pick up the goose" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=30 \
  --dataset.video=false \
  --dataset.push_to_hub=false || echo "(policy exited)"

sleep 1

echo
echo "=== 2/2: moving to handoff pose (gripper held closed) ==="
./.venv/bin/python /home/sunrise/arm_poses.py goto handoff --hold-gripper

echo
echo "=== done -- arm is holding the goose at handoff position ==="
echo "To release: ./.venv/bin/python /home/sunrise/arm_poses.py ... or power cycle the arm"
