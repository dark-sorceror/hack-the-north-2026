#!/usr/bin/env bash
# Entrypoint for the Baseten training job. Invoked by config.py start_commands.
set -euo pipefail

SMOKE="${SMOKE:-1}"          # 1 = short pipeline check, 0 = real training run
MULTI_GPU="${MULTI_GPU:-0}"  # 1 = launch under accelerate (set GPU_COUNT>1 too)
NPROC="${NPROC:-4}"

DATA_TARBALL="${DATA_TARBALL:-pickup_v1_10ep.tar.gz}"
DATA_DIR="${DATA_DIR:-archive_pickup_v1_10ep_0920_1126}"
REPO_ID="${REPO_ID:-local/pickup_v1}"

echo "=== environment ==="
python -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), 'gpus', torch.cuda.device_count())"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader || true

echo "=== deps ==="
# git    : the pytorch runtime image has no git, and lerobot 0.5.x installs
#          from a git URL, so pip cannot fetch it without this.
# ffmpeg : episodes are AV1 mp4 and decode is what feeds the GPU. Missing it
#          fails at the first video frame, not at startup.
# Deliberately not silenced with "|| true": both are hard requirements, and
# swallowing a failed apt here just moves the error somewhere less obvious.
apt-get update -qq
apt-get install -y -qq git ffmpeg
command -v git >/dev/null || { echo "git missing after apt install"; exit 1; }
command -v ffmpeg >/dev/null || { echo "ffmpeg missing after apt install"; exit 1; }
echo "  git $(git --version | awk '{print $3}') | ffmpeg $(ffmpeg -version 2>/dev/null | head -1 | awk '{print $3}')"
# PyPI only publishes lerobot up to 0.4.4; the 0.5.x line exists solely as
# git tags, which is why D-Robotics' guide clones rather than pip installs.
# v0.5.1 is the newest tag and matches the board's fork exactly (their guide
# cites "v0.5.2", but no such tag exists).
# Training touches no motors, so stock lerobot is right here - none of the
# Hiwonder fork's patches are needed.
pip install --quiet "lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git@v0.5.1" "accelerate"

# Installing lerobot can drag in its own torch and silently replace the CUDA
# build this image ships with. Catch that here rather than after the dataset
# has loaded and a GPU hour is gone.
python - <<'PYCHK'
import sys, torch
print(f"  post-install torch {torch.__version__} cuda={torch.cuda.is_available()} gpus={torch.cuda.device_count()}")
if not torch.cuda.is_available():
    sys.exit("CUDA disappeared after installing lerobot - a CPU torch was pulled in")
PYCHK

echo "=== dataset ==="
if [ ! -d "$DATA_DIR" ]; then
  [ -f "$DATA_TARBALL" ] || { echo "missing $DATA_TARBALL next to config.py"; exit 1; }
  tar -xzf "$DATA_TARBALL"
fi
python - <<'PY'
import json, os, glob
root = os.environ.get("DATA_DIR", "archive_pickup_v1_10ep_0920_1126")
d = json.load(open(f"{root}/meta/info.json"))
cams = [f.split(".")[-1] for f in d["features"] if f.startswith("observation.images")]
print(f"  episodes {d['total_episodes']}  frames {d['total_frames']}  fps {d['fps']}")
print(f"  cameras  {cams}")
print(f"  videos   {len(glob.glob(root + '/videos/**/*.mp4', recursive=True))}")
PY

OUT="${BT_CHECKPOINT_DIR:-./outputs}/act_pickup"
mkdir -p "$OUT"

if [ "$SMOKE" = "1" ]; then
  STEPS=2000; BATCH=32; SAVE=1000
else
  STEPS=100000; BATCH=64; SAVE=20000
fi

ARGS=(
  --dataset.repo_id="$REPO_ID"
  --dataset.root="./$DATA_DIR"
  --policy.type=act
  --output_dir="$OUT"
  --job_name=act_pickup
  --steps="$STEPS"
  --batch_size="$BATCH"
  --save_freq="$SAVE"
  --num_workers=8          # ACT is small; video decode is the bottleneck
  --wandb.enable=false
)
# Deliberately no --policy.device: accelerate auto-detects the device and
# ignores it, per the note in lerobot's train script.

echo "=== training (smoke=$SMOKE multi_gpu=$MULTI_GPU steps=$STEPS batch=$BATCH) ==="
if [ "$MULTI_GPU" = "1" ]; then
  accelerate launch --multi_gpu --num_processes="$NPROC" "$(which lerobot-train)" "${ARGS[@]}"
else
  lerobot-train "${ARGS[@]}"
fi

echo "=== checkpoints written ==="
find "$OUT" -name "*.safetensors" -o -name "config.json" | head -20
du -sh "$OUT"
