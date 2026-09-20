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
apt-get install -y -qq git ffmpeg curl
command -v git >/dev/null || { echo "git missing after apt install"; exit 1; }
command -v ffmpeg >/dev/null || { echo "ffmpeg missing after apt install"; exit 1; }
echo "  git $(git --version | awk '{print $3}') | ffmpeg $(ffmpeg -version 2>/dev/null | head -1 | awk '{print $3}')"

# The pytorch:2.7.0 image ships Python 3.11, but lerobot declares
# requires-python >=3.12, so its own interpreter cannot install it. Rather than
# hunt for a CUDA image with 3.12, use uv to fetch a managed 3.12 and build a
# venv - the same approach the RDK board uses. CUDA itself comes from the host
# driver, so the base image's torch is not needed; we install a cu128 build.
export PATH="$HOME/.local/bin:$PATH"
command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
echo "  uv $(uv --version | awk '{print $2}')"

VENV=/opt/lerobot-venv
uv venv --python 3.12 "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -c "import sys; print('  venv python', sys.version.split()[0])"

# CUDA wheel index, or pip resolves a CPU build and the H100 sits idle.
uv pip install --quiet torch --index-url https://download.pytorch.org/whl/cu128
uv pip install --quiet "lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git@v0.5.1" accelerate

python - <<'PYCHK'
import sys, torch
print(f"  torch {torch.__version__} cuda={torch.cuda.is_available()} gpus={torch.cuda.device_count()}")
if not torch.cuda.is_available():
    sys.exit("no CUDA in the venv - a CPU torch was resolved")
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
