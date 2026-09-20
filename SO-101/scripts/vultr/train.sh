#!/usr/bin/env bash
# Train the ACT policy on a rented GPU instance (Vultr, RunPod, Lambda - any
# bare Ubuntu box with an NVIDIA driver). Self-contained: installs everything.
#
# From your laptop:
#   scp train.sh pickup_v1_10ep.tar.gz root@<instance-ip>:~/
#   ssh root@<instance-ip>
#   chmod +x train.sh && ./train.sh                 # smoke: 2000 steps
#   SMOKE=0 ./train.sh                              # real: 100k steps
#   SMOKE=0 MULTI_GPU=1 NPROC=4 ./train.sh          # 4 GPUs
#
# Then pull the checkpoint back BEFORE destroying the instance - the disk does
# not persist. The script prints the exact scp command at the end.
#
# Pick an L40S or A100 over an H100: ACT is small (ResNet18 backbones + a
# 512-dim transformer) and the bottleneck is video decode feeding the GPU, not
# matmul. An L40S at ~$1/hr trains it about as fast as an H100 at ~$3.
set -euo pipefail

SMOKE="${SMOKE:-1}"
MULTI_GPU="${MULTI_GPU:-0}"
NPROC="${NPROC:-4}"
DATA_TARBALL="${DATA_TARBALL:-pickup_v2_56ep.tar.gz}"
DATA_DIR="${DATA_DIR:-pickup_v2}"
REPO_ID="${REPO_ID:-local/pickup_v2}"
OUT="${OUT:-$HOME/outputs/act_pickup}"
# Video decode feeds the GPU, so match this to the vCPU count.
NUM_WORKERS="${NUM_WORKERS:-8}"
# Lower = less lost to a dead/preempted instance. 10k is ~20 min.
SAVE_FREQ="${SAVE_FREQ:-10000}"

echo "=== host ==="
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader \
  || { echo "no NVIDIA driver visible - wrong instance type?"; exit 1; }

echo "=== deps ==="
# git    : lerobot 0.5.x is only published as git tags, never to PyPI.
# ffmpeg : episodes are AV1 mp4; decode is what feeds the GPU.
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq git ffmpeg curl
echo "  git $(git --version | awk '{print $3}') | ffmpeg $(ffmpeg -version 2>/dev/null | head -1 | awk '{print $3}')"

# lerobot needs Python >=3.12; most CUDA images ship 3.10/3.11. uv fetches a
# managed 3.12 rather than fighting the system interpreter.
export PATH="$HOME/.local/bin:$PATH"
command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"

VENV="$HOME/lerobot-venv"
[ -d "$VENV" ] || uv venv --python 3.12 "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -c "import sys; print('  python', sys.version.split()[0])"

# Default PyPI torch already bundles CUDA on linux/x86_64. Deliberately not
# pinning a cuXXX index here: the right one depends on the host driver, and
# guessing wrong gives a torch that imports but sees no GPU.
uv pip install --quiet torch
uv pip install --quiet "lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git@v0.5.1" accelerate

python - <<'PYCHK'
import sys, torch
print(f"  torch {torch.__version__} cuda={torch.cuda.is_available()} gpus={torch.cuda.device_count()}")
if not torch.cuda.is_available():
    sys.exit("torch cannot see the GPU - driver/CUDA mismatch")
PYCHK

echo "=== dataset ==="
if [ ! -d "$DATA_DIR" ]; then
  [ -f "$DATA_TARBALL" ] || { echo "missing $DATA_TARBALL - scp it up first"; exit 1; }
  tar -xzf "$DATA_TARBALL"
fi
DATA_DIR="$DATA_DIR" python - <<'PY'
import json, os, glob
root = os.environ["DATA_DIR"]
d = json.load(open(f"{root}/meta/info.json"))
cams = [f.split(".")[-1] for f in d["features"] if f.startswith("observation.images")]
print(f"  episodes {d['total_episodes']}  frames {d['total_frames']}  fps {d['fps']}")
print(f"  cameras  {cams}")
print(f"  videos   {len(glob.glob(root + '/videos/**/*.mp4', recursive=True))}")
if not cams:
    raise SystemExit("no camera features - dataset did not extract properly")
PY

mkdir -p "$OUT"
if [ "$SMOKE" = "1" ]; then STEPS=2000;   BATCH=32; SAVE=1000
else                        STEPS=100000; BATCH=64; SAVE="$SAVE_FREQ"; fi

ARGS=(
  --dataset.repo_id="$REPO_ID"
  --dataset.root="./$DATA_DIR"
  --policy.type=act
  --output_dir="$OUT"
  --job_name=act_pickup
  --steps="$STEPS"
  --batch_size="$BATCH"
  --save_freq="$SAVE"
  --num_workers="$NUM_WORKERS"
  --wandb.enable=false
)

echo "=== training (smoke=$SMOKE steps=$STEPS batch=$BATCH) ==="
START=$(date +%s)
if [ "$MULTI_GPU" = "1" ]; then
  accelerate launch --multi_gpu --num_processes="$NPROC" "$(which lerobot-train)" "${ARGS[@]}"
else
  lerobot-train "${ARGS[@]}"
fi
echo "  elapsed $(( ($(date +%s) - START) / 60 )) min"

echo
echo "=== checkpoints ==="
du -sh "$OUT"
find "$OUT" -name "*.safetensors" | head
cat <<EOF

    Pull this down BEFORE destroying the instance:

      scp -r root@\$(curl -s ifconfig.me 2>/dev/null || echo '<instance-ip>'):$OUT ./checkpoints

EOF
