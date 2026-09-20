#!/usr/bin/env bash
# Launch ACT training as a Hugging Face Job.
#
#   ./train_job.sh                 # real run, L40S
#   FLAVOR=a10g-large ./train_job.sh
#   SMOKE=1 ./train_job.sh         # 2000 steps, to prove the pipeline
#
# Watch it:   hf jobs logs <job-id> --follow
# Stop it:    hf jobs cancel <job-id>
#
# The dataset is pulled from the Hub, and checkpoints are pushed back to a Hub
# model repo - so nothing depends on the job's disk surviving, and losing CLI
# access cannot strand a finished run (which is exactly how Baseten lost us a
# trained model tonight).
set -euo pipefail

USER_NS="${USER_NS:-jqyy}"
DATASET="${DATASET:-$USER_NS/pickup_v2}"
OUT_REPO="${OUT_REPO:-$USER_NS/act-pickup-v2}"
# a10g-large: 12 vCPU / 24GB GPU / $1.50-hr. vCPU count matters more than
# GPU class here - AV1 decode feeds the GPU and is the real bottleneck.
FLAVOR="${FLAVOR:-rtx-pro-6000}"
# Match the flavor vCPU count: AV1 decode feeds the GPU and is the real
# bottleneck, so this buys more speed than a bigger GPU.
NUM_WORKERS="${NUM_WORKERS:-20}"
SMOKE="${SMOKE:-0}"
TIMEOUT="${TIMEOUT:-6h}"
IMAGE="${IMAGE:-pytorch/pytorch:2.6.0-cuda12.4-cudnn9-devel}"

if [ "$SMOKE" = "1" ]; then STEPS=2000; BATCH=32; SAVE=1000
else                        STEPS=100000; BATCH=64; SAVE=10000; fi

# Runs inside the container. Kept as one string because `hf jobs run` takes the
# command inline.
read -r -d '' REMOTE <<EOF || true
set -euo pipefail
echo "=== host ==="
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader

echo "=== deps ==="
# ffmpeg: episodes are AV1 mp4 and decode feeds the GPU.
# git: lerobot 0.5.x exists only as git tags, never on PyPI.
apt-get update -qq && apt-get install -y -qq git ffmpeg curl

# lerobot needs python >=3.12; CUDA images ship 3.10/3.11.
export PATH="/root/.local/bin:\$PATH"
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="/root/.local/bin:\$PATH"
uv venv --python 3.12 /opt/venv
source /opt/venv/bin/activate
uv pip install --quiet torch
uv pip install --quiet "lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git@v0.5.1"
python -c "import torch,sys; print('torch',torch.__version__,'cuda',torch.cuda.is_available()); sys.exit(0 if torch.cuda.is_available() else 1)"

echo "=== train ==="
lerobot-train \
  --dataset.repo_id=$DATASET \
  --policy.type=act \
  --policy.push_to_hub=true \
  --policy.repo_id=$OUT_REPO \
  --output_dir=/tmp/act_out \
  --job_name=act_pickup \
  --steps=$STEPS \
  --batch_size=$BATCH \
  --save_freq=$SAVE \
  --num_workers=$NUM_WORKERS \
  --wandb.enable=false
echo "=== done ==="
EOF

echo "dataset : $DATASET"
echo "output  : $OUT_REPO (model repo on the Hub)"
echo "flavor  : $FLAVOR   steps: $STEPS   batch: $BATCH"
echo

exec hf jobs run \
  --flavor "$FLAVOR" \
  --timeout "$TIMEOUT" \
  --secrets HF_TOKEN \
  --name "act-pickup-$(date +%H%M)" \
  --detach \
  "$IMAGE" \
  bash -c "$REMOTE"
