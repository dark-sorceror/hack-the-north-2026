#!/usr/bin/env bash
# Launch ACT training as a Hugging Face Job.
#
#   ./train_job.sh                 # real run
#   SMOKE=1 ./train_job.sh         # 2000 steps, proves the pipeline
#   FLAVOR=a100-large ./train_job.sh
#
# Watch:  hf jobs logs <job-id> --follow
# Stop:   hf jobs cancel <job-id>
#
# The dataset is pulled from the Hub and checkpoints are pushed back to a Hub
# model repo, so a finished run cannot be stranded by losing CLI access.
set -euo pipefail

USER_NS="${USER_NS:-jqyy}"
DATASET="${DATASET:-$USER_NS/pickup_v2}"
OUT_REPO="${OUT_REPO:-$USER_NS/act-pickup-v2}"
# vCPU count matters more than GPU class here: AV1 decode feeds the GPU and is
# the real bottleneck. rtx-pro-6000 gives 23 vCPU for $2.75/hr, against h200's
# identical 23 vCPU for $5.
FLAVOR="${FLAVOR:-rtx-pro-6000}"
NUM_WORKERS="${NUM_WORKERS:-20}"
SMOKE="${SMOKE:-0}"
TIMEOUT="${TIMEOUT:-6h}"
IMAGE="${IMAGE:-pytorch/pytorch:2.6.0-cuda12.4-cudnn9-devel}"

if [ "$SMOKE" = "1" ]; then STEPS=2000; BATCH=32; SAVE=1000
else                        STEPS=100000; BATCH=64; SAVE=10000; fi

# Quoted heredoc: nothing expands locally. Config arrives via --env instead,
# which sidesteps escaping entirely - an earlier version interpolated $PATH
# and shipped the laptop's Windows PATH into the container.
REMOTE=$(cat <<'EOF'
set -euo pipefail
echo "=== host ==="
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
echo "vCPUs: $(nproc)"

echo "=== deps ==="
# git    : lerobot 0.5.x exists only as git tags, never on PyPI.
# ffmpeg : episodes are AV1 mp4 and decode feeds the GPU.
apt-get update -qq && apt-get install -y -qq git ffmpeg curl

# lerobot needs python >=3.12; CUDA images ship 3.10/3.11.
curl -LsSf https://astral.sh/uv/install.sh | sh
# Absolute paths throughout, and never touch PATH. Git Bash mangles a PATH
# assignment on its way into the container: the Windows PATH replaced the
# container's, /usr/bin vanished, and even basename stopped resolving.
UV=/root/.local/bin/uv
VPY=/opt/venv/bin/python
$UV venv --python 3.12 /opt/venv
$UV pip install --quiet --python $VPY torch
$UV pip install --quiet --python $VPY "lerobot[feetech] @ git+https://github.com/huggingface/lerobot.git@v0.5.1"
$VPY - <<'PYCHK'
import sys, torch
print("torch", torch.__version__, "cuda", torch.cuda.is_available())
sys.exit(0 if torch.cuda.is_available() else 1)
PYCHK

# lerobot only pushes to the Hub AFTER the training loop ends, so a job that
# is cancelled or times out yields nothing - /tmp dies with the container.
# Sync checkpoints up as they appear instead, so the run can be stopped at any
# point and the newest checkpoint is already safe on the Hub.
# Deliberately NOT creating /tmp/act_out here: lerobot refuses to start if
# output_dir already exists and resume is false. The sync loop below
# tolerates it being absent until training creates it.
(
  while true; do
    sleep 240
    if compgen -G "/tmp/act_out/checkpoints/*" > /dev/null; then
      /opt/venv/bin/hf upload "$OUT_REPO" /tmp/act_out/checkpoints checkpoints --type model --commit-message "checkpoint sync" 2>&1 | tail -2 | sed 's/^/[sync] /'
    fi
  done
) &
SYNC_PID=$!
trap 'kill $SYNC_PID 2>/dev/null || true' EXIT

echo "=== train: $STEPS steps, batch $BATCH, $NUM_WORKERS workers ==="
/opt/venv/bin/lerobot-train \
  --dataset.repo_id="$DATASET" \
  --policy.type=act \
  --policy.push_to_hub=true \
  --policy.repo_id="$OUT_REPO" \
  --output_dir=/tmp/act_out \
  --job_name=act_pickup \
  --steps="$STEPS" \
  --batch_size="$BATCH" \
  --save_freq="$SAVE" \
  --num_workers="$NUM_WORKERS" \
  --wandb.enable=false
echo "=== done ==="
EOF
)

echo "dataset : $DATASET"
echo "output  : $OUT_REPO"
echo "flavor  : $FLAVOR   steps: $STEPS   batch: $BATCH   workers: $NUM_WORKERS"
echo

# NB: this hf CLI (1.18) has no --name and no --detach. Options must precede
# IMAGE, and an unsupported flag gets silently swallowed as the image name.
export MSYS2_ARG_CONV_EXCL="*"
exec hf jobs run \
  --flavor "$FLAVOR" \
  --timeout "$TIMEOUT" \
  --secrets HF_TOKEN \
  --env "DATASET=$DATASET" \
  --env "OUT_REPO=$OUT_REPO" \
  --env "STEPS=$STEPS" \
  --env "BATCH=$BATCH" \
  --env "SAVE=$SAVE" \
  --env "NUM_WORKERS=$NUM_WORKERS" \
  "$IMAGE" \
  bash -c "$REMOTE"
