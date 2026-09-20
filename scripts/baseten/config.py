"""Baseten Training job for the ACT policy.

    baseten train push --config config.py
    baseten train job logs --job-id <job_id> --tail

The whole directory containing this file is uploaded with the job, so keep the
dataset tarball and run.sh next to it.

Smoke test first (SMOKE=1 in run.sh, 1x H100, ~minutes) before spending hours:
10 episodes cannot produce a usable policy, so the first run is only proving
that the dataset loads, both camera streams decode, the policy builds with the
right input shapes, and a checkpoint lands in $BT_CHECKPOINT_DIR.
"""

from truss.base.truss_config import AcceleratorSpec
from truss_train import (
    CacheConfig,
    CheckpointingConfig,
    Compute,
    Image,
    Runtime,
    TrainingJob,
    TrainingProject,
)

# CUDA image; lerobot brings its own torch pin on top.
BASE_IMAGE = "pytorch/pytorch:2.7.0-cuda12.8-cudnn9-runtime"

# 1 for the smoke test. Raise to 4 for the real run and set MULTI_GPU=1 in
# run.sh so it launches under accelerate.
GPU_COUNT = 1

training_runtime = Runtime(
    start_commands=["chmod +x ./run.sh && ./run.sh"],
    # Reuses the pip install and any cached dataset across jobs - worth having
    # when iterating, since installing lerobot is the slowest part of a short run.
    cache_config=CacheConfig(enabled=True),
    checkpointing_config=CheckpointingConfig(enabled=True),
)

training_compute = Compute(
    accelerator=AcceleratorSpec(accelerator="H100", count=GPU_COUNT),
)

training_job = TrainingJob(
    image=Image(base_image=BASE_IMAGE),
    compute=training_compute,
    runtime=training_runtime,
)

training_project = TrainingProject(
    name="act-soarm101-pickup",
    job=training_job,
)
