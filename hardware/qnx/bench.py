"""M4: run an oss.qnx.com AI module (tflite-runtime) on the QNX Pi and measure it.

usage: python3 bench.py MODEL.tflite IMAGE.jpg [threads] [runs]
Prints the model's I/O, the detections, latency percentiles, and CPU use
(process CPU time / wall time, across all threads; 400% = all 4 cores).
"""
import sys, time
import numpy as np
import cv2
from tflite_runtime.interpreter import Interpreter

model, image = sys.argv[1], sys.argv[2]
threads = int(sys.argv[3]) if len(sys.argv) > 3 else 4
runs = int(sys.argv[4]) if len(sys.argv) > 4 else 50

it = Interpreter(model_path=model, num_threads=threads)
it.allocate_tensors()
inp = it.get_input_details()[0]
outs = it.get_output_details()
_, h, w, _ = inp["shape"]

bgr = cv2.imread(image)
x = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), (w, h))
if inp["dtype"] == np.float32:
    x = (x.astype(np.float32) - 127.5) / 127.5
x = np.expand_dims(x.astype(inp["dtype"]), 0)

def infer():
    it.set_tensor(inp["index"], x)
    it.invoke()

for _ in range(5):
    infer()

lat = []
c0, w0 = time.process_time(), time.perf_counter()
for _ in range(runs):
    t = time.perf_counter(); infer(); lat.append((time.perf_counter() - t) * 1e3)
cpu, wall = time.process_time() - c0, time.perf_counter() - w0

# TFLite_Detection_PostProcess outputs: boxes [1,N,4] (ymin,xmin,ymax,xmax), classes [1,N], scores [1,N], count [1]
arrs = [it.get_tensor(o["index"]) for o in outs]
boxes = next(a for a in arrs if a.ndim == 3 and a.shape[-1] == 4)[0]
flat = [a[0] for a in arrs if a.ndim == 2]
classes, scores = (flat[0], flat[1]) if np.all(flat[0] == np.round(flat[0])) else (flat[1], flat[0])
H, W = bgr.shape[:2]
people = [(float(s), b) for c, s, b in zip(classes, scores, boxes) if int(c) == 0 and s >= 0.4]

lat = np.array(lat)
print(f"model={model.split('/')[-1]} input={list(inp['shape'])} {inp['dtype'].__name__} threads={threads} runs={runs}")
print(f"latency ms: mean={lat.mean():.1f} p50={np.percentile(lat,50):.1f} p99={np.percentile(lat,99):.1f} max={lat.max():.1f}")
print(f"cpu: {100*cpu/wall:.0f}% of one core ({100*cpu/wall/4:.0f}% of 4 cores) -> ~{1000/lat.mean():.0f} inferences/s")
print(f"people (score>=0.4): {len(people)}")
for s, (y0, x0, y1, x1) in sorted(people, key=lambda p: p[0], reverse=True):
    print(f"  person {s:.2f}  box h={100*(y1-y0):.0f}% of frame  at x={int(x0*W)}..{int(x1*W)} y={int(y0*H)}..{int(y1*H)}")
