"""Synthesize a 'person walks up to the robot' clip: zoom into one person and back out."""
import os, cv2
img = cv2.imread("people.jpg"); H, W = img.shape[:2]
cx, cy = 250, 770            # the person SSD found at x=223..276, y=699..842
os.makedirs("approach", exist_ok=True)
hs = [int(900 - (900 - 190) * i / 29) for i in range(30)]
for k, h in enumerate(hs + hs[::-1]):
    w = int(h * W / H)
    x0 = min(max(cx - w // 2, 0), W - w); y0 = min(max(cy - h // 2, 0), H - h)
    cv2.imwrite(f"approach/f{k:03d}.jpg", cv2.resize(img[y0:y0 + h, x0:x0 + w], (640, 426)))
print("wrote", len(hs) * 2, "frames")
