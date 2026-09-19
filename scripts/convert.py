from pathlib import Path
from PIL import Image

w, h, stride = 1280, 800, 1792
raw = Path("cam11.raw").read_bytes()
assert len(raw) == stride * h, f"Unexpected size: {len(raw)}"

gray = bytearray(w * h)
for y in range(h):
    row = raw[y * stride:y * stride + 1600]
    for x in range(0, w, 4):
        i = x * 5 // 4
        a, b, c, d, e = row[i:i + 5]
        gray[y*w+x:y*w+x+4] = bytes((
            (a >> 2) | ((b & 3) << 6),
            (b >> 4) | ((c & 15) << 4),
            (c >> 6) | ((d & 63) << 2),
            e,
        ))

Image.frombytes("L", (w, h), gray).save("frame.jpg", quality=95)
