#!/usr/bin/env python3
"""Export the released ZipDepth NPU checkpoint to an RV1103 INT8 model.

Run on a PC with Python 3.11, torch==2.4.0+cpu, rknn-toolkit2==2.5.0,
numpy==1.26.4, onnx<=1.18, onnxruntime, opencv-python<4.12, setuptools<81.
The dataset is a text file containing paths to representative RGB images;
use three identical grayscale channels for the OV9281 cameras.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types

UPSTREAM_COMMIT = "91f3fd21e131641f51e8d35736d1958350180e3a"
CHECKPOINT_SHA256 = "627c04fda584133ead4310074884a4a037061b4c01ba86e73e492ea30fab570d"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def export(args):
    import cv2
    import numpy as np
    import onnxruntime as ort
    import torch
    import torch.nn.functional as functional
    from rknn.api import RKNN

    upstream = args.upstream.resolve()
    revision = subprocess.check_output(["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True).strip()
    if revision != UPSTREAM_COMMIT:
        raise ValueError(f"Expected ZipDepth checkout {UPSTREAM_COMMIT}, got {revision}")
    checkpoint = upstream / "checkpoints/zipdepth_base_npu.pth"
    if digest(checkpoint) != CHECKPOINT_SHA256:
        raise ValueError("ZipDepth checkpoint checksum mismatch")
    images = [Path(line.strip()) for line in args.dataset.read_text().splitlines() if line.strip()]
    if len(images) < 20 or not all(path.is_file() for path in images):
        raise ValueError("Provide at least 20 representative calibration images")
    for path in images:
        image = cv2.imread(str(path))
        if image is None or image.shape != (160, 256, 3):
            raise ValueError(f"Expected a 256x160 three-channel calibration image: {path}")
        if not np.array_equal(image[:, :, 0], image[:, :, 1]) or not np.array_equal(image[:, :, 1], image[:, :, 2]):
            raise ValueError(f"Calibration images must match the monochrome camera input: {path}")

    sys.path.insert(0, str(upstream))
    from zipdepth.model.architecture import create_model
    from zipdepth.utils.model_utils import fuse_remaining_conv_bn, strip_state_dict_prefixes

    torch.set_num_threads(4)
    model = create_model(variant="base", global_mode="balanced", upsample_unfold=False).eval()
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    model.load_state_dict(strip_state_dict_prefixes(state.get("model_state_dict", state)), strict=True)
    model.fuse_for_inference()
    fuse_remaining_conv_bn(model)

    sample = cv2.imread(str(images[0]))[:, :, ::-1].copy()
    tensor = torch.from_numpy(sample.transpose(2, 0, 1)).unsqueeze(0).float() / 255
    with torch.no_grad():
        reference = model(tensor).numpy()

    # Express the learned weighted context as elementwise weighting + pooling.
    # This preserves the trained attention; replacing it with uniform averaging
    # would change the model. Avoid MatMul without dropping context_weight.
    for module in model.modules():
        if type(module).__name__ == "GlobalContextBlock":
            def context_forward(self, x):
                height, width = x.shape[2:]
                weights = functional.softmax(self.context_weight(x).flatten(2), dim=-1)
                weighted = x * weights.reshape(1, 1, height, width)
                context = functional.avg_pool2d(weighted, (height, width)) * (height * width)
                return x + self.transform(context)
            module.forward = types.MethodType(context_forward, module)
        elif type(module).__name__ == "StripPoolingAttention":
            def strip_forward(self, x):
                height, width = x.shape[2:]
                horizontal = functional.avg_pool2d(x, (1, width))
                vertical = functional.avg_pool2d(x, (height, 1))
                return x * self.gate_conv(horizontal + vertical)
            module.forward = types.MethodType(strip_forward, module)
    cross_scale = model.encoder.cross_scale
    def cross_forward(self, high, low):
        up = functional.interpolate(self.low_to_high(low), size=high.shape[2:], mode="nearest")
        down = functional.avg_pool2d(self.high_to_low(high), 2, 2)
        return high + up * 0.3, low + down * 0.3
    cross_scale.forward = types.MethodType(cross_forward, cross_scale)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    onnx = args.output.with_suffix(".onnx")
    torch.onnx.export(model, tensor, str(onnx), opset_version=17,
                      input_names=["image"], output_names=["depth"], do_constant_folding=True)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(onnx), options, providers=["CPUExecutionProvider"])
    actual = session.run(None, {"image": tensor.numpy()})[0]
    np.testing.assert_allclose(actual, reference, atol=1e-4, rtol=1e-4)
    maximum_error = float(np.max(np.abs(actual - reference)))
    print(f"ONNX vs trained PyTorch model: maximum absolute error {maximum_error:.8g}", flush=True)

    compiler = RKNN(verbose=True, verbose_file=str(args.output.with_suffix(".build.log")))
    try:
        if compiler.config(target_platform="rv1103", mean_values=[[0, 0, 0]],
                               std_values=[[255, 255, 255]],
                               quantized_dtype="asymmetric_quantized-8", optimization_level=3):
            raise RuntimeError("RKNN compiler configuration failed")
        if compiler.load_onnx(model=str(onnx)):
            raise RuntimeError("RKNN ONNX import failed")
        if compiler.build(do_quantization=True, dataset=str(args.dataset.resolve())):
            raise RuntimeError("RKNN INT8 compilation failed")
        temporary = args.output.with_suffix(".tmp.rknn")
        if compiler.export_rknn(str(temporary)):
            raise RuntimeError("RKNN model export failed")
        temporary.replace(args.output)
    finally:
        compiler.release()
    metadata = {
        "model": "ZipDepth-base NPU", "source": "https://github.com/fabiotosi92/ZipDepth",
        "source_commit": revision, "checkpoint_sha256": CHECKPOINT_SHA256,
        "rknn_toolkit": "2.5.0", "target": "rv1103", "precision": "INT8",
        "input": [1, 3, 160, 256], "output": "relative inverse depth",
        "grayscale_channels": 3, "calibration_images": len(images),
        "attention": "learned weighted context preserved through exact pooling reformulation",
        "onnx_maximum_absolute_error": maximum_error, "sha256": digest(args.output),
    }
    args.output.with_name("provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Wrote {args.output}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    export(parser.parse_args())
