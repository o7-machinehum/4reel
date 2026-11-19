#!/usr/bin/env python3
"""Fixed 320x224 export retaining the official checkpoint's learned attention.

Unlike upstream scripts/export.py, this never replaces GlobalContextBlock with
uniform average pooling. Strict loading rejects missing or unexpected weights.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import onnxruntime as ort
import torch

CHECKPOINT_SHA256 = "627c04fda584133ead4310074884a4a037061b4c01ba86e73e492ea30fab570d"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--upstream", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    sys.path.insert(0, str(a.upstream.resolve()))
    from zipdepth.model.architecture import create_model
    from zipdepth.utils.model_utils import strip_state_dict_prefixes, fuse_remaining_conv_bn
    checkpoint = a.upstream / "checkpoints/zipdepth_base_npu.pth"
    assert hashlib.sha256(checkpoint.read_bytes()).hexdigest() == CHECKPOINT_SHA256
    torch.set_num_threads(4)
    model = create_model(variant="base", global_mode="balanced", upsample_unfold=False)
    weights = torch.load(checkpoint, map_location="cpu", weights_only=True)
    model.load_state_dict(strip_state_dict_prefixes(weights.get("model_state_dict", weights)), strict=True)
    model.eval()
    model.fuse_for_inference()
    fuse_remaining_conv_bn(model)
    dummy = torch.zeros(1, 3, 224, 320)
    with torch.no_grad():
        torch.onnx.export(model, dummy, str(a.output), opset_version=17,
                          input_names=["image"], output_names=["depth"], do_constant_folding=True)
    session = ort.InferenceSession(str(a.output), providers=["CPUExecutionProvider"])
    errors = []
    for seed in range(3):
        rng = np.random.default_rng(seed)
        mono = rng.random((1, 1, 224, 320), dtype=np.float32)
        mono[:, :, :12] = mono[:, :, -12:] = 0
        image = np.repeat(mono, 3, axis=1)
        with torch.no_grad():
            reference = model(torch.from_numpy(image)).numpy()
        actual = session.run(None, {"image": image})[0]
        error = float(np.max(np.abs(reference-actual)))
        assert error < 1e-5, error
        errors.append(error)
    a.output.with_suffix(".parity.json").write_text(json.dumps({
        "checkpoint_sha256": CHECKPOINT_SHA256,
        "pytorch_vs_onnx_max_abs": errors,
        "test_inputs": "three seeded mono arrays; export equivalence only, not inference-quality measurements",
    }, indent=2) + "\n")


if __name__ == "__main__":
    main()
