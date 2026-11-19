#!/usr/bin/env python3
"""Convert the official fixed ZipDepth export; never substitute generated weights.

Run export_fixed.py first; upstream's exporter changes learned attention.
The RKNN input is NHWC uint8 grayscale. Normalization stays inside the graph,
apart from dividing uint8 by 255 using RKNN's input preprocessing.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

import cv2
import numpy as np
import onnx
from onnx import helper
from onnxsim import simplify
from rknn.api import RKNN


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.onnx = args.onnx.resolve()
    args.images = args.images.resolve()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    os.chdir(args.output)  # Toolkit diagnostics must never fill the source tree.
    graph = onnx.load(args.onnx)
    inp = graph.graph.input[0]
    assert [d.dim_value for d in inp.type.tensor_type.shape.dim] == [1, 3, 224, 320]
    # Replicate native mono, preserving all three trained channel weights and
    # their different ImageNet means/stds, including convolution edge padding.
    for node in graph.graph.node:
        for i, name in enumerate(node.input):
            if name == inp.name:
                node.input[i] = "replicated_mono"
    graph.graph.node.insert(0, helper.make_node(
        "Concat", [inp.name] * 3, ["replicated_mono"], axis=1))
    inp.type.tensor_type.shape.dim[1].dim_value = 1
    graph, valid = simplify(graph)
    assert valid
    onnx.checker.check_model(graph)
    mono = args.output / "zipdepth-mono-320x224.onnx"
    onnx.save(graph, mono)

    records = []
    for source in sorted(args.images.rglob("im[01].png")):
        image = cv2.imread(str(source), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Unreadable calibration image: {source}")
        image = cv2.resize(image, (320, 200), interpolation=cv2.INTER_AREA)
        tensor = np.pad(image, ((12, 12), (0, 0)))
        dest = args.output / f"calibration-{len(records):03}.npy"
        # NPY inputs use model NCHW shape, values before RKNN std=255.
        np.save(dest, tensor[np.newaxis, np.newaxis].astype(np.float32))
        records.append({"source": str(source.relative_to(args.images)),
                        "sha256": sha256(source), "tensor": str(dest.resolve())})
    if len(records) < 20:
        raise ValueError("Need at least 20 real monochrome calibration images")
    dataset = args.output / "dataset.txt"
    dataset.write_text("".join(r["tensor"] + "\n" for r in records))
    manifest = {"source_onnx_sha256": sha256(args.onnx),
                "mono_onnx_sha256": sha256(mono), "images": records,
                "corpus": "ETH3D low-res two-view training, real monochrome stereo",
                "limitations": "Not OV9281/M32076M20 fisheye data; target-domain validation required",
                "input": "uint8 1x224x320x1; average 320x200; zero pad top/bottom 12",
                "target": "rv1103", "quantization": "asymmetric_quantized-8"}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    rknn = RKNN(verbose=True)
    try:
        def check(result, operation):
            if result != 0:
                raise RuntimeError(f"{operation} failed: {result}")
        check(rknn.config(target_platform="rv1103", mean_values=[[0]],
                          std_values=[[255]], quantized_dtype="asymmetric_quantized-8"), "config")
        check(rknn.load_onnx(model=str(mono)), "load_onnx")
        check(rknn.build(do_quantization=True, dataset=str(dataset)), "build")
        model = args.output / "zipdepth-mono-rv1103.rknn"
        check(rknn.export_rknn(str(model)), "export_rknn")
        manifest["rknn_sha256"] = sha256(model)
        # The simulator is a real quantized graph execution, not a substitute
        # for the on-board NPU. Compare to float ONNX on calibration-domain data.
        import onnxruntime as ort
        reference = ort.InferenceSession(str(mono), providers=["CPUExecutionProvider"])
        check(rknn.init_runtime(), "simulator init")
        errors = []
        for record in records[::max(1, len(records)//6)]:
            nchw = np.load(record["tensor"])
            expected = reference.run(None, {reference.get_inputs()[0].name: nchw / 255})[0]
            actual = rknn.inference(inputs=[nchw.transpose(0, 2, 3, 1).astype(np.uint8)],
                                    data_format=["nhwc"])[0].reshape(expected.shape)
            if not np.isfinite(actual).all():
                raise RuntimeError("Nonfinite simulator output")
            e = actual.astype(np.float64).ravel()
            r = expected.astype(np.float64).ravel()
            errors.append({"source": record["source"], "rmse": float(np.sqrt(np.mean((e-r)**2))),
                           "reference_std": float(r.std()),
                           "correlation": float(np.corrcoef(e, r)[0, 1])})
        manifest["simulator_comparison"] = errors
        (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    finally:
        rknn.release()


if __name__ == "__main__":
    main()
