#!/usr/bin/env python3
"""Assemble the ignored local Buildroot input bundle from verified downloads."""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

PINS = {
    "zipdepth": "91f3fd21e131641f51e8d35736d1958350180e3a",
    "rknn": "2c723177a94fac65cbc036d722907ddbc91980cf",
    "rga": "2b32edcb97b601b25683e2941d888c8515da6d55",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    for name in ["zipdepth", "rknn", "rga", "glibc", "model", "output", "hash-list"]:
        p.add_argument("--"+name, type=Path, required=True)
    p.add_argument("--parity", type=Path, required=True)
    a = p.parse_args()
    for name, revision in PINS.items():
        actual = subprocess.check_output(["git", "-C", str(getattr(a, name)), "rev-parse", "HEAD"], text=True).strip()
        if actual != revision:
            raise ValueError(f"Wrong {name} source revision: {actual}")
    manifest = json.loads((a.model / "manifest.json").read_text())
    model = a.model / "zipdepth-mono-rv1103.rknn"
    if digest(model) != manifest["rknn_sha256"] or manifest["target"] != "rv1103":
        raise ValueError("Model checksum/target mismatch")
    parity = json.loads(a.parity.read_text())
    if max(parity["pytorch_vs_onnx_max_abs"]) > 1e-5:
        raise ValueError("Float export is not equivalent to the checkpoint")
    manifest.update(source_revisions=PINS, export_parity=parity,
                    weights_license="MIT (official repository LICENSE; no separate checkpoint restriction found)",
                    glibc="Debian libc6-armhf-cross 2.36-8cross1 (private helper only)",
                    dataset_url="https://www.eth3d.net/data/two_view_training.zip",
                    dataset_archive_sha256="e45587e72079fbbb746023b24537cd8d278fc7221c6f146333fb4a2ae407ccfc",
                    hardware_validated=False)
    for image in manifest["images"]:
        image.pop("tensor", None)  # retain source names/hashes, no ephemeral host paths
    a.output.mkdir(parents=True, exist_ok=True)

    def copy(source, relative):
        target = a.output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)

    shutil.copytree(a.glibc / "usr/arm-linux-gnueabihf/include", a.output / "glibc/include", dirs_exist_ok=True)
    for name in ["crt1.o", "crti.o", "crtn.o", "libc.so.6", "libm.so.6", "libpthread.so.0", "ld-linux-armhf.so.3"]:
        copy(a.glibc / "usr/arm-linux-gnueabihf/lib" / name, "glibc/lib/"+name)
    sdk = a.rknn / "rknpu2/runtime/Linux/librknn_api"
    copy(sdk / "include/rknn_api.h", "include/rknn_api.h")
    copy(sdk / "armhf/librknnmrt.so", "lib/librknnmrt.so")
    copy(a.rga / "libs/Linux/gcc-armhf-c/librga.so", "lib/librga.so")
    for header in (a.rga / "include").glob("*.h"):
        copy(header, "include/"+header.name)
    copy(model, model.name)
    copy(a.zipdepth / "LICENSE", "licenses/ZipDepth-MIT")
    copy(a.rknn / "LICENSE", "licenses/RKNN-SDK")
    copy(a.rga / "COPYING", "licenses/RGA-Apache-2.0")
    copy(a.glibc / "usr/share/doc/libc6-armhf-cross/copyright", "licenses/glibc-copyright")
    (a.output / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    a.hash_list.write_text("".join(f"{digest(f)}  {f.relative_to(a.output)}\n"
                                 for f in sorted(a.output.rglob("*")) if f.is_file()))


if __name__ == "__main__":
    main()
