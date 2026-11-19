#!/usr/bin/env python3
"""Read-only inspection of a regular SD image file; never mounts or flashes it."""
import argparse
import hashlib
import json
import stat
import struct
import subprocess
from pathlib import Path


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--images", type=Path, required=True)
    p.add_argument("--work", type=Path, required=True)
    p.add_argument("--host", type=Path, required=True)
    p.add_argument("--report", type=Path, required=True)
    a = p.parse_args()
    image = a.images / "sdcard.img"
    if not stat.S_ISREG(image.stat().st_mode):
        raise ValueError("Only regular image files may be inspected")
    a.work.mkdir(parents=True, exist_ok=True)
    with image.open("rb") as f:
        mbr = f.read(512)
        assert mbr[510:] == b"\x55\xaa"
        partitions = []
        for i in range(2):
            entry = mbr[446+i*16:462+i*16]
            start, sectors = struct.unpack_from("<II", entry, 8)
            partitions.append(dict(start=start, sectors=sectors, type=entry[4], bootable=entry[0] == 128))
        assert partitions == [dict(start=32768, sectors=65536, type=12, bootable=True),
                              dict(start=98304, sectors=262144, type=131, bootable=False)]
        for part, name in zip(partitions, ["boot.vfat", "rootfs.ext4"]):
            f.seek(part["start"]*512)
            expected = (a.images / name).read_bytes()
            assert len(expected) == part["sectors"]*512
            assert f.read(len(expected)) == expected
        f.seek(32768)
        uboot = (a.images / "u-boot-rockchip.bin").read_bytes()
        assert f.read(len(uboot)) == uboot
        f.seek(partitions[1]["start"]*512)
        root = a.work / "rootfs-from-sd.ext4"
        root.write_bytes(f.read(partitions[1]["sectors"]*512))
    fsck = subprocess.run([str(a.host / "sbin/e2fsck"), "-fn", str(root)], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    assert fsck.returncode == 0, fsck.stdout
    files = ["/usr/bin/4reel-stereo", "/usr/bin/4reel-web",
             "/opt/4reel-rknn/bin/zipdepth-runner",
             "/usr/share/4reel-stereo/zipdepth-mono-rv1103.rknn",
             "/usr/share/4reel-stereo/manifest.json",
             "/etc/init.d/S85fourreel-web", "/etc/init.d/S50dropbear",
             "/etc/network/interfaces", "/etc/dnsmasq.conf"]
    files += ["/opt/4reel-rknn/lib/"+x for x in
              ["libc.so.6", "libm.so.6", "libpthread.so.0", "ld-linux-armhf.so.3", "librga.so", "librknnmrt.so"]]
    files += ["/usr/share/4reel-stereo/licenses/"+x for x in
              ["ZipDepth-MIT", "RGA-Apache-2.0", "RKNN-SDK", "glibc-copyright"]]
    hashes = {}
    for name in files:
        dest = a.work / name.lstrip("/")
        dest.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([str(a.host / "sbin/debugfs"), "-R", f"dump {name} {dest}", str(root)],
                       check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert dest.is_file() and dest.stat().st_size > 0, name
        hashes[name] = sha(dest)
    manifest = json.loads((a.work / "usr/share/4reel-stereo/manifest.json").read_text())
    assert hashes["/usr/share/4reel-stereo/zipdepth-mono-rv1103.rknn"] == manifest["rknn_sha256"]
    absent = subprocess.check_output([str(a.host / "sbin/debugfs"), "-R", "stat /etc/4reel/unit.ds", str(root)],
                                     stderr=subprocess.STDOUT, text=True)
    assert "not found" in absent
    boot = subprocess.check_output([str(a.host / "bin/mdir"), "-i", str(a.images / "boot.vfat"), "::"], text=True)
    assert "zImage" in boot and "rv1103g-4reel.dtb" in boot
    binaries = {}
    for name in ["/usr/bin/4reel-stereo", "/opt/4reel-rknn/bin/zipdepth-runner"]:
        info = subprocess.check_output(["readelf", "-h", "-l", "-d", str(a.work / name.lstrip("/"))], text=True)
        assert "ARM" in info
        interpreter = "/lib/ld-musl-armhf.so.1" if name.startswith("/usr/") else "/opt/4reel-rknn/lib/ld-linux-armhf.so.3"
        assert interpreter in info
        binaries[name] = info
    a.report.write_text(json.dumps(dict(image_sha256=sha(image), bytes=image.stat().st_size,
        partitions=partitions, uboot_region_matches=True, boot_partition_matches=True,
        rootfs_partition_matches=True, filesystem_check=fsck.stdout, files=hashes,
        no_unit_calibration=True, boot_listing=boot, elf=binaries,
        hardware_accessed=False, hardware_validated=False), indent=2)+"\n")
    print(f"PASS: partitions, U-Boot, boot files, ext4, ARM ELFs, libraries, services, real model and licenses: {a.report}")


if __name__ == "__main__":
    main()
