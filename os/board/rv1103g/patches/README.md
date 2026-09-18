# Pending upstream support

Buildroot applies these queues through `BR2_GLOBAL_PATCH_DIR`, in filename
order, after extracting each source tree.

## Linux

There is no Linux patch queue. The pinned Rockchip 6.1 source already contains
the RV1103/RV1106 clock, pinctrl, and device-tree support. The obsolete
mainline RV1103B queue was removed because it targets different silicon and
conflicts with the vendor source. `linux/linux.hash` verifies the pinned source
archive and its license files.

## U-Boot

The U-Boot queue is the complete 15-patch v3 submission with cover
Message-ID `20260707153135.2048115-1-sjg@chromium.org`. It applies to exact
base commit:

```text
ee5d46b45ec0c63f8f9dd1e816e0dac3452ccc3d
```

Keeping the full series preserves the dependencies between the early RV1103B
patches and the later RV1106/RV1103 changes. The RV1103G board DTS, default
environment, and Kconfig fragment override the reference boards without
modifying the submitted patches.
