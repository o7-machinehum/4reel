# Pending upstream support

Buildroot applies these queues through `BR2_GLOBAL_PATCH_DIR`, in filename
order, after extracting each source tree.

## Linux

The pinned Rockchip 6.1 source already contains the RV1103/RV1106 clock,
pinctrl, and device-tree support. The Linux patch queue only adds board-required
behavior missing from that vendor tree. `linux/linux.hash` verifies the pinned
source archive and its license files.

The board patches make the left-camera rotation persistent and make CIF scale
capture start its sensor/CSI source without full-resolution memory DMA.
RV1103/RV1106 has one shared scaler: the left camera uses it; the right camera
uses normal CSI capture and software downsampling. Both sensors use VC0 on
their separate CSI receivers.
The CMA heap patch supplies the physical-address query and cache synchronization
needed by the RVE SAD backend; physical-address access requires root privileges.

## U-Boot

The U-Boot queue is the complete 15-patch v3 submission with cover
Message-ID `20260707153135.2048115-1-sjg@chromium.org`. It applies to exact
base commit:

```text
ee5d46b45ec0c63f8f9dd1e816e0dac3452ccc3d
```

Keeping the full series preserves the dependencies between the early RV1103B
patches and the later RV1106/RV1103 changes. The RV1103G board DTS, default
environment, and standalone `uboot_defconfig` configure 4reel without loading
the Luckfox defconfig or modifying the submitted patches. The existing
`CONFIG_TARGET_LUCKFOX_PICO_RV1103` selection supplies the shared 64 MiB board
initialization code; the device tree and boot environment are 4reel's.
