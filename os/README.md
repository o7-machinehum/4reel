# 4reel OS

Buildroot-based Linux image for the 4reel boards (RV1103 and RV1106).

```sh
git submodule update --init
cd buildroot
make BR2_EXTERNAL=$PWD/../ rv1103g_defconfig
make
```

The SD-card image is written to `os/output/rv1103g/images/sdcard.img`.
