# 4reel OS

Buildroot-based Linux image for the 4reel boards (RV1103 and RV1106).

```sh
git submodule update --init
cd buildroot
make BR2_EXTERNAL=$PWD/../ rv1103g_defconfig
make
```

The SD-card image is written to `os/buildroot/output/images/sdcard.img`.

Linux and U-Boot use standalone configs in `board/rv1103g/{linux,uboot}_defconfig`.
After menuconfig changes, save them with `make linux-update-defconfig` or
`make uboot-update-defconfig`. Omit the machine-specific
`CONFIG_ENV_DEFAULT_ENV_TEXT_FILE` line from the saved U-Boot config; Buildroot
sets it from `uboot.env`.

The device-hosted camera dashboard is at `http://192.168.77.1:8080/`;
[calibration instructions](board/rv1103g/WEB_DEPTH.md) cover its first depth map.

On the RV1103G board, USB-C presents a network adapter to the host. The host
gets an address via DHCP; connect with `ssh root@192.168.77.1`. Root logs in
without a password in this development image.
