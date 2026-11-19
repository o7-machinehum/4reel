#!/bin/sh
# Build a glibc process with the same ARM EABI compiler as Buildroot. All libc
# headers, CRT objects and DSOs come from the private, hash-checked sysroot.
set -eu
cc=$1
kernel_headers=$2
bundle=$3
source=$4
output=$5
lib=$bundle/glibc/lib
"$cc" -std=c11 -Wall -Wextra -Werror -O2 -fno-pie -no-pie -nostdinc \
    -isystem "$bundle/glibc/include" -isystem "$("$cc" -print-file-name=include)" \
    -idirafter "$kernel_headers" -I "$bundle/include" -nostdlib \
    "$lib/crt1.o" "$lib/crti.o" "$("$cc" -print-file-name=crtbegin.o)" "$source" \
    -L "$lib" -L "$bundle/lib" -Wl,--disable-new-dtags,-rpath,/opt/4reel-rknn/lib \
    -Wl,--dynamic-linker=/opt/4reel-rknn/lib/ld-linux-armhf.so.3 \
    -Wl,-rpath-link,"$lib" -lrga -lrknnmrt -l:libc.so.6 -l:libm.so.6 \
    -l:ld-linux-armhf.so.3 -lgcc "$("$cc" -print-file-name=crtend.o)" \
    "$lib/crtn.o" -o "$output"
