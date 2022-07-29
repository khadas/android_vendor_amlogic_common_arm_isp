#!/bin/bash

export PATH=/opt/clang-r377782b/bin/:/opt/gcc-linaro-6.3.1-2017.02-x86_64_aarch64-linux-gnu/bin/:$PATH
export CROSS_COMPILE=aarch64-linux-gnu-
export CLANG_TRIPLE=aarch64-linux-gnu-
clang_flags="ARCH=arm64 CC=clang HOSTCC=clang LD=ld.lld NM=llvm-nm OBJCOPY=llvm-objcopy"

make ${clang_flags} $1
