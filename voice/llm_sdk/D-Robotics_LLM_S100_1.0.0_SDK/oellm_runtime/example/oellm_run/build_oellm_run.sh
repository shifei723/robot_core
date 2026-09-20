#!/bin/bash

if [ ${LINARO_GCC_ROOT} ]; then
  LINARO_GCC_ROOT=${LINARO_GCC_ROOT}
else
  echo "Please set environment LINARO_GCC_ROOT correctly"
  LINARO_GCC_ROOT=/opt/aarch64/arm-gnu-toolchain-12.2.rel1-x86_64-aarch64-none-linux-gnu
fi

export CC="${LINARO_GCC_ROOT}/bin/aarch64-none-linux-gnu-gcc"
export CXX="${LINARO_GCC_ROOT}/bin/aarch64-none-linux-gnu-g++"

if [ -d "build" ]; then
  rm -rf build
fi

mkdir build
cd build
cmake ..
make
