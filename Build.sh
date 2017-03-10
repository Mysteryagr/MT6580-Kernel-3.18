#!/bin/bash

#Mysteryagr
#Compile kernel with a build script to make things simple

mkdir -p out

#Change toolchain path before using build script!
#export CROSS_COMPILE=~/toolchains/arm-eabi-linaro-4.7.3/bin/arm-eabi-

#Enable when needed:
export USE_CCACHE=1

export ARCH=arm ARCH_MTK_PLATFORM=mt6580

#Enable only when needed:
#make clean
#make mrproper
#Or simply delete out directory to clean source

#Defconfig for Wijo Lenny 3
make -C $PWD O=$PWD/out ARCH=arm v3702_defconfig

#Defconfig for Infinix Hot 2
#make -C $PWD O=$PWD/out ARCH=arm x510_defconfig

#Edit the number according to the number of CPUs you have in your PC:
make -j4 -C $PWD O=$PWD/out ARCH=arm
