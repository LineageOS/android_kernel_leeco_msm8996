export CROSS_COMPILE="~/android/cm13/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-"
export JOBS=16
make -C $PWD O=$PWD/out ARCH=arm64 msm8996-le_x2-perf_defconfig
make -j$JOBS -C $PWD O=$PWD/out ARCH=arm64 KCFLAGS=-mno-android
make -j$JOBS -C $PWD O=$PWD/out ARCH=arm64 KCFLAGS=-mno-android modules
