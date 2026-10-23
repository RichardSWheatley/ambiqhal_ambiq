# Zephyr environment for ~/zephyrproject.
#   source ~/zephyrproject/zephyr-env.sh          # Xcode Apple clang (default)
#   source ~/zephyrproject/zephyr-env.sh gcc      # Zephyr SDK GCC 14.3
WS="$HOME/zephyrproject"
source "$WS/.venv312/bin/activate"
export ZEPHYR_BASE="$WS/zephyr"

zephyr_use() {
  case "${1:-apple}" in
    gcc)
      export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
      export CROSS_COMPILE="$WS/toolchains/gcc/arm-zephyr-eabi/bin/arm-zephyr-eabi-"
      unset LLVM_TOOLCHAIN_PATH EXTRA_CONF_FILE ;;
    *)
      export ZEPHYR_TOOLCHAIN_VARIANT=host/llvm
      export LLVM_TOOLCHAIN_PATH="$WS/toolchains/apple-clang"
      # lld + compiler-rt, so no -D flags are needed on each build
      export EXTRA_CONF_FILE="$WS/toolchains/apple-clang/apple-clang.conf"
      unset CROSS_COMPILE ;;
  esac
  echo "Zephyr toolchain: ${1:-apple} ($ZEPHYR_TOOLCHAIN_VARIANT)"
}
zephyr_use "$1"

cd "$WS"
cat <<'TXT'
Zephyr workspace: ~/zephyrproject   (switch compiler: zephyr_use gcc | zephyr_use apple)
  QEMU Cortex-M3 : west build -p -b qemu_cortex_m3 zephyr/samples/hello_world -d build/m3 && west build -t run -d build/m3
                   (quit QEMU: Ctrl-A then X)
  QEMU Cortex-M55: west build -p -b mps3/corstone300/an547 zephyr/samples/hello_world -d build/m55 && west build -t run -d build/m55
  M55 MVE/FPU test: west build -p -b mps3/corstone300/an547 apps/apple-clang-proof -d build/m55-proof && west build -t run -d build/m55-proof
  Apollo510 EVB  : west build -p -b apollo510_evb apps/apple-clang-proof -d build/apollo && west flash -d build/apollo
  Serial console : tio -b 115200 /dev/cu.usbmodem0011600032921   (quit: Ctrl-T then Q)
TXT
