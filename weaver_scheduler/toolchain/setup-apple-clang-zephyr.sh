#!/bin/zsh
# Build Zephyr for Ambiq Cortex-M55 parts with Xcode's Apple clang (unmodified).
#   compiler : Xcode's clang, via a thin wrapper (adds -resource-dir and -B only)
#   linker   : Homebrew lld          binutils : Homebrew llvm
#   builtins : compiler-rt 21.1.8, built here with Xcode's clang (M55 soft/hard float, M3)
#   libc     : picolibc, built from source by Zephyr with the same compiler
#
# Usage: ./setup-apple-clang-zephyr.sh [workspace]   (default ~/zephyrproject)
set -eo pipefail

WS="${1:-$HOME/zephyrproject}"
TC="$WS/toolchains/apple-clang"
LLVM_TAG=llvmorg-21.1.8          # matches Apple clang 21 (clang-2100.x)
PY=/opt/homebrew/bin/python3.12

echo "== Homebrew packages"
brew install lld llvm cmake ninja dtc gperf ccache python@3.12 qemu
# pyserial (for serial-capture.py) comes with Zephyr's run-test requirements below.

echo "== Zephyr workspace at $WS"
mkdir -p "$WS"; cd "$WS"
[[ -d .venv312 ]] || "$PY" -m venv .venv312
source .venv312/bin/activate
pip install -q west
[[ -d .west ]] || west init -m https://github.com/zephyrproject-rtos/zephyr --mr main .
# Only the modules Ambiq + QEMU builds need. Drop this line for a full checkout.
west config manifest.project-filter -- '-.*,+cmsis,+cmsis_6,+hal_ambiq,+picolibc'
west update
# USE_SWITCH FPU context-switch fixes (handoff doc 2.8), same as zephyrproject-rtos/zephyr PR 122749.
PATCH="$(cd "$(dirname "$0")" && pwd)/zephyr-arm-m-switch-clang-fpu.patch"
if [[ -f "$PATCH" ]] && git -C zephyr apply --check "$PATCH" 2>/dev/null; then
  git -C zephyr apply "$PATCH"
fi
pip install -q -r zephyr/scripts/requirements-base.txt \
               -r zephyr/scripts/requirements-build-test.txt \
               -r zephyr/scripts/requirements-run-test.txt

echo "== Toolchain directory $TC"
mkdir -p "$TC/bin"
for t in ld.lld; do ln -sf /opt/homebrew/opt/lld/bin/$t "$TC/bin/$t"; done
for t in llvm-objcopy llvm-ar llvm-readelf llvm-nm llvm-objdump llvm-ranlib llvm-strip llvm-size; do
  ln -sf /opt/homebrew/opt/llvm/bin/$t "$TC/bin/$t"
done
cat > "$TC/bin/clang" <<'EOF'
#!/bin/sh
# Thin wrapper around Xcode's Apple clang (binary is not modified).
# Adds: a resource dir holding Cortex-M compiler-rt builtins (soft or hard float ABI,
# chosen from the -mfloat-abi flag), and -B so -fuse-ld=lld finds Homebrew ld.lld.
TC="$(cd "$(dirname "$0")/.." && pwd)"
XCLANG="$(xcrun --find clang)"
ABI=soft
for a in "$@"; do case "$a" in -mfloat-abi=hard) ABI=hard;; -mfloat-abi=soft|-mfloat-abi=softfp) ABI=soft;; esac; done
exec "$XCLANG" -resource-dir "$TC/resource-$ABI" -B"$TC/bin" "$@"
EOF
chmod +x "$TC/bin/clang"; ln -sf clang "$TC/bin/clang++"

echo "== compiler-rt builtins ($LLVM_TAG) built with Xcode clang"
SRC="$WS/toolchains/src/llvm-project"
if [[ ! -d "$SRC" ]]; then
  git clone -q --depth 1 --filter=blob:none --sparse --branch "$LLVM_TAG" \
      https://github.com/llvm/llvm-project "$SRC"
  git -C "$SRC" sparse-checkout set compiler-rt cmake llvm/cmake llvm/utils/llvm-lit runtimes
fi
XRES="$(dirname "$(xcrun --find clang)")/../lib/clang/$(xcrun clang -dumpversion | cut -d. -f1)"
# name        target                    cpu         float ABI  triple dirs clang may look in
BUILTINS=(
  "m55-soft     armv8.1m.main-none-eabi   cortex-m55  soft  armv8.1m.main,armv8m.main,thumbv8.1m.main,thumbv8m.main"
  "m55-hard     armv8.1m.main-none-eabi   cortex-m55  hard  armv8.1m.main,armv8m.main,thumbv8.1m.main,thumbv8m.main"
  "m3           armv7m-none-eabi          cortex-m3   soft  armv7m,thumbv7m"
)
for spec in "${BUILTINS[@]}"; do
  read -r name target cpu abi dirs <<< "$spec"
  B="$WS/toolchains/build/builtins-$name"; rm -rf "$B"
  FL="-mcpu=$cpu -mthumb -mfloat-abi=$abi"
  cmake -G Ninja -S "$SRC/compiler-rt" -B "$B" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_SYSTEM_NAME=Generic -DCMAKE_SYSTEM_PROCESSOR=arm \
    -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
    -DCMAKE_C_COMPILER="$TC/bin/clang" -DCMAKE_ASM_COMPILER="$TC/bin/clang" \
    -DCMAKE_C_COMPILER_TARGET=$target -DCMAKE_ASM_COMPILER_TARGET=$target \
    -DCMAKE_AR="$TC/bin/llvm-ar" -DCMAKE_RANLIB="$TC/bin/llvm-ranlib" -DCMAKE_NM="$TC/bin/llvm-nm" \
    -DCMAKE_C_FLAGS="$FL -ffunction-sections -fdata-sections" -DCMAKE_ASM_FLAGS="$FL" \
    -DCOMPILER_RT_BAREMETAL_BUILD=ON -DCOMPILER_RT_DEFAULT_TARGET_ONLY=ON \
    -DCOMPILER_RT_BUILD_BUILTINS=ON -DCOMPILER_RT_BUILD_CRT=OFF -DCOMPILER_RT_BUILD_SANITIZERS=OFF \
    -DCOMPILER_RT_BUILD_XRAY=OFF -DCOMPILER_RT_BUILD_LIBFUZZER=OFF -DCOMPILER_RT_BUILD_PROFILE=OFF \
    -DCOMPILER_RT_BUILD_MEMPROF=OFF -DCOMPILER_RT_BUILD_ORC=OFF -DCOMPILER_RT_BUILD_CTX_PROFILE=OFF \
    -DCOMPILER_RT_INCLUDE_TESTS=OFF -DLLVM_ENABLE_PER_TARGET_RUNTIME_DIR=ON -DCOMPILER_RT_OS_DIR=baremetal >/dev/null
  ninja -C "$B" >/dev/null
  LIB="$(find "$B/lib" -name libclang_rt.builtins.a | head -1)"
  R="$TC/resource-$abi"; mkdir -p "$R/lib"; ln -sfn "$XRES/include" "$R/include"
  for tr in ${(s:,:)dirs}; do
    mkdir -p "$R/lib/$tr-unknown-none-eabi"; cp "$LIB" "$R/lib/$tr-unknown-none-eabi/"
  done
done

echo "== Environment script and Desktop shortcut"
HERE="$(cd "$(dirname "$0")" && pwd)"
cp "$HERE/env/zephyr-env.sh" "$WS/zephyr-env.sh"
mkdir -p "$WS/.zsh-zephyr"; cp "$HERE/env/zshrc" "$WS/.zsh-zephyr/.zshrc"
cp "$HERE/env/apple-clang.conf" "$TC/apple-clang.conf"
cp "$HERE/env/Zephyr Shell.command" "$HOME/Desktop/Zephyr Shell.command"
chmod +x "$HOME/Desktop/Zephyr Shell.command"

cat <<EOF

Done. Double-click "Zephyr Shell" on the Desktop, or:
  source $WS/zephyr-env.sh
  west build -p -b qemu_cortex_m3 zephyr/samples/hello_world -d build/m3 && west build -t run -d build/m3
EOF
