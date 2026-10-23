# Apple Clang toolchain for Zephyr (macOS)

Builds Zephyr for Cortex-M with Xcode's Apple clang, unmodified. A thin
wrapper adds only `-resource-dir` and `-B`. The linker and binutils come
from Homebrew lld/llvm, and compiler-rt builtins are built locally with
Xcode's clang.

| File | Purpose |
|------|---------|
| `setup-apple-clang-zephyr.sh` | Reproduces the whole setup: Homebrew packages, Zephyr workspace, wrapper, compiler-rt (M55 soft/hard, M3) |
| `env/zephyr-env.sh` | `source` it to select Apple clang (default) or `zephyr_use gcc` |
| `env/apple-clang.conf` | `CONFIG_LLVM_USE_LLD=y`, `CONFIG_COMPILER_RT_RTLIB=y`; passed as `EXTRA_CONF_FILE` |
| `env/zshrc`, `env/Zephyr Shell.command` | Desktop shortcut that opens a shell with the environment loaded |
| `zephyr-arm-m-switch-clang-fpu.patch` | Zephyr 4.5 ARM USE_SWITCH FPU context-switch fix that clang needs (upstream PR 122749) |

```sh
./setup-apple-clang-zephyr.sh [workspace]   # default ~/zephyrproject
source ~/zephyrproject/zephyr-env.sh
```

## Running the Weaver suites with this toolchain

`native_sim` does not build on macOS (Zephyr's POSIX arch is Linux-only),
so the ztest suites run on `qemu_cortex_m3` instead. CI covers `native_sim`.

`ambiqzephyr` at `ambiq-stable` is Zephyr 4.3, where the toolchain variant
is `llvm` (4.5 renamed it `host/llvm`). Use the `cmsis_6` and `picolibc`
revisions from its own `west.yml`.

```sh
git clone --depth 1 --branch ambiq-stable https://github.com/AmbiqMicro/ambiqzephyr.git
cd ambiqzephyr
git apply /path/to/weaver_scheduler/0001-weaver-scheduler.patch

WS=~/zephyrproject
export ZEPHYR_BASE=$PWD ZEPHYR_TOOLCHAIN_VARIANT=llvm
export LLVM_TOOLCHAIN_PATH=$WS/toolchains/apple-clang
export EXTRA_CONF_FILE=$WS/toolchains/apple-clang/apple-clang.conf
$WS/.venv312/bin/python ./scripts/twister -p qemu_cortex_m3 \
    -T tests/kernel/weaver_api -T tests/kernel/weaver_stress \
    --fixture weaver_api --fixture weaver_stress \
    --force-toolchain --force-platform \
    -x "ZEPHYR_MODULES=<cmsis>;<cmsis_6>;<picolibc>"
```

Host-only quick check:

```sh
xcrun clang -std=c11 -O2 -Wall tests/kernel/weaver_paths_equiv/src/main.c -o /tmp/pe && /tmp/pe
# ALL THREE PATHS EQUIVALENT
```
