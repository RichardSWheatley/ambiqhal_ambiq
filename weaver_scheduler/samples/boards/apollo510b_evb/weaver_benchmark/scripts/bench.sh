#!/usr/bin/env bash
#
# Build and flash the canonical benchmark with each variant in turn.
# Captures the per-variant log + parses the CSV line into a comparison
# table.
#
# Usage: cd <zephyr-root> && samples/boards/apollo510b_evb/weaver_benchmark/scripts/bench.sh
# Requires: west, J-Link, pylink, a serial terminal capture tool.

set -euo pipefail

SAMPLE=samples/boards/apollo510b_evb/weaver_benchmark
LOGDIR=${WEAVER_BENCH_LOGDIR:-/tmp/weaver_bench}
mkdir -p "$LOGDIR"

run_variant() {
    local name=$1
    local extra_conf=$2

    echo "=== build [$name] ==="
    west build -p -b apollo510b_evb "$SAMPLE" \
        ${extra_conf:+-- -DEXTRA_CONF_FILE=$extra_conf}

    echo "=== flash [$name] ==="
    west flash

    echo "=== capture 65 s for [$name] ==="
    # Adjust the device path for your host.
    timeout 65 cat /dev/ttyUSB0 | tee "$LOGDIR/$name.log" || true
}

run_variant stock          stock.conf
run_variant weaver-scalar  ""
run_variant weaver-hybrid  hybrid.conf
run_variant weaver-mve     mve.conf

echo
echo "=== comparison table ==="
printf "%-15s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n" \
       variant imu_ovr ppg_ovr gatt_ovr fus_p50 fus_p99 hr_p50 hr_p99 tick_cyc promo prewarp
for v in stock weaver-scalar weaver-hybrid weaver-mve; do
    line=$(grep "^CSV " "$LOGDIR/$v.log" | tail -1 | sed 's/^CSV //')
    IFS=',' read -ra fields <<< "$line"
    printf "%-15s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n" "${fields[@]}"
done
