#!/usr/bin/env bash
#
# Run the five Weaver scenarios under stock and Weaver builds, then
# diff the SCEN-CSV lines to produce a per-scenario delta table that
# maps directly to RFC §7 acceptance criteria M1-M7.
#
# Usage:
#   cd <zephyr-root>
#   samples/boards/apollo510b_evb/weaver_scenarios/scripts/run.sh

set -euo pipefail

SAMPLE=samples/boards/apollo510b_evb/weaver_scenarios
LOGDIR=${WEAVER_SCEN_LOGDIR:-/tmp/weaver_scen}
TTY=${WEAVER_SCEN_TTY:-/dev/ttyUSB0}
RUN_S=85   # ~75 s of scenarios + setup overhead
mkdir -p "$LOGDIR"

build_flash_capture() {
    local name=$1
    local extra=$2

    echo "=== build [$name] ==="
    west build -p -b apollo510b_evb "$SAMPLE" \
        ${extra:+-- -DEXTRA_CONF_FILE=$extra}

    echo "=== flash [$name] ==="
    west flash

    echo "=== capture ${RUN_S}s for [$name] ==="
    timeout $RUN_S cat "$TTY" | tee "$LOGDIR/$name.log" || true
}

build_flash_capture stock  stock.conf
build_flash_capture weaver ""

echo
echo "=== per-scenario comparison ==="
printf "%-15s %-10s %-10s %-10s\n" scenario metric stock weaver
echo "---------------------------------------------"

for scen in imu_overrun ble_deadline ppg_latency burst_shed display_defer; do
    s_line=$(grep "^SCEN-CSV stock,$scen,"  "$LOGDIR/stock.log"  | tail -1 || true)
    w_line=$(grep "^SCEN-CSV weaver,$scen," "$LOGDIR/weaver.log" | tail -1 || true)
    # Fields after the label: pass,overruns,misses,p50,p99,max,throt_peak,misc
    [[ -z "$s_line" || -z "$w_line" ]] && { echo "MISSING LINE for $scen"; continue; }
    IFS=, read -ra s <<< "${s_line#SCEN-CSV }"
    IFS=, read -ra w <<< "${w_line#SCEN-CSV }"

    case "$scen" in
        imu_overrun)    printf "%-15s %-10s %-10s %-10s\n" "$scen" "overruns" "${s[3]}" "${w[3]}" ;;
        ble_deadline)   printf "%-15s %-10s %-10s %-10s\n" "$scen" "misses"   "${s[4]}" "${w[4]}" ;;
        ppg_latency)    printf "%-15s %-10s %-10s %-10s\n" "$scen" "p50_us"   "${s[5]}" "${w[5]}"
                        printf "%-15s %-10s %-10s %-10s\n" ""      "p99_us"   "${s[6]}" "${w[6]}"
                        printf "%-15s %-10s %-10s %-10s\n" ""      "max_us"   "${s[7]}" "${w[7]}" ;;
        burst_shed)     printf "%-15s %-10s %-10s %-10s\n" "$scen" "thr_peak" "${s[8]}" "${w[8]}"
                        printf "%-15s %-10s %-10s %-10s\n" ""      "thr_ent"  "${s[9]}" "${w[9]}" ;;
        display_defer)  printf "%-15s %-10s %-10s %-10s\n" "$scen" "peak_fifo" "${s[9]}" "${w[9]}" ;;
    esac
done

echo
echo "RFC acceptance gates:"
echo "  M1: weaver overruns <= 0.5 * stock overruns      (scen 1)"
echo "  M3: weaver misses == 0                           (scen 2 - hard gate)"
echo "  M2: weaver ppg p99 <= 0.7 * stock p99            (scen 3)"
echo "  M6: weaver throttle_peak >= 192, entries == 3    (scen 4)"
echo "  M1': weaver peak_fifo <= stock peak_fifo         (scen 5 - regression check)"
