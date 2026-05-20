#!/usr/bin/env bash
#
# Run the five Weaver scenarios under stock and Weaver builds, diff
# the SCEN-CSV lines, and SCORE each result against the RFC §7
# acceptance gates. Exits non-zero on any HARD gate failure (M3).
#
# Usage: cd <zephyr-root> &&
#        samples/boards/apollo510b_evb/weaver_scenarios/scripts/run.sh

set -euo pipefail

SAMPLE=samples/boards/apollo510b_evb/weaver_scenarios
LOGDIR=${WEAVER_SCEN_LOGDIR:-/tmp/weaver_scen}
TTY=${WEAVER_SCEN_TTY:-/dev/ttyUSB0}
RUN_S=${WEAVER_SCEN_RUN_S:-85}
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
    timeout "$RUN_S" cat "$TTY" | tee "$LOGDIR/$name.log" || true
}

build_flash_capture stock  stock.conf
build_flash_capture weaver ""

# SCEN-CSV format from src/main.c:
#   SCEN-CSV variant,name,pass,overruns,misses,p50,p99,max,thr_peak,misc

declare -A S_OVR S_MIS S_P99 S_MAX S_THR S_THRENT S_MISC
declare -A W_OVR W_MIS W_P99 W_MAX W_THR W_THRENT W_MISC

parse_scen() {
    local prefix=$1 logfile=$2 var=$3
    for scen in imu_overrun ble_deadline ppg_latency burst_shed display_defer; do
        local line
        line=$(grep "^SCEN-CSV $var,$scen," "$logfile" | tail -1 || true)
        [[ -z "$line" ]] && continue
        line=${line#SCEN-CSV }
        IFS=',' read -ra f <<< "$line"
        # f[0]=var f[1]=name f[2]=pass f[3]=overruns f[4]=misses
        # f[5]=p50 f[6]=p99 f[7]=max f[8]=thr_peak f[9]=misc
        eval "${prefix}_OVR[$scen]=${f[3]}"
        eval "${prefix}_MIS[$scen]=${f[4]}"
        eval "${prefix}_P99[$scen]=${f[6]}"
        eval "${prefix}_MAX[$scen]=${f[7]}"
        eval "${prefix}_THR[$scen]=${f[8]}"
        eval "${prefix}_THRENT[$scen]=${f[9]}"
        eval "${prefix}_MISC[$scen]=${f[9]}"
    done
}

parse_scen S /tmp/weaver_scen/stock.log  stock
parse_scen W /tmp/weaver_scen/weaver.log weaver

gate() { # name expr-rendered-as-string actual_pass
    local name=$1 expr=$2 ok=$3
    if [[ "$ok" -eq 1 ]]; then
        printf "  PASS  %-50s %s\n" "$name" "$expr"
    else
        printf "  FAIL  %-50s %s\n" "$name" "$expr"
    fi
}

echo
echo "=== per-scenario comparison ==="
printf "%-15s %-12s %-10s %-10s\n" scenario metric stock weaver
echo "------------------------------------------------------"
for scen in imu_overrun ble_deadline ppg_latency burst_shed display_defer; do
    case "$scen" in
        imu_overrun)   printf "%-15s %-12s %-10s %-10s\n" "$scen" "overruns"  "${S_OVR[$scen]:-?}"  "${W_OVR[$scen]:-?}";;
        ble_deadline)  printf "%-15s %-12s %-10s %-10s\n" "$scen" "misses"    "${S_MIS[$scen]:-?}"  "${W_MIS[$scen]:-?}";;
        ppg_latency)   printf "%-15s %-12s %-10s %-10s\n" "$scen" "p99_us"    "${S_P99[$scen]:-?}"  "${W_P99[$scen]:-?}"
                       printf "%-15s %-12s %-10s %-10s\n" ""      "max_us"    "${S_MAX[$scen]:-?}"  "${W_MAX[$scen]:-?}";;
        burst_shed)    printf "%-15s %-12s %-10s %-10s\n" "$scen" "thr_peak"  "${S_THR[$scen]:-?}"  "${W_THR[$scen]:-?}"
                       printf "%-15s %-12s %-10s %-10s\n" ""      "thr_ent"   "${S_THRENT[$scen]:-?}" "${W_THRENT[$scen]:-?}";;
        display_defer) printf "%-15s %-12s %-10s %-10s\n" "$scen" "peak_fifo" "${S_MISC[$scen]:-?}" "${W_MISC[$scen]:-?}";;
    esac
done

echo
echo "=== RFC §7 acceptance gates ==="
hard_fail=0
soft_fail=0

# M1 — IMU overruns: weaver <= 0.5 * stock
if [[ -n "${S_OVR[imu_overrun]:-}" && -n "${W_OVR[imu_overrun]:-}" ]]; then
    s=${S_OVR[imu_overrun]}; w=${W_OVR[imu_overrun]}
    thresh=$(( s / 2 ))
    if [[ "$w" -le "$thresh" ]]; then ok=1; else ok=0; soft_fail=$((soft_fail+1)); fi
    gate "M1 imu_overrun: weaver <= 0.5*stock" "weaver=$w  stock=$s  thresh=$thresh" $ok
fi

# M3 — BLE deadline misses: weaver MUST be zero (HARD GATE)
if [[ -n "${W_MIS[ble_deadline]:-}" ]]; then
    w=${W_MIS[ble_deadline]}
    if [[ "$w" -eq 0 ]]; then ok=1; else ok=0; hard_fail=$((hard_fail+1)); fi
    gate "M3 ble_deadline: weaver_misses == 0 (HARD)" "weaver=$w" $ok
fi

# M2 — PPG P99 latency: weaver <= 0.7 * stock
if [[ -n "${S_P99[ppg_latency]:-}" && -n "${W_P99[ppg_latency]:-}" ]]; then
    s=${S_P99[ppg_latency]}; w=${W_P99[ppg_latency]}
    thresh=$(( (s * 7) / 10 ))
    if [[ "$w" -le "$thresh" ]]; then ok=1; else ok=0; soft_fail=$((soft_fail+1)); fi
    gate "M2 ppg_latency: weaver_p99 <= 0.7*stock_p99" "weaver=$w  stock=$s  thresh=$thresh" $ok
fi

# M6 — fuzzy throttle: weaver thr_peak >= 192, thr_ent == 3
if [[ -n "${W_THR[burst_shed]:-}" && -n "${W_THRENT[burst_shed]:-}" ]]; then
    tp=${W_THR[burst_shed]}; te=${W_THRENT[burst_shed]}
    if [[ "$tp" -ge 192 && "$te" -eq 3 ]]; then ok=1; else ok=0; soft_fail=$((soft_fail+1)); fi
    gate "M6 burst_shed: thr_peak>=192 AND thr_ent==3" "thr_peak=$tp thr_ent=$te" $ok
fi

# M1' — display defer: weaver peak_fifo <= stock peak_fifo (regression)
if [[ -n "${S_MISC[display_defer]:-}" && -n "${W_MISC[display_defer]:-}" ]]; then
    s=${S_MISC[display_defer]}; w=${W_MISC[display_defer]}
    if [[ "$w" -le "$s" ]]; then ok=1; else ok=0; soft_fail=$((soft_fail+1)); fi
    gate "M1' display_defer: weaver_peak_fifo <= stock" "weaver=$w stock=$s" $ok
fi

echo
if [[ "$hard_fail" -gt 0 ]]; then
    echo "RESULT: HARD GATE FAILURE ($hard_fail). Soft failures: $soft_fail."
    exit 1
elif [[ "$soft_fail" -gt 0 ]]; then
    echo "RESULT: hard gates OK; $soft_fail soft-gate failures (tuning targets)."
    exit 0
else
    echo "RESULT: all gates passed."
    exit 0
fi
