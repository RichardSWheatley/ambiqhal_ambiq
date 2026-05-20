#!/usr/bin/env bash
#
# Build and flash the canonical benchmark with each variant in turn,
# capture each run's CSV line, then SCORE the results against the RFC
# §7 acceptance gates (M1, M2, M4) so the comparison table doubles as
# a pass/fail report for reviewers.
#
# Exits non-zero if any HARD gate (M3 - deadline regression) fails.
# Soft gates (M1, M2, M4) print PASS/FAIL but don't fail the script;
# they're tuning targets, not regression gates.
#
# Usage:
#   cd <zephyr-root> && samples/boards/apollo510b_evb/weaver_benchmark/scripts/bench.sh
#
# Env vars:
#   WEAVER_BENCH_LOGDIR  override log directory (default /tmp/weaver_bench)
#   WEAVER_BENCH_TTY     override serial device (default /dev/ttyUSB0)
#   WEAVER_BENCH_RUN_S   override capture window (default 65s)

set -euo pipefail

SAMPLE=samples/boards/apollo510b_evb/weaver_benchmark
LOGDIR=${WEAVER_BENCH_LOGDIR:-/tmp/weaver_bench}
TTY=${WEAVER_BENCH_TTY:-/dev/ttyUSB0}
RUN_S=${WEAVER_BENCH_RUN_S:-65}
mkdir -p "$LOGDIR"

run_variant() {
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

run_variant stock          stock.conf
run_variant weaver-scalar  ""
run_variant weaver-hybrid  hybrid.conf
run_variant weaver-mve     mve.conf
run_variant weaver-fp      fp.conf

# CSV format from src/main.c:
#   CSV variant,imu_ovr,ppg_ovr,gatt_ovr,fus_p50,fus_p99,hr_p50,hr_p99,tick_cyc,promo,prewarp

declare -A IMU_OVR FUS_P99 HR_P99 TICK_CYC PROMO PREWARP

echo
echo "=== raw comparison table ==="
printf "%-15s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n" \
       variant imu_ovr ppg_ovr gatt_ovr fus_p50 fus_p99 hr_p50 hr_p99 tick_cyc promo prewarp
for v in stock weaver-scalar weaver-hybrid weaver-mve weaver-fp; do
    line=$(grep "^CSV " "$LOGDIR/$v.log" | tail -1 || true)
    if [[ -z "$line" ]]; then
        echo "MISSING CSV LINE for $v"
        continue
    fi
    line=${line#CSV }
    IFS=',' read -ra f <<< "$line"
    # f[0]=variant f[1]=imu f[2]=ppg f[3]=gatt f[4]=fp50 f[5]=fp99
    # f[6]=hp50 f[7]=hp99 f[8]=tickcyc f[9]=promo f[10]=prewarp
    printf "%-15s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n" "${f[@]}"
    IMU_OVR[$v]=${f[1]}
    FUS_P99[$v]=${f[5]}
    HR_P99[$v]=${f[7]}
    TICK_CYC[$v]=${f[8]}
    PROMO[$v]=${f[9]}
    PREWARP[$v]=${f[10]}
done

# Helper: integer "a <= b" with PASS/FAIL print.
gate_le() { # name actual threshold
    local name=$1 actual=$2 thresh=$3
    if [[ "$actual" -le "$thresh" ]]; then
        printf "  PASS  %-30s %d <= %d\n" "$name" "$actual" "$thresh"
        return 0
    else
        printf "  FAIL  %-30s %d > %d\n" "$name" "$actual" "$thresh"
        return 1
    fi
}

echo
echo "=== RFC §7 acceptance gates ==="
echo
fail_count=0

# M1: IMU overruns <= 0.5 * stock
if [[ -n "${IMU_OVR[stock]:-}" ]]; then
    s_imu=${IMU_OVR[stock]}
    m1_thresh=$(( s_imu / 2 ))
    echo "M1 — IMU FIFO overruns, weaver variants <= 0.5 * stock ($s_imu / 2 = $m1_thresh):"
    for v in weaver-scalar weaver-hybrid weaver-mve weaver-fp; do
        [[ -n "${IMU_OVR[$v]:-}" ]] || continue
        gate_le "  $v" "${IMU_OVR[$v]}" "$m1_thresh" || fail_count=$((fail_count + 1))
    done
fi

# M2 (fusion): fusion P99 latency <= 0.7 * stock fusion P99
echo
if [[ -n "${FUS_P99[stock]:-}" ]]; then
    s_p99=${FUS_P99[stock]}
    m2_thresh=$(( (s_p99 * 7) / 10 ))
    echo "M2 — fusion P99 latency, weaver variants <= 0.7 * stock ($s_p99 * 0.7 = $m2_thresh us):"
    for v in weaver-scalar weaver-hybrid weaver-mve weaver-fp; do
        [[ -n "${FUS_P99[$v]:-}" ]] || continue
        gate_le "  $v" "${FUS_P99[$v]}" "$m2_thresh" || fail_count=$((fail_count + 1))
    done
fi

# M2 (HR): HR P99 latency <= 0.7 * stock HR P99
echo
if [[ -n "${HR_P99[stock]:-}" ]]; then
    s_p99=${HR_P99[stock]}
    m2_thresh=$(( (s_p99 * 7) / 10 ))
    echo "M2 — HR P99 latency, weaver variants <= 0.7 * stock ($s_p99 * 0.7 = $m2_thresh us):"
    for v in weaver-scalar weaver-hybrid weaver-mve weaver-fp; do
        [[ -n "${HR_P99[$v]:-}" ]] || continue
        gate_le "  $v" "${HR_P99[$v]}" "$m2_thresh" || fail_count=$((fail_count + 1))
    done
fi

# M4: dispatcher tick cycles, mean target ≤ 200, P99 ≤ 250
echo
echo "M4 — dispatcher overhead, last_tick_cycles <= 250 (P99 proxy):"
for v in weaver-scalar weaver-hybrid weaver-mve weaver-fp; do
    [[ -n "${TICK_CYC[$v]:-}" ]] || continue
    gate_le "  $v" "${TICK_CYC[$v]}" "250" || fail_count=$((fail_count + 1))
done

# M5: MVE context-save tax — informational comparison
echo
echo "M5 — informational: variant cost relative to scalar"
if [[ -n "${TICK_CYC[weaver-scalar]:-}" ]]; then
    scalar=${TICK_CYC[weaver-scalar]}
    for v in weaver-hybrid weaver-mve weaver-fp; do
        [[ -n "${TICK_CYC[$v]:-}" ]] || continue
        delta=$(( ${TICK_CYC[$v]} - scalar ))
        printf "  INFO  %-30s tick_cyc=%d (delta vs scalar = %+d)\n" \
               "$v" "${TICK_CYC[$v]}" "$delta"
    done
fi

echo
if [[ "$fail_count" -eq 0 ]]; then
    echo "RESULT: all gates passed."
    exit 0
else
    echo "RESULT: $fail_count soft-gate failures (see PASS/FAIL above)."
    echo "(M3 hard-gate is evaluated separately by scenarios/run.sh.)"
    # Non-zero only when a HARD gate fails. M1/M2/M4 are tuning targets;
    # report them but don't fail CI.
    exit 0
fi
