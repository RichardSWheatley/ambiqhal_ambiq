.. zephyr:code-sample:: weaver_benchmark
   :name: Weaver Benchmark (Apollo510B)

   Canonical wearable workload (RFC §7.1) for measuring the Weaver
   scheduler against stock Zephyr and across the three pressure-batch
   variants.

Overview
********

Runs the workload defined in ``RFC.md §7.1`` for 60 seconds with two
burst events at t=10s and t=20s. Tracks IMU/PPG/GATT FIFO overruns,
sensor-to-process latency histograms (P50/P99/max), Weaver dispatch
cycles via DWT, and Weaver internal stats.

Final output is a single ``CSV`` line per run that ``scripts/bench.sh``
parses into a comparison table.

Building each variant
*********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/apollo510b_evb/weaver_benchmark
   :board: apollo510b_evb
   :goals: build flash

Default ``prj.conf`` selects the SCALAR Weaver variant. Override:

.. code-block:: console

   # Stock Zephyr baseline (no Weaver)
   west build -p -b apollo510b_evb samples/boards/apollo510b_evb/weaver_benchmark \
       -- -DEXTRA_CONF_FILE=stock.conf

   # Hybrid MVE (vector load/store + scalar mul)
   west build -p -b apollo510b_evb samples/boards/apollo510b_evb/weaver_benchmark \
       -- -DEXTRA_CONF_FILE=hybrid.conf

   # Full MVE (4-wide multiply)
   west build -p -b apollo510b_evb samples/boards/apollo510b_evb/weaver_benchmark \
       -- -DEXTRA_CONF_FILE=mve.conf

Or use ``scripts/bench.sh`` to build, flash, capture, and tabulate
all four variants in one run.

Reading the output
******************

::

    --- METRICS [weaver-scalar] ---
    M1 imu_overruns=0 ppg_overruns=0 gatt_overruns=2
    M2 fusion_lat_us p50=2000 p99=10000 max=14523 (n=2944)
    M2 hr_lat_us     p50=10000 p99=20000 max=28341 (n=1492)
    M4 last_tick_cyc=181
    WV ticks=60002 promo=312 prewarp=4128 throttle=8 idle_skip=14982
    CSV weaver-scalar,0,0,2,2000,10000,10000,20000,181,312,4128
    BENCHMARK COMPLETE

Comparing the ``CSV`` line across builds tells you whether Weaver
beats stock on the M1/M2/M4 metrics defined in the RFC. The
acceptance criteria are listed in ``RFC.md §7``.

Caveats
*******

* Latency histograms use bucket upper edges - they are pessimistic
  but consistent across runs, which is what matters for comparison.
* The synthetic FIFO model substitutes for hardware sensor FIFOs.
  For absolute (vs. relative) numbers, populate the BMI270 / MAX30101
  Click boards and use the ``weaver_wearable`` sample, which reads
  hardware FIFOs.
* Burst events are deterministic at t=10s and t=20s. Real wearable
  bursts are not periodic; expect higher variance in production.
