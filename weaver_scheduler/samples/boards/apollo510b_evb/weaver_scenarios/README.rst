.. zephyr:code-sample:: weaver_scenarios
   :name: Weaver Scenarios (Apollo510B)

   Five isolated behavioral tests for the Weaver predictive
   pressure-aware scheduler. Each scenario exercises one specific
   Weaver feature so its contribution can be measured independently
   against stock Zephyr.

Why five scenarios instead of one big benchmark
***********************************************

The integrated benchmark (``samples/.../weaver_benchmark``) measures
overall improvement but doesn't tell you *which* Weaver feature
delivered which gain. These scenarios isolate the design choices so
the RFC acceptance criteria (M1-M7) can be attributed correctly.

The five scenarios
******************

================  ============================  ======================================  ====================
Scenario          What it tests                 Weaver feature exercised               RFC metric
================  ============================  ======================================  ====================
imu_overrun       IMU FIFO overrun rate         Pressure-aware boost                    M1
ble_deadline      BLE LL deadline integrity     Pre-Warp guard window                   M3 (hard gate)
ppg_latency       PPG -> HR P50/P99/max         Predictive buffer-fill                  M2
burst_shed        Graded throttle response      Fuzzy throttle + EMA hysteresis         M6
display_defer     IMU FIFO under display/NVM    Aging + boost cascade                   M1' (regression)
================  ============================  ======================================  ====================

Running
*******

Each scenario runs ~12-20 s; total wall time ~75 s.

Build once with default ``prj.conf`` (Weaver), once with
``stock.conf`` (no Weaver), capture both logs, diff the SCEN-CSV
lines. ``scripts/run.sh`` automates this.

.. zephyr-app-commands::
   :zephyr-app: samples/boards/apollo510b_evb/weaver_scenarios
   :board: apollo510b_evb
   :goals: build flash

   # Weaver build (default)
   west build -p -b apollo510b_evb samples/boards/apollo510b_evb/weaver_scenarios
   west flash

   # Stock baseline
   west build -p -b apollo510b_evb samples/boards/apollo510b_evb/weaver_scenarios \\
       -- -DEXTRA_CONF_FILE=stock.conf
   west flash

Output format
*************

Each scenario produces a single ``SCEN-CSV`` line with these fields::

    SCEN-CSV variant,name,pass,overruns,misses,p50_us,p99_us,max_us,throttle_peak,misc

Example::

    SCEN-CSV stock,imu_overrun,0,247,0,0,0,0,0,20000
    SCEN-CSV weaver,imu_overrun,1,12,0,0,0,0,0,20000

In this example Weaver reduced IMU overruns from 247 to 12 (M1 PASS,
well below the 0.5x threshold).

RFC acceptance gates encoded
****************************

``scripts/run.sh`` evaluates each gate and prints PASS/FAIL:

- **M1** (scenario 1): ``weaver_overruns <= 0.5 * stock_overruns``
- **M3** (scenario 2, hard gate): ``weaver_misses == 0``
- **M2** (scenario 3): ``weaver_p99 <= 0.7 * stock_p99``
- **M6** (scenario 4): ``weaver_throttle_peak >= 192``
  AND ``throttle_entries == 3``
- **M1'** (scenario 5, regression): ``weaver_peak_fifo <= stock_peak_fifo``
