.. zephyr:code-sample:: weaver_wearable
   :name: Weaver Wearable (Apollo510 LP @ 96 MHz)

   Wearable sensor pipeline demo for the Weaver pressure-aware scheduler
   on Ambiq Apollo510 in 96 MHz Low-Power mode.

Overview
********

Models a fitness/health wearable workload with three Warp threads
(BLE link-layer, IMU, PPG) and six Weft threads (sensor fusion, HR
algorithm, GATT TX, activity classifier, display, NVM flush).

Each tick (1 ms), Weaver computes Q16.16 pressure for every Weft
based on its associated FIFO depth and how long it has waited, then
elevates the highest-pressure Weft by two priority levels. When a
BLE/IMU/PPG deadline is within ``CONFIG_WEAVER_PREWARP_GUARD_TICKS``,
promotion is suppressed so the deadline-bound thread sees a quiet
system (Pattern Slicing).

Building
********

.. zephyr-app-commands::
   :zephyr-app: samples/boards/apollo510_evb/weaver_wearable
   :board: apollo510_evb
   :goals: build flash

Tuning
******

The wearable preset (``CONFIG_WEAVER_WEARABLE_PRESET=y``) sets:

* ``WEAVER_MAX_THREADS=12``
* ``WEAVER_THROTTLE_THRESHOLD=0x30000`` (3.0 in Q16.16)
* ``WEAVER_DEFAULT_BOOST_LEVELS=2``
* ``WEAVER_POWER_AWARE=y``

Override individual settings in ``prj.conf`` if your sensor lineup
differs (more PPG channels, multi-mic audio, etc.).
