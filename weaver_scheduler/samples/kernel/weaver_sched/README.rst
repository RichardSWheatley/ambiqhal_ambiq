.. zephyr:code-sample:: weaver_sched
   :name: Weaver scheduler

   Demonstrates the Weaver predictive pressure-aware scheduling layer.

Overview
********

This sample exercises the Weaver scheduler:

* One **Warp** (hard real-time) thread with infinite pressure.
* Two **Weft** (opportunistic) threads representing data producers.
* A periodic timer drives ``weaver_tick()`` every 10 ms, recomputing
  Q16.16 pressure scores and elevating the highest-pressure Weft
  thread by one Zephyr priority level.

The main loop ramps Weft A's simulated buffer fill ratio from 0 to
1.0 over 100 ticks while Weft B stays low, so you can observe the
dynamic priority handoff and the rising aggregate ``system_pressure``.

Building
********

.. zephyr-app-commands::
   :zephyr-app: samples/kernel/weaver_sched
   :board: qemu_cortex_m3
   :goals: build run

Sample Output
*************

.. code-block:: console

   tick=0  sys_pressure=0x000051fa throttle=0 p_a=0x00028f5c p_b=0x0002a29e
   tick=50 sys_pressure=0x0008c000 throttle=1 p_a=0x00064000 p_b=0x00028000
   ...
   Weaver Test Complete
