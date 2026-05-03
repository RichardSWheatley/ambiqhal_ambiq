.. zephyr:code-sample:: weaver_wearable_apollo510b
   :name: Weaver Wearable on Apollo510 Blue EVB

   Drives real wearable sensors via mikroBUS Click boards on the
   Apollo510 Blue EVB at 96 MHz LP mode, scheduled by the Weaver
   pressure-aware fixed-point dispatcher.

Hardware
********

Required:

* **Apollo510 Blue EVB** (``apollo510b_evb``) — has BLE + mikroBUS slot
* **6DOF IMU 14 Click** (BMI270) — wired to mikroBUS slot 1 → IOM0/I2C0
* **Heart Rate 4 Click** (MAX30101) — wired to mikroBUS slot 2 → IOM1/I2C1
  (or move BMI270 to slot 2 if your EVB has only one mikroBUS)

Optional:

* **ap510_disp** display shield (CO5300 AMOLED 468x468 + CHSC5x touch)
* Onboard BLE controller is used by the Zephyr Bluetooth host

Building
********

.. zephyr-app-commands::
   :zephyr-app: samples/boards/apollo510b_evb/weaver_wearable
   :board: apollo510b_evb
   :shield: ap510_disp
   :goals: build flash

Sample Output
*************

.. code-block:: console

   Weaver wearable demo on Apollo510B EVB (LP @ 96 MHz, mikroBUS Click)
   BMI270   ready: yes
   MAX30101 ready: yes
   [t= 500ms] sys_p=0x004210 throttle=0 next_warp=12 ticks=500 promo=18 clear=4 idle=0
   ...
   Weaver Wearable Demo Complete

Predictive scheduling
*********************

Each sensor producer calls
``weaver_set_buffer_fill_predictive_q16()`` instead of the plain
setter. That function performs a one-step linear extrapolation
(``next ~= fill + (fill - last_fill)``) so the dispatcher acts on the
buffer's predicted state next tick rather than its current state.
This is the "Predictive" half of "Predictive and Pressure-Aware" —
implemented in 6 cycles of integer arithmetic instead of an ML
inference path.

Single-mikroBUS-slot variant
****************************

If your EVB only has one mikroBUS slot, populate it with the BMI270
click and edit ``boards/apollo510b_evb.overlay``: remove the ``&iom1``
block. The PPG pipeline becomes inactive but the scheduler still
demonstrates Warp/Weft handoff with IMU traffic.
