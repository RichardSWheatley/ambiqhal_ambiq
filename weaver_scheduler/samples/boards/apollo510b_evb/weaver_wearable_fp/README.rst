.. zephyr:code-sample:: weaver_wearable_fp
   :name: Weaver Wearable FP (Apollo510 Blue EVB)

   Floating-point variant of the Weaver wearable demo. Same workload
   as ``weaver_wearable`` but using ``CONFIG_WEAVER_SCHED_FP=y`` (the
   single-precision FPU implementation) instead of the Q16.16
   fixed-point one.

When to pick this sample
************************

Pick this variant when:

* Your Weft threads (sensor fusion, HR FFT, graphics) already use
  the FPU, so the lazy floating-point context-save tax is already
  being paid;
* You want a code path that's easier to audit than the Q16.16 one;
* Apollo510 is in HP mode (192/250 MHz) where FP cost is dwarfed
  by available cycles.

Pick the fixed-point variant (``weaver_wearable``) when:

* Battery-constrained LP-mode (96 MHz) and no other thread uses
  the FPU;
* The MCU has no FPU at all (Cortex-M0, M3);
* Bit-identical reproducibility is required.

See ``FP_VS_FIXED.md`` for the full comparison.

Building
********

.. zephyr-app-commands::
   :zephyr-app: samples/boards/apollo510b_evb/weaver_wearable_fp
   :board: apollo510b_evb
   :shield: ap510_disp
   :goals: build flash
