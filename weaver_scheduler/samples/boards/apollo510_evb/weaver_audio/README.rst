.. zephyr:code-sample:: weaver_audio
   :name: Weaver Audio (Apollo510 EVB, on-board only)

   Weaver scheduler exercising the on-board hardware of the
   Apollo510 EVB (the non-Blue variant) plus the SoC's built-in
   PDM peripheral driving a DMIC. No external Click board, no
   external shield required.

What it uses
************

================  =================================  ====================
Resource          Where it comes from                What Weaver does
================  =================================  ====================
LED0/LED1/LED2    on the EVB                         visual indicators of
                                                     thread activity
button0/button1   on the EVB                         occasional Warp
                                                     wake events
PDM0              SoC peripheral                     audio sampling Warp
                                                     thread (16 kHz)
================  =================================  ====================

The PDM0 input pins on the apollo510_evb are wired to the on-board
DMIC if one is fitted, OR to an external connector — this sample
treats the audio source as "the click peripheral" regardless of
where the microphone physically sits.

Threads
*******

  - **audio_sample** (Warp, 16 kHz nominal): reads DMIC frames via
    Zephyr's DMIC API; pushes 32-sample chunks into a ring buffer
    and calls ``weaver_set_buffer_fill_predictive_q16``.
  - **vad** (Weft, event-driven): consumes the ring, runs a simple
    sum-of-squares energy detector, drives an output flag.
  - **led_indicator** (Weft, 30 Hz): blinks LED0 with the VAD
    output, LED1 with sample-rate heartbeat, LED2 with throttle
    state.
  - **button_watch** (Warp, poll 100 Hz): records button presses,
    cycles weight presets via ``weaver_set_weights`` so you can
    interact with Weaver live on hardware.
  - **log** (Weft, 1 Hz): mock NVM-flush thread for diversity.

Caveats for this sample
***********************

* The DTS overlay enables ``pdm0`` with a generic configuration.
  If your apollo510_evb wires the DMIC differently, edit
  ``boards/apollo510_evb.overlay`` to match the actual pinmux from
  the board schematic.
* If no DMIC is fitted, the audio_sample thread will produce zero
  data; VAD will idle. LEDs and buttons still work, so Weaver's
  behaviour under "Warp present, Weft mostly idle" is still
  exercised - just not the FIFO-pressure case.
* No "DMIC audio shield" overlay exists in this branch as of this
  write. If Ambiq publishes one later (under
  ``boards/shields/ap5_dmic/`` or similar), this sample can be
  switched to use it via ``-DSHIELD=...``.

Building
********

.. zephyr-app-commands::
   :zephyr-app: samples/boards/apollo510_evb/weaver_audio
   :board: apollo510_evb
   :goals: build flash

Output
******

::

    Weaver Audio demo on Apollo510 EVB
    PDM0 ready: yes (or no - sample still runs, just no audio)
    [t= 500ms] audio_frames=8000 vad_hits=3 led_state=0x4
    [t=1000ms] audio_frames=16000 vad_hits=7 throttle=0 tick_cyc=181
    ...
    Weaver Audio Demo Complete

Press button0 to cycle weight presets:
   default -> 0.5/0.4/0.1 (urgency-heavy)
   -> 0.2/0.7/0.1 (density-heavy) -> 0.3/0.3/0.4 (aging-heavy) -> default
Press button1 to reset stats.
