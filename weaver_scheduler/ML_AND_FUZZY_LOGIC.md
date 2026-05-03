# Why no fuzzy logic / ML / AI in the scheduler?

Short answer: **the current pressure formula is already a one-layer
linear classifier**, and going further than that costs determinism,
RAM, energy, and certifiability — none of which a wearable can spare
in the scheduler hot path. But there ARE places where lightweight
learning helps, and `weaver_set_weights()` is the hook for them.

## What we already have

```
pressure = w_urgency * priority + w_density * buffer_fill + w_aging * wait_ticks
```

Mathematically that is a perceptron with three inputs, three weights,
no bias, and identity activation. It is the absolute simplest member
of the ML family. We are not avoiding ML — we are using its smallest
useful instance.

What changes when you go bigger:

| Algorithm class | What it adds | What it costs |
|---|---|---|
| Multi-layer NN | Non-linear feature interactions | KB-MB of weights, FPU or quantization, ~µs–ms per inference |
| Fuzzy logic (Mamdani) | Smooth threshold transitions | Membership-function eval per input + defuzzification (centroid integral) per output |
| Fuzzy logic (TSK / Sugeno) | Same but with linear consequents | Cheaper than Mamdani; still 5–20× our current cost |
| RL (Q-learning / bandit) | Adapts weights from observed outcomes | State-space explosion, reward signal needed |
| Decision tree | Cheap inference, interpretable | Training pipeline; brittle to drift |

## Why each one is a poor fit for the scheduler hot path

### 1. Determinism

A real-time scheduler must run in **bounded** time on **every** tick.
Our 1.875 µs at 96 MHz is constant — same cycle count whether the
system is idle or maxed out. An ML inference path with branches,
table lookups, or non-uniform sparsity violates that. WCET (worst-
case execution time) analysis becomes a research problem.

The pre-warp guard window is 1 ms. If a scheduler invocation can
take 200 µs in the bad case, you have already eaten 20% of that
budget before doing anything useful.

### 2. Memory

Apollo510 has 4 MB MRAM and 3 MB SRAM. Every KB the scheduler eats
is a KB you can't spend on:

- BLE Mesh routing tables
- Flash-backed sensor logs
- ML model for HR arrhythmia detection (which actually deserves a
  neural network!)
- Watch-face graphics

A modest TFLM-quantized model (32 KB) is 40× the entire current
scheduler binary footprint.

### 3. Energy per tick

At 1 ms tick we run `weaver_tick()` 86,400,000 times per day.

| Approach | Cycles/tick | µJ/tick @ 96 MHz, 0.4V | Per day |
|---|---|---|---|
| Current linear | ~180 | ~0.0007 | 60 µWh |
| Fuzzy (10 rules) | ~3,000 | ~0.012 | 1 mWh |
| Tiny MLP (3-8-1) | ~5,000 | ~0.020 | 1.7 mWh |

A 1 mWh/day baseline overhead means roughly 1 hour off battery life
on a 250 mAh wearable cell. For *the scheduler itself*. Not the
sensors. Not the ML model that actually does HR or activity detection.

### 4. Certification

If this product touches medical claims (heart-rate detection, atrial
fibrillation, fall detection), an unbounded inference path inside the
scheduler is going to make the regulatory submission much harder. Linear,
inspectable code is easier to defend.

### 5. The ML belongs one layer up

The right place for ML on a wearable is **inside a Weft thread**, not
inside the scheduler. The scheduler's job is to *give your ML model
CPU time when its input buffer is filling*. If the ML model also
controls the scheduler, you have a feedback loop with no ground truth.

## Where lightweight learning DOES belong

These are good candidates for a v2 — none of them put ML in the hot
dispatch path:

### A. Online weight adaptation (recommended first step)

Run a slow-loop controller (1 Hz) outside the scheduler that calls
`weaver_set_weights()` based on observed outcomes:

```c
// Example controller pseudo-code, runs in a 1 Hz workqueue
struct weaver_stats now, prev;
weaver_get_stats(&now);
uint32_t throttles_this_sec = now.throttle_events - prev.throttle_events;

if (throttles_this_sec > 5) {
    /* System overloaded: prioritize by buffer fill more. */
    weaver_set_weights(W_URGENCY * 0.8, W_DENSITY * 1.2, W_AGING * 1.0);
} else if (now.skipped_idle_ticks > 800) {
    /* System mostly idle: spread attention more evenly. */
    weaver_set_weights(W_URGENCY * 1.0, W_DENSITY * 1.0, W_AGING * 1.1);
}
prev = now;
```

This is a 1-D feedback controller. ~30 cycles per second of overhead.
If you want something fancier, swap it for a multi-armed bandit (e.g.
ε-greedy with three arms = three weight presets) and you have an
honest learning algorithm.

### B. Predictive buffer fill (the "Predictive" half of "Predictive
Pressure-Aware")

Right now `buffer_fill_q16` is reactive — pressure goes up when the
buffer fills up. The "predictive" part isn't really there yet. A
lightweight linear extrapolation:

```c
/* Run inside the producer when it updates its FIFO depth. */
uint32_t prev = wd->buffer_fill_q16;
uint32_t now  = current_fifo_depth_q16;
uint32_t predicted = now + (now - prev);  /* one-step linear */
weaver_set_buffer_fill_q16(wd, predicted);
```

3 cycles. Catches a filling FIFO one tick earlier. No ML library
needed.

### C. Fuzzy throttle (instead of binary)

The current `weaver_should_throttle()` is a hard threshold. A TSK
fuzzy controller would return a 0–255 throttle level:

- `LOW` rule: if pressure < 0.3 then 0
- `MED` rule: if 0.3 ≤ pressure < 0.7 then 128
- `HIGH` rule: if pressure ≥ 0.7 then 255

With linear membership functions, three rules cost ~30 cycles. Worth
it if your producers can act on a graded throttle (e.g., "drop sensor
rate by 25%" instead of "drop entirely"). I left this out of v1
because the producers in the sample are bang-bang.

### D. Workload classifier for preset switching

Detect "active" (steps + HR climbing) vs "resting" (no movement) vs
"sleep" states from current pressure history, swap to a different
weight preset for each. Done with three running averages and two
comparators. No NN.

### E. RL for autotuning the throttle threshold

`CONFIG_WEAVER_THROTTLE_THRESHOLD` is hand-picked. It could instead
be a learned parameter that minimizes (FIFO overruns + screen
re-render misses). Single-parameter contextual bandit, 10 lines of
code.

## My recommendation for the wearable

1. **Ship v1 as-is** with the linear pressure model. It's 1.875 µs and
   testable.
2. **Add option B (predictive buffer fill)** as a producer-side
   helper. Three lines, zero scheduler-side cost.
3. **Add option A (1 Hz weight controller)** as a Weft thread once
   you have field data on what overruns happen and when. The hook
   `weaver_set_weights()` is already in place.
4. **Skip C/D/E unless data shows they pay off.** Premature
   sophistication is how scheduler bugs get filed.

## Summary table

| Question | Answer |
|---|---|
| "Are we using ML?" | Yes — a linear perceptron. The simplest one. |
| "Should we use a neural net?" | No, in the scheduler. Yes, in the Weft threads consuming sensor data. |
| "Should we use fuzzy logic?" | Optional for `weaver_should_throttle()`. Not for dispatch. |
| "Can we adapt weights at runtime?" | Yes — `weaver_set_weights()` is now exposed. |
| "What's the next step if we want learning?" | 1 Hz outer-loop controller calling `weaver_set_weights()`. |

The Weaver model is intentionally interpretable. If you ever want to
explain to a regulator (or yourself, six months from now) why a
specific thread got promoted at a specific tick, you can do it with
three multiplications and an addition — not by inspecting 50,000
weights of a black-box model.
