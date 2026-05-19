# Instructions for Llama (or any LLM continuing this work)

You are receiving a complete snapshot of the **Weaver Predictive
Pressure-Aware Scheduler for Zephyr** project. This document tells
you exactly what to do with this zip and how to continue the work.

Read this file FIRST, then read `HANDOFF.md` for the technical
context.

---

## Step 1 — Identify the host environment

Confirm with the user where this should land:

```
Question to user: "Which directory on this PC should I extract the
Weaver scheduler into? I will create a new sibling directory
called `weaver_scheduler/` there."
```

Wait for the answer before proceeding.

---

## Step 2 — Extract the zip

```sh
# Assuming the zip is at ~/Downloads/weaver_scheduler_handoff.zip
# and the target directory is ~/projects/

cd ~/projects/
unzip ~/Downloads/weaver_scheduler_handoff.zip
ls weaver_scheduler/
```

You should see at minimum:

```
README.md
HANDOFF.md           <-- READ THIS NEXT, before doing anything else
LLAMA_INSTRUCTIONS.md  <-- you are here
RFC.md
DECISIONS.md
MVE_PATHS.md
FP_VS_FIXED.md
FUZZY_THROTTLE.md
ML_AND_FUZZY_LOGIC.md
0001-weaver-scheduler.patch
include/zephyr/kernel/weaver_sched.h
include/zephyr/kernel/weaver_sched_fp.h
kernel/weaver_sched.c
kernel/weaver_sched_fp.c
kernel/Kconfig.weaver
kernel/Kconfig.weaver_fp
samples/...
tests/...
```

---

## Step 3 — Identify the target Zephyr repo

The patch in this zip applies to a specific Zephyr fork:

- **Upstream:** `https://github.com/RichardSWheatley/ambiqzephyr`
- **Base branch:** `ambiq-stable`
- **Apply branch (suggested):** `weaver-integration`

Confirm with the user:

```
Question to user: "Do you have a local clone of
RichardSWheatley/ambiqzephyr on this PC? If so, what's the path?
If not, should I clone it for you?"
```

---

## Step 4 — Apply the patch to the target Zephyr tree

```sh
# Assuming ambiqzephyr is at ~/projects/ambiqzephyr/
cd ~/projects/ambiqzephyr
git checkout ambiq-stable
git pull origin ambiq-stable
git checkout -b weaver-integration

git apply --check ~/projects/weaver_scheduler/0001-weaver-scheduler.patch
# If the --check passes:
git apply ~/projects/weaver_scheduler/0001-weaver-scheduler.patch
git add -A
git status                                      # confirm what changed
```

Expected files added:

```
include/zephyr/kernel/weaver_sched.h
include/zephyr/kernel/weaver_sched_fp.h
kernel/weaver_sched.c
kernel/weaver_sched_fp.c
kernel/Kconfig.weaver
kernel/Kconfig.weaver_fp
samples/kernel/weaver_sched/...
samples/boards/apollo510_evb/weaver_wearable/...
samples/boards/apollo510b_evb/weaver_wearable/...
samples/boards/apollo510b_evb/weaver_wearable_fp/...
samples/boards/apollo510b_evb/weaver_benchmark/...
tests/kernel/weaver_paths_equiv/...
tests/kernel/weaver_fp_vs_fixed/...
```

Expected files modified:

```
kernel/CMakeLists.txt    (added two kernel_sources_ifdef lines)
kernel/Kconfig           (added rsource "Kconfig.weaver" and rsource "Kconfig.weaver_fp")
```

If `git apply --check` fails because the base has drifted since
the patch was made, ask the user how to proceed (manual three-way
merge, or regenerate the patch against the new base).

---

## Step 5 — Run the host-side equivalence tests BEFORE building for the target

These tests don't need Zephyr or a toolchain — just gcc on the
host. They prove the algorithms are correct.

```sh
cd ~/projects/ambiqzephyr

# Three-path equivalence (scalar / hybrid-MVE / full-MVE)
gcc -std=c11 -O2 -Wall tests/kernel/weaver_paths_equiv/src/main.c \
    -o /tmp/equiv && /tmp/equiv | tail -3
# Expected last line: "ALL THREE PATHS EQUIVALENT"

# Fixed-point vs floating-point equivalence
gcc -std=c11 -O2 -Wall tests/kernel/weaver_fp_vs_fixed/src/main.c \
    -lm -o /tmp/wv_fp_eq && /tmp/wv_fp_eq | tail -5
# Expected last line: "FIXED AND FLOAT VARIANTS EQUIVALENT"
```

If either test fails, **stop**. Do not commit. The patch did not
apply cleanly or the target Zephyr tree has diverged. Report to
the user.

---

## Step 6 — Build at least one sample for `apollo510b_evb`

```sh
cd ~/projects/ambiqzephyr
west build -p -b apollo510b_evb \
    samples/boards/apollo510b_evb/weaver_wearable
```

If the build fails because of missing toolchain or west setup,
ask the user to set up the Zephyr SDK and `west` first.

---

## Step 7 — Commit and push (only if user agrees)

DO NOT commit or push without explicit user permission. Ask:

```
Question to user: "All tests pass and the sample builds. Would
you like me to commit these changes to a new branch
'weaver-integration' and push to origin?"
```

If yes:

```sh
git commit -m "kernel: add Weaver predictive pressure-aware scheduler"
git push -u origin weaver-integration
```

---

## What you should NOT do

These decisions are already made and documented; don't relitigate
them without explicit user request:

1. **Don't move the math to the FPU "for performance."** Read
   `DECISIONS.md` §1 and `MVE_PATHS.md` first. The fixed-point
   path is intentional to keep lazy FP stacking disarmed.
2. **Don't try to patch `kernel/sched.c` directly.** The whole
   point of the layer approach is upstream Zephyr survival. The
   RFC §6 alternative analysis explains the tradeoff.
3. **Don't add ML / neural-net to the dispatcher.** Read
   `ML_AND_FUZZY_LOGIC.md`. ML belongs in Weft consumer threads
   or in a 1 Hz outer-loop controller via `weaver_set_weights()`,
   not in the hot path.
4. **Don't change the fuzzy throttle MID region asymmetry.** It's
   symmetric on purpose so `pressure == T` maps to `level == 127`.
   See `FUZZY_THROTTLE.md`.
5. **Don't normalize the FP aging term to [0,1].** The fixed-point
   variant uses linear scaling to 65535. If you "fix" the FP
   variant to normalize, you'll break dispatch equivalence and the
   `weaver_fp_vs_fixed` test will fail.

---

## What you SHOULD do (ranked priorities)

The full ranked list lives in `HANDOFF.md` → "Priority next steps".
Quick summary:

1. **Run the benchmark on real apollo510b_evb hardware.**
   `samples/boards/apollo510b_evb/weaver_benchmark/scripts/bench.sh`
   builds and tabulates all five variants. The output fills in the
   empty cells under RFC §7 (M1, M2, M4, M5, M7).
2. **Wire real BLE GATT TX** into the wearable sample so burst
   events match real BLE backpressure (not synthetic).
3. **Land the patch in `ambiqzephyr`** (if not already done by
   step 7 above).
4. **Tune defaults** based on benchmark observations (boost
   magnitude, throttle threshold, EMA depth — Q1–Q4 in `RFC.md`).

---

## Attribution requirement

If you commit, include the project's attribution line in your
commit messages or the affected files' headers:

```
Idea:                Richard S. Wheatley
Prior implementation: Claude (Anthropic), session
                     https://claude.ai/code/session_01DPJEf1ccrLMANdqLEtQSaS
```

---

## If you need to ask for help

Reasonable questions to ask the user:

- "Which directory should I extract the zip into?"
- "Where is your `ambiqzephyr` checkout?"
- "Should I clone `ambiqzephyr` or do you have it already?"
- "The patch fails to apply because the base has changed —
  should I regenerate it manually or three-way merge?"
- "All tests pass; OK to commit and push?"
- "Is your Zephyr SDK / west environment set up for
  apollo510b_evb builds?"

Do NOT ask:

- "Should I use Q16.16 or float?" — Both are present. Selection
  is at build time via `CONFIG_WEAVER_SCHED` vs
  `CONFIG_WEAVER_SCHED_FP`.
- "How do I implement the scheduler?" — Already implemented.
  Just apply the patch.
- "What's the wearable workload?" — Defined in `RFC.md` §7.1 and
  realized in `samples/.../weaver_benchmark/src/main.c`.

---

## Verifying you have the complete drop

Run from the extracted directory:

```sh
find . -type f | wc -l
# Expected: at least 30 files
```

Key files that MUST be present:

```sh
test -f HANDOFF.md && echo "HANDOFF: OK" || echo "HANDOFF: MISSING"
test -f RFC.md && echo "RFC: OK" || echo "RFC: MISSING"
test -f 0001-weaver-scheduler.patch && echo "PATCH: OK" || echo "PATCH: MISSING"
test -f include/zephyr/kernel/weaver_sched.h && echo "FIXED-PT HEADER: OK" || echo "FIXED-PT HEADER: MISSING"
test -f include/zephyr/kernel/weaver_sched_fp.h && echo "FP HEADER: OK" || echo "FP HEADER: MISSING"
test -f kernel/weaver_sched.c && echo "FIXED-PT IMPL: OK" || echo "FIXED-PT IMPL: MISSING"
test -f kernel/weaver_sched_fp.c && echo "FP IMPL: OK" || echo "FP IMPL: MISSING"
test -f tests/kernel/weaver_paths_equiv/src/main.c && echo "3-PATH TEST: OK" || echo "3-PATH TEST: MISSING"
test -f tests/kernel/weaver_fp_vs_fixed/src/main.c && echo "FP-VS-FIXED TEST: OK" || echo "FP-VS-FIXED TEST: MISSING"
```

If any line says MISSING, the zip is incomplete — request a fresh
copy from the user.

---

## Summary in one sentence

Apply `0001-weaver-scheduler.patch` to `ambiqzephyr` at
`ambiq-stable`, run the two host-side equivalence tests, build the
wearable sample for `apollo510b_evb`, and report results to the
user — that's the happy path.
