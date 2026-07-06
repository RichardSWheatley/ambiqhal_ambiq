/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Weaver Asymmetric Cognitive Offload: NPU coprocessor scheduling
 *        layer (DRAFT, Phase 3)
 *
 * STATUS: Design sketch. **No implementation exists yet.** This header
 * is shipped now so reviewers can see how the Weaver architecture
 * extends to an on-die NPU (e.g., the Ethos-U55 that pairs with the
 * Cortex-M55 on Apollo510 SoC variants).
 *
 * NOT to be merged into a product build until weaver_sched_npu.c lands.
 * The header is guarded by CONFIG_WEAVER_NPU which does not exist yet
 * either; including it without setting the guard yields an empty
 * translation unit so consumer code can probe for it via #ifdef without
 * breaking.
 *
 * ============================================================
 * ARCHITECTURAL FRAMING: ASYMMETRIC COGNITIVE OFFLOAD
 * ============================================================
 *
 * The primary CPU is a real-time execution engine. It should not spend
 * cycles computing heuristics that another silicon block can compute
 * asynchronously and better. This layer offloads three distinct
 * classes of policy work to the NPU, keeping the CPU's dispatch hot
 * path deterministic and integer-only:
 *
 *   1. POLICY ADJUSTOR   - reshapes Weft scheduling weights based on
 *                          observed workload patterns. Runs at 1-10 Hz
 *                          on the NPU; writes to weaver_set_weights()
 *                          on completion. The CPU dispatcher keeps
 *                          consuming the same Q16.16 pressure formula
 *                          - only the weights inside it change.
 *
 *   2. ARBITER FEEDER    - orders NPU inference jobs (Looms) queued by
 *                          the CPU. This is the classical scheduling
 *                          role the NPU could plausibly host: when the
 *                          NPU is idle, pick the highest-pressure Loom.
 *                          Runs on the CPU integer path (~50 cycles);
 *                          NPU is the target of the dispatch, not the
 *                          decision-maker.
 *
 *   3. ANOMALY DETECTOR  - watches kernel telemetry (cache miss rate
 *                          proxy, context-switch delay, IOM stall
 *                          bits) via a shared no-cache SRAM ring and
 *                          predicts starvation events 10-100 ms ahead.
 *                          Emits a graded warning that producers
 *                          consume before pressure would spike.
 *
 * All three run on the NPU asynchronously. The CPU-side dispatcher
 * (Phase 1-2 code) is unchanged: still Q16.16 fixed-point, still
 * integer-only, still deterministic sub-microsecond decision. What
 * changes is the QUALITY of the inputs to that dispatcher: better
 * weights, better arbitration, earlier warning.
 *
 * ============================================================
 * WHAT STAYS ON THE CPU (deliberate boundary)
 * ============================================================
 *
 * The per-tick dispatch decision itself STAYS ON THE CPU integer ALU.
 * NPU inference latency is 5-50 ms; a 1 ms scheduler tick cannot wait.
 * The rule: NPU handles POLICY (slow, learnable), CPU handles DISPATCH
 * (fast, deterministic).
 *
 * This split is what makes the layer asymmetric: two different silicon
 * blocks handling two different classes of decision, coupled through
 * a shared-memory mailbox rather than through a synchronous call path.
 *
 * ============================================================
 * WHY NOT USE THE USER-SPACE NPU DRIVER
 * ============================================================
 *
 * Vendor NPU stacks (Arm Ethos-U, AMD XDNA, Intel NPU) are designed
 * to expose the accelerator UPWARD to user-space AI runtimes (TFLite,
 * ONNX Runtime, DirectML). That path is unsuitable here for three
 * reasons:
 *
 *   - Kernel-latency budget: user-space round-trip adds tens of
 *     microseconds; policy updates need to complete before the next
 *     dispatch tick observes them.
 *   - Priority inversion risk: a user-space policy engine gets
 *     preempted by other user work; the NPU sits idle.
 *   - Ownership: a shared NPU means the scheduler competes with user
 *     apps for inference slots.
 *
 * The Weaver Loom driver claims **exclusive kernel-space ownership**
 * of a partition (or full instance) of the NPU. This is what the
 * asymmetric offload architecture requires and it is the concrete
 * hardware-software co-design claim distinguishing this work from
 * higher-level "AI-native scheduling" software wrappers.
 *
 * The Loom class represents one NPU graph (e.g. KWS, activity
 * classifier, AFib detector). A wearable typically has 3-6 Looms
 * competing for one NPU. Weaver decides:
 *
 *   1. When the NPU goes idle, which ready Loom to dispatch next.
 *   2. Whether dispatching now respects the pre-Warp guard window so
 *      a 25 ms inference doesn't stall a 2 ms audio sampling deadline.
 *   3. Whether the system's energy budget allows the inference at all,
 *      or whether the Loom should be downgraded / deferred.
 *
 * The dispatcher math is identical to the existing Weft tournament:
 * Q16.16 fixed-point, saturating add, no FPU/MVE in the hot path. The
 * only new state is per-Loom (input/output fill, latency EMA, energy
 * cost estimate) and a tiny arbiter that runs after the Weft tournament
 * inside weaver_tick().
 *
 * ============================================================
 * METAPHOR
 * ============================================================
 *
 * The "Loom" is the machine that does the actual weaving; the Warp
 * (fixed structure) and Weft (threads moving through) just feed it.
 * Naming the NPU layer after the loom keeps the metaphor consistent:
 * threads supply tensors, the loom does the inference, the fabric
 * (output stream) leaves the other side.
 */

#ifndef ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_NPU_H_
#define ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_NPU_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/weaver_sched.h>  /* WEAVER_Q16_* macros */

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------
 * Q16.16 weights for Loom pressure calculation.
 *
 * Different default apportionment from the Weft weights because Loom
 * priorities are different: aging matters MORE (a 1-second-old NPU
 * result is useless), input-fill matters somewhat less (a partly-full
 * tensor can wait for completion), and there's no "urgency" in the
 * Weft sense (Loom priority is captured in the static priority field).
 *
 * Weights roughly: input 0.35, backlog 0.25, aging 0.4.
 * ----------------------------------------------------------------- */
#define WEAVER_LOOM_W_INPUT    0x00005999U   /* ~0.35 in Q16.16 */
#define WEAVER_LOOM_W_BACKLOG  0x00004000U   /* ~0.25 in Q16.16 */
#define WEAVER_LOOM_W_AGING    0x00006666U   /* ~0.40 in Q16.16 */

/**
 * @brief Per-NPU-graph scheduling state.
 *
 * The owner allocates this struct (typically file-static in the
 * driver wrapping the graph) and registers it. Loom data is updated
 * by producers via weaver_loom_push_input() and by the NPU completion
 * ISR via weaver_loom_complete().
 *
 * Size: ~48 bytes on Cortex-M55. Bounded by CONFIG_WEAVER_MAX_LOOMS.
 */
struct weaver_loom_data {
	void    *npu_graph;            /**< Opaque graph handle (TFLM
					    interpreter, Ethos-U command
					    stream, etc.) */
	uint32_t input_fill_q16;       /**< Input tensor queue fill 0..ONE */
	uint32_t output_backlog_q16;   /**< Unconsumed result fill 0..ONE */
	uint32_t wait_ticks;           /**< Ticks since last dispatch */
	uint32_t avg_inference_us;     /**< EMA of measured latencies */
	uint32_t expected_energy_uj;   /**< Per-inference energy estimate */
	uint32_t pressure_q16;         /**< Computed each tick */
	uint8_t  priority;             /**< Static rank within Loom set
					    (lower = higher priority, mirrors
					    Zephyr thread priorities) */
	uint8_t  in_use;               /**< Slot occupancy */
	uint8_t  dispatching;          /**< 1 while NPU is running this
					    Loom; cleared by complete() */
	uint8_t  reserved;
};

/**
 * @brief System-wide NPU + energy budget tracker.
 *
 * Maintained internally by the Loom layer. Exposed read-only via
 * weaver_loom_get_budget() so producers (or a higher-level controller)
 * can decide whether to even bother queuing more work.
 */
struct weaver_loom_budget {
	uint32_t epoch_ms;             /**< Budget refresh window */
	int32_t  remaining_uj;         /**< Energy left this epoch (can go
					    slightly negative under bursts) */
	uint32_t inferences_in_epoch;  /**< Count, informational */
	uint32_t deferrals_in_epoch;   /**< Count of pressure-meets-budget
					    deferrals - useful for tuning */
};

/* -----------------------------------------------------------------
 * Registration / lifecycle
 * ----------------------------------------------------------------- */

/**
 * @brief Register an NPU graph with the Loom dispatcher.
 *
 * @param ld                Caller-owned, zero-initialized slot.
 * @param npu_graph         Opaque graph handle (driver-defined).
 * @param expected_latency_us Initial latency estimate; updated by EMA
 *                            after the first weaver_loom_complete().
 * @param expected_energy_uj  Initial per-inference energy estimate (uJ).
 * @param priority           Static rank, lower = higher.
 *
 * @retval 0       success
 * @retval -ENOMEM Loom registry full (CONFIG_WEAVER_MAX_LOOMS)
 * @retval -EINVAL bad arguments
 */
int weaver_loom_register(struct weaver_loom_data *ld,
			 void *npu_graph,
			 uint32_t expected_latency_us,
			 uint32_t expected_energy_uj,
			 uint8_t  priority);

/** @brief Remove a Loom from the registry. */
int weaver_loom_unregister(struct weaver_loom_data *ld);

/* -----------------------------------------------------------------
 * Producer-side updates
 * ----------------------------------------------------------------- */

/** @brief Update input tensor queue fill (e.g. audio ring depth). */
void weaver_loom_push_input(struct weaver_loom_data *ld, uint32_t fill_q16);

/** @brief Mark one result as consumed downstream. */
void weaver_loom_consume_output(struct weaver_loom_data *ld);

/* -----------------------------------------------------------------
 * Dispatch + arbitration
 * ----------------------------------------------------------------- */

/**
 * @brief Pure function: compute Q16.16 pressure for a single Loom.
 *
 * Pressure = input_fill_q16 * W_INPUT
 *          + output_backlog_q16 * W_BACKLOG
 *          + TO_Q16(wait_ticks) * W_AGING
 *
 * Same algebraic structure as the Weft pressure formula. No FPU.
 */
uint32_t weaver_loom_calculate_pressure(const struct weaver_loom_data *ld);

/**
 * @brief Pick the highest-pressure ready Loom and dispatch it.
 *
 * Called from inside weaver_tick() AFTER the Weft tournament has
 * applied priority elevations. Steps:
 *   1. If NPU is busy, return NULL (next tick will retry).
 *   2. Scan registry; find Loom with max pressure AND non-empty input.
 *   3. Check pre-Warp guard: if any Warp deadline is within
 *      avg_inference_us of now, defer (return NULL).
 *   4. Check energy budget: if remaining_uj < expected_energy_uj,
 *      defer and increment deferrals_in_epoch.
 *   5. Dispatch to the NPU driver (driver-specific call).
 *   6. Mark Loom as dispatching, reset wait_ticks.
 *
 * @return The dispatched Loom, or NULL if nothing should run this tick.
 */
struct weaver_loom_data *weaver_loom_arbitrate(void);

/**
 * @brief Completion notification from the NPU IRQ handler.
 *
 * @param ld           Loom whose inference just finished.
 * @param actual_us    Measured inference latency in microseconds.
 *                     Folded into avg_inference_us as a 1/4 EMA.
 *
 * Called from ISR context; safe to use from any priority.
 */
void weaver_loom_complete(struct weaver_loom_data *ld, uint32_t actual_us);

/* -----------------------------------------------------------------
 * Throttle and budget queries
 * ----------------------------------------------------------------- */

/**
 * @brief Graded NPU-dispatch throttle level (0..255).
 *
 * Distinct from weaver_throttle_level() which gates CPU producer
 * back-off. When this returns >= 128, NPU producers should slow down
 * (e.g., process every Nth audio frame instead of every frame).
 *
 * Same TSK fuzzy + EMA structure as the existing throttle, fed by:
 * sum(input_fill_q16 + output_backlog_q16) across all Looms vs.
 * CONFIG_WEAVER_LOOM_THROTTLE_THRESHOLD.
 */
uint8_t weaver_loom_throttle_level(void);

/** @brief Read-only snapshot of the current epoch's energy budget. */
void weaver_loom_get_budget(struct weaver_loom_budget *out);

/**
 * @brief Recharge the energy budget for a new epoch.
 *
 * Called by the system PM layer (or a periodic timer) to refresh
 * remaining_uj at the start of each budget epoch. Typical: 1 Hz with
 * an epoch_uj proportional to the wearable's mW power envelope.
 */
void weaver_loom_refresh_budget(int32_t epoch_uj);

/* -----------------------------------------------------------------
 * Compile-time interface contract (extends Weft contract)
 * ----------------------------------------------------------------- */

/**
 * @brief NPU driver hook the Loom dispatcher calls to start a job.
 *
 * Provided by the platform NPU driver. The Loom layer is driver-
 * agnostic; this indirection keeps it portable across Ethos-U,
 * Cadence Tensilica HiFi, and custom Ambiq accelerators.
 */
struct weaver_loom_driver_ops {
	int (*dispatch)(void *npu_graph);
	int (*abort)(void);
	uint32_t (*get_remaining_us)(void);
};

/** @brief Register the platform NPU driver. Must be called once at boot. */
int weaver_loom_set_driver_ops(const struct weaver_loom_driver_ops *ops);

/* =================================================================
 * ROLE 1: POLICY ADJUSTOR
 *
 * The NPU periodically consumes telemetry, runs a quantized policy
 * model (linear bandit / lightweight RL), and emits new weights for
 * the CPU dispatcher's Q16.16 pressure formula. The CPU keeps using
 * the same formula; only the weights inside it change.
 *
 * Runs at 1-10 Hz. Latency of the update is ~milliseconds; the CPU
 * dispatcher is never blocked waiting for it.
 * ================================================================= */

/**
 * @brief Signature for a policy-adjustor callback the NPU invokes
 *        after each inference cycle completes.
 *
 * The NPU driver calls this from ISR context with the three new
 * Q16.16 weights it just inferred. The implementation typically
 * forwards to weaver_set_weights().
 */
typedef void (*weaver_policy_apply_fn)(uint32_t w_urgency_q16,
				       uint32_t w_density_q16,
				       uint32_t w_aging_q16);

/**
 * @brief Register the policy graph and its apply callback.
 *
 * @param graph      Opaque NPU graph handle for the policy model.
 * @param period_ms  Target inference cadence (typical: 100-1000 ms).
 * @param apply      Called from NPU IRQ context with new weights.
 */
int weaver_policy_register(void *graph, uint32_t period_ms,
			   weaver_policy_apply_fn apply);

/* =================================================================
 * ROLE 2: TELEMETRY CAPTURE
 *
 * Kernel-side telemetry the NPU consumes via a shared no-cache
 * SRAM ring. Populated by the dispatcher and low-level kernel hooks;
 * never read by CPU code, only written and then produced to NPU.
 *
 * The metrics are chosen so that measuring them is nearly free on
 * Cortex-M55: DWT gives cycle count and cache miss surrogate; the
 * scheduler already tracks context-switch delta; IOM stall bits are
 * peripheral MMIO reads.
 * ================================================================= */

struct weaver_telemetry_frame {
	uint64_t timestamp_cycles;
	uint32_t dispatcher_cycles;      /**< From weaver_get_last_tick_cycles() */
	uint32_t context_switch_delta;   /**< Kernel-tracked */
	uint32_t cache_miss_surrogate;   /**< From DWT LSU miss counter */
	uint32_t iom_stall_bits;         /**< MMIO snapshot */
	uint32_t weft_promotions;
	uint32_t throttle_level;
	uint32_t loom_dispatched_this_epoch;
	uint32_t reserved[8];
};

/**
 * @brief Bind a shared-memory ring the NPU reads asynchronously.
 *
 * @param ring         Caller-provided ring memory (SRAM_NO_CACHE region).
 * @param n_frames     Ring depth. Typical: 32 frames.
 *
 * The CPU dispatcher writes into this ring at the end of every
 * weaver_tick(); the NPU reads a batch (~10-100 frames) per
 * inference cycle.
 */
int weaver_telemetry_bind_ring(struct weaver_telemetry_frame *ring,
			       size_t n_frames);

/* =================================================================
 * ROLE 3: ANOMALY DETECTOR
 *
 * The NPU inferences over recent telemetry frames and predicts
 * upcoming starvation (thread that will miss its next window) or
 * imminent NPU energy-budget exhaustion. The prediction is
 * consumed by producers as an early warning, letting them
 * downgrade or defer BEFORE the throttle threshold trips.
 * ================================================================= */

/**
 * @brief Graded starvation-warning level (0..255) from the NPU
 *        anomaly detector.
 *
 * 0 = no predicted starvation in the next window.
 * 128 = starvation likely within the next ~50 ms.
 * 255 = starvation predicted imminently (< 10 ms).
 *
 * Producers should consume this ALONGSIDE the existing CPU-side
 * fuzzy throttle (weaver_throttle_level). The two are complementary:
 * the CPU throttle is reactive; this one is predictive.
 */
uint8_t weaver_anomaly_level(void);

/**
 * @brief Register the anomaly-detector graph.
 *
 * @param graph      Opaque NPU graph handle.
 * @param period_ms  Inference cadence (typical: 20-100 ms).
 */
int weaver_anomaly_register(void *graph, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KERNEL_WEAVER_SCHED_NPU_H_ */
