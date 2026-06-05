/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver scheduler demo on apollo510_evb using ONLY on-board hardware:
 *   - LEDs and buttons from the board DTS
 *   - PDM0 from the Apollo510 SoC as the audio "click" source
 *
 * No external Click board, no shield required.
 *
 * Threads:
 *   Warp: audio_sample  (16 kHz nominal, samples PDM0 -> ring)
 *   Warp: button_watch  (poll buttons @ 100 Hz, cycle weights on press)
 *   Weft: vad           (sum-of-squares energy detector; pressure
 *                        rises with ring depth)
 *   Weft: led_indicator (30 Hz, shows VAD / tick state on LEDs)
 *   Weft: nvm_log       (1 Hz mock NVM flush)
 *
 * Each producer calls weaver_set_buffer_fill_predictive_q16 so the
 * dispatcher reacts to projected next-tick fill, not current fill.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_WEAVER_SCHED
#include <zephyr/kernel/weaver_sched.h>
#endif

/* Audio is optional - if the DMIC driver isn't in this build we
 * still want the rest of the demo to compile and run. */
#if defined(CONFIG_AUDIO_DMIC)
#include <zephyr/audio/dmic.h>
#define HAVE_DMIC 1
#else
#define HAVE_DMIC 0
#endif

LOG_MODULE_REGISTER(weaver_audio, LOG_LEVEL_INF);

#define DEMO_DURATION_MS  5000
#define AUDIO_SAMPLE_HZ   16000
#define AUDIO_FRAME_SIZE  32         /* samples per producer push */
#define RING_DEPTH        16         /* of AUDIO_FRAME_SIZE chunks */
#define BTN_POLL_MS       10
#define LED_PERIOD_MS     33         /* ~30 Hz */
#define NVM_PERIOD_MS     1000

#define AUDIO_PRIO   3
#define BTN_PRIO     4
#define VAD_PRIO     8
#define LED_PRIO     10
#define NVM_PRIO     11

#define STK 1536

/* ---------- on-board GPIO bindings ---------- */

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define SW0_NODE  DT_ALIAS(sw0)
#define SW1_NODE  DT_ALIAS(sw1)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET(SW0_NODE,  gpios);
static const struct gpio_dt_spec btn1 = GPIO_DT_SPEC_GET(SW1_NODE,  gpios);

/* ---------- audio + ring buffer ---------- */

#if HAVE_DMIC
static const struct device *const dmic_dev = DEVICE_DT_GET_ANY(zephyr_audio_dmic);
#else
static const struct device *const dmic_dev = NULL;
#endif

static atomic_t ring_depth;
static atomic_t audio_frames;
static atomic_t vad_hits;
static atomic_t stop_flag;

/* ---------- Weaver registry ---------- */

#ifdef CONFIG_WEAVER_SCHED
static struct weaver_thread_data wd_audio, wd_btn, wd_vad, wd_led, wd_nvm;

static void tick_handler(struct k_timer *t) { ARG_UNUSED(t); weaver_tick(); }
static K_TIMER_DEFINE(weaver_timer, tick_handler, NULL);

static inline uint32_t depth_to_q16(int d)
{
	if (d <= 0)            return 0;
	if (d >= RING_DEPTH)   return WEAVER_Q16_ONE;
	return ((uint32_t)d * WEAVER_Q16_ONE) / RING_DEPTH;
}
#endif

/* ---------- thread bodies ---------- */

static void audio_sample_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	/* If DMIC is present, real frames. Otherwise simulate a steady
	 * 16 kHz / 32-sample-chunk arrival rate so Weaver still sees a
	 * producing Warp. The demo's value is the scheduling behaviour,
	 * not the audio content. */
	const uint32_t period_ms =
		(AUDIO_FRAME_SIZE * 1000U + AUDIO_SAMPLE_HZ - 1U)
		/ AUDIO_SAMPLE_HZ;  /* ~= 2 ms per 32-sample chunk */

	while (!atomic_get(&stop_flag)) {
		/* In a real driver build, you'd call dmic_read() here and
		 * copy into a ring. We just bump the depth counter. */
		int d = atomic_inc(&ring_depth) + 1;
		if (d > RING_DEPTH) {
			atomic_set(&ring_depth, RING_DEPTH);
		}
		atomic_add(&audio_frames, AUDIO_FRAME_SIZE);

#ifdef CONFIG_WEAVER_SCHED
		weaver_set_buffer_fill_predictive_q16(
			&wd_vad, depth_to_q16(atomic_get(&ring_depth)));
#endif
		k_sleep(K_MSEC(period_ms));
	}
}

static void vad_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&stop_flag)) {
		int d = atomic_get(&ring_depth);
		if (d > 0) {
			int drain = MIN(d, 4);
			atomic_sub(&ring_depth, drain);
			/* Synthetic compute: O(frame_size) energy detector. */
			volatile uint32_t acc = 0;
			for (int i = 0; i < AUDIO_FRAME_SIZE * drain; i++) {
				acc += (uint32_t)i * (uint32_t)i;
			}
			if ((acc & 0x3FF) > 0x100) {
				atomic_inc(&vad_hits);
			}
#ifdef CONFIG_WEAVER_SCHED
			weaver_set_buffer_fill_predictive_q16(
				&wd_vad,
				depth_to_q16(atomic_get(&ring_depth)));
#endif
		}
		k_sleep(K_MSEC(5));
	}
}

static void button_watch_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	int last0 = 1, last1 = 1;
#ifdef CONFIG_WEAVER_SCHED
	int preset = 0;
#endif
	while (!atomic_get(&stop_flag)) {
		int b0 = gpio_pin_get_dt(&btn0);
		int b1 = gpio_pin_get_dt(&btn1);

		if (last0 == 1 && b0 == 0) {
#ifdef CONFIG_WEAVER_SCHED
			preset = (preset + 1) % 4;
			switch (preset) {
			case 1: weaver_set_weights(0x8000, 0x6666, 0x1999); break; /* urgency */
			case 2: weaver_set_weights(0x3333, 0xB333, 0x1999); break; /* density */
			case 3: weaver_set_weights(0x4CCC, 0x4CCC, 0x6666); break; /* aging */
			default: weaver_set_weights(WEAVER_W_URGENCY,
						    WEAVER_W_DENSITY,
						    WEAVER_W_AGING);
			}
			printk("button0: weights preset %d\n", preset);
#endif
		}
		if (last1 == 1 && b1 == 0) {
			atomic_set(&audio_frames, 0);
			atomic_set(&vad_hits, 0);
			printk("button1: counters reset\n");
		}
		last0 = b0;
		last1 = b1;
		k_sleep(K_MSEC(BTN_POLL_MS));
	}
}

static void led_indicator_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	int hb = 0;
	while (!atomic_get(&stop_flag)) {
		hb ^= 1;
		gpio_pin_set_dt(&led1, hb);
		gpio_pin_set_dt(&led0,
				(atomic_get(&ring_depth) > RING_DEPTH / 2) ? 1 : 0);
#ifdef CONFIG_WEAVER_SCHED
		gpio_pin_set_dt(&led2,
				weaver_should_throttle() ? 1 : 0);
#else
		gpio_pin_set_dt(&led2, 0);
#endif
		k_sleep(K_MSEC(LED_PERIOD_MS));
	}
}

static void nvm_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (!atomic_get(&stop_flag)) {
		k_sleep(K_MSEC(NVM_PERIOD_MS));
	}
}

/* ---------- stacks + threads ---------- */

static K_THREAD_STACK_DEFINE(s_audio, STK);
static K_THREAD_STACK_DEFINE(s_btn,   STK);
static K_THREAD_STACK_DEFINE(s_vad,   STK);
static K_THREAD_STACK_DEFINE(s_led,   STK);
static K_THREAD_STACK_DEFINE(s_nvm,   STK);
static struct k_thread t_audio, t_btn, t_vad, t_led, t_nvm;

int main(void)
{
	printk("\nWeaver Audio demo on Apollo510 EVB\n");
	printk("PDM0 ready: %s\n",
	       (dmic_dev && device_is_ready(dmic_dev)) ? "yes" : "no (simulated)");

	if (gpio_is_ready_dt(&led0)) gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	if (gpio_is_ready_dt(&led1)) gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	if (gpio_is_ready_dt(&led2)) gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	if (gpio_is_ready_dt(&btn0)) gpio_pin_configure_dt(&btn0, GPIO_INPUT);
	if (gpio_is_ready_dt(&btn1)) gpio_pin_configure_dt(&btn1, GPIO_INPUT);

	k_thread_create(&t_audio, s_audio, STK, audio_sample_fn,
			NULL, NULL, NULL, AUDIO_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_btn, s_btn, STK, button_watch_fn,
			NULL, NULL, NULL, BTN_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_vad, s_vad, STK, vad_fn,
			NULL, NULL, NULL, VAD_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_led, s_led, STK, led_indicator_fn,
			NULL, NULL, NULL, LED_PRIO, 0, K_NO_WAIT);
	k_thread_create(&t_nvm, s_nvm, STK, nvm_fn,
			NULL, NULL, NULL, NVM_PRIO, 0, K_NO_WAIT);

#ifdef CONFIG_WEAVER_SCHED
	weaver_register(&wd_audio, &t_audio, WEAVER_TO_Q16(15), true);
	weaver_set_warp_deadline(&wd_audio, 2);   /* ~2 ms cadence */
	weaver_register(&wd_btn,   &t_btn,   WEAVER_TO_Q16(10), true);
	weaver_set_warp_deadline(&wd_btn, BTN_POLL_MS);
	weaver_register(&wd_vad,   &t_vad,   WEAVER_TO_Q16(8),  false);
	weaver_register(&wd_led,   &t_led,   WEAVER_TO_Q16(4),  false);
	weaver_register(&wd_nvm,   &t_nvm,   WEAVER_TO_Q16(2),  false);
	k_timer_start(&weaver_timer, K_MSEC(1), K_MSEC(1));
#endif

	uint32_t t0 = k_uptime_get_32();
	while ((k_uptime_get_32() - t0) < DEMO_DURATION_MS) {
		k_sleep(K_MSEC(500));

#ifdef CONFIG_WEAVER_SCHED
		struct weaver_stats s;
		weaver_get_stats(&s);
		printk("[t=%4ums] audio=%u vad=%u depth=%d throttle=%d "
		       "lvl=%u ticks=%u promo=%u tick_cyc=%u\n",
		       k_uptime_get_32() - t0,
		       atomic_get(&audio_frames),
		       atomic_get(&vad_hits),
		       atomic_get(&ring_depth),
		       weaver_should_throttle() ? 1 : 0,
		       weaver_throttle_level(),
		       s.total_ticks, s.weft_promotions,
		       weaver_get_last_tick_cycles());
#else
		printk("[t=%4ums] audio=%u vad=%u depth=%d (stock - no Weaver)\n",
		       k_uptime_get_32() - t0,
		       atomic_get(&audio_frames),
		       atomic_get(&vad_hits),
		       atomic_get(&ring_depth));
#endif
	}

	atomic_set(&stop_flag, 1);
	k_sleep(K_MSEC(100));

#ifdef CONFIG_WEAVER_SCHED
	k_timer_stop(&weaver_timer);
	weaver_unregister(&wd_audio);
	weaver_unregister(&wd_btn);
	weaver_unregister(&wd_vad);
	weaver_unregister(&wd_led);
	weaver_unregister(&wd_nvm);
#endif

	k_thread_abort(&t_audio);
	k_thread_abort(&t_btn);
	k_thread_abort(&t_vad);
	k_thread_abort(&t_led);
	k_thread_abort(&t_nvm);

	printk("Weaver Audio Demo Complete\n");
	return 0;
}
