/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver scheduler demo:
 *   - One Warp thread (deterministic, infinite pressure).
 *   - Two Weft threads simulating logging / sensor producers whose
 *     buffer fill ratio drives their dynamic pressure.
 *   - A periodic timer drives weaver_tick() every 10 ms.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/weaver_sched.h>
#include <zephyr/sys/printk.h>

#define WARP_PRIO  4
#define WEFT_PRIO  6
#define TICK_MS    10
#define RUN_TICKS  100

static struct weaver_thread_data warp_wd;
static struct weaver_thread_data weft_a_wd;
static struct weaver_thread_data weft_b_wd;

static K_THREAD_STACK_DEFINE(warp_stack, 1024);
static K_THREAD_STACK_DEFINE(weft_a_stack, 1024);
static K_THREAD_STACK_DEFINE(weft_b_stack, 1024);

static struct k_thread warp_thread;
static struct k_thread weft_a_thread;
static struct k_thread weft_b_thread;

static void warp_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(5));
	}
}

static void weft_fn(void *self, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1); ARG_UNUSED(unused2);
	while (1) {
		k_sleep(K_MSEC(15));
		(void)self;
	}
}

static void tick_timer_handler(struct k_timer *t)
{
	ARG_UNUSED(t);
	weaver_tick();
}

static K_TIMER_DEFINE(weaver_timer, tick_timer_handler, NULL);

int main(void)
{
	k_thread_create(&warp_thread, warp_stack, K_THREAD_STACK_SIZEOF(warp_stack),
			warp_fn, NULL, NULL, NULL, WARP_PRIO, 0, K_NO_WAIT);
	k_thread_create(&weft_a_thread, weft_a_stack, K_THREAD_STACK_SIZEOF(weft_a_stack),
			weft_fn, &weft_a_wd, NULL, NULL, WEFT_PRIO, 0, K_NO_WAIT);
	k_thread_create(&weft_b_thread, weft_b_stack, K_THREAD_STACK_SIZEOF(weft_b_stack),
			weft_fn, &weft_b_wd, NULL, NULL, WEFT_PRIO, 0, K_NO_WAIT);

	weaver_register(&warp_wd,   &warp_thread,   WEAVER_TO_Q16(10), true);
	weaver_register(&weft_a_wd, &weft_a_thread, WEAVER_TO_Q16(2),  false);
	weaver_register(&weft_b_wd, &weft_b_thread, WEAVER_TO_Q16(2),  false);

	k_timer_start(&weaver_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));

	for (int i = 0; i < RUN_TICKS; i++) {
		/* Drive Weft A's buffer fill ratio up over time; B stays low. */
		uint32_t fill_a = (uint32_t)i * (WEAVER_Q16_ONE / RUN_TICKS);
		uint32_t fill_b = WEAVER_Q16_ONE / 8U;

		weaver_set_buffer_fill_q16(&weft_a_wd, fill_a);
		weaver_set_buffer_fill_q16(&weft_b_wd, fill_b);

		if ((i % 10) == 0) {
			printk("tick=%d sys_pressure=0x%08x throttle=%d "
			       "p_a=0x%08x p_b=0x%08x\n",
			       i, weaver_system_pressure(),
			       weaver_should_throttle() ? 1 : 0,
			       weaver_calculate_pressure(&weft_a_wd),
			       weaver_calculate_pressure(&weft_b_wd));
		}

		k_sleep(K_MSEC(TICK_MS));
	}

	k_timer_stop(&weaver_timer);
	weaver_unregister(&warp_wd);
	weaver_unregister(&weft_a_wd);
	weaver_unregister(&weft_b_wd);

	printk("Weaver Test Complete\n");
	return 0;
}
