/*
 * Copyright (c) 2026 Ambiq Micro, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Weaver isolation scenarios driver.
 *
 * Runs 5 focused scenarios sequentially. Each isolates one specific
 * Weaver feature so its contribution can be attributed cleanly.
 * Build twice (stock.conf and default) and diff the SCEN-CSV lines
 * to populate the RFC M1-M7 acceptance table.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "scen_common.h"

int main(void)
{
	printk("\n=== Weaver Scenarios [%s] on Apollo510B EVB LP @ 96 MHz ===\n",
	       BUILD_LABEL);
	printk("5 isolation scenarios, ~75 s total\n\n");

	struct scen_result results[5] = {0};

	printk("[scen 1/5] imu_overrun       (20 s) ...\n");
	scen_imu_overrun(&results[0]);
	printk("[scen 2/5] ble_deadline      (15 s) ...\n");
	scen_ble_deadline(&results[1]);
	printk("[scen 3/5] ppg_latency       (12 s) ...\n");
	scen_ppg_latency(&results[2]);
	printk("[scen 4/5] burst_shed        (12 s) ...\n");
	scen_burst_shed(&results[3]);
	printk("[scen 5/5] display_defer     (15 s) ...\n");
	scen_display_defer(&results[4]);

	printk("\n--- SCENARIO RESULTS [%s] ---\n", BUILD_LABEL);
	for (int i = 0; i < 5; i++) {
		struct scen_result *r = &results[i];
		printk("S%d %-15s pass=%d overruns=%-6u misses=%-4u "
		       "p50=%-6u p99=%-6u max=%-6u throttle_peak=%-3u "
		       "%s=%u\n",
		       i + 1, r->name, r->pass,
		       r->overruns, r->deadline_misses,
		       r->latency_p50_us, r->latency_p99_us, r->latency_max_us,
		       r->throttle_peak,
		       r->misc_label ? r->misc_label : "n/a", r->misc);
	}

	/* Machine-readable single line per scenario for diff scripts. */
	printk("\n");
	for (int i = 0; i < 5; i++) {
		struct scen_result *r = &results[i];
		printk("SCEN-CSV %s,%s,%d,%u,%u,%u,%u,%u,%u,%u\n",
		       BUILD_LABEL, r->name, r->pass,
		       r->overruns, r->deadline_misses,
		       r->latency_p50_us, r->latency_p99_us, r->latency_max_us,
		       r->throttle_peak, r->misc);
	}

	printk("\nSCENARIOS COMPLETE\n");
	return 0;
}
