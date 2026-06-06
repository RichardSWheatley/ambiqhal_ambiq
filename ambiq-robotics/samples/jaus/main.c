/*
 * ARB sample (host): JAUS-driven differential-drive node.
 *
 * A runnable demonstration of the JAUS bridge wired to a (simulated) diff-drive
 * robot. It ties together the OS-agnostic core, the host platform port, and the
 * JAUS-over-UDP transport:
 *
 *   JAUS SetWrenchEffort (UDP) --> bridge --> ARB /cmd_vel --> robot sim
 *   robot sim --> ARB /odom --> bridge --> JAUS Report{VelocityState,LocalPose}
 *
 * The "robot" is a kinematic integrator so the whole pipeline runs on a PC with
 * no hardware. Point a JAUS controller (or the OpenJAUS adapter, or a second
 * instance) at it.
 *
 * Usage: arb_jaus_sample [bind_port] [peer_ip] [peer_port] [run_seconds]
 *        defaults:        3794        127.0.0.1 3795        0 (forever)
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/msg.h"
#include "arb/bridge/jaus.h"
#include "jaus_udp.h"

#define TOPIC_CMD_VEL 1
#define TOPIC_ODOM    2

#define STEP_MS 20

/* latest commanded twist (set by /cmd_vel subscriber, used by the sim) */
static volatile float g_cmd_v, g_cmd_w;

/* integrated pose */
static float g_x, g_y, g_th;
static uint32_t g_seq;

static void on_cmd_vel(arb_topic_id_t topic, const void *msg, size_t len,
		       void *user)
{
	(void)topic; (void)user;
	if (len != sizeof(arb_twist_t)) {
		return;
	}
	const arb_twist_t *t = (const arb_twist_t *)msg;
	g_cmd_v = t->linear.x;
	g_cmd_w = t->angular.z;
}

static void step_sim_and_publish(float dt)
{
	g_th += g_cmd_w * dt;
	g_x  += g_cmd_v * cosf(g_th) * dt;
	g_y  += g_cmd_v * sinf(g_th) * dt;

	arb_odom_t odom;
	arb_header_init(&odom.header, ARB_MSG_ODOM, 0, arb_platform_time_us(),
			g_seq++);
	odom.pose.x = g_x;
	odom.pose.y = g_y;
	odom.pose.theta = g_th;
	odom.linear_vel = g_cmd_v;
	odom.angular_vel = g_cmd_w;
	arb_topic_publish(TOPIC_ODOM, &odom, sizeof(odom));
}

int main(int argc, char **argv)
{
	uint16_t bind_port = (argc > 1) ? (uint16_t)atoi(argv[1]) : 3794;
	const char *peer_ip = (argc > 2) ? argv[2] : "127.0.0.1";
	uint16_t peer_port = (argc > 3) ? (uint16_t)atoi(argv[3]) : 3795;
	int run_seconds = (argc > 4) ? atoi(argv[4]) : 0;

	arb_platform_init();
	arb_topic_init();

	arb_topic_advertise(TOPIC_CMD_VEL, sizeof(arb_twist_t));
	arb_topic_advertise(TOPIC_ODOM, sizeof(arb_odom_t));
	arb_topic_subscribe(TOPIC_CMD_VEL, on_cmd_vel, NULL);

	arb_jaus_udp_t udp;
	if (arb_jaus_udp_open(&udp, bind_port, peer_ip, peer_port) != 0) {
		fprintf(stderr, "failed to open UDP transport on port %u\n",
			bind_port);
		return 1;
	}

	arb_jaus_bridge_t bridge;
	arb_jaus_cfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.self.subsystem = 1;
	cfg.self.node = 1;
	cfg.self.component = 1;
	cfg.send = arb_jaus_udp_send;
	cfg.send_ctx = &udp;
	cfg.cmd_vel_topic = TOPIC_CMD_VEL;
	cfg.odom_topic = TOPIC_ODOM;
	cfg.max_linear = 1.5f;
	cfg.max_angular = 3.0f;
	strncpy(cfg.identification, "arb-diffdrive", sizeof(cfg.identification) - 1);
	if (arb_jaus_init(&bridge, &cfg) != ARB_OK) {
		fprintf(stderr, "bridge init failed\n");
		return 1;
	}

	printf("ARB JAUS diff-drive node: bind %u, peer %s:%u%s\n",
	       bind_port, peer_ip, peer_port,
	       run_seconds ? "" : " (Ctrl-C to stop)");

	const float dt = STEP_MS * 1e-3f;
	long max_iters = run_seconds ? (run_seconds * 1000 / STEP_MS) : -1;
	long iters = 0, printed = 0;

	for (;;) {
		arb_jaus_udp_poll(&udp, &bridge); /* JAUS in -> /cmd_vel */
		arb_platform_dispatch();          /* deliver to subscribers */
		step_sim_and_publish(dt);         /* /odom -> JAUS reports   */
		arb_platform_dispatch();          /* flush odom reports      */

		if (++printed >= (1000 / STEP_MS)) { /* ~1 Hz status */
			printed = 0;
			printf("pose: x=%.2f y=%.2f th=%.2f  cmd: v=%.2f w=%.2f\n",
			       g_x, g_y, g_th, g_cmd_v, g_cmd_w);
		}

		if (max_iters >= 0 && ++iters >= max_iters) {
			break;
		}
		usleep(STEP_MS * 1000);
	}

	arb_jaus_udp_close(&udp);
	return 0;
}
