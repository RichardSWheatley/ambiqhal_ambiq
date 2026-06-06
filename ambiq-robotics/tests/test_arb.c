/*
 * ARB core unit tests (host).
 *
 * Exercises the OS-agnostic core against the host platform port:
 *   - pub/sub delivery, multiple subscribers, deferred dispatch
 *   - services (req/resp, sequencing, not-found)
 *   - HAL logic: motor scaling/inversion/deadband, encoder decode, IMU remap
 *   - PID: clamping, anti-windup, convergence
 *
 * Build via tests/CMakeLists.txt (ctest) or directly with gcc; see tests/README.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "arb/platform.h"
#include "arb/topic.h"
#include "arb/service.h"
#include "arb/msg.h"
#include "arb/hal/motor.h"
#include "arb/hal/encoder.h"
#include "arb/hal/imu.h"
#include "arb/control/pid.h"

static int g_failed;
static int g_checks;

#define CHECK(cond)                                                          \
	do {                                                                 \
		g_checks++;                                                  \
		if (!(cond)) {                                               \
			g_failed++;                                          \
			printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__,     \
			       #cond);                                       \
		}                                                           \
	} while (0)

#define CHECK_NEAR(a, b, eps) CHECK(fabsf((float)(a) - (float)(b)) <= (eps))

/* ---- pub/sub ---------------------------------------------------------- */

#define TOPIC_A 100
#define TOPIC_B 101

static int s_a_count, s_b1_count, s_b2_count;
static arb_twist_t s_last_a;

static void on_a(arb_topic_id_t t, const void *m, size_t l, void *u)
{
	(void)t; (void)u;
	CHECK(l == sizeof(arb_twist_t));
	s_last_a = *(const arb_twist_t *)m;
	s_a_count++;
}
static void on_b1(arb_topic_id_t t, const void *m, size_t l, void *u)
{
	(void)t; (void)m; (void)l; (void)u; s_b1_count++;
}
static void on_b2(arb_topic_id_t t, const void *m, size_t l, void *u)
{
	(void)t; (void)m; (void)l; (void)u; s_b2_count++;
}

static void test_pubsub(void)
{
	printf("test_pubsub\n");
	arb_topic_init();
	s_a_count = s_b1_count = s_b2_count = 0;

	CHECK(arb_topic_advertise(TOPIC_A, sizeof(arb_twist_t)) == ARB_OK);
	CHECK(arb_topic_advertise(TOPIC_B, sizeof(arb_twist_t)) == ARB_OK);
	/* re-advertise same size = ok, different size = error */
	CHECK(arb_topic_advertise(TOPIC_A, sizeof(arb_twist_t)) == ARB_OK);
	CHECK(arb_topic_advertise(TOPIC_A, 8) == ARB_ERR_INVAL);

	CHECK(arb_topic_subscribe(TOPIC_A, on_a, NULL) == ARB_OK);
	CHECK(arb_topic_subscribe(TOPIC_B, on_b1, NULL) == ARB_OK);
	CHECK(arb_topic_subscribe(TOPIC_B, on_b2, NULL) == ARB_OK);

	arb_twist_t tw;
	memset(&tw, 0, sizeof(tw));
	arb_header_init(&tw.header, ARB_MSG_TWIST, 0, arb_platform_time_us(), 0);
	tw.linear.x = 2.5f;

	/* wrong length rejected */
	CHECK(arb_topic_publish(TOPIC_A, &tw, 4) == ARB_ERR_TOOBIG);
	/* unknown topic rejected */
	CHECK(arb_topic_publish(999, &tw, sizeof(tw)) == ARB_ERR_NOTFOUND);

	CHECK(arb_topic_publish(TOPIC_A, &tw, sizeof(tw)) == ARB_OK);
	/* deferred: nothing delivered until dispatch */
	CHECK(s_a_count == 0);
	arb_platform_dispatch();
	CHECK(s_a_count == 1);
	CHECK_NEAR(s_last_a.linear.x, 2.5f, 1e-6f);

	/* two subscribers both fire on TOPIC_B */
	CHECK(arb_topic_publish(TOPIC_B, &tw, sizeof(tw)) == ARB_OK);
	arb_platform_dispatch();
	CHECK(s_b1_count == 1);
	CHECK(s_b2_count == 1);
}

/* ---- services --------------------------------------------------------- */

#define SVC_DOUBLE 7

static uint32_t s_last_seq;

static int svc_double(uint32_t seq, const void *req, size_t rl, void *resp,
		      size_t cap, size_t *ro, void *user)
{
	(void)user; (void)rl; (void)cap;
	s_last_seq = seq;
	int v = *(const int *)req;
	*(int *)resp = v * 2;
	*ro = sizeof(int);
	return 0;
}

static void test_service(void)
{
	printf("test_service\n");
	arb_service_init();

	CHECK(arb_service_register(SVC_DOUBLE, svc_double, NULL) == ARB_OK);

	int in = 21, out = 0;
	size_t ol = 0;
	CHECK(arb_service_call(SVC_DOUBLE, &in, sizeof(in), &out, sizeof(out),
			       &ol) == 0);
	CHECK(out == 42);
	CHECK(ol == sizeof(int));
	uint32_t first = s_last_seq;

	/* sequence increments across calls */
	(void)arb_service_call(SVC_DOUBLE, &in, sizeof(in), &out, sizeof(out),
			       &ol);
	CHECK(s_last_seq == first + 1);

	/* unknown service */
	CHECK(arb_service_call(4242, &in, sizeof(in), &out, sizeof(out), &ol)
	      == ARB_ERR_NOTFOUND);
}

/* ---- motor ------------------------------------------------------------ */

static int g_duty;
static int mock_set_duty(void *c, int16_t d) { (void)c; g_duty = d; return ARB_OK; }
static const arb_motor_ops_t mock_motor = { .set_duty = mock_set_duty };

static void test_motor(void)
{
	printf("test_motor\n");
	arb_motor_t m;
	CHECK(arb_motor_init(&m, &mock_motor, NULL) == ARB_OK);

	arb_motor_set_duty(&m, 1.0f);
	CHECK(g_duty == 32767);
	arb_motor_set_duty(&m, -1.0f);
	CHECK(g_duty == -32767);
	arb_motor_set_duty(&m, 0.5f);
	CHECK(g_duty > 16000 && g_duty < 16600);

	/* clamp beyond range */
	arb_motor_set_duty(&m, 2.0f);
	CHECK(g_duty == 32767);

	/* inversion flips sign */
	m.inverted = true;
	arb_motor_set_duty(&m, 0.5f);
	CHECK(g_duty < 0);
	m.inverted = false;

	/* deadband zeroes small commands */
	m.deadband_q15 = 1000;
	arb_motor_set_duty_q15(&m, 500);
	CHECK(g_duty == 0);
	arb_motor_set_duty_q15(&m, 1500);
	CHECK(g_duty == 1500);

	/* stop */
	arb_motor_stop(&m);
	CHECK(g_duty == 0);
}

/* ---- encoder ---------------------------------------------------------- */

static int32_t g_enc_raw;
static int mock_enc_read(void *c, int32_t *v) { (void)c; *v = g_enc_raw; return ARB_OK; }
static const arb_encoder_ops_t mock_enc = { .read_count = mock_enc_read };

static void test_encoder(void)
{
	printf("test_encoder\n");
	arb_encoder_t e;
	CHECK(arb_encoder_init(&e, &mock_enc, NULL, 1000) == ARB_OK);

	arb_encoder_reading_t r;
	g_enc_raw = 0;
	CHECK(arb_encoder_read(&e, &r) == ARB_OK); /* primes */
	CHECK(r.velocity_rad_s == 0.0f);

	/* quarter turn = 250 counts of 1000 cpr -> pi/2 rad */
	g_enc_raw = 250;
	CHECK(arb_encoder_read(&e, &r) == ARB_OK);
	CHECK(r.count == 250);
	CHECK_NEAR(r.position_rad, (float)M_PI / 2.0f, 0.01f);

	/* reset clears accumulated angle */
	CHECK(arb_encoder_reset(&e) == ARB_OK);
	g_enc_raw = 0;
	CHECK(arb_encoder_read(&e, &r) == ARB_OK);
	CHECK_NEAR(r.position_rad, 0.0f, 1e-6f);
}

/* ---- imu axis remap --------------------------------------------------- */

static int mock_imu_read(void *c, arb_imu_sample_t *o)
{
	(void)c;
	memset(o, 0, sizeof(*o));
	o->accel.x = 1.0f;
	o->accel.y = 2.0f;
	o->accel.z = 3.0f;
	o->gyro.x = 0.1f;
	o->gyro.y = 0.2f;
	o->gyro.z = 0.3f;
	o->has_mag = false;
	return ARB_OK;
}
static const arb_imu_ops_t mock_imu = { .read = mock_imu_read };

static void test_imu(void)
{
	printf("test_imu\n");
	arb_imu_t_inst imu;
	CHECK(arb_imu_init(&imu, &mock_imu, NULL) == ARB_OK);

	arb_imu_sample_t s;
	CHECK(arb_imu_sample(&imu, &s) == ARB_OK);
	CHECK_NEAR(s.accel.x, 1.0f, 1e-6f); /* identity map */

	/* swap X and Y, negate Z */
	arb_axis_map_t map[3] = {
		{ .src = 1, .sign = 1 },
		{ .src = 0, .sign = 1 },
		{ .src = 2, .sign = -1 },
	};
	CHECK(arb_imu_set_axis_map(&imu, map) == ARB_OK);
	CHECK(arb_imu_sample(&imu, &s) == ARB_OK);
	CHECK_NEAR(s.accel.x, 2.0f, 1e-6f);
	CHECK_NEAR(s.accel.y, 1.0f, 1e-6f);
	CHECK_NEAR(s.accel.z, -3.0f, 1e-6f);

	/* message packing stamps the header */
	arb_imu_t msg;
	CHECK(arb_imu_sample_msg(&imu, 5, 9, &msg) == ARB_OK);
	CHECK(msg.header.type == ARB_MSG_IMU);
	CHECK(msg.header.source == 5);
	CHECK(msg.header.seq == 9);
}

/* ---- pid -------------------------------------------------------------- */

static void test_pid(void)
{
	printf("test_pid\n");
	arb_pid_t pid;
	arb_pid_init(&pid, 1.0f, 0.5f, 0.0f, -10.0f, 10.0f);

	/* output is clamped */
	float u = arb_pid_update(&pid, 1000.0f, 0.0f, 0.1f);
	CHECK(u <= 10.0f);

	/* anti-windup: integrator cannot exceed the output range */
	for (int i = 0; i < 1000; i++) {
		arb_pid_update(&pid, 1000.0f, 0.0f, 0.1f);
	}
	CHECK(pid.i_term <= 10.0f + 1e-3f);

	/* convergence: simple integrator plant should approach the setpoint */
	arb_pid_init(&pid, 2.0f, 1.0f, 0.05f, -50.0f, 50.0f);
	float x = 0.0f, target = 5.0f, dt = 0.01f;
	for (int i = 0; i < 5000; i++) {
		float ctl = arb_pid_update(&pid, target, x, dt);
		x += ctl * dt; /* plant: dx/dt = u */
	}
	CHECK_NEAR(x, target, 0.05f);
}

int main(void)
{
	arb_platform_init();

	test_pubsub();
	test_service();
	test_motor();
	test_encoder();
	test_imu();
	test_pid();

	printf("\n%d checks, %d failures\n", g_checks, g_failed);
	if (g_failed == 0) {
		printf("ALL TESTS PASSED\n");
	}
	return g_failed ? 1 : 0;
}
