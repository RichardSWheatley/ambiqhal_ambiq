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
#include "arb/control/diff_drive.h"
#include "arb/bridge/jaus.h"
#include "arb/bridge/serial.h"

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

/* ---- diff-drive kinematics ------------------------------------------- */

static void test_diff_drive(void)
{
	printf("test_diff_drive\n");
	arb_diffdrive_t dd = { .wheel_base = 0.2f, .wheel_radius = 0.05f };
	float wl, wr, v, w;

	/* straight ahead: both wheels equal, no rotation */
	arb_diffdrive_twist_to_wheels(&dd, 0.5f, 0.0f, &wl, &wr);
	CHECK_NEAR(wl, wr, 1e-6f);
	CHECK_NEAR(wl, 0.5f / 0.05f, 1e-4f);

	/* spin in place: wheels equal and opposite, zero forward speed */
	arb_diffdrive_twist_to_wheels(&dd, 0.0f, 1.0f, &wl, &wr);
	CHECK_NEAR(wl, -wr, 1e-6f);

	/* round-trip: twist -> wheels -> twist is identity */
	arb_diffdrive_twist_to_wheels(&dd, 0.3f, 0.8f, &wl, &wr);
	arb_diffdrive_wheels_to_twist(&dd, wl, wr, &v, &w);
	CHECK_NEAR(v, 0.3f, 1e-4f);
	CHECK_NEAR(w, 0.8f, 1e-4f);
}

/* ---- JAUS bridge ------------------------------------------------------ */

#define JAUS_CMD_VEL 200
#define JAUS_ODOM    201

static int s_jaus_cmd_count;
static arb_twist_t s_jaus_cmd;

static void on_jaus_cmd(arb_topic_id_t t, const void *m, size_t l, void *u)
{
	(void)t; (void)u;
	if (l == sizeof(arb_twist_t)) {
		s_jaus_cmd = *(const arb_twist_t *)m;
		s_jaus_cmd_count++;
	}
}

/* capture outbound JAUS messages from the bridge */
static int s_jaus_sent;
static uint16_t s_jaus_last_code;
static int jaus_capture(void *ctx, const arb_jaus_addr_t *dest,
			const uint8_t *msg, size_t len)
{
	(void)ctx; (void)dest;
	if (len >= 2) {
		s_jaus_last_code = (uint16_t)(msg[0] | (msg[1] << 8));
		s_jaus_sent++;
	}
	return 0;
}

static void test_jaus(void)
{
	printf("test_jaus\n");

	/* scaled-int round trip */
	uint32_t i = arb_jaus_scale_to_uint(2.5, -327.68, 327.67, 32);
	double back = arb_jaus_scale_from_uint(i, -327.68, 327.67, 32);
	CHECK_NEAR(back, 2.5, 0.001);
	/* clamping */
	CHECK(arb_jaus_scale_to_uint(1000.0, 0.0, 100.0, 16) == 0xFFFF);
	CHECK(arb_jaus_scale_to_uint(-1.0, 0.0, 100.0, 16) == 0);

	/* bridge setup */
	arb_topic_init();
	s_jaus_cmd_count = 0;
	s_jaus_sent = 0;
	CHECK(arb_topic_advertise(JAUS_CMD_VEL, sizeof(arb_twist_t)) == ARB_OK);
	CHECK(arb_topic_advertise(JAUS_ODOM, sizeof(arb_odom_t)) == ARB_OK);
	CHECK(arb_topic_subscribe(JAUS_CMD_VEL, on_jaus_cmd, NULL) == ARB_OK);

	arb_jaus_bridge_t br;
	arb_jaus_cfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.self.subsystem = 1;
	cfg.send = jaus_capture;
	cfg.cmd_vel_topic = JAUS_CMD_VEL;
	cfg.odom_topic = JAUS_ODOM;
	cfg.max_linear = 2.0f;
	cfg.max_angular = 3.0f;
	strcpy(cfg.identification, "arb-bot");
	CHECK(arb_jaus_init(&br, &cfg) == ARB_OK);

	/* inbound SetWrenchEffort -> /cmd_vel twist */
	uint8_t wbuf[16];
	int wn = arb_jaus_build_set_wrench_effort(50.0f, -100.0f, wbuf,
						  sizeof(wbuf));
	CHECK(wn > 0);
	arb_jaus_addr_t ctrl = { .subsystem = 9, .node = 1, .component = 1 };
	CHECK(arb_jaus_rx(&br, &ctrl, wbuf, (size_t)wn) == ARB_OK);
	arb_platform_dispatch();
	CHECK(s_jaus_cmd_count == 1);
	/* 50% of max_linear=2.0 -> 1.0 m/s; -100% of max_angular=3.0 -> -3.0 */
	CHECK_NEAR(s_jaus_cmd.linear.x, 1.0f, 0.01f);
	CHECK_NEAR(s_jaus_cmd.angular.z, -3.0f, 0.01f);

	/* publishing /odom triggers velocity + pose reports to the controller */
	arb_odom_t odom;
	memset(&odom, 0, sizeof(odom));
	odom.linear_vel = 0.5f;
	odom.angular_vel = 0.2f;
	odom.pose.x = 1.0f;
	CHECK(arb_topic_publish(JAUS_ODOM, &odom, sizeof(odom)) == ARB_OK);
	s_jaus_sent = 0;
	arb_platform_dispatch();
	CHECK(s_jaus_sent == 2); /* ReportVelocityState + ReportLocalPose */

	/* a query is answered with a report */
	uint8_t q[2] = { ARB_JAUS_QUERY_IDENTIFICATION & 0xFF,
			 ARB_JAUS_QUERY_IDENTIFICATION >> 8 };
	s_jaus_sent = 0;
	CHECK(arb_jaus_rx(&br, &ctrl, q, sizeof(q)) == ARB_OK);
	CHECK(s_jaus_sent == 1);
	CHECK(s_jaus_last_code == ARB_JAUS_REPORT_IDENTIFICATION);

	/* unknown command code is reported as not-found */
	uint8_t bad[2] = { 0xEF, 0xBE };
	CHECK(arb_jaus_rx(&br, &ctrl, bad, sizeof(bad)) == ARB_ERR_NOTFOUND);
}

/* ---- serial transport ------------------------------------------------- */

#define SER_TOPIC 210

/* loopback sink: feed everything written by the tx endpoint into rx */
static arb_serial_t *s_ser_rx;
static int ser_loopback(void *ctx, const uint8_t *buf, size_t len)
{
	(void)ctx;
	arb_serial_rx(s_ser_rx, buf, len);
	return 0;
}

static int s_ser_got;
static arb_odom_t s_ser_last;
static void on_ser(arb_topic_id_t t, const void *m, size_t l, void *u)
{
	(void)t; (void)u;
	if (l == sizeof(arb_odom_t)) {
		s_ser_last = *(const arb_odom_t *)m;
		s_ser_got++;
	}
}

static void test_serial(void)
{
	printf("test_serial\n");

	/* CRC sanity: known-length compute is stable/non-trivial */
	uint8_t sample[4] = { 1, 2, 3, 4 };
	uint16_t c = arb_serial_crc16(0xFFFF, sample, sizeof(sample));
	CHECK(c != 0 && c != 0xFFFF);

	arb_topic_init();
	s_ser_got = 0;
	CHECK(arb_topic_advertise(SER_TOPIC, sizeof(arb_odom_t)) == ARB_OK);
	CHECK(arb_topic_subscribe(SER_TOPIC, on_ser, NULL) == ARB_OK);

	arb_serial_t tx, rx;
	arb_serial_init(&rx, NULL, NULL);
	rx.publish_on_rx = true;
	s_ser_rx = &rx;
	arb_serial_init(&tx, ser_loopback, NULL);

	arb_odom_t odom;
	memset(&odom, 0, sizeof(odom));
	odom.pose.x = 3.14f;
	odom.linear_vel = 1.5f;

	/* frame it out; loopback feeds the parser; parser publishes locally */
	CHECK(arb_serial_send(&tx, SER_TOPIC, &odom, sizeof(odom)) == ARB_OK);
	arb_platform_dispatch();
	CHECK(s_ser_got == 1);
	CHECK_NEAR(s_ser_last.pose.x, 3.14f, 1e-6f);
	CHECK(rx.rx_frames == 1 && rx.rx_crc_errors == 0);

	/* a corrupted byte stream must not produce a frame */
	uint8_t junk[] = { 0x7E, 0x00, 0x00, 0xFF, 0xFF, 0xDE, 0xAD };
	arb_serial_rx(&rx, junk, sizeof(junk)); /* bogus length -> resync */
	arb_platform_dispatch();
	CHECK(s_ser_got == 1); /* unchanged */
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
	test_diff_drive();
	test_jaus();
	test_serial();

	printf("\n%d checks, %d failures\n", g_checks, g_failed);
	if (g_failed == 0) {
		printf("ALL TESTS PASSED\n");
	}
	return g_failed ? 1 : 0;
}
