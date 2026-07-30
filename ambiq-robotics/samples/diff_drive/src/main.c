/*
 * ARB sample - closed-loop differential-drive base (Apollo510 EVB).
 *
 * Zephyr-native throughout: PWM/GPIO/I2C device APIs for the hardware, zbus
 * for all messaging, ARB for the vocabulary, node lifecycle / e-stop, PID and
 * kinematics.
 *
 * Dataflow:
 *   cmd_vel (zbus, local or via transport) -> PID per wheel -> PWM
 *   encoders (GPIO IRQ quadrature)  -> arb_chan_encoder @ 50 Hz
 *   ICM-42688 (I2C)                 -> arb_chan_imu     @ 200 Hz
 *   wheel odometry                  -> arb_chan_odom    @ 50 Hz
 *   arb_chan_estop                  -> motors brake
 *
 * Hardware (see boards/apollo510_evb.overlay):
 *   motors:   PWM P12/P18 (CT12/CT18), DIR P13/P19 -> DRV8833/TB6612
 *   encoders: P24/P25 (left A/B), P26/P27 (right A/B), x4 decode
 *   IMU:      ICM-42688-P on IOM0 I2C (SCL P5, SDA P6), addr 0x68
 *   link:     UART1 (TX P41, RX P43) via chosen "arb,uart"
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/zbus/zbus.h>

#include "arb/topics.h"
#include "arb/node.h"
#include "arb/transport.h"
#include "arb/control/pid.h"
#include "arb/control/diff_drive.h"

LOG_MODULE_REGISTER(diff_drive, LOG_LEVEL_INF);

#define ZUSER DT_PATH(zephyr_user)

/* ---- hardware ----------------------------------------------------------- */

static const struct pwm_dt_spec motor_pwm[2] = {
	PWM_DT_SPEC_GET_BY_IDX(ZUSER, 0),
	PWM_DT_SPEC_GET_BY_IDX(ZUSER, 1),
};
static const struct gpio_dt_spec motor_dir[2] = {
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, dir_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, dir_gpios, 1),
};
/* enc-gpios: left A, left B, right A, right B */
static const struct gpio_dt_spec enc_gpio[4] = {
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, enc_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, enc_gpios, 1),
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, enc_gpios, 2),
	GPIO_DT_SPEC_GET_BY_IDX(ZUSER, enc_gpios, 3),
};

static const struct device *const i2c_dev =
	DEVICE_DT_GET(DT_NODELABEL(i2c0));
#define IMU_ADDR 0x68

/* robot geometry / limits */
#define WHEEL_BASE_M    0.30f
#define WHEEL_RADIUS_M  0.04f
#define ENC_CPR         360u          /* lines; x4 decode -> 1440 counts/rev */
#define MAX_WHEEL_RADPS 25.0f

/* ---- encoders (x4 quadrature in GPIO ISR) ------------------------------- */

static const int8_t quad[4][4] = {
	{  0, +1, -1,  0 },
	{ -1,  0,  0, +1 },
	{ +1,  0,  0, -1 },
	{  0, -1, +1,  0 },
};

struct enc {
	struct gpio_callback cb;
	volatile int32_t     count;
	uint8_t              prev;
	uint8_t              base; /* index of channel A in enc_gpio[] */
	int32_t              last_count;
	int64_t              last_ms;
	float                vel_radps;
};
static struct enc enc[2] = { { .base = 0 }, { .base = 2 } };

static void enc_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	ARG_UNUSED(dev); ARG_UNUSED(pins);
	struct enc *e = CONTAINER_OF(cb, struct enc, cb);
	uint8_t a = (uint8_t)gpio_pin_get_dt(&enc_gpio[e->base]);
	uint8_t b = (uint8_t)gpio_pin_get_dt(&enc_gpio[e->base + 1]);
	uint8_t st = (uint8_t)((a << 1) | b);

	e->count += quad[e->prev][st];
	e->prev = st;
}

static float enc_velocity(struct enc *e)
{
	int64_t now = k_uptime_get();
	int64_t dt  = now - e->last_ms;

	if (dt <= 0) {
		return e->vel_radps;
	}
	int32_t c  = e->count;
	int32_t dc = c - e->last_count;

	e->last_count = c;
	e->last_ms    = now;
	e->vel_radps  = ((float)dc / (float)(ENC_CPR * 4)) * 2.0f *
			3.14159265f * 1000.0f / (float)dt;
	return e->vel_radps;
}

/* ---- motors ------------------------------------------------------------- */

static void motor_out(int m, float duty) /* duty -1..1 */
{
	duty = CLAMP(duty, -1.0f, 1.0f);
	gpio_pin_set_dt(&motor_dir[m], duty < 0.0f);
	float mag = duty < 0.0f ? -duty : duty;

	pwm_set_pulse_dt(&motor_pwm[m],
			 (uint32_t)(mag * (float)motor_pwm[m].period));
}

static void motors_brake(void)
{
	motor_out(0, 0.0f);
	motor_out(1, 0.0f);
}

/* ---- control state ------------------------------------------------------ */

static const arb_diffdrive_t geom = {
	.wheel_base   = WHEEL_BASE_M,
	.wheel_radius = WHEEL_RADIUS_M,
};
static arb_pid_t pid_l, pid_r;
static float sp_wl, sp_wr;        /* wheel setpoints, rad/s */
static bool  drive_enabled;
static struct { float x, y, th; } odom_pose;
/* shared by the control workqueue and the IMU thread */
static atomic_t seq;
static arb_node_t *drive_node;

static uint64_t stamp_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

/* ---- zbus observers ----------------------------------------------------- */

static void cmd_vel_cb(const struct zbus_channel *chan)
{
	const arb_twist_t *t = zbus_chan_const_msg(chan);

	arb_diffdrive_twist_to_wheels(&geom, t->linear.x, t->angular.z,
				      &sp_wl, &sp_wr);
	sp_wl = CLAMP(sp_wl, -MAX_WHEEL_RADPS, MAX_WHEEL_RADPS);
	sp_wr = CLAMP(sp_wr, -MAX_WHEEL_RADPS, MAX_WHEEL_RADPS);
}
ZBUS_LISTENER_DEFINE(cmd_vel_listener, cmd_vel_cb);
ZBUS_CHAN_ADD_OBS(arb_chan_cmd_vel, cmd_vel_listener, 3);

static void estop_cb(const struct zbus_channel *chan)
{
	ARG_UNUSED(chan);
	drive_enabled = false;
	sp_wl = sp_wr = 0.0f;
	motors_brake();
}
ZBUS_LISTENER_DEFINE(estop_listener, estop_cb);
ZBUS_CHAN_ADD_OBS(arb_chan_estop, estop_listener, 3);

/* ---- node lifecycle ----------------------------------------------------- */

static int drive_activate(void *ctx)
{
	ARG_UNUSED(ctx);
	arb_pid_reset(&pid_l);
	arb_pid_reset(&pid_r);
	sp_wl = sp_wr = 0.0f;
	drive_enabled = true;
	return ARB_OK;
}

static int drive_stop(void *ctx)
{
	ARG_UNUSED(ctx);
	drive_enabled = false;
	motors_brake();
	return ARB_OK;
}

static const arb_node_ops_t drive_ops = {
	.on_activate   = drive_activate,
	.on_deactivate = drive_stop,
	.on_shutdown   = drive_stop,
};

/* ---- 100 Hz control loop + 50 Hz telemetry ------------------------------ */

#define CTRL_PERIOD_MS 10

static void control_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(control_work, control_fn);

static void control_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	static unsigned int tick;
	const float dt = CTRL_PERIOD_MS / 1000.0f;

	float wl = enc_velocity(&enc[0]);
	float wr = enc_velocity(&enc[1]);

	if (drive_enabled) {
		motor_out(0, arb_pid_update(&pid_l, sp_wl, wl, dt));
		motor_out(1, arb_pid_update(&pid_r, sp_wr, wr, dt));
	}

	/* integrate planar odometry */
	float v, w;

	arb_diffdrive_wheels_to_twist(&geom, wl, wr, &v, &w);
	odom_pose.th += w * dt;
	odom_pose.x  += v * dt * cosf(odom_pose.th);
	odom_pose.y  += v * dt * sinf(odom_pose.th);

	if ((++tick % 2) == 0) { /* 50 Hz */
		arb_encoder_msg_t em = { 0 };

		for (int i = 0; i < 2; i++) {
			arb_header_init(&em.header, ARB_MSG_ENCODER,
					drive_node->id, stamp_us(),
					(uint32_t)atomic_inc(&seq));
			em.encoder_id     = (uint8_t)i;
			em.count          = enc[i].count;
			em.position_rad   = (float)enc[i].count * 2.0f *
					    3.14159265f / (ENC_CPR * 4);
			em.velocity_rad_s = i ? wr : wl;
			(void)zbus_chan_pub(&arb_chan_encoder, &em, K_NO_WAIT);
		}

		arb_odom_t od = { 0 };

		arb_header_init(&od.header, ARB_MSG_ODOM, drive_node->id,
				stamp_us(), (uint32_t)atomic_inc(&seq));
		od.pose.x      = odom_pose.x;
		od.pose.y      = odom_pose.y;
		od.pose.theta  = odom_pose.th;
		od.linear_vel  = v;
		od.angular_vel = w;
		(void)zbus_chan_pub(&arb_chan_odom, &od, K_NO_WAIT);
	}

	k_work_schedule(&control_work, K_MSEC(CTRL_PERIOD_MS));
}

/* ---- ICM-42688 over Zephyr I2C ------------------------------------------ */

#define ICM_WHO_AM_I      0x75
#define ICM_WHO_AM_I_VAL  0x47
#define ICM_PWR_MGMT0     0x4E
#define ICM_GYRO_CONFIG0  0x4F
#define ICM_ACCEL_CONFIG0 0x50
#define ICM_TEMP_DATA1    0x1D

#define ACCEL_SCALE (9.80665f / 4096.0f)   /* +/-8g          */
#define GYRO_SCALE  (0.0174533f / 32.8f)   /* +/-1000 dps    */

static int imu_init(void)
{
	uint8_t who = 0;

	if (i2c_reg_read_byte(i2c_dev, IMU_ADDR, ICM_WHO_AM_I, &who) ||
	    who != ICM_WHO_AM_I_VAL) {
		LOG_ERR("ICM-42688 WHO_AM_I 0x%02x", who);
		return -ENODEV;
	}
	/* +/-8g, +/-1000dps, 200 Hz ODR; accel+gyro low-noise */
	i2c_reg_write_byte(i2c_dev, IMU_ADDR, ICM_ACCEL_CONFIG0, 0x47);
	i2c_reg_write_byte(i2c_dev, IMU_ADDR, ICM_GYRO_CONFIG0,  0x47);
	i2c_reg_write_byte(i2c_dev, IMU_ADDR, ICM_PWR_MGMT0,     0x0F);
	k_msleep(50);
	return 0;
}

static void imu_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	if (imu_init() != 0) {
		return;
	}

	while (1) {
		uint8_t raw[14];

		if (i2c_burst_read(i2c_dev, IMU_ADDR, ICM_TEMP_DATA1, raw,
				   sizeof(raw)) == 0) {
			arb_imu_t m = { 0 };
			int16_t t  = (int16_t)((raw[0] << 8) | raw[1]);
			int16_t ax = (int16_t)((raw[2] << 8) | raw[3]);
			int16_t ay = (int16_t)((raw[4] << 8) | raw[5]);
			int16_t az = (int16_t)((raw[6] << 8) | raw[7]);
			int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
			int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
			int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

			arb_header_init(&m.header, ARB_MSG_IMU, 0, stamp_us(),
					(uint32_t)atomic_inc(&seq));
			m.accel.x = ax * ACCEL_SCALE;
			m.accel.y = ay * ACCEL_SCALE;
			m.accel.z = az * ACCEL_SCALE;
			m.gyro.x  = gx * GYRO_SCALE;
			m.gyro.y  = gy * GYRO_SCALE;
			m.gyro.z  = gz * GYRO_SCALE;
			m.temperature = t / 132.48f + 25.0f;

			(void)zbus_chan_pub(&arb_chan_imu, &m, K_NO_WAIT);
		}
		k_msleep(5); /* 200 Hz */
	}
}
K_THREAD_DEFINE(imu_tid, 2048, imu_thread, NULL, NULL, NULL, 6, 0, 200);

/* ---- main --------------------------------------------------------------- */

int main(void)
{
	LOG_INF("ARB diff-drive sample (zbus)");

	for (int i = 0; i < 2; i++) {
		if (!pwm_is_ready_dt(&motor_pwm[i]) ||
		    !gpio_is_ready_dt(&motor_dir[i])) {
			LOG_ERR("motor %d hw not ready", i);
			return -ENODEV;
		}
		gpio_pin_configure_dt(&motor_dir[i], GPIO_OUTPUT_INACTIVE);
		motor_out(i, 0.0f);
	}

	for (int i = 0; i < 4; i++) {
		gpio_pin_configure_dt(&enc_gpio[i], GPIO_INPUT);
	}
	for (int e = 0; e < 2; e++) {
		uint8_t a = (uint8_t)gpio_pin_get_dt(&enc_gpio[enc[e].base]);
		uint8_t b =
			(uint8_t)gpio_pin_get_dt(&enc_gpio[enc[e].base + 1]);

		enc[e].prev    = (uint8_t)((a << 1) | b);
		enc[e].last_ms = k_uptime_get();
		gpio_init_callback(&enc[e].cb, enc_isr,
				   BIT(enc_gpio[enc[e].base].pin) |
				   BIT(enc_gpio[enc[e].base + 1].pin));
		gpio_add_callback(enc_gpio[enc[e].base].port, &enc[e].cb);
		gpio_pin_interrupt_configure_dt(&enc_gpio[enc[e].base],
						GPIO_INT_EDGE_BOTH);
		gpio_pin_interrupt_configure_dt(&enc_gpio[enc[e].base + 1],
						GPIO_INT_EDGE_BOTH);
	}

	arb_pid_init(&pid_l, 0.06f, 0.9f, 0.0f, -1.0f, 1.0f);
	arb_pid_init(&pid_r, 0.06f, 0.9f, 0.0f, -1.0f, 1.0f);

	drive_node = arb_node_create("diff_drive", &drive_ops, NULL);
	arb_node_set_heartbeat(drive_node, 500);
	arb_node_configure(drive_node);
	arb_node_activate(drive_node);

	IF_ENABLED(CONFIG_ARB_TRANSPORT, (
		arb_transport_export(&arb_chan_odom);
		arb_transport_export(&arb_chan_imu);
		arb_transport_export(&arb_chan_encoder);
		arb_transport_export(&arb_chan_heartbeat);
		arb_transport_export(&arb_chan_estop);
	))

	k_work_schedule(&control_work, K_MSEC(CTRL_PERIOD_MS));
	return 0;
}
