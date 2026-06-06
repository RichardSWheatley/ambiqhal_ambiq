/*
 * ARB JAUS transport - OpenJAUS SDK adapter.
 *
 * Connects the portable ARB JAUS bridge (core/src/bridge/jaus.c) to a real JAUS
 * network using the OpenJAUS SDK, so the AS5669A transport, discovery, and node
 * management are handled by OpenJAUS while ARB owns the robot behavior.
 *
 *   outbound: ARB bridge send hook -> arb_openjaus_send() -> OpenJAUS message
 *   inbound : OpenJAUS message callback -> arb_jaus_rx() -> ARB topics
 *
 * This file is NOT built by default: it depends on the OpenJAUS headers/libs.
 * Define ARB_WITH_OPENJAUS and add the SDK to your include/link path to enable
 * it. The OpenJAUS API differs across major versions (the open-source v3.3 C API
 * vs. the v4 C++ SDK); the calls below follow the v3.x C "OjCmpt" style and are
 * marked where they must be matched to your SDK version.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 */
#ifdef ARB_WITH_OPENJAUS

#include <string.h>

#include "openJaus.h"          /* OpenJAUS SDK (path provided by your build) */
#include "arb/bridge/jaus.h"

/* Adapter state: the OpenJAUS component plus the ARB bridge it feeds. */
typedef struct {
	OjCmpt             cmpt;   /* OpenJAUS component handle */
	arb_jaus_bridge_t *bridge;
} arb_openjaus_t;

static arb_openjaus_t s_oj;

/* ---- ARB bridge -> OpenJAUS (transport send hook) --------------------- */

int arb_openjaus_send(void *ctx, const arb_jaus_addr_t *dest,
		      const uint8_t *msg, size_t len)
{
	(void)ctx;

	/* Wrap the raw ARB JAUS message bytes in an OpenJAUS message and send
	 * it via the component. With the v3.x API this is typically a
	 * UserMessage whose data is the command-code+payload buffer; v4 uses
	 * typed message classes. Adjust to your SDK. */
	JausMessage jmsg = jausMessageCreate();
	jmsg->commandCode    = (uint16_t)(msg[0] | (msg[1] << 8));
	jmsg->destination->subsystem = dest->subsystem;
	jmsg->destination->node      = dest->node;
	jmsg->destination->component = dest->component;
	jmsg->dataSize = (unsigned int)(len - 2);
	jmsg->data     = (unsigned char *)&msg[2];

	ojCmptSendMessage(s_oj.cmpt, jmsg);

	jmsg->data = NULL; /* we don't own msg */
	jausMessageDestroy(jmsg);
	return 0;
}

/* ---- OpenJAUS -> ARB bridge (inbound message callback) ---------------- */

static void arb_openjaus_recv(OjCmpt cmpt, JausMessage jmsg)
{
	(void)cmpt;

	arb_jaus_addr_t src;
	src.subsystem = jmsg->source->subsystem;
	src.node      = jmsg->source->node;
	src.component = jmsg->source->component;

	/* Re-linearize to command-code + payload for the portable codec. */
	uint8_t buf[256];
	if (jmsg->dataSize + 2u > sizeof(buf)) {
		return;
	}
	buf[0] = (uint8_t)(jmsg->commandCode & 0xFF);
	buf[1] = (uint8_t)(jmsg->commandCode >> 8);
	memcpy(&buf[2], jmsg->data, jmsg->dataSize);

	arb_jaus_rx(s_oj.bridge, &src, buf, jmsg->dataSize + 2u);
}

/* ---- setup ------------------------------------------------------------ */

/**
 * @brief Create the OpenJAUS component and bind it to an ARB JAUS bridge.
 *
 * Fill @p cfg with cmd_vel/odom topic ids and limits, set cfg->send to
 * arb_openjaus_send, then call arb_jaus_init() yourself, or let this helper do
 * both. Register arb_openjaus_recv for the command codes you care about.
 */
int arb_openjaus_start(arb_jaus_bridge_t *bridge, arb_jaus_cfg_t *cfg,
		       const char *name, uint8_t component_id, double rate_hz)
{
	s_oj.bridge = bridge;
	s_oj.cmpt = ojCmptCreate((char *)name, component_id, rate_hz);
	if (!s_oj.cmpt) {
		return ARB_ERR_AGAIN;
	}

	cfg->send     = arb_openjaus_send;
	cfg->send_ctx = &s_oj;
	int rc = arb_jaus_init(bridge, cfg);
	if (rc != ARB_OK) {
		return rc;
	}

	/* Route the JAUS commands we handle into our receive callback. */
	ojCmptSetMessageCallback(s_oj.cmpt, ARB_JAUS_SET_WRENCH_EFFORT,
				 arb_openjaus_recv);
	ojCmptSetMessageCallback(s_oj.cmpt, ARB_JAUS_QUERY_VELOCITY_STATE,
				 arb_openjaus_recv);
	ojCmptSetMessageCallback(s_oj.cmpt, ARB_JAUS_QUERY_LOCAL_POSE,
				 arb_openjaus_recv);
	ojCmptSetMessageCallback(s_oj.cmpt, ARB_JAUS_QUERY_IDENTIFICATION,
				 arb_openjaus_recv);

	ojCmptRun(s_oj.cmpt);
	return ARB_OK;
}

void arb_openjaus_stop(void)
{
	if (s_oj.cmpt) {
		ojCmptDestroy(s_oj.cmpt);
		s_oj.cmpt = NULL;
	}
}

#endif /* ARB_WITH_OPENJAUS */
