/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <bluetooth/hci.h>


#include <drivers/sensor.h>
#include <stdio.h>
#include "mesh.h"
#include "board.h"

#define MOD_LF            0x0000
#define OP_HELLO          0xbb
#define OP_HEARTBEAT      0xbc
#define OP_BADUSER        0xbd
#define OP_DISTANCE	   0xbe
#define OP_VND_HELLO      BT_MESH_MODEL_OP_3(OP_HELLO, BT_COMP_ID_LF)
#define OP_VND_HEARTBEAT  BT_MESH_MODEL_OP_3(OP_HEARTBEAT, BT_COMP_ID_LF)
#define OP_VND_BADUSER    BT_MESH_MODEL_OP_3(OP_BADUSER, BT_COMP_ID_LF)
#define OP_VND_DISTANCE   BT_MESH_MODEL_OP_3(OP_DISTANCE, BT_COMP_ID_LF)


#define IV_INDEX          0
#define DEFAULT_TTL       31
#define GROUP_ADDR        0xc123
#define NET_IDX           0x000
#define APP_IDX           0x000
#define FLAGS             0

/* Maximum characters in "hello" message */
#define HELLO_MAX         8

#define MAX_SENS_STATUS_LEN 8

#define SENS_PROP_ID_PRESENT_DEVICE_TEMP 0x0054

#define SLEEP_TIME_MS   1
enum {
	SENSOR_HDR_A = 0,
	SENSOR_HDR_B = 1,
};

struct sensor_hdr_a {
	uint16_t prop_id:11;
	uint16_t length:4;
	uint16_t format:1;
} __packed;

struct sensor_hdr_b {
	uint8_t length:7;
	uint8_t format:1;
	uint16_t prop_id;
} __packed;

static struct k_work hello_work;
static struct k_work baduser_work;
static struct k_work mesh_start_work;


/* Definitions of models user data (Start) */
static struct led_onoff_state led_onoff_state[] = {
	/* Use LED 0 for this model */
	{ .dev_id = 0 },
};

static void heartbeat(uint8_t hops, uint16_t feat)
{
	board_show_text("Heartbeat Received", false, K_SECONDS(2));
}

static struct bt_mesh_cfg_srv cfg_srv = {
	.relay = BT_MESH_RELAY_ENABLED,
	.beacon = BT_MESH_BEACON_DISABLED,
	.default_ttl = DEFAULT_TTL,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),
	.relay_retransmit = BT_MESH_TRANSMIT(3, 20),

	.hb_sub.func = heartbeat,
};

static struct bt_mesh_cfg_cli cfg_cli = {
};

static void attention_on(struct bt_mesh_model *model)
{
	
	board_show_text("Attention!", false, K_SECONDS(2));
}

static void attention_off(struct bt_mesh_model *model)
{
	
	board_refresh_display();
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

/* Generic OnOff Server message handlers */
static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
	struct led_onoff_state *state = model->user_data;

	printk("addr 0x%04x onoff 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, state->current);
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
	net_buf_simple_add_u8(&msg, state->current);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send On Off Status response\n");
	}
}

static void gen_onoff_set_unack(struct bt_mesh_model *model,
				struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{	
	struct net_buf_simple *msg = model->pub->msg;
	struct led_onoff_state *state = model->user_data;
	int err;
	uint8_t tid, onoff;
	int64_t now;

	onoff = net_buf_simple_pull_u8(buf);
	tid = net_buf_simple_pull_u8(buf);

	if (onoff > STATE_ON) {
		printk("Wrong state received\n");

		return;
	}

	now = k_uptime_get();
	if (state->last_tid == tid && state->last_tx_addr == ctx->addr &&
	    (now - state->last_msg_timestamp <= (6 * MSEC_PER_SEC))) {
		printk("Already received message\n");
	}

	state->current = onoff;
	state->last_tid = tid;
	state->last_tx_addr = ctx->addr;
	state->last_msg_timestamp = now;

	printk("addr 0x%02x state 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, state->current);

	if (set_led_state(state->dev_id, onoff)) {
		printk("Failed to set led state\n");

		return;
	}

	/*
	 * If a server has a publish address, it is required to
	 * publish status on a state change
	 *
	 * See Mesh Profile Specification 3.7.6.1.2
	 *
	 * Only publish if there is an assigned address
	 */

	if (state->previous != state->current &&
	    model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		printk("publish last 0x%02x cur 0x%02x\n",
		       state->previous, state->current);
		state->previous = state->current;
		bt_mesh_model_msg_init(msg,
				       BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
		net_buf_simple_add_u8(msg, state->current);
		err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d\n", err);
		}
	}
}

static void gen_onoff_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{	
	gen_onoff_set_unack(model, ctx, buf);
	gen_onoff_get(model, ctx, buf);
}

static void sensor_desc_get(struct bt_mesh_model *model,
			    struct bt_mesh_msg_ctx *ctx,
			    struct net_buf_simple *buf)
{	
	/* TODO */
}

static void sens_temperature_celsius_fill(struct net_buf_simple *msg)
{	
	struct sensor_hdr_a hdr;
	/* TODO Get only temperature from sensor */
	struct sensor_value val[2];
	int16_t temp_degrees;

	hdr.format = SENSOR_HDR_A;
	hdr.length = sizeof(temp_degrees);
	hdr.prop_id = SENS_PROP_ID_PRESENT_DEVICE_TEMP;

	get_hdc1010_val(val);
	temp_degrees = sensor_value_to_double(&val[0]) * 100;

	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));
	net_buf_simple_add_le16(msg, temp_degrees);
}

static void sens_unknown_fill(uint16_t id, struct net_buf_simple *msg)
{	
	struct sensor_hdr_b hdr;

	/*
	 * When the message is a response to a Sensor Get message that
	 * identifies a sensor property that does not exist on the element, the
	 * Length field shall represent the value of zero and the Raw Value for
	 * that property shall be omitted. (Mesh model spec 1.0, 4.2.14).
	 *
	 * The length zero is represented using the format B and the special
	 * value 0x7F.
	 */
	hdr.format = SENSOR_HDR_B;
	hdr.length = 0x7FU;
	hdr.prop_id = id;

	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));
}

static void sensor_create_status(uint16_t id, struct net_buf_simple *msg)
{	
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

	switch (id) {
	case SENS_PROP_ID_PRESENT_DEVICE_TEMP:
		sens_temperature_celsius_fill(msg);
		break;
	default:
		sens_unknown_fill(id, msg);
		break;
	}
}

static void sensor_get(struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{	
	NET_BUF_SIMPLE_DEFINE(msg, 1 + MAX_SENS_STATUS_LEN + 4);
	uint16_t sensor_id;

	sensor_id = net_buf_simple_pull_le16(buf);
	sensor_create_status(sensor_id, &msg);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send Sensor get status response\n");
	}
}

static void sensor_col_get(struct bt_mesh_model *model,
			   struct bt_mesh_msg_ctx *ctx,
			   struct net_buf_simple *buf)
{	
	/* TODO */
}

static void sensor_series_get(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{	
	/* TODO */
}

/* Definitions of models publication context (Start) */
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_srv_pub_root, NULL, 2 + 3);

/* Mapping of message handlers for Generic OnOff Server (0x1000) */
static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_GET, 0, gen_onoff_get },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET, 2, gen_onoff_set },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, 2, gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

/* Mapping of message handlers for Sensor Server (0x1100) */
static const struct bt_mesh_model_op sensor_srv_op[] = {
	{ BT_MESH_MODEL_OP_SENS_DESC_GET, 0, sensor_desc_get },
	{ BT_MESH_MODEL_OP_SENS_GET, 0, sensor_get },
	{ BT_MESH_MODEL_OP_SENS_COL_GET, 2, sensor_col_get },
	{ BT_MESH_MODEL_OP_SENS_SERIES_GET, 2, sensor_series_get },
};

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV,
		      gen_onoff_srv_op, &gen_onoff_srv_pub_root,
		      &led_onoff_state[0]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV,
		      sensor_srv_op, NULL, NULL),
};


static void vnd_hello(struct bt_mesh_model *model,
		      struct bt_mesh_msg_ctx *ctx,
		      struct net_buf_simple *buf)
{	
	char str[32];
	size_t len;

	printk("Hello message from 0x%04x\n", ctx->addr);

	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		printk("Ignoring message from self\n");
		return;
	}

	len = MIN(buf->len, HELLO_MAX);
	memcpy(str, buf->data, len);
	str[len] = '\0';
	board_add_hello(ctx->addr,ctx->recv_rssi, str);
	
	strcat(str, " says hi!");
	
	board_show_text(str, false, K_SECONDS(1));
	
	board_blink_leds();
}

/* Model for publishing the data*/

static void vnd_distance(struct bt_mesh_model *model,
		      struct bt_mesh_msg_ctx *ctx,
		      struct net_buf_simple *buf)
{
	
	char str[32];
	size_t len;
	printk("Hello message from 0x%04x\n", ctx->addr);	
	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		printk("Ignoring message from self\n");
		return;
	}
	
	len = MIN(buf->len, HELLO_MAX);
	memcpy(str, buf->data, len);
	str[len] = '\0';
	board_add_hello(ctx->addr, ctx->recv_rssi,str);
	
	printk("if other board exist, the rssi %d\n", ctx->recv_rssi);
	sprintf(str,"0x%x", ctx->addr);
	board_blink_leds();	
}

static void vnd_baduser(struct bt_mesh_model *model,
			struct bt_mesh_msg_ctx *ctx,
			struct net_buf_simple *buf)
{	
	char str[32];
	size_t len;

	printk("\"Bad user\" message from 0x%04x\n", ctx->addr);

	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		printk("Ignoring message from self\n");
		return;
	}

	len = MIN(buf->len, HELLO_MAX);
	memcpy(str, buf->data, len);
	str[len] = '\0';

	strcat(str, " is misbehaving!");
	board_show_text(str, false, K_SECONDS(3));

	board_blink_leds();
}

static void vnd_heartbeat(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{	
	uint8_t init_ttl, hops;

	/* Ignore messages from self */
	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		return;
	}

	init_ttl = net_buf_simple_pull_u8(buf);
	hops = init_ttl - ctx->recv_ttl + 1;

	printk("Heartbeat from 0x%04x over %u hop%s\n", ctx->addr,
	       hops, hops == 1U ? "" : "s");

	board_add_heartbeat(ctx->addr, hops);
}

static const struct bt_mesh_model_op vnd_ops[] = {
	{ OP_VND_HELLO, 1, vnd_hello },
	{ OP_VND_HEARTBEAT, 1, vnd_heartbeat },
	{ OP_VND_BADUSER, 1, vnd_baduser },
	{ OP_VND_DISTANCE, 1, vnd_distance },
	BT_MESH_MODEL_OP_END,
};

static int pub_update(struct bt_mesh_model *mod)
{	
	struct net_buf_simple *msg = mod->pub->msg;

	printk("Preparing to send Hello Message\n");

	bt_mesh_model_msg_init(msg, OP_VND_DISTANCE);
	net_buf_simple_add_u8(msg, DEFAULT_TTL);

	return 0;
}

BT_MESH_MODEL_PUB_DEFINE(vnd_pub, pub_update, 3 + 1);

static struct bt_mesh_model vnd_models[] = {
	BT_MESH_MODEL_VND(BT_COMP_ID_LF, MOD_LF, vnd_ops, &vnd_pub, NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, vnd_models),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static size_t first_name_len(const char *name)
{	
	size_t len;

	for (len = 0; *name; name++, len++) {
		switch (*name) {
		case ' ':
		case ',':
		case '\n':
			return len;
		}
	}

	return len;
}

static void send_hello(struct k_work *work)
{	
	
	NET_BUF_SIMPLE_DEFINE(msg, 3 + HELLO_MAX + 4);
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = APP_IDX,
		.addr = GROUP_ADDR,
		.send_ttl = DEFAULT_TTL,
	};
	const char *name = bt_get_name();
	printk("Board_name %d\n",*name);
	printk("Message Group Addr : %x\n", GROUP_ADDR);
	printk("Message APP_IDX : %x\n", APP_IDX);
	bt_mesh_model_msg_init(&msg, OP_VND_HELLO);
	net_buf_simple_add_mem(&msg, name,
			       MIN(HELLO_MAX, first_name_len(name)));

	if (bt_mesh_model_send(&vnd_models[0], &ctx, &msg, NULL, NULL) == 0) {
		board_show_text("Saying \"hi!\" to everyone", false,
				K_SECONDS(2));
	} else {
		board_show_text("Sending Failed!", false, K_SECONDS(2));
	}
}

void mesh_send_hello(void)
{	
	k_work_submit(&hello_work);
}

static void send_baduser(struct k_work *work)
{	
	NET_BUF_SIMPLE_DEFINE(msg, 3 + HELLO_MAX + 4);
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = APP_IDX,
		.addr = GROUP_ADDR,
		.send_ttl = DEFAULT_TTL,
	};
	const char *name = bt_get_name();

	bt_mesh_model_msg_init(&msg, OP_VND_BADUSER);
	net_buf_simple_add_mem(&msg, name,
			       MIN(HELLO_MAX, first_name_len(name)));

	if (bt_mesh_model_send(&vnd_models[0], &ctx, &msg, NULL, NULL) == 0) {
		board_show_text("Bad user!", false, K_SECONDS(2));
	} else {
		board_show_text("Sending Failed!", false, K_SECONDS(2));
	}
}

void mesh_send_baduser(void)
{	
	k_work_submit(&baduser_work);
}

static int provision_and_configure(void)
{	
	static const uint8_t net_key[16] = {
		0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
		0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
	};
	static const uint8_t app_key[16] = {
		0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
		0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	};
	struct bt_mesh_cfg_mod_pub pub = {
		.addr = GROUP_ADDR,
		.app_idx = APP_IDX,
		.ttl = DEFAULT_TTL,
		.period = BT_MESH_PUB_PERIOD_SEC(10),
	};
	uint8_t dev_key[16];
	uint16_t addr;
	int err;

	err = bt_rand(dev_key, sizeof(dev_key));
	if (err) {
		return err;
	}

	do {
		err = bt_rand(&addr, sizeof(addr));
		if (err) {
			return err;
		}
	} while (!addr);

	/* Make sure it's a unicast address (highest bit unset) */
	addr &= ~0x8000;

	err = bt_mesh_provision(net_key, NET_IDX, FLAGS, IV_INDEX, addr,
				dev_key);
	if (err) {
		return err;
	}

	printk("Configuring...\n");

	/* Add Application Key */
	bt_mesh_cfg_app_key_add(NET_IDX, addr, NET_IDX, APP_IDX, app_key, NULL);

	/* Bind to vendor model */
	bt_mesh_cfg_mod_app_bind_vnd(NET_IDX, addr, addr, APP_IDX,
				     MOD_LF, BT_COMP_ID_LF, NULL);

	bt_mesh_cfg_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
				 BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);

	bt_mesh_cfg_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
				 BT_MESH_MODEL_ID_SENSOR_SRV, NULL);

	/* Bind to Health model */
	bt_mesh_cfg_mod_app_bind(NET_IDX, addr, addr, APP_IDX,
				 BT_MESH_MODEL_ID_HEALTH_SRV, NULL);

	/* Add model subscription */
	bt_mesh_cfg_mod_sub_add_vnd(NET_IDX, addr, addr, GROUP_ADDR,
				    MOD_LF, BT_COMP_ID_LF, NULL);

	bt_mesh_cfg_mod_pub_set_vnd(NET_IDX, addr, addr, MOD_LF, BT_COMP_ID_LF,
				    &pub, NULL);
	
	printk("Configuration complete\n");

	return addr;
}

static void start_mesh(struct k_work *work)
{	
	int err;

	err = provision_and_configure();
	if (err < 0) {
		board_show_text("Starting Mesh Failed", false,
				K_SECONDS(2));
	} else {
		char buf[32];

		snprintk(buf, sizeof(buf),
			 "Mesh Started\nAddr: 0x%04x", err);
		board_show_text(buf, false, K_SECONDS(4));
	}
}

void mesh_start(void)
{	
	k_work_submit(&mesh_start_work);
}

bool mesh_is_initialized(void)
{	
	return elements[0].addr != BT_MESH_ADDR_UNASSIGNED;
}

uint16_t mesh_get_addr(void)
{	
	return elements[0].addr;
}

int mesh_init(void)
{	
	static const uint8_t dev_uuid[16] = { 0xc0, 0xff, 0xee };
	static const struct bt_mesh_prov prov = {
		.uuid = dev_uuid,
	};

	k_work_init(&hello_work, send_hello);
	k_work_init(&baduser_work, send_baduser);
	k_work_init(&mesh_start_work, start_mesh);

	return bt_mesh_init(&prov, &comp);
}/*
 * Copyright (c) 2018 Phytec Messtechnik GmbH
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <display/cfb.h>
#include <sys/printk.h>

struct font_info {
	uint8_t columns;
} fonts[] = {
	[FONT_BIG] =    { .columns = 12 },
	[FONT_MEDIUM] = { .columns = 16 },
	[FONT_SMALL] =  { .columns = 25 },
};

#define LONG_PRESS_TIMEOUT K_SECONDS(1)

#define STAT_COUNT 128

static const struct device *epd_dev;
static bool pressed;
static uint8_t screen_id = SCREEN_MAIN;
static const struct device *gpio;
static struct k_delayed_work epd_work;
static struct k_delayed_work long_press_work;
static char str_buf[256];

static int status;
static struct {
	const struct device *dev;
	const char *name;
	gpio_pin_t pin;
	gpio_flags_t flags;
} leds[] = {
	{ .name = DT_GPIO_LABEL(DT_ALIAS(led0), gpios),
	  .pin = DT_GPIO_PIN(DT_ALIAS(led0), gpios),
	  .flags = DT_GPIO_FLAGS(DT_ALIAS(led0), gpios)},
	{ .name = DT_GPIO_LABEL(DT_ALIAS(led1), gpios),
	  .pin = DT_GPIO_PIN(DT_ALIAS(led1), gpios),
	  .flags = DT_GPIO_FLAGS(DT_ALIAS(led1), gpios)},
	{ .name = DT_GPIO_LABEL(DT_ALIAS(led2), gpios),
	  .pin = DT_GPIO_PIN(DT_ALIAS(led2), gpios),
	  .flags = DT_GPIO_FLAGS(DT_ALIAS(led2), gpios)}
};

struct k_delayed_work led_timer;
int show_sensors_data(k_timeout_t interval);
void post_sensor_data( char * data);

static size_t print_line(enum font_size font_size, int row, const char *text,
			 size_t len, bool center)
{	
	uint8_t font_height, font_width;
	uint8_t line[fonts[FONT_SMALL].columns + 1];
	int pad;

	cfb_framebuffer_set_font(epd_dev, font_size);

	len = MIN(len, fonts[font_size].columns);
	memcpy(line, text, len);
	line[len] = '\0';

	if (center) {
		pad = (fonts[font_size].columns - len) / 2U;
	} else {
		pad = 0;
	}

	cfb_get_font_size(epd_dev, font_size, &font_width, &font_height);

	if (cfb_print(epd_dev, line, font_width * pad, font_height * row)) {
		printk("Failed to print a string\n");
	}
	
	return len;
}

static size_t get_len(enum font_size font, const char *text)
{	
	const char *space = NULL;
	size_t i;

	for (i = 0; i <= fonts[font].columns; i++) {
		switch (text[i]) {
		case '\n':
		case '\0':
			return i;
		case ' ':
			space = &text[i];
			break;
		default:
			continue;
		}
	}

	/* If we got more characters than fits a line, and a space was
	 * encountered, fall back to the last space.
	 */
	if (space) {
		return space - text;
	}

	return fonts[font].columns;
}

void board_blink_leds(void)
{	
	status =1;
	k_delayed_work_submit(&led_timer, K_MSEC(100));
}

int board_show_text(const char *text, bool center, k_timeout_t duration)
{	
	int i;

	cfb_framebuffer_clear(epd_dev, false);
	
	for (i = 0; i < 3; i++) {
		size_t len;

		while (*text == ' ' || *text == '\n') {
			text++;
		}

		len = get_len(FONT_BIG, text);
		if (!len) {
			break;
		}

		text += print_line(FONT_BIG, i, text, len, center);
		if (!*text) {
			break;
		}
	}

	cfb_framebuffer_finalize(epd_dev);

	if (!K_TIMEOUT_EQ(duration, K_FOREVER)) {
		k_delayed_work_submit(&epd_work, duration);
	}
	return 0;
}

static struct stat {
	uint16_t addr;
	char name[9];
	int8_t  rssi;
	unsigned long d;
	int temp1,temp2;
	uint8_t min_hops;
	uint8_t max_hops;
	uint16_t hello_count;
	uint16_t heartbeat_count;
} stats[STAT_COUNT] = {
	[0 ... (STAT_COUNT - 1)] = {
		.min_hops = BT_MESH_TTL_MAX,
		.max_hops = 0,
	},
};

static uint32_t stat_count;

#define NO_UPDATE -1

static int add_hello(uint16_t addr, int8_t  rssi,const char *name)
{	


	int i;
	
	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		struct stat *stat = &stats[i];

		if (!stat->addr) {
			stat->addr = addr;
			stat->rssi=rssi;
			strncpy(stat->name, name, sizeof(stat->name) - 1);
			stat->hello_count = 1U;
			stat_count++;
			return i;
		}

		if (stat->addr == addr) {
			/* Update name, incase it has changed */
			strncpy(stat->name, name, sizeof(stat->name) - 1);

			if (stat->hello_count < 0xffff) {
				stat->rssi = rssi;
				stat->hello_count++;
				return i;
			}

			return NO_UPDATE;
		}
	}

	return NO_UPDATE;
}

static int add_heartbeat(uint16_t addr, uint8_t hops)
{	
	int i;

	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		struct stat *stat = &stats[i];

		if (!stat->addr) {
			stat->addr = addr;
			stat->heartbeat_count = 1U;
			stat->min_hops = hops;
			stat->max_hops = hops;
			stat_count++;
			return i;
		}

		if (stat->addr == addr) {
			if (hops < stat->min_hops) {
				stat->min_hops = hops;
			} else if (hops > stat->max_hops) {
				stat->max_hops = hops;
			}

			if (stat->heartbeat_count < 0xffff) {
				stat->heartbeat_count++;
				return i;
			}

			return NO_UPDATE;
		}
	}

	return NO_UPDATE;
}

void board_add_hello(uint16_t addr, int8_t  rssi,const char *name)
{	
	uint32_t sort_i;

	sort_i = add_hello(addr,rssi, name);
	if (sort_i != NO_UPDATE) {
	}
}

void board_add_heartbeat(uint16_t addr, uint8_t hops)
{	
	uint32_t sort_i;

	sort_i = add_heartbeat(addr, hops);
	if (sort_i != NO_UPDATE) {
	}
}

float_t distance(int8_t rssi)
{
	float_t cal=0.0, temp=0.0;
	
	float p;
	
	cal = -69 -(rssi);
	temp = cal/20;
	p = (pow(10,temp)*100); 
	return p;
}
void Display(k_timeout_t interval)
{	
	
	int i;
	struct stat *stat;
	char str[100], str1[32];
	
	
	/* Get the Board Address */
	struct sensor_value val[3];
	sprintf(str,"0x%04x", mesh_get_addr());
	
	
	
	/* hdc1010 get Temperature and Humidity */
	if (get_hdc1010_val(val)) {
		goto _error_get;
	}
	sprintf(str1,",%d", val[0].val1 );
	strcat(str, str1);
	
	sprintf(str1,".%d", val[0].val2 / 100000 );
	strcat(str, str1);
	
	
	sprintf(str1,",%d", val[1].val1);
	strcat(str, str1);
	
	
	sprintf(str1,",%u", stat_count );
	strcat(str, str1);
	
	/* Get the Information of the neighbour node : Address, Disance */
	if (stat_count > 0) {
		
		for (i = 0; i < ARRAY_SIZE(stats); i++) {
		
		stat = &stats[i];
		if (!stat->addr) {
			break;
		}

		if (!stat->hello_count) {
			continue;
		}

		stat = &stats[i];
			
		stat->d = distance(stat->rssi);
			
		sprintf(str1,",0x%04x", stat->addr);
		strcat(str, str1);
			
		sprintf(str1,",%ld", stat->d/100);
		strcat(str, str1);
			
			
		}
	}

	
	k_delayed_work_submit(&epd_work, interval);
	post_sensor_data(str);					// Calling Function post_sensor_data
	return;
_error_get:
	printk("Failed to get sensor data or print a string\n");
}

void post_sensor_data( char * data)
{
 printk("Posting Data : %s\n", data);

}

/* To display the statistic on the Board */
static void show_statistics(k_timeout_t interval)
{	
	
	int len, i, line = 0;
	struct stat *stat;
	char str[32];
	int val;
	
	cfb_framebuffer_clear(epd_dev, false);
	/* Current Node Address */
	len = snprintk(str, sizeof(str),
		       "Own Address: 0x%04x", mesh_get_addr());
	print_line(FONT_SMALL, line++, str, len, false);
	
	/* Total Number of Nodes present in the mesh network*/
	len = snprintk(str, sizeof(str),
		       "Node Count:  %u", stat_count + 1);
	print_line(FONT_SMALL, line++, str, len, false);
	
	/* Display the Proximity sensor Value*/
	val=show_sensors_data(K_SECONDS(2));
	printk("Proximity:%d\n", val);
	len = snprintf(str_buf, sizeof(str_buf), "Proximity:%d\n", val);
	print_line(FONT_SMALL, line++, str_buf, len, false);
	
	/* If Neighbour node present display the detail : Message_Count, Address, RSSI, Distance(cm) */
	if (stat_count > 0) {
		len = snprintk(str, sizeof(str), "Msg,NodeId,rssi,d(cm)");
		print_line(FONT_SMALL, line++, str, len, false);

		for (i = 0; i < ARRAY_SIZE(stats); i++) {
		
		stat = &stats[i];
		if (!stat->addr) {
			break;
		}

		if (!stat->hello_count) {
			continue;
		}

			stat = &stats[i];
			
			stat->d = distance(stat->rssi);


			len = snprintk(str, sizeof(str), "%-3u 0x%04x %d  %lu",
				       stat->hello_count, stat->addr,stat->rssi,stat->d);
			print_line(FONT_SMALL, line++, str, len, false);
		}
	}

	cfb_framebuffer_finalize(epd_dev);
	k_delayed_work_submit(&epd_work, interval);
}

/* To Alert the user using red led if distance < 2 meters*/
static void show_display(k_timeout_t interval)
{	
	
	int len, i, line = 0;
	struct stat *stat;
	char str[32];
	int count = 0;
	cfb_framebuffer_clear(epd_dev, false);

	/* Display node ID : */
	len = snprintk(str, sizeof(str),"My ID: 0x%04x", mesh_get_addr());
	print_line(FONT_SMALL, line++, str, len, false);
	
	if(status)
	{
	cfb_print(epd_dev,"Recieving...",150,0);                        // Blinks if neighbour input is recieved 
	status =0;
	}
	cfb_print(epd_dev, "--------------------",0,16);
	
	cfb_framebuffer_set_font(epd_dev,0);
	cfb_print(epd_dev,"*",0,32);
	

	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		

		stat = &stats[i];
		if (!stat->addr) {
			break;
		}

		if (!stat->hello_count) {
			continue;
		}
		stat->d = distance(stat->rssi);
		if(stat->d < 200)				// if distance < 2 meter alert RED
		{	
			cfb_print(epd_dev,"+",32,32);
			getled();
		}
		else	
			//sprintf (str,"%x",stat->addr);
			cfb_print(epd_dev,"#", 150, 32);

		
	}
	cfb_framebuffer_set_font(epd_dev,0);
	cfb_print(epd_dev, "--------------------",0,48);
	
	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		
		stat = &stats[i];
		if (!stat->addr) {
			break;
		}

		if (!stat->hello_count) {
			continue;
		}
		stat->d = distance(stat->rssi);
		if(stat->d < 200 )
		 { 
		   count++;
		   sprintf (str,"0x%x",stat->addr);
		   cfb_print(epd_dev,str,0,80+(16*i));
		 }
	}
	
	line=4;
	len = snprintk(str, sizeof(str),
		       "No. of devices near: %d", count);
	print_line(FONT_SMALL, line++, str, len, false);
	
	
	cfb_framebuffer_finalize(epd_dev);
	k_delayed_work_submit(&epd_work, interval);
}

/* Get Proximity Data */
int show_sensors_data(k_timeout_t interval)
{	
	struct sensor_value val[3];
	
	/* apds9960 */
	if (get_apds9960_val(val)) {
		goto _error_get;
	}

	k_delayed_work_submit(&epd_work, interval);

	return val[1].val1;
_error_get:
	printk("Failed to get sensor data or print a string\n");
}


static void show_main(void)
{	
	char buf[CONFIG_BT_DEVICE_NAME_MAX];
	int i;

	strncpy(buf, bt_get_name(), sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	/* Convert commas to newlines */
	for (i = 0; buf[i] != '\0'; i++) {
		if (buf[i] == ',') {
			buf[i] = '\n';
		}
	}
	
	board_show_text(buf, true, K_FOREVER);
	
}

static void epd_update(struct k_work *work)
{	
	switch (screen_id) {
	case SCREEN_DIS:
		show_display( K_SECONDS(1));
		return;
	case SCREEN_STATS:
		show_statistics( K_SECONDS(2));
		return;
	case SCREEN_MAIN:	
		show_main();
		//Display(K_SECONDS(2));
		return;
	}
}

static void long_press(struct k_work *work)
{	
	/* Treat as release so actual release doesn't send messages */
	pressed = false;
	screen_id = (screen_id + 1) % SCREEN_LAST;
	printk("Change screen to id = %d\n", screen_id);
	board_refresh_display();
}

static bool button_is_pressed(void)
{	
	return gpio_pin_get(gpio, DT_GPIO_PIN(DT_ALIAS(sw0), gpios)) > 0;
}

static void button_interrupt(const struct device *dev,
			     struct gpio_callback *cb,
			     uint32_t pins)
{	
	if (button_is_pressed() == pressed) {
		return;
	}

	pressed = !pressed;
	printk("Button %s\n", pressed ? "pressed" : "released");

	if (pressed) {
		k_delayed_work_submit(&long_press_work, LONG_PRESS_TIMEOUT);
		return;
	}

	k_delayed_work_cancel(&long_press_work);

	if (!mesh_is_initialized()) {
		return;
	}

	/* Short press for views */
	switch (screen_id) {
	case SCREEN_DIS:
	case SCREEN_STATS:
		return;
	case SCREEN_MAIN:
		if (pins & BIT(DT_GPIO_PIN(DT_ALIAS(sw0), gpios))) {
			uint32_t uptime = k_uptime_get_32();
			static uint32_t bad_count, press_ts;

			if (uptime - press_ts < 500) {
				bad_count++;
			} else {
				bad_count = 0U;
			}

			press_ts = uptime;

			if (bad_count) {
				if (bad_count > 5) {
					mesh_send_baduser();
					bad_count = 0U;
				} else {
					printk("Ignoring press\n");
				}
			} else {
				mesh_send_hello();
			}
		}
		return;
	default:
		return;
	}
}

static int configure_button(void)						
{	
	static struct gpio_callback button_cb;

	gpio = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(sw0), gpios));
	if (!gpio) {
		return -ENODEV;
	}

	gpio_pin_configure(gpio, DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
			   GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios));

	gpio_pin_interrupt_configure(gpio, DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
				     GPIO_INT_EDGE_BOTH);

	gpio_init_callback(&button_cb, button_interrupt,
			   BIT(DT_GPIO_PIN(DT_ALIAS(sw0), gpios)));

	gpio_add_callback(gpio, &button_cb);

	return 0;
}

int set_led_state(uint8_t id, bool state)
{	
	return gpio_pin_set(leds[id].dev, leds[id].pin, state);
}

static void led_timeout(struct k_work *work)
{	
	static int led_cntr;
	int i;

	/* Disable all LEDs */
	for (i = 0; i < ARRAY_SIZE(leds); i++) {
		set_led_state(i, 0);
	}

	/* Stop after 5 iterations */
	if (led_cntr >= (ARRAY_SIZE(leds) * 5)) {
		led_cntr = 0;
		return;
	}

	/* Select and enable current LED */
	i = led_cntr++ % ARRAY_SIZE(leds);
	set_led_state(i, 1);

	k_delayed_work_submit(&led_timer, K_MSEC(100));
}

static int configure_leds(void)
{	
	int i;

	for (i = 0; i < ARRAY_SIZE(leds); i++) {
		leds[i].dev = device_get_binding(leds[i].name);
		if (!leds[i].dev) {
			printk("Failed to get %s device\n", leds[i].name);
			return -ENODEV;
		}

		gpio_pin_configure(leds[i].dev, leds[i].pin,
				   leds[i].flags |
				   GPIO_OUTPUT_INACTIVE);
	}

	k_delayed_work_init(&led_timer, led_timeout);
	return 0;
}

static int erase_storage(void)
{	
	const struct device *dev;

	dev = device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);

	return flash_erase(dev, FLASH_AREA_OFFSET(storage),
			   FLASH_AREA_SIZE(storage));
}

void board_refresh_display(void)
{	
	k_delayed_work_submit(&epd_work, K_NO_WAIT);
}

int board_init(void)						//2
{	
	epd_dev = device_get_binding(DT_LABEL(DT_INST(0, solomon_ssd16xxfb)));
	if (epd_dev == NULL) {
		printk("SSD16XX device not found\n");
		return -ENODEV;
	}

	if (cfb_framebuffer_init(epd_dev)) {
		printk("Framebuffer initialization failed\n");
		return -EIO;
	}

	cfb_framebuffer_clear(epd_dev, true);

	if (configure_button()) {
		printk("Failed to configure button\n");
		return -EIO;
	}

	if (configure_leds()) {
		printk("LED init failed\n");
		return -EIO;
	}

	k_delayed_work_init(&epd_work, epd_update);
	k_delayed_work_init(&long_press_work, long_press);

	pressed = button_is_pressed();
	if (pressed) {
		printk("Erasing storage\n");
		board_show_text("Resettin
