/*
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
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <drivers/sensor.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh/access.h>

#include "mesh.h"
#include "board.h"
#include "getled.h"

#if CONFIG_NET_IPV4
#include "http_util.h"
#endif

enum font_size {
	FONT_SMALL = 0,
	FONT_MEDIUM = 1,
	FONT_BIG = 2,
};

enum screen_ids {
	SCREEN_MAIN = 0,
	SCREEN_STATS = 1,
	SCREEN_DIS = 2,
	SCREEN_LAST,
};

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
		printk("stats cound: %d\n", stat_count);
	}

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

	k_delayed_work_submit(&epd_work, interval);

#if CONFIG_NET_IPV4
	post_sensor_data(str);
	return;
#else
	printk("I am skipping HTTP post\n");
#endif

_error_get:
	printk("Failed to get sensor data or print a string\n");
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
		Display(K_SECONDS(2));
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

static int configure_button(void)						//3
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
		board_show_text("Resetting Device", false, K_SECONDS(1));
		erase_storage();
	}

	return 0;
}
