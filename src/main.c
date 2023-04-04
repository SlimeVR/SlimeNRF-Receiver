/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <math.h>

#include <zephyr/init.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#define pi 3.141592653589793238462643383279502884f

#define INT16_TO_UINT16(x) ((uint16_t)32768 + (uint16_t)(x))
#define UINT16_TO_INT16(x) (int16_t)((uint16_t)(x) - (uint16_t)32768)
#define TO_FIXED_14(x) ((int16_t)((x) * (1 << 14)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))
#define FIXED_14_TO_DOUBLE(x) (((double)(x)) / (1 << 14))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

static const struct gpio_dt_spec leds[] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios),
};

BUILD_ASSERT(DT_SAME_NODE(DT_GPIO_CTLR(DT_ALIAS(led0), gpios),
			  DT_GPIO_CTLR(DT_ALIAS(led1), gpios)) &&
	     DT_SAME_NODE(DT_GPIO_CTLR(DT_ALIAS(led0), gpios),
			  DT_GPIO_CTLR(DT_ALIAS(led2), gpios)) &&
	     DT_SAME_NODE(DT_GPIO_CTLR(DT_ALIAS(led0), gpios),
			  DT_GPIO_CTLR(DT_ALIAS(led3), gpios)),
	     "All LEDs must be on the same port");

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);

static int leds_init(void)
{
	if (!device_is_ready(leds[0].port)) {
		LOG_ERR("LEDs port not ready");
		return -ENODEV;
	}

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		int err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT);

		if (err) {
			LOG_ERR("Unable to configure LED%u, err %d.", i, err);
			return err;
		}
	}

	return 0;
}

static void leds_update(uint8_t value)
{
	bool led0_status = !(value % 8 > 0 && value % 8 <= 4);
	bool led1_status = !(value % 8 > 1 && value % 8 <= 5);
	bool led2_status = !(value % 8 > 2 && value % 8 <= 6);
	bool led3_status = !(value % 8 > 3);

	gpio_port_pins_t mask = BIT(leds[0].pin) | BIT(leds[1].pin) |
				BIT(leds[2].pin) | BIT(leds[3].pin);

	gpio_port_value_t val = led0_status << leds[0].pin |
				led1_status << leds[1].pin |
				led2_status << leds[2].pin |
				led3_status << leds[3].pin;

	gpio_port_set_masked_raw(leds[0].port, mask, val);
}

static struct k_work report_send;

static struct tracker_report {
	uint8_t imu_id;
	uint8_t battery;
	uint16_t qi;
	uint16_t qj;
	uint16_t qk;
	uint16_t ql;
	uint16_t ax;
	uint16_t ay;
	uint16_t az;
} __packed report = {
	.imu_id = 0,
	.battery = 0,
	.qi = 0,
	.qj = 0,
	.qk = 0,
	.ql = 0,
	.ax = 0,
	.ay = 0,
	.az = 0
};;

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
//			LOG_DBG("Packet received, len %d : "
//				"0x%02x, 0x%02x, 0x%02x, 0x%02x, "
//				"0x%02x, 0x%02x, 0x%02x, 0x%02x",
//				rx_payload.length, rx_payload.data[0],
//				rx_payload.data[1], rx_payload.data[2],
//				rx_payload.data[3], rx_payload.data[4],
//				rx_payload.data[5], rx_payload.data[6],
//				rx_payload.data[7]);

//double rx_buf[7];
//rx_buf[0] = FIXED_14_TO_DOUBLE(UINT16_TO_INT16(((uint16_t)rx_payload.data[2] << 8) | rx_payload.data[3]));
//rx_buf[1] = FIXED_14_TO_DOUBLE(UINT16_TO_INT16(((uint16_t)rx_payload.data[4] << 8) | rx_payload.data[5]));
//rx_buf[2] = FIXED_14_TO_DOUBLE(UINT16_TO_INT16(((uint16_t)rx_payload.data[6] << 8) | rx_payload.data[7]));
//rx_buf[3] = FIXED_14_TO_DOUBLE(UINT16_TO_INT16(((uint16_t)rx_payload.data[8] << 8) | rx_payload.data[9]));
//rx_buf[4] = FIXED_10_TO_DOUBLE(UINT16_TO_INT16(((uint16_t)rx_payload.data[10] << 8) | rx_payload.data[11]));
//rx_buf[5] = FIXED_10_TO_DOUBLE(UINT16_TO_INT16(((uint16_t)rx_payload.data[12] << 8) | rx_payload.data[13]));
//rx_buf[6] = FIXED_10_TO_DOUBLE(UINT16_TO_INT16(((uint16_t)rx_payload.data[14] << 8) | rx_payload.data[15]));
//float q[4];
//q[0] = rx_buf[0];
//q[1] = rx_buf[1];
//q[2] = rx_buf[2];
//q[3] = rx_buf[3];
//float a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
//float a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
//float a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
//float a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
//float a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
//float pitch = -asinf(a32);
//float roll = atan2f(a31, a33);
//float yaw = atan2f(a12, a22);
//pitch *= 180.0f / pi;
//yaw *= 180.0f / pi;
//if (yaw < 0) yaw += 360.0f; // Ensure yaw stays between 0 and 360
//roll *= 180.0f / pi;
report.imu_id=rx_payload.data[0];
report.battery=rx_payload.data[1];
report.qi=(((uint16_t)rx_payload.data[2] << 8) | rx_payload.data[3]);
report.qj=(((uint16_t)rx_payload.data[4] << 8) | rx_payload.data[5]);
report.qk=(((uint16_t)rx_payload.data[6] << 8) | rx_payload.data[7]);
report.ql=(((uint16_t)rx_payload.data[8] << 8) | rx_payload.data[9]);
report.ax=(((uint16_t)rx_payload.data[10] << 8) | rx_payload.data[11]);
report.ay=(((uint16_t)rx_payload.data[12] << 8) | rx_payload.data[13]);
report.az=(((uint16_t)rx_payload.data[14] << 8) | rx_payload.data[15]);
	k_work_submit(&report_send);

//			LOG_DBG("ID:%d BAT:%d I:%0.2f J:%0.2f K:%0.2f L:%0.2f P:%0.2f R:%0.2f Y:%0.2f",
//				rx_payload.data[0],rx_payload.data[1],q[0],q[1],q[2],q[3],pitch,roll,yaw);

//			leds_update(rx_payload.data[1]);
		} else {
			LOG_ERR("Error while reading rx packet");
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
}

static bool configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_SECONDS(2)

static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_LOGICAL_MIN8(0),
		HID_LOGICAL_MAX8(255),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(2),
		HID_INPUT(0x02),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_LOGICAL_MIN8(0),
		HID_LOGICAL_MAX16(255, 255),
		HID_REPORT_SIZE(16),
		HID_REPORT_COUNT(7),
		HID_INPUT(0x02),
	HID_END_COLLECTION,
};

static void send_report(struct k_work *work)
{
	int ret, wrote;

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		ret = hid_int_ep_write(hdev, (uint8_t *)&report,
				       sizeof(report), &wrote);
		if (ret != 0) {
			/*
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		} else {
			LOG_DBG("Report submitted");
		}
	} else {
		LOG_DBG("HID IN endpoint busy");
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
}

/*
 * On Idle callback is available here as an example even if actual use is
 * very limited. In contrast to report_event_handler(),
 * report value is not incremented here.
 */
static void on_idle_cb(const struct device *dev, uint16_t report_id)
{
	LOG_DBG("On idle callback");
	k_work_submit(&report_send);
}

static void report_event_handler(struct k_timer *dummy)
{
	/* Increment reported data */
	//report_1.value++;
	k_work_submit(&report_send);
}

static void protocol_cb(const struct device *dev, uint8_t protocol)
{
	LOG_INF("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
	.on_idle = on_idle_cb,
	.protocol_change = protocol_cb,
};

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		if (!configured) {
			int_in_ready_cb(hdev);
			configured = true;
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static int composite_pre_init(const struct device *dev)
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_INF("HID Device: dev %p", hdev);

	usb_hid_register_device(hdev, hid_report_desc, sizeof(hid_report_desc),
				&ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	k_timer_start(&event_timer, REPORT_PERIOD, REPORT_PERIOD);

	if (usb_hid_set_proto_code(hdev, HID_BOOT_IFACE_CODE_NONE)) {
		LOG_WRN("Failed to set Protocol Code");
	}

	return usb_hid_init(hdev);
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void main(void)
{
	int err;

	LOG_INF("Enhanced ShockBurst prx sample");

	err = clocks_start();
	if (err) {
		return;
	}

	err = leds_init();
	if (err) {
		return;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}

	LOG_INF("Initialization complete");

	err = esb_write_payload(&tx_payload);
	if (err) {
		LOG_ERR("Write payload, err %d", err);
		return;
	}

	LOG_INF("Setting up for packet receiption");

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return;
	}

	int ret;

	LOG_INF("Starting application");

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	k_work_init(&report_send, send_report);

	/* return to idle thread */
	return;
}
