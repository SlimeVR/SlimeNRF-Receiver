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
#include <zephyr/sys/crc.h>

#include <zephyr/init.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include <zephyr/console/console.h>

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define STORED_TRACKERS 1
#define STORED_TRACKER_ADDR 2

#define RBT_CNT_ID 2
#define STORED_ADDR_0 3
// 0-15 -> id 3-18
// 0-255 -> id 3-258

#include "retained.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define MAX_TRACKERS 256
#define DETECTION_THRESHOLD 16

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884f
#endif

// Saturate int to 16 bits
// Optimized to a single ARM assembler instruction
#define SATURATE_INT16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

#define SATURATE_UINT11(x) ((x) > 2047 ? 2047 : ((x) < 0 ? 0 : (x)))
#define SATURATE_UINT10(x) ((x) > 1023 ? 1023 : ((x) < 0 ? 0 : (x)))

#define TO_FIXED_15(x) ((int16_t)SATURATE_INT16((x) * (1 << 15)))
#define TO_FIXED_11(x) ((int16_t)((x) * (1 << 11)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))
#define TO_FIXED_7(x) ((int16_t)SATURATE_INT16((x) * (1 << 7)))
#define FIXED_15_TO_DOUBLE(x) (((double)(x)) / (1 << 15))
#define FIXED_11_TO_DOUBLE(x) (((double)(x)) / (1 << 11))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))
#define FIXED_7_TO_DOUBLE(x) (((double)(x)) / (1 << 7))

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

uint8_t stored_trackers = 0;
uint64_t stored_tracker_addr[MAX_TRACKERS] = {0};

uint8_t discovered_trackers[256] = {0};

uint8_t pairing_buf[8] = {0};

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
	0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_timer = ESB_CREATE_PAYLOAD(0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_sync = ESB_CREATE_PAYLOAD(0,
	0, 0, 0, 0);

const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(ZEPHYR_USER_NODE, led_gpios, led0);

static struct k_work report_send;

static struct tracker_report {
	uint8_t data[16];
} __packed report = {
	.data = {0}
};;

uint8_t reports[256*sizeof(report)];
uint8_t report_count = 0;
uint8_t report_sent = 0;

int blink = 0;

#include <nrfx_timer.h>
static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

//|b0      |b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |
//|type    |id      |packet data                                                                                                                  |
//|0       |id      |proto   |batt    |batt_v  |temp    |brd_id  |mcu_id  |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |rssi    |
//|1       |id      |q0               |q1               |q2               |q3               |a0               |a1               |a2               |
//|2       |id      |batt    |batt_v  |temp    |q_buf                              |a0               |a1               |a2               |rssi    |
//|3	   |id      |svr_stat|status  |resv                                                                                              |rssi    |
//|255     |id      |addr                                                 |resv                                                                   |

void packet_device_addr(uint8_t *report, uint16_t id) // associate id and tracker address
{
	report[0] = 255; // receiver packet 0
	report[1] = id;
	memcpy(&report[2], &stored_tracker_addr[id], 6);
	memset(&report[8], 0, 8); // last 8 bytes unused for now
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:
	// make tx payload for ack here
		if (!esb_read_rx_payload(&rx_payload)) {
			switch (rx_payload.length)
			{
			case 8:
				LOG_INF("RX Pairing Packet");
				memcpy(pairing_buf, rx_payload.data, 8);
				esb_write_payload(&tx_payload_pair); // Add to TX buffer
				break;
			case 16:
				uint8_t imu_id = rx_payload.data[1];
				if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) { // garbage filtering of nonexistent tracker
					discovered_trackers[imu_id]++;
					return;
				}
				if (rx_payload.data[0] > 223) // reserved for receiver only
					break;
				memcpy(&report.data, &rx_payload.data, 16); // all data can be passed through
				if (rx_payload.data[0] != 1) // packet 1 is full precision quat and accel, no room for rssi
					report.data[15]=rx_payload.rssi;
				// TODO: this sucks
				for (int i = 0; i < report_count; i++) { // replace existing entry instead
					if (reports[sizeof(report) * (report_sent + i) + 1] == report.data[1]) {
						memcpy(&reports[sizeof(report) * (report_sent + i)], &report, sizeof(report));
//						k_work_submit(&report_send);
						break;
					}
				}
				if (report_count > 100) // overflow
					break;
				memcpy(&reports[sizeof(report) * (report_sent + report_count)], &report, sizeof(report));
				report_count++;
//				k_work_submit(&report_send);
				break;
			default:
				break;
			}
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
	int fetch_attempts = 0;

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
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

uint8_t base_addr_0[4] = {0,0,0,0};
uint8_t base_addr_1[4] = {0,0,0,0};
uint8_t addr_prefix[8] = {0,0,0,0,0,0,0,0};

int esb_initialize_tx(void)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	// config.mode = ESB_MODE_PTX;
	config.event_handler = event_handler;
	// config.bitrate = ESB_BITRATE_2MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 8;
	// config.retransmit_delay = 600;
	config.retransmit_count = 0;
	config.tx_mode = ESB_TXMODE_MANUAL;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

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

int esb_initialize(void)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	// config.bitrate = ESB_BITRATE_2MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 8;
	// config.retransmit_delay = 600;
	// config.retransmit_count = 3;
	// config.tx_mode = ESB_TXMODE_AUTO;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

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
//#define REPORT_PERIOD		K_SECONDS(5)
#define REPORT_PERIOD		K_MSEC(1) // streaming reports

static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),
	HID_END_COLLECTION,
};

uint16_t sent_device_addr = 0;
bool usb_enabled = false;
int64_t last_registration_sent = 0;

static void send_report(struct k_work *work)
{
	if (!usb_enabled) return;
	if (!stored_trackers) return;
	if (report_count == 0 && k_uptime_get() - 100 < last_registration_sent) return; // send registrations only every 100ms
	int ret, wrote;

	last_registration_sent = k_uptime_get();

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		// TODO: this really sucks, how can i send as much or as little as i want instead??
//		for (int i = report_count; i < 4; i++) memcpy(&reports[sizeof(report) * (report_sent+i)], &reports[sizeof(report) * report_sent], sizeof(report)); // just duplicate first entry a bunch, this will definitely cause problems
		// cycle through devices and send associated address for server to register
		for (int i = report_count; i < 4; i++) {
			packet_device_addr(&reports[sizeof(report) * (report_sent + i)], sent_device_addr);
			sent_device_addr++;
			sent_device_addr %= stored_trackers;
		}
//		ret = hid_int_ep_write(hdev, &reports, sizeof(report) * report_count, &wrote);
		ret = hid_int_ep_write(hdev, &reports[sizeof(report) * report_sent], sizeof(report) * 4, &wrote);
		if (report_count > 4) {
			LOG_INF("left %u reports on the table", report_count - 4);
		}
		report_sent += report_count;
		report_sent += 3; // this is a hack to make sure the ep isnt reading the same bits as trackers write to
		if (report_sent > 128) report_sent = 0; // an attempt to make ringbuffer so the ep isnt reading the same bits as trackers write to
		report_count = 0;
		if (ret != 0) {
			/*
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		} else {
			//LOG_DBG("Report submitted");
		}
	} else { // busy with what
		//LOG_DBG("HID IN endpoint busy");
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
	if (usb_enabled)
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

static int composite_pre_init()
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

uint16_t led_clock = 0;

static void timer_handler(nrf_timer_event_t event_type, void *p_context) {
	if (event_type == NRF_TIMER_EVENT_COMPARE0) {
		//esb_write_payload(&tx_payload_sync);
		esb_start_tx();
	} else if (event_type == NRF_TIMER_EVENT_COMPARE1) {
		esb_stop_rx();
		esb_disable();
		esb_initialize_tx();
		esb_write_payload(&tx_payload_sync);
	} else if (event_type == NRF_TIMER_EVENT_COMPARE2) {
		esb_disable();
		esb_initialize();
		esb_start_rx();
		led_clock++;
		led_clock%=17*600/3;
		tx_payload_sync.data[0]=(led_clock >> 8) & 255;
		tx_payload_sync.data[1]=led_clock & 255;
	}
}

void timer_init(void) {
    //nrfx_err_t err;
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);
	//timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    //timer_cfg.mode = NRF_TIMER_MODE_TIMER;
    //timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
    //timer_cfg.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
    //timer_cfg.p_context = NULL;
	nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
    uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 3);
    nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true); // timeslot to send sync
    nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks * 20 / 21, true); // switch to tx
    nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL2, ticks * 1 / 21, true); // switch to rx
    nrfx_timer_enable(&m_timer);
	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);
}

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;
int64_t press_time;
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (gpio_pin_get_dt(&button0))
	{
		press_time = k_uptime_get();
	}
	else
	{
		if (press_time != 0 && k_uptime_get() - press_time > 50) // Debounce
			sys_reboot(SYS_REBOOT_COLD); // treat like pin reset but without pin reset reason
		press_time = 0;
	}
}

bool button_init;
void sys_button_init(void)
{
#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	if (!button_init)
	{
		gpio_pin_configure_dt(&button0, GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
		gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
		gpio_add_callback(button0.port, &button_cb_data);
		button_init = true;
	}
#endif
}

void button_thread(void)
{
	sys_button_init();
	while (1)
	{
		k_msleep(10);
		if (press_time != 0 && k_uptime_get() - press_time > 50 && gpio_pin_get_dt(&button0)) // Button is being pressed
			sys_reboot(SYS_REBOOT_COLD);
	}
}

K_THREAD_DEFINE(button_thread_id, 256, button_thread, NULL, NULL, NULL, 6, 0, 0);
#endif

void usb_init_thread(void)
{
	k_msleep(1000);
	usb_disable();
	k_msleep(1000); // Wait before enabling USB // TODO: why does it need to wait so long
	usb_enable(status_cb);
	k_work_init(&report_send, send_report);
	usb_enabled = true;
}

K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

void console_thread(void)
{
//	console_init();
	console_getline_init();
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk("reboot                       Soft reset the device\n");
	printk("pair                         Enter pairing mode\n");
	printk("clear                        Clear stored devices\n");
	printk("dfu                          Enter DFU bootloader\n");

	uint8_t command_reboot[] = "reboot";
	uint8_t command_pair[] = "pair";
	uint8_t command_clear[] = "clear";
	uint8_t command_dfu[] = "dfu";

	uint8_t reboot_counter = 0;

	while (1) {
		uint8_t *line = console_getline();
		for (uint8_t *p = line; *p; ++p) {
			*p = tolower(*p);
		}

		if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0) {
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0) {
			reboot_counter = 1;
			nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (memcmp(line, command_clear, sizeof(command_clear)) == 0) {
			reboot_counter = 2;
			nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0) {
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
		}
		else {
			printk("Unknown command\n");
		}
	}
}

K_THREAD_DEFINE(console_thread_id, 512, console_thread, NULL, NULL, NULL, 6, 0, 0);

static int nvs_init(void)
{
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
	nvs_mount(&fs);
	return 0;
}

SYS_INIT(nvs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

uint8_t reset_mode = 0;
int main(void)
{
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;

	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 1); // Boot LED

	// TODO: change pin reset to using memory, so there is less delay between resets

#if CONFIG_BUILD_OUTPUT_UF2 // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	ram_range_retain(dbl_reset_mem, sizeof(dbl_reset_mem), true);
#endif

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	sys_button_init();
	reset_reason |= gpio_pin_get_dt(&button0);
#endif

	//int64_t time_begin = k_uptime_get();
	nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	reset_mode = reboot_counter;
	if (reset_reason & 0x01) { // Count pin resets
		reboot_counter++;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		k_msleep(1000); // Wait before clearing counter and continuing
	}
	reboot_counter = 0;
	nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));

	if (reset_mode == 2) { // Clear stored data
		reset_mode = 1; // Enter pairing mode
		nvs_write(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		LOG_INF("NVS Reset");
	} else {
		nvs_read(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		for (int i = 0; i < stored_trackers; i++) {
			nvs_read(&fs, STORED_ADDR_0+i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
		}
		LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);
	}

#if CONFIG_BUILD_OUTPUT_UF2 // Using Adafruit bootloader
	if (reset_mode >= 3) { // DFU_MAGIC_UF2_RESET, Reset mode DFU
		NRF_POWER->GPREGRET = 0x57;
		sys_reboot(SYS_REBOOT_COLD);
	}
#endif

	gpio_pin_set_dt(&led, 0);

	clocks_start();


	if (stored_trackers == 0 || reset_mode == 1) { // Pairing mode
		reset_mode = 0;
		LOG_INF("Starting in pairing mode");
		for (int i = 0; i < 4; i++) {
			base_addr_0[i] = discovery_base_addr_0[i];
			base_addr_1[i] = discovery_base_addr_1[i];
		}
		for (int i = 0; i < 8; i++) {
			addr_prefix[i] = discovery_addr_prefix[i];
		}
		esb_initialize();
		esb_start_rx();
		tx_payload_pair.noack = false;
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		memcpy(&tx_payload_pair.data[2], addr, 6);
		LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
		while (true) { // Run indefinitely (User must reset/unplug dongle)
			uint64_t found_addr = (*(uint64_t *)pairing_buf >> 16) & 0xFFFFFFFFFFFF;
			uint16_t send_tracker_id = stored_trackers; // Use new tracker id
			for (int i = 0; i < stored_trackers; i++) { // Check if the device is already stored
				if (found_addr != 0 && stored_tracker_addr[i] == found_addr) {
					//LOG_INF("Found device linked to id %d with address %012llX", i, found_addr);
					send_tracker_id = i;
				}
			}
			uint8_t checksum = crc8_ccitt(0x07, &pairing_buf[2], 6); // make sure the packet is valid
			if (checksum == 0)
				checksum = 8;
			if (checksum == pairing_buf[0] && found_addr != 0 && send_tracker_id == stored_trackers && stored_trackers < MAX_TRACKERS) { // New device, add to NVS
				LOG_INF("Added device on id %d with address %012llX", stored_trackers, found_addr);
				stored_tracker_addr[stored_trackers] = found_addr;
				nvs_write(&fs, STORED_ADDR_0+stored_trackers, &stored_tracker_addr[stored_trackers], sizeof(stored_tracker_addr[0]));
				stored_trackers++;
				nvs_write(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
			}
			if (checksum == pairing_buf[0] && send_tracker_id < MAX_TRACKERS) { // Make sure the dongle is not full
				tx_payload_pair.data[0] = pairing_buf[0]; // Use checksum sent from device to make sure packet is for that device
			} else {
				tx_payload_pair.data[0] = 0; // Invalidate packet
			}
			tx_payload_pair.data[1] = send_tracker_id; // Add tracker id to packet
			//esb_flush_rx();
			//esb_flush_tx();
			//esb_write_payload(&tx_payload_pair); // Add to TX buffer
			if (blink == 0) {
				gpio_pin_set_dt(&led, 1);
				k_msleep(100);
				gpio_pin_set_dt(&led, 0);
				k_msleep(400);
			}
			else
				k_msleep(500);
			blink++;
			blink %= 2;
		}
	}

	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[8] = {0,0,0,0,0,0,0,0};
	for (int i = 0; i < 6; i++) {
		buf[i+2] = (*addr >> (8 * i)) & 0xFF;
	}
	uint8_t buf2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	for (int i = 0; i < 4; i++) {
		buf2[i] = buf[i+2];
		buf2[i+4] = buf[i+2] + buf[6];
	}
	for (int i = 0; i < 8; i++) {
		buf2[i+8] = buf[7] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (buf2[i] == 0x00 || buf2[i] == 0x55 || buf2[i] == 0xAA) {
			buf2[i] += 8;
		};
	}
	for (int i = 0; i < 4; i++) {
		base_addr_0[i] = buf2[i];
		base_addr_1[i] = buf2[i+4];
	}
	for (int i = 0; i < 8; i++) {
		addr_prefix[i] = buf2[i+8];
	}

	int err;

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}

	tx_payload_timer.noack = true;
	tx_payload_sync.noack = true;

	//timer_init();

	memset(discovered_trackers, 0, sizeof(discovered_trackers));

	while (true) { // this should be a timer but lazy; reset count if its not above threshold
		if (blink == 0) {
			gpio_pin_set_dt(&led, 1);
			k_msleep(300);
			gpio_pin_set_dt(&led, 0);
			k_msleep(700);
		}
		else
			k_msleep(1000);
		blink++;
		blink %= 10;
		for (int i = 0; i < 256; i++) {
			if (discovered_trackers[i] < DETECTION_THRESHOLD) {
				discovered_trackers[i] = 0;
			}
		}
	}

	/* return to idle thread */
	return 0;
}