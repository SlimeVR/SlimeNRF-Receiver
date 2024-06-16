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

#include <zephyr/sys/reboot.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define STORED_TRACKERS 1
#define STORED_TRACKER_ADDR 2

#define RBT_CNT_ID 2
#define STORED_ADDR_0 3
// 0-15 -> id 3-18

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define pi 3.141592653589793238462643383279502884f

#define INT16_TO_UINT16(x) ((uint16_t)32768 + (uint16_t)(x))
#define UINT16_TO_INT16(x) (int16_t)((uint16_t)(x) - (uint16_t)32768)
#define TO_FIXED_14(x) ((int16_t)((x) * (1 << 14)))
#define TO_FIXED_10(x) ((int16_t)((x) * (1 << 10)))
#define FIXED_14_TO_DOUBLE(x) (((double)(x)) / (1 << 14))
#define FIXED_10_TO_DOUBLE(x) (((double)(x)) / (1 << 10))

LOG_MODULE_REGISTER(esb_prx, 4);

uint8_t stored_trackers = 0;
uint64_t stored_tracker_addr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t discovered_trackers[256];

uint8_t pairing_buf[8] = {0,0,0,0,0,0,0,0};

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
	uint8_t type; //reserved
	uint8_t imu_id;
	uint8_t rssi;
	uint8_t battery;
	uint16_t batt_mV;
	uint16_t qi;
	uint16_t qj;
	uint16_t qk;
	uint16_t ql;
	uint16_t ax;
	uint16_t ay;
	uint16_t az;
} __packed report = {
	.type = 0,
	.imu_id = 0,
	.rssi = 0,
	.battery = 0,
	.batt_mV = 0,
	.qi = 0,
	.qj = 0,
	.qk = 0,
	.ql = 0,
	.ax = 0,
	.ay = 0,
	.az = 0
};;

uint8_t reports[256*sizeof(report)];
uint8_t report_count = 0;
uint8_t report_sent = 0;

int blink = 0;

#include <nrfx_timer.h>
static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		//LOG_DBG("TX SUCCESS");
		break;
	case ESB_EVENT_TX_FAILED:
		break;
	case ESB_EVENT_RX_RECEIVED:
	// make tx payload for ack here
		if (esb_read_rx_payload(&rx_payload) == 0) {
			if (rx_payload.length == 8) {
				LOG_INF("RX Pairing Packet");
				for (int i = 0; i < 8; i++) {
					pairing_buf[i] = rx_payload.data[i];
				}
				esb_write_payload(&tx_payload_pair); // Add to TX buffer
			} else {
//				int32_t cc_timer = nrfx_timer_capture(&m_timer, NRF_TIMER_CC_CHANNEL3);
//				uint8_t id = (rx_payload.data[1] >> 4) & 15;
//				//tx_payload_timer.data[0] = stored_tracker_addr[id] & 255;
//				//tx_payload_timer.data[1] = id;
//				//tx_payload_timer.data[2] = (cc_timer >> 8) & 255;
//				//tx_payload_timer.data[3] = cc_timer & 255;
//				uint32_t length = nrfx_timer_ms_to_ticks(&m_timer, 3);
//				int32_t offset = length * id / 16;
//				if (id == 8) {
//					offset = 2333;
//				} else {
//					offset = 667;
//				}
//				int32_t correction = (offset - cc_timer) / 2;
//				LOG_INF("ID %u, Trigger %ld", id, cc_timer);
//				int8_t out;
//				if (correction >= -64 && correction <= 64) {
//					out = correction;
//				} else if (correction > 0) {
//					out = 64;
//					out = correction / 32 + 62;
//				} else {
//					out = -64;
//					out = correction / 32 - 62;
//				}
//				tx_payload_timer.data[id+1] = (uint8_t)out + 127;
//				esb_write_payload(&tx_payload_timer);
				// instead of making specific ack packet for all devices maybe instead have generic timestamp?
				// or send same ack packet which contains all necessary data for every device?
				// or use pipes and contain data for two devices, but you will need a bigger tx buffer..
				//report.type=rx_payload.data[0]; // for later
				uint8_t imu_id = rx_payload.data[1];
				if (discovered_trackers[imu_id] < 16) { // garbage filtering of nonexistant tracker
					discovered_trackers[imu_id]++;
					break;
				}
				report.imu_id=imu_id;
				report.battery=rx_payload.data[2];
				report.batt_mV=((int)rx_payload.data[3] + 245) * 10;
				report.rssi=rx_payload.rssi;
				report.qi=(((uint16_t)rx_payload.data[4] << 8) | rx_payload.data[5]);
				report.qj=(((uint16_t)rx_payload.data[6] << 8) | rx_payload.data[7]);
				report.qk=(((uint16_t)rx_payload.data[8] << 8) | rx_payload.data[9]);
				report.ql=(((uint16_t)rx_payload.data[10] << 8) | rx_payload.data[11]);
				report.ax=(((uint16_t)rx_payload.data[12] << 8) | rx_payload.data[13]);
				report.ay=(((uint16_t)rx_payload.data[14] << 8) | rx_payload.data[15]);
				report.az=(((uint16_t)rx_payload.data[16] << 8) | rx_payload.data[17]);
				// TODO: this sucks
				for (int i = 0; i < report_count; i++) { // replace existing entry instead
					if (reports[sizeof(report) * (report_sent+i) + 1] == imu_id) {
						memcpy(&reports[sizeof(report) * (report_sent+i)], &report, sizeof(report));
						k_work_submit(&report_send);
						break;
					}
				}
				memcpy(&reports[sizeof(report) * (report_sent+report_count)], &report, sizeof(report));
				report_count++;
				k_work_submit(&report_send);
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
#define REPORT_PERIOD		K_SECONDS(5)

static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);

static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(60),
		HID_INPUT(0x02),
	HID_END_COLLECTION,
};

static void send_report(struct k_work *work)
{
	if (report_count == 0) return;
	int ret, wrote;

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		// TODO: this really sucks, how can i send as much or as little as i want instead??
		for (int i = report_count; i < 3; i++) memcpy(&reports[sizeof(report) * (report_sent+i)], &reports[sizeof(report) * report_sent], sizeof(report)); // just duplicate first entry a bunch, this will definitely cause problems
//		ret = hid_int_ep_write(hdev, &reports, sizeof(report) * report_count, &wrote);
		ret = hid_int_ep_write(hdev, &reports[sizeof(report) * report_sent], sizeof(report) * 3, &wrote);
		if (report_count > 3) {
			LOG_INF("left %u reports on the table", report_count - 3);
		}
		report_sent += report_count;
		report_sent += 2;
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
static struct gpio_callback button_cb_data;
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	sys_reboot(SYS_REBOOT_COLD); // treat like pin reset but without pin reset reason
}
#endif

uint8_t reset_mode = 0;
int main(void)
{
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;

	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 1); // Boot LED

	// TODO: change pin reset to using memory, so there is less delay between resets
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
	nvs_mount(&fs);

#if CONFIG_BOARD_SUPERMINI|CONFIG_BOARD_ETEE_DONGLE // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
#endif

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
	const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	reset_reason |= gpio_pin_get_dt(&button0);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
#endif

	//int64_t time_begin = k_uptime_get();
	if (reset_reason & 0x01) { // Count pin resets
		nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		reset_mode = reboot_counter;
		reboot_counter++;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter = 0;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	}

	if (reset_mode == 2) { // Clear stored data
		reset_mode = 1; // Enter pairing mode
		nvs_write(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		for (int i = 0; i < 16; i++) {
			nvs_write(&fs, STORED_ADDR_0+i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
		}
		LOG_INF("NVS Reset");
	} else {
		nvs_read(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		for (int i = 0; i < 16; i++) {
			nvs_read(&fs, STORED_ADDR_0+i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
		}
		LOG_INF("%d devices stored", stored_trackers);
	}

#if CONFIG_BOARD_SUPERMINI|CONFIG_BOARD_ETEE_DONGLE // Using Adafruit bootloader
	if (reset_mode >= 3) { // DFU_MAGIC_UF2_RESET, Reset mode DFU
		NRF_POWER->GPREGRET = 0x57;
		sys_reboot(SYS_REBOOT_COLD);
	}
#endif

	gpio_pin_set_dt(&led, 0);

	usb_enable(status_cb);

	k_work_init(&report_send, send_report);

	clocks_start();

	if (stored_trackers == 0 || reset_mode == 1) { // Pairing mode
		reset_mode = 0;
		LOG_INF("Starting in pairing mode");
		for (int i = 0; i < stored_trackers; i++) {
			report.imu_id = i << 4;
			k_work_submit(&report_send);
		}
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
		uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF;
		LOG_INF("Device address %lld", addr);
		for (int i = 0; i < 6; i++) {
			tx_payload_pair.data[i+2] = (addr >> (8 * i)) & 0xFF;
		}
		while (true) { // Run indefinitely (User must reset/unplug dongle)
			uint64_t found_addr = 0;
			for (int i = 0; i < 6; i++) { // Take device address from RX buffer
				found_addr |= (uint64_t)pairing_buf[i+2] << (8*i);
			}
			uint16_t send_tracker_id = stored_trackers; // Use new tracker id
			for (int i = 0; i < stored_trackers; i++) { // Check if the device is already stored
				if (found_addr != 0 && stored_tracker_addr[i] == found_addr) {
					//LOG_INF("Found device linked to id %d with address %lld", i, found_addr);
					send_tracker_id = i;
				}
			}
			if (found_addr != 0 && send_tracker_id == stored_trackers && stored_trackers < 16) { // New device, add to NVS
				LOG_INF("Added device on id %d with address %lld", stored_trackers, found_addr);
				report.imu_id = stored_trackers << 4;
				stored_tracker_addr[stored_trackers] = found_addr;
				nvs_write(&fs, STORED_ADDR_0+stored_trackers, &stored_tracker_addr[stored_trackers], sizeof(stored_tracker_addr[0]));
				stored_trackers++;
				nvs_write(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
				k_work_submit(&report_send);
			}
			if (send_tracker_id < 16) { // Make sure the dongle is not full
				tx_payload_pair.data[0] = pairing_buf[0]; // Use int sent from device to make sure packet is for that device
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
	uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF;
	uint8_t buf[8] = {0,0,0,0,0,0,0,0};
	for (int i = 0; i < 6; i++) {
		buf[i+2] = (addr >> (8 * i)) & 0xFF;
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

	LOG_INF("Enhanced ShockBurst prx sample");

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}

	LOG_INF("Initialization complete");

	LOG_INF("Setting up for packet reception");

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}

	LOG_INF("Starting application");

	tx_payload_timer.noack = true;
	tx_payload_sync.noack = true;

	timer_init();

	for (int i = 0; i < 256; i++) {
		discovered_trackers[i] = 0;
	}

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
			if (discovered_trackers[i] < 16) {
				discovered_trackers[i] = 0;
			}
		}
	}

	/* return to idle thread */
	return 0;
}