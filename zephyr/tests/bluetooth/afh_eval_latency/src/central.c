/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#define ITERATIONS 500

static struct bt_conn *default_conn;
#define CHAR_SIZE_MAX           512
#define BT_EVAL_UUID BT_UUID_DECLARE_128( 0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12)
#define BT_EVAL_UUID_TEST BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3))


static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

bool test_ready = false;

u32_t start_time;
u32_t stop_time;
u32_t cycles_spent;
u32_t nanoseconds_spent;

u16_t service_handle;

static int cmd_write(u16_t handle, char* data, int len);
static int cmd_read(u16_t handle);
static struct bt_gatt_read_params read_params;

static u8_t read_func(struct bt_conn *conn, u8_t err,
			 struct bt_gatt_read_params *params,
			 const void *data, u16_t length)
{
	// for( int i = 0; i < length; i++) {
	// 	printk("%c", ((u8_t*)data)[i]);
	// }
	// printk("\n");
	if (!data) {
    stop_time = k_cycle_get_32();
    cycles_spent = stop_time - start_time;
    nanoseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
    // printk("Nanoseconds for read: %uns\n", nanoseconds_spent);
	printk("\rSeconds for read: %d,%ds", nanoseconds_spent / 1000000000, nanoseconds_spent % 1000000000);
		(void)memset(params, 0, sizeof(*params));
   
		// cmd_write(service_handle, vnd_long_value, 456);
		return BT_GATT_ITER_STOP;
	}
	return BT_GATT_ITER_CONTINUE;
}

static int cmd_read(u16_t handle)
{
	int err;

	if (!default_conn) {
		printk("Not connected");
		return -ENOEXEC;
	}

	read_params.func = read_func;
	read_params.handle_count = 1;
	read_params.single.handle = handle;
	read_params.single.offset = 0U;

  start_time = k_cycle_get_32();
	err = bt_gatt_read(default_conn, &read_params);
	if (err) {
		printk("Read failed (err %d)", err);
	} else {
		// printk("Read pending");
	}

	return err;
}

static struct bt_gatt_write_params write_params;
static u8_t gatt_write_buf[CHAR_SIZE_MAX];

static void write_func(struct bt_conn *conn, u8_t err,
		       struct bt_gatt_write_params *params)
{
  	stop_time = k_cycle_get_32();
	u32_t end_ms = k_uptime_get_32();
  	cycles_spent = stop_time - start_time;
  	nanoseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
  	// printk("Sent data in %u ys\n", nanoseconds_spent / 1000);
	printk("aaaaaaaaa%u,%u\n", end_ms, nanoseconds_spent);
	test_ready = true;
//   printk("\rNanoseconds for write: %uns", nanoseconds_spent);
	// printk("\rSeconds for write: %d,%ds", nanoseconds_spent / 1000000000, nanoseconds_spent % 1000000000);
	// printk("Write complete");
	// (void)memset(&write_params, 0, sizeof(write_params));
	// cmd_read(service_handle);
}

static int test_loop(u16_t handle, int iterations)
{
	int err;

	if (!default_conn) {
		printk("Not connected");
		return -ENOEXEC;
	}

	// if (write_params.func) {
	// 	// printk("Write ongoing");
	// 	return -ENOEXEC;
	// }

	gatt_write_buf[0] = 42;
	
	write_params.data = gatt_write_buf;
	write_params.length = 1;
	write_params.handle = handle;
	write_params.offset = 0;
	write_params.func = write_func;

  	start_time = k_cycle_get_32();
	u32_t start_ms = k_uptime_get_32();
	// struct bt_conn_info info;
	// bt_conn_get_info(default_conn, &info);
	// printk("Interval: %u\nLatency: %u\n", info.le.interval, info.le.latency); 
		// printk("ITERATION: %d\n", i);
	err = bt_gatt_write(default_conn, &write_params);
	if (err) {
		printk("Write failed (err %d)", err);
	}

	// err = bt_gatt_write(default_conn, &write_params);
	// stop_time = k_cycle_get_32();
  	// cycles_spent = stop_time - start_time;
  	// nanoseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);

	// printk("Sent data in %u ys\n", nanoseconds_spent / 1000);
	return err;
}


static u8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, u16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u\n", data, length);

	return BT_GATT_ITER_CONTINUE;
}

static u8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	char str[37];

	// bt_uuid_to_str(attr->uuid, str, sizeof(str));
	// printk("UUID: %s \n", str);
	// printk("Handle: %u \n", attr->handle);
	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	// printk("In discover\n");
	

	// char str[37];

	bt_uuid_to_str(attr->uuid, str, sizeof(str));
	printk("%s", str);
	if (0 == strcmp(str, "22345678-1234-5678-1234-56789abcdef3")) {
		// if(attr->handle == 42) {
		printk("Found Service!\n");

		service_handle = attr->handle;

		return BT_GATT_ITER_STOP;
	}

	discover_params.start_handle = attr->handle + 1;

	return BT_GATT_ITER_CONTINUE;
}

static void exchange_func(struct bt_conn *conn, u8_t err,
			  struct bt_gatt_exchange_params *params)
{
	printk("Exchange done\n");
	test_ready = true;
}
struct bt_gatt_exchange_params exchange_params;
static void connected(struct bt_conn *conn, u8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);
		return;
	}

	printk("Connected: %s\n", addr);

	if (conn == default_conn) {
		memcpy(&uuid, BT_EVAL_UUID_TEST, sizeof(uuid));
		// discover_params.uuid = &uuid.uuid;
		discover_params.func = discover_func;
		discover_params.start_handle = 0x0001;
		discover_params.end_handle = 0xffff;
		discover_params.type = BT_GATT_DISCOVER_ATTRIBUTE;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("Discover failed(err %d)\n", err);
			return;
		} else {
			
			exchange_params.func = exchange_func;
			printk("Before exchange\n");
			err = bt_gatt_exchange_mtu(default_conn, &exchange_params);
			if(err) {
				printk("Error Code for exchange: %d\n", err);
			}
			// cmd_write(service_handle, vnd_long_value, sizeof(vnd_long_value));
		}
	}
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;

	printk("[AD]: %u data_len %u\n", data->type, data->data_len);

			u16_t u16;
			int err;

			err = bt_le_scan_stop();
			if (err) {
				printk("Stop LE scan failed (err %d)\n", err);
				// continue;
			}

			// default_conn = bt_conn_create_le(addr,
							//  BT_LE_CONN_PARAM_DEFAULT);
			default_conn = bt_conn_create_le(addr,
							 BT_LE_CONN_PARAM(0x08, \
						  0x08, \
						  0, 400));
			printk("Connected\n");
			return false;

	return true;
}

static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
			 struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	// printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	//        dev, type, ad->len, rssi);

	/* We're only interested in connectable events */
	if ((strcmp(dev, "f0:9d:fa:8f:55:91 (random)") == 0) && (type == BT_LE_ADV_IND || type == BT_LE_ADV_DIRECT_IND)) {
		printk("Starting data parse\n");
		bt_data_parse(ad, eir_found, (void *)addr);
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param) {
	printk("LE conn  param req: int (0x%04x, 0x%04x) lat %d to %d\n", param->interval_min, param->interval_max,
		    param->latency, param->timeout);
	return false;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req
};

static void test() {
	test_ready = false;
	// printk("Test started\n");
	// printk("Service handle: %u\n", service_handle);

	test_loop(service_handle, ITERATIONS);
}

void central() {
  // while(true) {
	// 	printk("LALALA");
	// }
	// k_sleep(5000);
	int err;
	// err = bt_enable(NULL);

	// if (err) {
	// 	printk("Bluetooth init failed (err %d)\n", err);
	// 	return;
	// }

	printk("Bluetooth initialized\n");

	
	bt_conn_cb_register(&conn_callbacks);
	// int err;
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);

	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");

	while(true) {
		if(test_ready && service_handle != 0) {
			// printk("In testing\n");
			test();
			k_sleep(9);
		}
		k_sleep(1);
	}
	printk("Test finished\n");
}


