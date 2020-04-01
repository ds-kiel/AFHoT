/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <bluetooth/bluetooth.h>

void main(void)
{
	k_sleep(10000);
	while(true) {
	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	bt_addr_le_t addr_array[CONFIG_BT_ID_MAX];
	size_t size = CONFIG_BT_ID_MAX;
	bt_id_get(addr_array, &size);
	for(int i = 0; i < size; i++) {
		char dev[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(&addr_array[i], dev, sizeof(dev));
		printf("%s\n", dev);
	}
	}
}
