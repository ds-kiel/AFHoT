#include <zephyr.h>
#include <sys/printk.h>
#include <bluetooth/bluetooth.h>
#include "peripheral.h"
#include "central.h"

int is_central;
bool bt_enabled;

void cb_bt_enabled(int err) {
	bt_enabled = true;
	while (is_central == -2) {
		k_sleep(10);
	}
	if (is_central == 0) {
		bt_ready(err);
	}
}

void main(void) {
  is_central = -2;
  bt_enabled = false;
	k_sleep(10000);
	int err = bt_enable(cb_bt_enabled);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	} else {
		printk("Bluetooth initialized\n");
	}

	printk("Before bt enabled\n");

	while (!bt_enabled) {
		k_sleep(10);
	}

	printk("After bt enabled\n");

	bt_addr_le_t addr_array[CONFIG_BT_ID_MAX];
	size_t size = CONFIG_BT_ID_MAX;
	bt_id_get(addr_array, &size);
	for (int i = 0; i < size; i++) {
		char dev[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(&addr_array[i], dev, sizeof(dev));
		if (strcmp(dev, "ee:96:32:de:da:e9 (random)") == 0) {
			printk("I am central!\n");
			is_central = 1;
		}
		if (strcmp(dev, "f0:9d:fa:8f:55:91 (random)") == 0) {
			printk("I am peripheral!\n");
			is_central = 0;
		}
		printk("%s\n", dev);
	}
	if (is_central == 1) {
		central();
	} else if (is_central == 0) {
		peripheral();
	} else {
		is_central = -1;
		printk("I succ!\n");
	}
}