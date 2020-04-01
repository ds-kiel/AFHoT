#include <zephyr.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>
#include <sys/ring_buffer.h>
#include <stdlib.h>

#include "util/util.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "hal/ticker.h"
#include "hal/ccm.h"
#include "ticker/ticker.h"

#include "pdu.h"
#include "ll.h"
#include "ll_feat.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_adv.h"
#include "lll_scan.h"
#include "lll_conn.h"
#include "lll_master.h"
#include "lll_filter.h"
#include "lll_tim_internal.h"

#include "ull_adv_types.h"
#include "ull_scan_types.h"
#include "ull_conn_types.h"
#include "ull_filter.h"

#include "ull_internal.h"
#include "ull_scan_internal.h"
#include "ull_conn_internal.h"
#include "ull_master_internal.h"
#include "ull_afh.h"

#define INIT_HISTORY(name)                                                     \
	struct data_history name = { .updated = 0U,                            \
				     .entry_count = 0,                         \
				     .current_index = 0 }

#define DATA_CHAN_COUNT 37
#define TESTING_CHAN_COUNT 1
#define MIN_HISTORY_ENTRIES 5
#define SUPPORTED_DATA_TYPES 2
#define PDR_THRESHOLD 45
#define RSSI_THRESHOLD 45
#define BEST_N_VAL 20

struct data_history rssi_stats[CONFIG_BT_MAX_CONN][DATA_CHAN_COUNT];
struct data_history pdr_stats[CONFIG_BT_MAX_CONN][DATA_CHAN_COUNT];
enum pdr_action pdr_last_actions[CONFIG_BT_MAX_CONN];

struct timer_data timer_data[CONFIG_BT_MAX_CONN];

enum strategy_type strat_type = rssi;
enum strategy_algo strat_algo = best_n;

bool afh_enabled = false;

struct timer_data *get_timer_data(u16_t handle)
{
	if (handle < CONFIG_BT_MAX_CONN) {
		return &timer_data[handle];
	} else {
		return NULL;
	}
}

u8_t get_history_entry(struct data_history *history, int index)
{
	int real_index = (history->current_index - index) % HISTORY_SIZE;
	return history->history[real_index];
}

void put_history_entry(struct data_history *history, u8_t entry)
{
	history->current_index = (history->current_index + 1) % HISTORY_SIZE;
	history->history[history->current_index] = entry;
	history->updated = 1U;
	if (history->entry_count < HISTORY_SIZE) {
		history->entry_count = history->entry_count + 1;
	}
}

void reset_history(struct data_history *history)
{
	history->entry_count = 0U;
	history->current_index = 0U;
}

void upd_conn_stat(u16_t handle, u32_t channel, struct upd_data *upd_data)
{
	struct data_history *rssi_history = &rssi_stats[handle][channel];
	struct data_history *pdr_history = &pdr_stats[handle][channel];
	switch (upd_data->data_type) {
	case data_type_rssi:
		put_history_entry(rssi_history, upd_data->data.rssi);
		break;
	case data_type_pdr:
		if (upd_data->data.pdr_action != pdr_last_actions[handle]) {
			put_history_entry(pdr_history, 1);
		} else {
			put_history_entry(pdr_history, 0);
		}
		pdr_last_actions[handle] = upd_data->data.pdr_action;
		break;
	default:
		printk("Invalid data_type supplied!");
		break;
	}
}

void set_chan_map_helper(u8_t *chan_map, u16_t handle)
{
	struct ll_conn *conn = ll_connected_get(handle);
	ll_conn_chm_update(chan_map, conn);
}

u8_t *transform_bool_to_chan_map(bool *channels, u8_t *chan_map)
{
	for (int i = 0; i < 5; i++) {
		chan_map[i] = 0;
	}
	// printk("Memset: %x %x %x %x %x\n", chan_map[0], chan_map[1],chan_map[2], chan_map[3],chan_map[4] );
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		if (channels[i]) {
			chan_map[i / 8] = chan_map[i / 8] | BIT(i % 8);
		}
	}
	// printk("Calculated channel map: %x %x %x %x %x\n", chan_map[0], chan_map[1],chan_map[2], chan_map[3],chan_map[4] );
	return chan_map;
}

void _calc_bool_map_rssi_threshhold(struct ll_conn *conn, u16_t handle,
				    bool *bool_map)
{
	struct data_history *channels = rssi_stats[handle];
	u8_t *old_chan_map = (u8_t *)&conn->conn_chan_map;
	struct data_history *history;
	int average;
	int entry_count;
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		history = &channels[i];

		if (history->entry_count < MIN_HISTORY_ENTRIES) {
			bool_map[i] = (old_chan_map[i / 8] & BIT(i % 8)) != 0;
			continue;
		}

		do {
			average = 0;
			entry_count = history->entry_count;
			history->updated = 0U;

			for (int j = 0; j < entry_count; j++) {
				average =
					average + get_history_entry(history, j);
			}

			if (entry_count > 0) {
				average = average / entry_count;
			}

			bool_map[i] = average < RSSI_THRESHOLD;

		} while (history->updated);
	}
}

static struct sort_struct {
	u8_t index;
	int value;
};

void swap(struct sort_struct *xp, struct sort_struct *yp)  
{  
    struct sort_struct temp = *xp;  
    *xp = *yp;  
    *yp = temp;  
}  
  
void selectionSort(struct sort_struct arr[], int n)  
{  
    int i, j, min_idx;  
  
    // One by one move boundary of unsorted subarray  
    for (i = 0; i < n-1; i++)  
    {  
        // Find the minimum element in unsorted array  
        min_idx = i;  
        for (j = i+1; j < n; j++)  
        if (arr[j].value < arr[min_idx].value)  
            min_idx = j;  
  
        // Swap the found minimum element with the first element  
        swap(&arr[min_idx], &arr[i]);  
    }  
}

void _calc_bool_map_rssi_best_n(struct ll_conn *conn, u16_t handle,
				bool *bool_map)
{
	struct data_history *channels = rssi_stats[handle];
	u8_t *old_chan_map = (u8_t *)&conn->conn_chan_map;
	struct data_history *history;
	int average;
	int entry_count;
	struct sort_struct averages[DATA_CHAN_COUNT];
	bool active = true;
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		history = &channels[i];

		if (history->entry_count < MIN_HISTORY_ENTRIES) {
			averages[i] =
				(struct sort_struct){ .index = i, .value = 1000 };
		} else {
		do {

			average = 0;
			entry_count = history->entry_count;
			history->updated = 0U;

			for (int j = 0; j < entry_count; j++) {
				average =
					average + get_history_entry(history, j);
			}

			if (entry_count > 0) {
				average = average / entry_count;
			}

			averages[i] = (struct sort_struct){ .index = i,
							    .value = average };

		} while (history->updated);
		}
	}

	selectionSort(averages, DATA_CHAN_COUNT);

	for (int i = 0; i < BEST_N_VAL; i++) {
		struct sort_struct item = averages[i];
		if (item.value != 1000) {
			bool_map[item.index] = 1;
		}
	}
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		if (averages[i].value == 1000) {
			bool_map[i] = (old_chan_map[i / 8] & BIT(i % 8)) != 0;
		}
	}
}

void _calc_bool_map_pdr_best_n(struct ll_conn *conn, u16_t handle,
			       bool *bool_map)
{
	struct data_history *historys = pdr_stats[handle];
	struct data_history *history;
	u8_t *old_chan_map = (u8_t *)&conn->conn_chan_map;
	int sent = 0;
	int acks = 0;
	struct sort_struct rates[DATA_CHAN_COUNT];
	int entry_count;
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		history = &historys[i];
		do {
			entry_count = history->entry_count;
			history->updated = 0U;
			if (entry_count < HISTORY_SIZE) {
				rates[i] = (struct sort_struct){ .index = i, .value = -1 };
			} else {
				acks = 0;
				for (int j = 0; j < entry_count; j++) {
					if (get_history_entry(history, j) ==
					    1) {
						acks++;
					}
				}
				sent = entry_count;
				int rate = (float) acks * 100.0 / (float)sent;
				rates[i] = (struct sort_struct){ .index = i,
						      .value = rate };
			}
		} while (history->updated);
	}
	selectionSort(rates, DATA_CHAN_COUNT);
	for (int i = 0; i < BEST_N_VAL; i++) {
		struct sort_struct item = rates[DATA_CHAN_COUNT - i - 1];
		if (item.value > -1) {
			bool_map[item.index] = 1;
		}
	}
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		if (rates[i].value == -1) {
      int index = rates[i].index;
			bool_map[index] = (old_chan_map[index / 8] & BIT(index % 8)) != 0;
		}
	}
}

void _calc_bool_map_pdr_threshold(struct ll_conn *conn, u16_t handle,
				  bool *bool_map)
{
	struct data_history *historys = pdr_stats[handle];
	struct data_history *history;
	u8_t *old_chan_map = (u8_t *)&conn->conn_chan_map;
	int sent = 0;
	int acks = 0;
	int entry_count;
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		history = &historys[i];
		do {
			entry_count = history->entry_count;
			history->updated = 0U;
			if (entry_count < HISTORY_SIZE) {
				bool_map[i] =
					(old_chan_map[i / 8] & BIT(i % 8)) != 0;
			} else {
				acks = 0;
				for (int j = 0; j < entry_count; j++) {
					if (get_history_entry(history, j) ==
					    1) {
						acks++;
					}
				}
				sent = entry_count;
				u32_t rate = (float)acks * 100 / (float)sent;
				bool_map[i] = rate >= PDR_THRESHOLD;
			}
		} while (history->updated);
	}
}

void include_testing_channels(bool *bool_map)
{
	int updated_channels = 0;
	int index;
	int disabled_chan_count = 0;
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		if (!bool_map[i]) {
			disabled_chan_count++;
		}
	}

	int channels_to_update = MIN(TESTING_CHAN_COUNT, disabled_chan_count);
	// int channels_to_update = 0;
	while (updated_channels < channels_to_update) {
		index = sys_rand32_get() % DATA_CHAN_COUNT;
		if (!bool_map[index]) {
			bool_map[index] = true;
			updated_channels++;
		}
	}
}

bool chan_map_equals(u8_t *chan_map_1, u8_t *chan_map_2)
{
	for (int i = 0; i < 5; i++) {
		if (chan_map_1[i] != chan_map_2[i]) {
			return false;
		}
	}
	return true;
}

void upd_conn_chm(u16_t handle)
{
	u8_t chan_map[5];
	bool bool_map[37] = { false };
	struct ll_conn *conn = ll_conn_get(handle);

	switch (strat_type) {
	case rssi:
		switch (strat_algo) {
		case threshold:
			_calc_bool_map_rssi_threshhold(conn, handle, bool_map);
			break;
		case best_n:
			_calc_bool_map_rssi_best_n(conn, handle, bool_map);
			break;
		case average:
			break;
		case weighted_average:
			break;
		case normal_dist:
			break;
		default:
			break;
		}
		break;
	case pdr:
		switch (strat_algo) {
		case threshold:
			_calc_bool_map_pdr_threshold(conn, handle, bool_map);
			break;
		case best_n:
			_calc_bool_map_pdr_best_n(conn, handle, bool_map);
			break;
		case average:
			break;
		case weighted_average:
			break;
		case normal_dist:
			break;
		default:
			break;
		}
		break;
	case rssi_and_pdr:
		switch (strat_algo) {
		case threshold:
			break;
		case best_n:
			break;
		case average:
			break;
		case weighted_average:
			break;
		case normal_dist:
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	// printk("upd_conn_chm_map\n");
	struct data_history *channels = rssi_stats[handle];

	include_testing_channels((bool *)&bool_map);
	transform_bool_to_chan_map(bool_map, chan_map);

	if (!chan_map_equals((u8_t *)&chan_map, (u8_t *)&conn->conn_chan_map)) {
		ll_conn_chm_update(chan_map, conn);
		printk("Channel map was set to: %x %x %x %x %x\n", chan_map[0],
		       chan_map[1], chan_map[2], chan_map[3], chan_map[4]);
	}

	struct data_history *rssi_historys = rssi_stats[handle];
	struct data_history *pdr_historys = pdr_stats[handle];
	/* Reset history of disabled channels */
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		if (!bool_map[i]) {
			reset_history(&rssi_historys[i]);
			reset_history(&pdr_historys[i]);
			// printk("Reset history for Channel %d\n", i);
		}
		// int sent = pdr_stats[handle][i].sent;
		// int ack = pdr_stats[handle][i].acks;
		// u32_t rate = (float)ack * 100 / (float)sent;
		// printk("Sent: %d, ACKs: %d, Rate: %d\n", sent, ack, rate) ;
	}
}

void reset_conn_stats(u16_t handle)
{
	printk("RESET CONN STATS \n");
	struct data_history *rssi_historys = rssi_stats[handle];
	struct data_history *pdr_historys = pdr_stats[handle];
	for (int i = 0; i < DATA_CHAN_COUNT; i++) {
		reset_history(&rssi_historys[i]);
		reset_history(&pdr_historys[i]);
	}
}

void work_handler(struct k_work *k_work)
{
	struct timer_data *timer_data =
		CONTAINER_OF(k_work, struct timer_data, work_timer_expired);
	upd_conn_chm(timer_data->handle);
}

void handle_timer(struct k_timer *timer)
{
	// printk("TIMER ABGELAUFEN");
	struct timer_data *timer_data =
		CONTAINER_OF(timer, struct timer_data, timer);
	struct k_work *work = &timer_data->work_timer_expired;
	k_work_init(work, work_handler);
	k_work_submit(work);
}

void work_handler_timer_stop(struct k_work *k_work)
{
	struct timer_data *timer_data =
		CONTAINER_OF(k_work, struct timer_data, work_timer_stopped);
	reset_conn_stats(timer_data->handle);
}

void handle_timer_stop(struct k_timer *timer)
{
	// u16_t handle = *(u16_t*) k_timer_user_data_get(timer);
	struct timer_data *timer_data =
		CONTAINER_OF(timer, struct timer_data, timer);
	struct k_work *work = &timer_data->work_timer_stopped;
	k_work_init(work, work_handler_timer_stop);
	k_work_submit(work);
}