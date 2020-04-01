# AFHoT
AFHoT - an open source AFH implementation for Zephyr's Bluetooth Low Energy stack

# Documentation
The main module is in the **ull_afh.c**. It is located here:
zephyr/subsys/bluetooth/controller/ll_sw/ull_afh.c

Data collection is in the **lll_conn.c**:
zephyr/subsys/bluetooth/controller/ll_sw/nordic/lll/lll_conn.c

For that, we call the upd_conn_stat function.

In the **ull_master.c**, we initialize the timer:
zephyr/subsys/bluetooth/controller/ll_sw/ull_master.c

Also, we changed the ll_conn_chm_update function here to support connection specific channel maps.

Our throughput and latency testing is here:
zephyr/tests/bluetooth/afh_eval_throughput/
zephyr/tests/bluetooth/afh_eval_latency/