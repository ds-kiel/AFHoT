#define HISTORY_SIZE 20
#define CHM_UPDATE_INTERVAL K_SECONDS(1)

extern enum strategy_type strat_type;
extern enum strategy_algo strat_algo;
extern bool afh_enabled;

enum data_type {data_type_rssi, data_type_pdr};
enum strategy_type {rssi, pdr, rssi_and_pdr};
enum strategy_algo {threshold, best_n, average, weighted_average, normal_dist};
enum pdr_action {ack, send};
struct upd_data
{
    enum data_type data_type;
    union
    {
        u8_t rssi;
        enum pdr_action pdr_action;
    } data;
};

struct data_history
{
    u8_t current_index;
    u8_t entry_count;
    u8_t updated;
    u8_t history[HISTORY_SIZE];
};

struct pdr_stat
{
  int sent;
  int acks;
};

struct upd_conn_chm_work {
    struct k_work work;
    u16_t handle;
};

struct timer_data {
	struct k_timer timer;
	struct k_work work_timer_expired;
	struct k_work work_timer_stopped;
	u16_t handle;
};

struct timer_data *get_timer_data(u16_t handle);
void upd_conn_stat(u16_t handle, u32_t channel, struct upd_data *upd_data);
void upd_conn_chm(u16_t handle);
void handle_timer(struct k_timer *timer);
void handle_timer_stop(struct k_timer *timer);
void init_conn_stats(u16_t handle);
void reset_conn_stats(u16_t handle);