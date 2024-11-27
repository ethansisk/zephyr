
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device_runtime.h>
#include <string.h>

#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/sys/atomic.h>

#include "gnss_nmea0183.h"
#include "gnss_nmea0183_match.h"
#include "gnss_parse.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(telit_mex10g1_gnss, CONFIG_GNSS_LOG_LEVEL);

#define GNSS_MODEM_NODE DT_ALIAS(modem)
MODEM_PIPELINK_DT_DECLARE(GNSS_MODEM_NODE, user_pipe_0);

struct telit_mex10g1_gnss_config {
	uint8_t x;
};

struct telit_mex10g1_data {
	struct k_work telit_mex10g1_gnss_open_pipe_work;
	struct k_work telit_mex10g1_gnss_attach_chat_work;
	struct k_work telit_mex10g1_gnss_release_chat_work;
	struct k_work_delayable telit_mex10g1_gnss_check_fix_work;

	struct modem_pipelink *telit_mex10g1_gnss_pipelink;

	struct modem_chat gnss_chat;
	uint8_t gnss_chat_rx_buf[1024];
	uint8_t *chat_argv[32];

	/* Pair chat script */
	uint8_t at_request_buf[32];
	uint8_t at_match_buf[32];
	struct modem_chat_match at_match;
	struct modem_chat_script_chat at_script_chat;
	struct modem_chat_script at_script;
};

static void modem_cellular_chat_on_gnss_acp(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	// if (argc != 2) {
	// 	return;
	// }

	LOG_INF("GPSACP response: ");
	for (int i = 0; i < argc; i++) {
		LOG_INF("%s", argv[i]);
	}
}

static void gnss_chat_callback_handler(struct modem_chat *chat,
						 enum modem_chat_script_result result,
						 void *user_data)
{
	struct telit_mex10g1_data *data = (struct telit_mex10g1_data *)(user_data);
	k_work_schedule(&data->telit_mex10g1_gnss_check_fix_work, K_SECONDS(10));
}

MODEM_CHAT_MATCH_DEFINE(ok_match, "OK", "", NULL);
MODEM_CHAT_MATCH_DEFINE(gnss_acp, "$GPSACP: ", ",", modem_cellular_chat_on_gnss_acp);
MODEM_CHAT_MATCHES_DEFINE(abort_matches, MODEM_CHAT_MATCH("ERROR", "", NULL));

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_mex10g1_gnss_check_fix_script_cmds,
MODEM_CHAT_SCRIPT_CMD_RESP("AT$GPSP?", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT$GPSACP", gnss_acp));

MODEM_CHAT_SCRIPT_DEFINE(telit_mex10g1_gnss_check_fix_script, telit_mex10g1_gnss_check_fix_script_cmds,
			 abort_matches, gnss_chat_callback_handler, 5);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_mex10g1_gnss_start_script_cmds,
	MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT#CEDRXS=2,4,\"1000\", \"0011\"", ok_match),
	// MODEM_CHAT_SCRIPT_CMD_RESP("AT$GNSSNMEA=1,8D", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT$GPSP=1", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(telit_mex10g1_gnss_start_script, telit_mex10g1_gnss_start_script_cmds,
			 abort_matches, gnss_chat_callback_handler, 5);

static void telit_mex10g1_gnss_check_fix_work_fn(struct k_work *work)
{
	struct k_work_delayable *d_work = (struct k_work_delayable *)(work);
	struct telit_mex10g1_data *data = CONTAINER_OF(d_work, struct telit_mex10g1_data, telit_mex10g1_gnss_check_fix_work);

	int err = modem_chat_run_script_async(&data->gnss_chat, &telit_mex10g1_gnss_check_fix_script);
	if (err) {
		LOG_ERR("Error %d running gps fix script", err);
	} else {
		LOG_INF("GPS fix script success");
	}
}

static void telit_mex10g1_gnss_attach_chat_handler(struct k_work *work)
{
	struct telit_mex10g1_data *data = CONTAINER_OF(work, struct telit_mex10g1_data, telit_mex10g1_gnss_attach_chat_work);

	int err = modem_chat_attach(&data->gnss_chat, modem_pipelink_get_pipe(data->telit_mex10g1_gnss_pipelink));
	if (err) {
		LOG_ERR("err %d, chat not attached", err);
	} else {
		LOG_INF("chat attached");
	}
	// atomic_set_bit(&at_shell_state, AT_SHELL_STATE_ATTACHED_BIT);

	err = modem_chat_run_script_async(&data->gnss_chat, &telit_mex10g1_gnss_start_script);
	if (err) {
		LOG_ERR("Error %d running gps start script", err);
	} else {
		LOG_INF("GPS start script success");
	}
}

static void telit_mex10g1_gnss_release_chat_handler(struct k_work *work)
{
	struct telit_mex10g1_data *data = CONTAINER_OF(work, struct telit_mex10g1_data, telit_mex10g1_gnss_release_chat_work);

	modem_chat_release(&data->gnss_chat);
	// atomic_clear_bit(&at_shell_state, AT_SHELL_STATE_ATTACHED_BIT);
	LOG_INF("chat released");
}

static void telit_mex10g1_gnss_pipe_callback(struct modem_pipe *pipe,
				   enum modem_pipe_event event,
				   void *user_data)
{
	struct telit_mex10g1_data *data = (struct telit_mex10g1_data *)(user_data);

	switch (event) {
	case MODEM_PIPE_EVENT_OPENED:
		LOG_INF("pipe opened");
		k_work_submit(&data->telit_mex10g1_gnss_attach_chat_work);
		break;

	default:
		break;
	}
}

static void telit_mex10g1_gnss_open_pipe_handler(struct k_work *work)
{
	struct telit_mex10g1_data *data = CONTAINER_OF(work, struct telit_mex10g1_data, telit_mex10g1_gnss_open_pipe_work);

	LOG_INF("opening pipe");

	modem_pipe_attach(modem_pipelink_get_pipe(data->telit_mex10g1_gnss_pipelink),
			  telit_mex10g1_gnss_pipe_callback,
			  data);

	modem_pipe_open_async(modem_pipelink_get_pipe(data->telit_mex10g1_gnss_pipelink));
}

void telit_mex10g1_gnss_pipelink_callback(struct modem_pipelink *link,
				enum modem_pipelink_event event,
				void *user_data)
{
	struct telit_mex10g1_data *data = (struct telit_mex10g1_data *)(user_data);

	switch (event) {
	case MODEM_PIPELINK_EVENT_CONNECTED:
		LOG_DBG("pipe connected");
		k_work_submit(&data->telit_mex10g1_gnss_open_pipe_work);
		break;

	case MODEM_PIPELINK_EVENT_DISCONNECTED:
		LOG_DBG("pipe disconnected");
		k_work_submit(&data->telit_mex10g1_gnss_release_chat_work);
		break;
	}
}

/** API for setting fix rate */
static int telit_mex10g1_set_fix_rate(const struct device *dev, uint32_t fix_interval_ms)
{
	return 0;
}

/** API for getting fix rate */
static int telit_mex10g1_get_fix_rate(const struct device *dev, uint32_t *fix_interval_ms)
{
	return 0;
}

/** API for setting navigation mode */
static int telit_mex10g1_set_nav_mode(const struct device *dev,
					  enum gnss_navigation_mode mode)
{
	return 0;
}

/** API for getting navigation mode */
static int telit_mex10g1_get_nav_mode(const struct device *dev,
					  enum gnss_navigation_mode *mode)
{
	return 0;
}

/** API for enabling systems */
static int telit_mex10g1_set_enabled_systems(const struct device *dev, gnss_systems_t systems)
{
	char sys_cfg = '0';
	if (systems & GNSS_SYSTEM_GLONASS) {
		sys_cfg = '1';
	} else if (systems & GNSS_SYSTEM_GALILEO) {
		sys_cfg = '2';
	} else if (systems & GNSS_SYSTEM_BEIDOU) {
		sys_cfg = '3';
	} else if (systems & GNSS_SYSTEM_QZSS) {
		sys_cfg = '4';
	} else if (!(systems & GNSS_SYSTEM_GPS)) {
		return -ENOTSUP;
	}

	char cfg_str[sizeof("AT$GPSCFG=2,") + 1];
	sprintf(cfg_str, "AT$GPSCFG=2,%c", sys_cfg);

	// MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_mex10g1_gnss_check_set_sys_cmds,
	// 	MODEM_CHAT_SCRIPT_CMD_RESP(cfg_str, ok_match));

	// MODEM_CHAT_SCRIPT_DEFINE(telit_mex10g1_gnss_check_set_sys_script, telit_mex10g1_gnss_check_set_sys_cmds,
	// 			abort_matches, gnss_chat_callback_handler, 1);
	
	// int err = modem_chat_run_script(&telit_mex10g1_gnss_chat, &telit_mex10g1_gnss_check_set_sys_script);
	// if (err) {
	// 	LOG_ERR("Err running set script");
	// }

	// return err;
	return 0;
}

/** API for getting enabled systems */
static gnss_systems_t curr_sys;
static void modem_cellular_chat_on_gpscfg(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	LOG_INF("$GPSCFG response: ");
	for (int i = 0; i < argc; i++) {
		LOG_INF("%s", argv[i]);
	}
	char *sys_char = argv[2];
	int sys_int = atoi(sys_char);
	switch (sys_int) {
		case 0:
			// based on MCC of camped network
			curr_sys = GNSS_SYSTEM_GPS;
			break;
		case 1:
			curr_sys = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS;
			break;
		case 2:
			curr_sys = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GALILEO;
			break;
		case 3:
			curr_sys = GNSS_SYSTEM_GPS | GNSS_SYSTEM_BEIDOU;
			break;
		case 4:
			curr_sys = GNSS_SYSTEM_GPS | GNSS_SYSTEM_QZSS;
			break;
		default:
			break;
	}
}

static int telit_mex10g1_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	struct telit_mex10g1_data *data = (struct telit_mex10g1_data *)(dev->data);
	// MODEM_CHAT_MATCH_DEFINE(gnss_cfg, "$GPSCFG: ", ",", modem_cellular_chat_on_gpscfg);
	// MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_mex10g1_gnss_check_get_sys_cmds,
	// 	MODEM_CHAT_SCRIPT_CMD_RESP("AT$GPSCFG", gnss_cfg));

	// MODEM_CHAT_SCRIPT_DEFINE(telit_mex10g1_gnss_check_get_sys_script, telit_mex10g1_gnss_check_get_sys_cmds,
	// 			abort_matches, gnss_chat_callback_handler, 1);
	
	// int err = modem_chat_run_script(&data->gnss_chat, &telit_mex10g1_gnss_check_get_sys_script);
	// if (err) {
	// 	LOG_ERR("Err running enabled script");
	// 	return err;
	// } else {
	// 	*systems = curr_sys;
	// }

	return 0;
}

/** API for getting enabled systems */
static int telit_mex10g1_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
	if (systems == NULL) {
		return -EINVAL;
	}

	*systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO | GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_QZSS;
	return 0;
}

static const struct gnss_driver_api gnss_api = {
	.set_fix_rate = telit_mex10g1_set_fix_rate,
	.get_fix_rate = telit_mex10g1_get_fix_rate,
	.set_navigation_mode = telit_mex10g1_set_nav_mode,
	.get_navigation_mode = telit_mex10g1_get_nav_mode,
	.set_enabled_systems = telit_mex10g1_set_enabled_systems,
	.get_enabled_systems = telit_mex10g1_get_enabled_systems,
	.get_supported_systems = telit_mex10g1_get_supported_systems
};

static int telit_mex10g1_gnss_init(const struct device *dev)
{
	struct telit_mex10g1_data *data = (struct telit_mex10g1_data *)(dev->data);

    k_work_init(&data->telit_mex10g1_gnss_open_pipe_work, telit_mex10g1_gnss_open_pipe_handler);
	k_work_init(&data->telit_mex10g1_gnss_attach_chat_work, telit_mex10g1_gnss_attach_chat_handler);
	k_work_init(&data->telit_mex10g1_gnss_release_chat_work, telit_mex10g1_gnss_release_chat_handler);
	k_work_init_delayable(&data->telit_mex10g1_gnss_check_fix_work, telit_mex10g1_gnss_check_fix_work_fn);

	data->telit_mex10g1_gnss_pipelink = MODEM_PIPELINK_DT_GET(GNSS_MODEM_NODE, user_pipe_0);
    modem_pipelink_attach(data->telit_mex10g1_gnss_pipelink, telit_mex10g1_gnss_pipelink_callback, data);

    const struct modem_chat_config telit_mex10g1_gnss_chat_cfg = {
		.receive_buf = data->gnss_chat_rx_buf,
		.receive_buf_size = sizeof(data->gnss_chat_rx_buf),
		.delimiter = "\r",
		.delimiter_size = sizeof("\r") - 1,
		.filter = "\n",
		.filter_size = sizeof("\n") - 1,
		.argv = data->chat_argv,
		.argv_size = ARRAY_SIZE(data->chat_argv),
		.user_data = data
	};

	modem_chat_init(&data->gnss_chat, &telit_mex10g1_gnss_chat_cfg);
    LOG_INF("initialized with pipelink %p", (void*)data->telit_mex10g1_gnss_pipelink);
	return 0;
}


#define TELIT_MEX10G1_GNSS_INST_NAME(inst, name) \
	_CONCAT(_CONCAT(_CONCAT(name, _), DT_DRV_COMPAT), inst)

#define TELIT_MEX10G1_GNSS_DEVICE(inst)								\
	static const struct telit_mex10g1_gnss_config TELIT_MEX10G1_GNSS_INST_NAME(inst, config); \
	\
	static struct telit_mex10g1_data TELIT_MEX10G1_GNSS_INST_NAME(inst, data); \
																		\
											\
											\
	DEVICE_DT_INST_DEFINE(inst, telit_mex10g1_gnss_init, NULL,	\
			 &TELIT_MEX10G1_GNSS_INST_NAME(inst, data), &TELIT_MEX10G1_GNSS_INST_NAME(inst, config),	\
			 POST_KERNEL, 99, &gnss_api);

#define DT_DRV_COMPAT telit_mex10g1_gnss
DT_INST_FOREACH_STATUS_OKAY(TELIT_MEX10G1_GNSS_DEVICE)
#undef DT_DRV_COMPAT