#include "driver/ledc.h"
#include "hal/ledc_types.h"

#include "esp_timer.h"
#include "esp_mac.h"

#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "esp_netif_types.h"

#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types.h"
#include "esp_http_server.h"

#include "esp_event.h"
#include "esp_event_base.h"

#include "esp_err.h"
#include "esp_log.h"

#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern const uint8_t index_html_pem_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_pem_end[] asm("_binary_index_html_end");

static const char * TAG = "main";

/* interpolator */

typedef struct {
	uint16_t time;
	uint16_t warm_duty;
	uint16_t cold_duty;
} gradient_stop_t;

// little-endian
typedef struct {
	char magic[4]; // "aura"
	uint32_t timestamp;
	uint32_t stops_len;
	gradient_stop_t stops[0];
} gradient_t;

// interpolate from a to b, using res' time to calculate its remaining fields.
void gradient_interp(const gradient_stop_t * a, const gradient_stop_t * b, 
	gradient_stop_t * res) {
	const uint32_t INTERP_BITS = 16;
	const uint32_t INTERP_MAX = 1 << INTERP_BITS;

	int32_t interp_value = a->time != b->time 
		? (res->time - a->time) * INTERP_MAX / (b->time - a->time) 
		: 0;

	if (interp_value > INTERP_MAX) interp_value = INTERP_MAX;
	if (interp_value < 0) interp_value = 0;

	res->warm_duty = (a->warm_duty * (INTERP_MAX - interp_value)
		+ b->warm_duty * interp_value) / INTERP_MAX;
	res->cold_duty = (a->cold_duty * (INTERP_MAX - interp_value)
		+ b->cold_duty * interp_value) / INTERP_MAX;
}

void gradient_get(const gradient_t * g, gradient_stop_t * res) {
	if (g == NULL) return;

	if (g->stops[0].time > res->time) {
		res->warm_duty = g->stops[0].warm_duty;
		res->cold_duty = g->stops[0].cold_duty;
		return;
	}

	for (size_t i = 1; i < g->stops_len; i++) {
		if (g->stops[i - 1].time <= res->time && g->stops[i].time > res->time) { // res lies between
			gradient_interp(&g->stops[i - 1], &g->stops[i], res);
			return;
		}
	}

	res->warm_duty = g->stops[g->stops_len - 1].warm_duty;
	res->cold_duty = g->stops[g->stops_len - 1].cold_duty;
}

/* server config */

const struct {

} server_cfg = {
	
};

struct {
	httpd_handle_t httpd;
	gradient_t * gradient;
} server = {
	.httpd = NULL,
	.gradient = NULL,
};

esp_err_t server_get_cb(httpd_req_t * req) {
	ESP_LOGI(TAG, "server get handler");

	ESP_ERROR_CHECK(httpd_resp_set_status(req, HTTPD_200));
	ESP_ERROR_CHECK(httpd_resp_set_type(req, "text/html"));

	return httpd_resp_send(req, (const char *)index_html_pem_start, index_html_pem_end - index_html_pem_start);
}

esp_err_t server_post_cb(httpd_req_t * req) {
	ESP_LOGI(TAG, "server post handler, expecting %u bytes", req->content_len);

	const size_t MAX_CONTENT_LEN = 2048;

	if (req->content_len > MAX_CONTENT_LEN) {
		ESP_LOGW(TAG, "rejected post, data exceeds max content length!");
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	size_t head = 0;

	free(server.gradient);
	server.gradient = malloc(req->content_len);

	while (head < req->content_len) {
		int ret = httpd_req_recv(req, &((char *)server.gradient)[head], req->content_len);

		if (ret > 0) head += ret;
		else if (ret != HTTPD_SOCK_ERR_TIMEOUT) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
			return ESP_FAIL;
		}
	}

	// check magic and version

	ESP_LOGI(TAG, "gradient, magic=\"%.4s\", t=%lu, len=%lu",
		server.gradient->magic, server.gradient->timestamp, 
		server.gradient->stops_len);

	for (size_t i = 0; i < server.gradient->stops_len; i++) {
		ESP_LOGI(TAG, "[%u] time=%u, warm=%u, cold=%u", 
			i, server.gradient->stops[i].time, 
			server.gradient->stops[i].warm_duty, server.gradient->stops[i].cold_duty);
	}

	return ESP_OK;
}

void server_init() {
	const gradient_stop_t init_stops[] = {
		{ 0, 0x0000, 0x0000 },
		{ 240, 0xffff, 0x0000 },
		{ 720, 0xc000, 0xc000 },
		{ 1380, 0xa000, 0xffff },
		{ 1440, 0x0000, 0x0000 },
	};

	const gradient_t init = {
		.magic = "aura",
		.timestamp = 0,
		.stops_len = sizeof(init_stops) / sizeof(gradient_stop_t),
	};

	server.gradient = malloc(sizeof(gradient_t) + sizeof(init_stops));

	memcpy(server.gradient, &init, sizeof(gradient_t));
	memcpy(server.gradient->stops, init_stops, sizeof(init_stops));
}

void server_start() {
	httpd_config_t httpd_cfg = HTTPD_DEFAULT_CONFIG();
	httpd_cfg.lru_purge_enable = true;

	ESP_LOGI(TAG, "starting server on port %u", httpd_cfg.server_port);

	ESP_ERROR_CHECK(httpd_start(&server.httpd, &httpd_cfg));

	const httpd_uri_t get_uri = {
		.uri = "/",
		.method = HTTP_GET,
		.handler = server_get_cb,
		.user_ctx = NULL,
	};

	const httpd_uri_t post_uri = {
		.uri = "/",
		.method = HTTP_POST,
		.handler = server_post_cb,
		.user_ctx = NULL,
	};

	ESP_ERROR_CHECK(httpd_register_uri_handler(server.httpd, &get_uri));
	ESP_ERROR_CHECK(httpd_register_uri_handler(server.httpd, &post_uri));
}

void server_stop() {
	ESP_ERROR_CHECK(httpd_stop(server.httpd));
	server.httpd = NULL;
}

/* wifi config */

#define AP_NAME "Circadia"

const struct {

} ap_cfg = {

};

void ap_wifi_event_cb(void * arg, esp_event_base_t event_base, int32_t event_id, void * event_data) {
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		server_start();
	}
}

void ap_init() {
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_ERROR_CHECK(esp_netif_init());

	esp_netif_t * netif = esp_netif_create_default_wifi_ap();
	
	ESP_ERROR_CHECK(esp_netif_set_hostname(netif, "circadia"));
	
	esp_netif_ip_info_t netif_ip_info;

	ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &netif_ip_info));
	esp_netif_set_ip4_addr(&netif_ip_info.ip, 4, 3, 2, 1);
	esp_netif_set_ip4_addr(&netif_ip_info.gw, 4, 3, 2, 1);

	ESP_ERROR_CHECK(esp_netif_dhcps_stop(netif));
	ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &netif_ip_info));
	ESP_ERROR_CHECK(esp_netif_dhcps_start(netif));

	wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
	wifi_init_cfg.ampdu_rx_enable = false;

	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ap_wifi_event_cb, NULL, NULL));

	wifi_config_t wifi_cfg = {
		.ap = {
			.ssid = AP_NAME,
			.ssid_len = strlen(AP_NAME),

			.channel = 8,
			.password = "",
			.max_connection = 1,
			.authmode = WIFI_AUTH_OPEN,
			.pmf_cfg = { .required = true },
		}
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
}

void ap_start() {
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_LOGI(TAG, "ap start");
}

void ap_stop() {
	ESP_ERROR_CHECK(esp_wifi_stop());
	ESP_LOGI(TAG, "ap stop");
}

/* lamp */

typedef enum { STRIP_WARM = 0, STRIP_COLD = 1, STRIP_MAX = 2 } lamp_strip_t;

const uint32_t LAMP_PWM_BITS = 10; // nooo touchye

const struct {
	gradient_t ** gradient;
	uint64_t update_period;

	ledc_timer_config_t ledc_timer_cfg;

	ledc_channel_t ledc_chans[STRIP_MAX];
	int chan_pins[STRIP_MAX];

	ledc_channel_config_t ledc_chan_cfg;
} lamp_cfg = {
	.gradient = &server.gradient,

	.ledc_timer_cfg = {
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = LAMP_PWM_BITS,
		.freq_hz = 50000,
		.clk_cfg = LEDC_AUTO_CLK,
	},

	.ledc_chans = { LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
	.chan_pins = { 32, 33 },
	.ledc_chan_cfg = {
		.intr_type = LEDC_INTR_DISABLE,
		.duty = 0,
		.hpoint = 0,
	},
};

void lamp_set_strip_duty(lamp_strip_t strip, uint32_t duty) {
	ESP_ERROR_CHECK(ledc_set_duty(lamp_cfg.ledc_timer_cfg.speed_mode, lamp_cfg.ledc_chans[strip], 
		duty >> (16 - LAMP_PWM_BITS)));
	ESP_ERROR_CHECK(ledc_update_duty(lamp_cfg.ledc_timer_cfg.speed_mode, lamp_cfg.ledc_chans[strip]));
}

void lamp_set_duties(uint32_t warm, uint32_t cold) {
	lamp_set_strip_duty(STRIP_WARM, warm);
	lamp_set_strip_duty(STRIP_COLD, cold);
}

void lamp_set_gradient(const gradient_stop_t * stop) {
	lamp_set_duties(stop->warm_duty, stop->cold_duty);
}

void lamp_update() {
	gradient_stop_t new_stop = {
		.time = (esp_timer_get_time() / (60 * 1000000 / 1440)) % 1440, // one day every 30 seconds
		.warm_duty = 1000,
		.cold_duty = 1000,
	};

	gradient_get(*lamp_cfg.gradient, &new_stop);

	/*ESP_LOGI(TAG, "lamp update called, time=%hu, %hu, %hu", new_stop.time, 
		new_stop.warm_duty, new_stop.cold_duty);*/

	lamp_set_gradient(&new_stop);
}

void lamp_init() {
	ESP_ERROR_CHECK(ledc_timer_config(&lamp_cfg.ledc_timer_cfg));
	for (lamp_strip_t s = 0; s < STRIP_MAX; s++) {
		ledc_channel_config_t chan_cfg;
		memcpy(&chan_cfg, &lamp_cfg.ledc_chan_cfg, sizeof(chan_cfg));

		chan_cfg.speed_mode = lamp_cfg.ledc_timer_cfg.speed_mode;
		chan_cfg.gpio_num = lamp_cfg.chan_pins[s];
		chan_cfg.channel = lamp_cfg.ledc_chans[s];
		chan_cfg.timer_sel = lamp_cfg.ledc_timer_cfg.timer_num;

		ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));
	}

	esp_timer_create_args_t update_timer_cfg = {
		.name = "lamp_update",
		.callback = lamp_update,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.skip_unhandled_events = true,
	};
	esp_timer_handle_t update_timer;

	esp_timer_create(&update_timer_cfg, &update_timer);
	esp_timer_start_periodic(update_timer, 10000);
}

void app_main() {
	server_init();
	lamp_init();
	ap_init();

	ap_start();

	while (true) {

		/*gradient_stop_t stop = { .time = , };

		gradient_stop_t a = { .time = 0, 0x0000, 0x0000 }, 
			b = { .time = 0xffff, (1 << LAMP_PWM_BITS) / 2 - 1, (1 << LAMP_PWM_BITS) / 2 - 1 };

		gradient_interp(&a, &b, &stop);
		lamp_set_gradient(&stop);*/

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	server_stop();
	ap_stop();
}