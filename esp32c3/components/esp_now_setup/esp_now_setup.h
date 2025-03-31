#pragma once

#include "esp_now.h"

void wifi_init(void);

void espnow_init(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb);