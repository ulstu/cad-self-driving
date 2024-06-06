#pragma once
#include <esp_http_server.h>

class Webserver {
public:
    static void init();
private:
    static inline const int OTA_BUFFSIZE = 1024;
    static inline char ota_write_data[OTA_BUFFSIZE + 1] = {0};

    static void reboot_timer_callback(void* arg);
    static esp_err_t upload_post_handler(httpd_req_t *req);

    static esp_err_t js_handler(httpd_req_t *req);
    static esp_err_t css_handler(httpd_req_t *req);
    static esp_err_t bootstrap_handler(httpd_req_t *req);

    static esp_err_t get_clutch_handler(httpd_req_t *req);
    static esp_err_t set_clutch_handler(httpd_req_t *req);
    static esp_err_t clutch_page_handler(httpd_req_t *req);

    static esp_err_t get_brake_handler(httpd_req_t *req);
    static esp_err_t set_brake_handler(httpd_req_t *req);
    static esp_err_t brake_page_handler(httpd_req_t *req);

    static esp_err_t get_system_handler(httpd_req_t *req);
    static esp_err_t set_system_handler(httpd_req_t *req);
    static esp_err_t system_page_handler(httpd_req_t *req);

    static esp_err_t index_page_handler(httpd_req_t *req);
    static esp_err_t get_status_handler(httpd_req_t *req);
};