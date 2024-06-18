#include "webserver.h"
#include <string>
#include <esp_http_server.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "variables.h"
#include "web_funcs.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include <esp_app_format.h>

#include "P_css.h"
#include "P_bootstrap.h"
#include "P_system.h"
#include "P_pedal.h"
#include "P_can.h"
#include "P_index.h"
#include "P_js.h"

static const char *TAG = "Webserver";

int64_t lastCommandTime = 0;

#define BUFFSIZE 1024
static char ota_write_data[BUFFSIZE + 1] = {0};

static void reboot_timer_callback(void* arg) {
    esp_restart();
}

const esp_timer_create_args_t reboot_timer_args = {
    .callback = &reboot_timer_callback,
    .name = "reboot-one-shot"
};

esp_timer_handle_t reboot_timer;

static esp_err_t upload_post_handler(httpd_req_t *req)
{
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA example");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 (unsigned int)configured->address, (unsigned int)running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, (unsigned int)running->address);

    // 开始升级
    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, (unsigned int)update_partition->address);

    int received;

    /* Content length of the request gives
     * the size of the file being uploaded */
    int remaining = req->content_len;
    bool image_header_was_checked = false;
    while (remaining > 0)
    {
        ESP_LOGI(TAG, "Remaining size : %d", remaining);
        /* Receive the file part by part into a buffer */
        if ((received = httpd_req_recv(req, ota_write_data, std::min(remaining, BUFFSIZE))) <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry if timeout occurred */
                continue;
            }

            ESP_LOGE(TAG, "File reception failed!");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
            return ESP_FAIL;
        }

        if (image_header_was_checked == false)
        {
            esp_app_desc_t new_app_info;
            if (received > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
            {
                // check current version with downloading
                memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                esp_app_desc_t running_app_info;
                if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                }

                const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                esp_app_desc_t invalid_app_info;
                if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                }

                image_header_was_checked = true;

                err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                    esp_ota_abort(update_handle);
                }
                ESP_LOGI(TAG, "esp_ota_begin succeeded");
            }
            else
            {
                ESP_LOGE(TAG, "received package is not fit len");
                esp_ota_abort(update_handle);
            }
        }
        err = esp_ota_write(update_handle, (const void *)ota_write_data, received);
        if (err != ESP_OK)
        {
            esp_ota_abort(update_handle);
        }
        ESP_LOGD(TAG, "Written image length %d", received);

        /* Keep track of remaining size of
         * the file left to be uploaded */
        remaining -= received;
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        else
        {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "File reception complete");

    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, "File uploaded successfully, rebooting");
    ESP_ERROR_CHECK(esp_timer_create(&reboot_timer_args, &reboot_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(reboot_timer, 5000000));
    return ESP_OK;
}


static esp_err_t js_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/javascript");
    httpd_resp_set_hdr(req, "Last-Modified", "Sat, 11 Mar 2023 10:00:00 GMT");
    httpd_resp_send(req, P_js, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t system_page_handler(httpd_req_t *req) {
    httpd_resp_send(req, P_system, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t pedal_page_handler(httpd_req_t *req) {
    httpd_resp_send(req, P_pedal, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t can_page_handler(httpd_req_t *req) {
    httpd_resp_send(req, P_can, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t get_wheel_state_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, getWheelStateJSON().c_str());
    return ESP_OK;
}

static esp_err_t get_can_state_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, getCANStateJSON().c_str());
    return ESP_OK;
}

static esp_err_t set_wheel_calibration_handler(httpd_req_t *req) {
    webSetWheelCalibration(req);
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, "{\"state\":\"ok\"}");
    return ESP_OK;
}

static esp_err_t get_wheel_calibration_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, webWheelCalibrationJSON().c_str());
    return ESP_OK;
}

static esp_err_t index_page_handler(httpd_req_t *req) {
    httpd_resp_send(req, P_index, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t css_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/css");
    httpd_resp_set_hdr(req, "Last-Modified", "Sat, 11 Mar 2023 10:00:00 GMT");
    httpd_resp_send(req, P_css, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t bootstrap_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/css");
    httpd_resp_set_hdr(req, "Last-Modified", "Sat, 11 Mar 2023 10:00:00 GMT");
    httpd_resp_send(req, P_bootstrap, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t get_pedal_params_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, webPedalJson().c_str());
    return ESP_OK;
}

static esp_err_t set_pedal_params_handler(httpd_req_t *req) {
    webPedalParamsSet(req);
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, "{\"state\":\"ok\"}");
    return ESP_OK;
}

static esp_err_t get_system_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, webSystemJson().c_str());
    return ESP_OK;
}

static esp_err_t set_system_handler(httpd_req_t *req) {
    webSystemSet(req);
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, "{\"state\":\"ok\"}");
    return ESP_OK;
}

esp_err_t general_handler(httpd_req_t *req) {
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");


    httpd_resp_send(req, "Response", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t start_web_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    //config.uri_match_fn = httpd_uri_match_wildcard;
    config.server_port = 80;
    config.stack_size = 32768;
    config.max_uri_handlers = 32;

    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }

    httpd_uri_t update_page = {
        .uri = "/system.html",
        .method = HTTP_GET,
        .handler = system_page_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &update_page);

    httpd_uri_t file_upload = {
        .uri = "/upload",
        .method = HTTP_POST,
        .handler = upload_post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &file_upload);

    httpd_uri_t bootstrap = {
        .uri = "/bootstrap.min.css",
        .method = HTTP_GET,
        .handler = bootstrap_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &bootstrap);

    httpd_uri_t css = {
        .uri = "/theme.css",
        .method = HTTP_GET,
        .handler = css_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &css);

    httpd_uri_t js = {
        .uri = "/script.js",
        .method = HTTP_GET,
        .handler = js_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &js);

    httpd_uri_t pedalPage = {
        .uri = "/pedal.html",
        .method = HTTP_GET,
        .handler = pedal_page_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &pedalPage);

    httpd_uri_t canPage = {
        .uri = "/can.html",
        .method = HTTP_GET,
        .handler = can_page_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &canPage);
    
    httpd_uri_t getPedalParams = {
        .uri = "/web/pedal.json",
        .method = HTTP_GET,
        .handler = get_pedal_params_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &getPedalParams);

    httpd_uri_t setPedalParams = {
        .uri = "/set/pedal",
        .method = HTTP_GET,
        .handler = set_pedal_params_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &setPedalParams);

    httpd_uri_t getSystem = {
        .uri = "/web/system.json",
        .method = HTTP_GET,
        .handler = get_system_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &getSystem);

    httpd_uri_t setSystem = {
        .uri = "/set/system",
        .method = HTTP_GET,
        .handler = set_system_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &setSystem);


    httpd_uri_t getWheelState = {
        .uri = "/get/wheel_state",
        .method = HTTP_POST,
        .handler = get_wheel_state_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &getWheelState);

    httpd_uri_t setWheelCalibration = {
        .uri = "/set/wheel_calibration",
        .method = HTTP_GET,
        .handler = set_wheel_calibration_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &setWheelCalibration);

    httpd_uri_t getWheelCalibration = {
        .uri = "/get/wheel_calibration.json",
        .method = HTTP_GET,
        .handler = get_wheel_calibration_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &getWheelCalibration);

    httpd_uri_t getCANState = {
        .uri = "/get/can_state",
        .method = HTTP_POST,
        .handler = get_can_state_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &getCANState);

    httpd_uri_t indexPage = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_page_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &indexPage);

    ESP_LOGI(TAG, "HTTP server started");

    return ESP_OK;
}