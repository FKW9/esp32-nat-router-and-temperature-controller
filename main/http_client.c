#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "mqtt_client.h"

const static char *TAG = "HTTP/MQTT Client";

esp_mqtt_client_handle_t client = NULL;
static char *response_data;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGE(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGW(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.8.42",
        // .credentials.authentication.password = "44155",
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

esp_err_t client_event_handler(esp_http_client_event_handle_t evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            // ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            // ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            // ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            // printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            response_data = (char*)evt->data;
            printf("%.*s", evt->data_len, response_data);
            break;
        case HTTP_EVENT_ON_FINISH:
            // ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            // ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
    return ESP_OK;
}

int mqtt_publish(float t1, float rh1, float t2, float rh2, float setpoint, uint8_t t_hyst_min, uint8_t t_hyst_max, uint8_t relay_state, uint8_t control_state) {
    static char mqtt_msg[256];
    int rssi;
    esp_wifi_sta_get_rssi(&rssi);
    // {"Time":"2023-08-18T19:39:10","ENERGY":{"TotalStartTime":"2023-08-18T15:07:16","Total":1.175,"Yesterday":0.000,"Today":1.175,"Power":13,"ApparentPower":625,"ReactivePower":625,"Factor":0.02,"Voltage":227,"Current":2.751}}
    sprintf(&mqtt_msg[0], "{\"\":\"\",\"ENERGY\":{\"TotalStartTime\":\"\",\"T1\":%.1f,\"RH1\":%.1f,\"T2\":%.1f,\"RH2\":%.1f,\"RELAY\":%d,\"SETPOINT\":%.1f,\"T_HYST_MIN\":%d,\"T_HYST_MAX\":%d,\"CONTROL\":%d,\"RSSI\":%d}}", t1, rh1, t2, rh2, relay_state, setpoint, t_hyst_min, t_hyst_max, control_state, rssi);
    return esp_mqtt_client_enqueue(client, "tele/tempctrl/SENSOR", mqtt_msg, 0, 1, 0, true);
}

esp_err_t set_tasmota_relay(uint8_t onoff)
{
    esp_err_t err = ESP_OK;
    switch (onoff)
    {
        case 0:
            esp_mqtt_client_enqueue(client, "cmnd/tempctrl/POWER", "OFF", 0, 1, 0, true);
            break;
        case 1:
            esp_mqtt_client_enqueue(client, "cmnd/tempctrl/POWER", "ON", 0, 1, 0, true);
            break;
        default:
            break;
    }

    // if(mqtt_response == -1 || mqtt_response == -2) {
    //     ESP_LOGW(TAG, "MQTT failed to set relay, tyring https...");
    //     esp_http_client_config_t config_post = {0};

    //     config_post.method = HTTP_METHOD_POST;
    //     config_post.cert_pem = NULL;
    //     config_post.event_handler = client_event_handler;

    //     switch (onoff)
    //     {
    //         case 0:
    //             config_post.url = "http://192.168.4.5/cm?user=admin&password=44155&cmnd=Power%20Off";
    //             break;
    //         case 1:
    //             config_post.url = "http://192.168.4.5/cm?user=admin&password=44155&cmnd=Power%20On";
    //             break;
    //         default:
    //             break;
    //     }

    //     ESP_LOGI(TAG, "%s", config_post.url);

    //     esp_http_client_handle_t client = esp_http_client_init(&config_post);

    //     esp_err_t err = esp_http_client_perform(client);
    //     if (err == ESP_OK) {
    //     ESP_LOGI(TAG, "Status = %d, content_length = %lld",
    //         esp_http_client_get_status_code(client),
    //         esp_http_client_get_content_length(client));
    //     }
    //     esp_http_client_cleanup(client);
    // }

    return err;
}

// uint8_t get_tasmota_relay(void)
// {
//     esp_http_client_config_t config_post = {0};

//     config_post.method = HTTP_METHOD_GET;
//     config_post.cert_pem = NULL;
//     config_post.event_handler = client_event_handler;
//     config_post.url = "http://192.168.4.5/cm?user=admin&password=44155&cmnd=Power";

//     ESP_LOGI(TAG, "%s", config_post.url);

//     esp_http_client_handle_t client = esp_http_client_init(&config_post);

//     esp_err_t err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//     ESP_LOGI(TAG, "Status = %d, content_length = %lld",
//         esp_http_client_get_status_code(client),
//         esp_http_client_get_content_length(client));
//     }
//     esp_http_client_cleanup(client);

//     ESP_LOGI(TAG, "Current Tasmota Relay State: %s", response_data);

//     return err;
// }
