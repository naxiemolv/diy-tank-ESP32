#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"


#include "config.h"
#include "udpManager.h"
#include "DCMotorController.h"
//===================================================================


//===================================================================


void heartBeat(void *pvParameters) {
    startHeartBeat();
}



void app_main(void)
{
    
    init_dc_motor_ctl();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init_sta();

    xTaskCreate(&udp_conn, "udp_conn", 4096, NULL, 5, NULL);
    xTaskCreate(&heartBeat, "heart_conn", 4096, NULL, 5, NULL);
}

