#include "udpManager.h"

#include <string.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "config.h"
#include "packageParser.h"

EventGroupHandle_t udp_event_group;
static int mysocket;
static int sendSocket;
static struct sockaddr_in remote_addr;
static struct sockaddr_in my_server_addr;

static unsigned int socklen;

int total_data = 0;
int success_pack = 0;
bool udpServerRunning = false;

int show_socket_error_reason(int socket);
int check_connected_socket();
int get_socket_error_code(int socket);

esp_err_t create_udp_server();
void close_socket();
unsigned char databuffRECV[EXAMPLE_DEFAULT_PKTSIZE];

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        wifi_init_sta(udp_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "event_handler:SYSTEM_EVENT_STA_GOT_IP!");
        ESP_LOGI(TAG, "got ip:%s\n",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(udp_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:" MACSTR " join,AID=%d\n",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        xEventGroupSetBits(udp_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:" MACSTR "leave,AID=%d\n",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        xEventGroupClearBits(udp_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void wifi_init_sta()
{
    udp_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_DEFAULT_SSID,
            .password = EXAMPLE_DEFAULT_PWD},
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s \n",
             EXAMPLE_DEFAULT_SSID, EXAMPLE_DEFAULT_PWD);
}

void udp_conn(void *pvParameters)
{
    ESP_LOGI(TAG, "task udp_conn start.");
    xEventGroupWaitBits(udp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "create udp server after 3s...");
    vTaskDelay(3000 / portTICK_RATE_MS);
    int socket_ret;

    socket_ret = create_udp_server();


    if (socket_ret == ESP_FAIL)
    {
        ESP_LOGI(TAG, "create udp socket error,stop.");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "task recv_data start!\n");
    int len;
    
    socklen = sizeof(remote_addr);
    memset(databuffRECV, EXAMPLE_PACK_BYTE_IS, EXAMPLE_DEFAULT_PKTSIZE);
    udpServerRunning = true;
    len = recvfrom(mysocket, databuffRECV, EXAMPLE_DEFAULT_PKTSIZE, 0, (struct sockaddr *)&remote_addr, &socklen);
    if (len > 0)
    {
        ESP_LOGI(TAG, "transfer data with %s:%u\n",
                 inet_ntoa(remote_addr.sin_addr), ntohs(remote_addr.sin_port));
        
    }
    else
    {
        show_socket_error_reason(mysocket);
        close(mysocket);
        vTaskDelete(NULL);
    } 

    while (1)
    {

        len = recvfrom(mysocket, databuffRECV, EXAMPLE_DEFAULT_PKTSIZE, 0, (struct sockaddr *)&remote_addr, &socklen);

        if (len > 0)
        {

            handlePackage(&databuffRECV[0], len);
        }
        else
        {
            if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG)
            {
                show_socket_error_reason(mysocket);
            }
            break;
        }
    }

    close(mysocket);
    vTaskDelete(NULL);
}

esp_err_t create_udp_server()
{
    ESP_LOGI(TAG, "create_udp_server() port:%d", EXAMPLE_DEFAULT_PORT);
    mysocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (mysocket < 0)
    {
        show_socket_error_reason(mysocket);
        return ESP_FAIL;
    }
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(EXAMPLE_DEFAULT_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(mysocket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        show_socket_error_reason(mysocket);
        close(mysocket);
        return ESP_FAIL;
    }
    return ESP_OK;
}

int show_socket_error_reason(int socket)
{
    int err = get_socket_error_code(socket);
    ESP_LOGW(TAG, "socket error %d %s", err, strerror(err));
    return err;
}
int check_connected_socket()
{
    int ret;
    ESP_LOGD(TAG, "check connect_socket");
    ret = get_socket_error_code(mysocket);
    if (ret != 0)
    {
        ESP_LOGW(TAG, "socket error %d %s", ret, strerror(ret));
    }
    return ret;
}

void close_socket()
{
    close(mysocket);
}

int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen) == -1)
    {
        ESP_LOGE(TAG, "getsockopt failed");
        return -1;
    }
    return result;
}

void startHeartBeat() {

    sendSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (sendSocket < 0) {
    	show_socket_error_reason(sendSocket);
    }
    /*for client remote_addr is also server_addr*/
    my_server_addr.sin_family = AF_INET;
    my_server_addr.sin_port = htons(EXAMPLE_DEFAULT_PORT);
    my_server_addr.sin_addr.s_addr = inet_addr(DEFAULT_SERVER_IP);

    unsigned char databuff[EXAMPLE_DEFAULT_PKTSIZE];

    
    int datalen = strlen(HAND_SHAKE_DATA);
    databuff[datalen] = 0;
    memcpy(databuff,HAND_SHAKE_DATA,datalen);
    
    xEventGroupWaitBits(udp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    ssize_t len;
    while (1) {
        if (udpServerRunning) {
            
            len = sendto(sendSocket, databuff, datalen, 0, (struct sockaddr *)&my_server_addr, sizeof(my_server_addr));
            printf("heartbeat    sent len:%d \n ",len);
                
        }
        
        vTaskDelay(5*1000);

    }
}


