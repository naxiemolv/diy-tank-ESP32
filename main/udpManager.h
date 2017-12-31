#ifndef _UDP_MANAGER_H_ 
#define _UDP_MANAGER_H_ 


#ifdef __cplusplus
extern "C" {
#endif


#define WIFI_CONNECTED_BIT BIT0

#define UDP_CONNCETED_SUCCESS BIT1

void udp_conn(void *pvParameters);

void wifi_init_sta();

void startHeartBeat();

#ifdef __cplusplus
}
#endif

#endif