/*
 * MQTTSim800.h
 *
 *  Created on: Jan 4, 2020
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 *
 *
 */

#include <main.h>
#include "stm32f4xx_hal.h"
// === CONFIG ===
#define UART_SIM800 &huart6
#define FREERTOS    1
#define CMD_DELAY   1000
// ==============

void Sim800_RxCallBack(void);

void A9G_GetTime(char * time);

int SIM800_SendCommand(char *command, char *reply, uint16_t delay);

int SIM800_Init(void);

int MQTT_Connect(char *apn, char *apn_user, char *apn_pass, char *host, uint16_t port, char *username, char *pass,
                 char *clientID, unsigned short keepAliveInterval);

int MQTT_Pub(char *topic, char *payload);

int MQTT_PingReq(void);
