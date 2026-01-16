/*! Copyright 2025 Bogdan Pilyugin
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *  	 \file sdmx.h
 *    \version 1.0
 * 		 \date 2025-12-06
 *     \author Bogdan Pilyugin
 * 	    \brief
 *    \details
 *	\copyright Apache License, Version 2.0
 */

#ifndef MAIN_DMX_DMX_H_
#define MAIN_DMX_DMX_H_

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include <esp_timer.h>


#include <driver/uart.h>
#include <driver/gpio.h>
#include <rom/gpio.h>
#include <stdint.h>

#define DMX_BAUD 		250000
#define DMX_PACKET_SIZE 512
#define DMX_PACKET_RATE	30 
#define DMX_BREAK_US 	100
#define DMX_MAB_US		10
#define DMX_START_BYTE  0x00

enum DMX_Direction
{
    input,
    output
};

enum DMX_State {
	DMX_IDLE,
	DMX_BREAK,
	DMX_DATA,
	DMX_ERROR	
};

typedef struct
{
    uint8_t idx;
    uart_port_t uart;
    uint8_t tx_pin;
    uint8_t rx_pin;
    uint8_t dc_pin;
    uint16_t circ_buff_size;
    enum DMX_Direction dmx_direction;
	uint8_t coreID;
	uint16_t packet_rate;
	uint16_t packet_size;
} sdmx_config_t;

typedef struct
{
    sdmx_config_t cfg;
	enum DMX_State state;
    QueueHandle_t dmx_rx_queue;
    SemaphoreHandle_t sync_dmx;
	esp_timer_handle_t tmr;
    EventGroupHandle_t dmx_events_group;
    uint16_t rx_cntr;
	uint8_t data[DMX_PACKET_SIZE];
    uint64_t last_dmx_packet;

} sdmx_handle_t;

esp_err_t InitDMXchannel(sdmx_handle_t *dmx, sdmx_config_t *cfg);
esp_err_t WriteDMX(sdmx_handle_t *dmx, uint8_t *data, uint16_t len);
esp_err_t ReadDMX(sdmx_handle_t *dmx, uint8_t *data, uint16_t len);
esp_err_t PacketReady(sdmx_handle_t *dmx);

#endif /* MAIN_DMX_DMX_H_ */
