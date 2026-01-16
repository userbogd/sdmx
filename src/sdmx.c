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
 *  	 \file sdmx.c
 *    \version 1.0
 * 		 \date 2025-12-06
 *     \author Bogdan Pilyugin
 * 	    \brief DMX512 driver
 *    \details DMX512 uart based driver
 *	\copyright Apache License, Version 2.0
 */

#include "sdmx.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <freertos/FreeRTOS.h>
#include "freertos/idf_additions.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include <stdint.h>
#include <string.h>

#define TAG "sDMX driver"

static const int SEND_DMX_BIT = BIT0;
static const int READ_DMX_BIT = BIT1;

IRAM_ATTR static void uart_tx_task(void *arg)
{
  sdmx_handle_t *dmx;
  dmx = (sdmx_handle_t *)arg;
  uint8_t start_byte = DMX_START_BYTE;
  while (1)
    {
      xEventGroupWaitBits(dmx->dmx_events_group, SEND_DMX_BIT, pdTRUE, pdFALSE,
          portMAX_DELAY);
      uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
      uart_set_line_inverse(dmx->cfg.uart, UART_SIGNAL_TXD_INV);
      esp_rom_delay_us(DMX_BREAK_US);
      uart_set_line_inverse(dmx->cfg.uart, 0);
      esp_rom_delay_us(DMX_MAB_US);
      xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
      uart_write_bytes(dmx->cfg.uart, (const char *)&start_byte, 1);
      uart_write_bytes(dmx->cfg.uart, (const char *)dmx->data, DMX_PACKET_SIZE);
      xSemaphoreGive(dmx->sync_dmx);
    }
}

IRAM_ATTR void uart_rx_task(void *arg)
{
  sdmx_handle_t *dmx;
  dmx = (sdmx_handle_t *)arg;
  int rxoffset = 0;

  uart_event_t event;
  uint8_t *tmp1 = (uint8_t *)malloc(dmx->cfg.circ_buff_size);
  uint8_t *tmp2 = (uint8_t *)malloc(DMX_PACKET_SIZE + 2);
  while (1)
    {
      if (xQueueReceive(dmx->dmx_rx_queue, (void *)&event, portMAX_DELAY))
        {
          bzero(tmp1, dmx->cfg.circ_buff_size);
          switch (event.type)
            {
              case UART_DATA:
                uart_read_bytes(dmx->cfg.uart, tmp1, event.size, portMAX_DELAY);
                if (dmx->state == DMX_BREAK)
                  {
                    xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
                    memcpy(dmx->data, &tmp2[2], sizeof(dmx->data));
                    xSemaphoreGive(dmx->sync_dmx);
                    // on_packet_received(dmx);
                    xEventGroupSetBits(dmx->dmx_events_group, READ_DMX_BIT);
                    if (tmp1[1] == 0)
                      {
                        dmx->state = DMX_DATA;
                        rxoffset = 0;
                        dmx->last_dmx_packet = xTaskGetTickCount();
                      }
                  }
                if (dmx->state == DMX_DATA)
                  {
                    for (int i = 0; i < event.size; i++)
                      {
                        if (rxoffset < DMX_PACKET_SIZE + 2)
                          tmp2[rxoffset++] = tmp1[i];
                      }
                  }

                break;
              case UART_BREAK:
                dmx->state = DMX_BREAK;
                break;

              case UART_FRAME_ERR:
              case UART_PARITY_ERR:
              case UART_BUFFER_FULL:
              case UART_FIFO_OVF:
              default:
                dmx->state = DMX_IDLE;
                uart_flush_input(dmx->cfg.uart);
                xQueueReset(dmx->dmx_rx_queue);
                ESP_LOGW(TAG, "FRAME ERROR DETECTED");
                break;
            }
        }
    }
}

const uart_config_t uart_config = { .baud_rate = DMX_BAUD,
  .data_bits = UART_DATA_8_BITS,
  .parity = UART_PARITY_DISABLE,
  .stop_bits = UART_STOP_BITS_2,
  .rx_flow_ctrl_thresh = 0,
  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

esp_err_t InitDMXchannel(sdmx_handle_t *dmx, sdmx_config_t *cfg)
{
  memcpy(&dmx->cfg, cfg, sizeof(sdmx_config_t));

  ESP_ERROR_CHECK(uart_driver_install(cfg->uart, cfg->circ_buff_size * 2,
      cfg->circ_buff_size * 2, 20, &dmx->dmx_rx_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(cfg->uart, &uart_config));
  ESP_ERROR_CHECK(
      uart_set_pin(cfg->uart, cfg->tx_pin, cfg->rx_pin, cfg->dc_pin, UART_PIN_NO_CHANGE));

  dmx->sync_dmx = xSemaphoreCreateMutex();
  dmx->dmx_events_group = xEventGroupCreate();

  gpio_pad_select_gpio(cfg->dc_pin);
  gpio_set_direction(cfg->dc_pin, GPIO_MODE_OUTPUT);
  dmx->state = DMX_IDLE;
  if (cfg->dmx_direction == output)
    {
      gpio_set_level(cfg->dc_pin, 1);
      xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 4 * 1024, dmx, 8, NULL,
          cfg->coreID);
    }
  else
    {
      gpio_set_level(cfg->dc_pin, 0);
      xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4 * 1024, dmx, 8, NULL,
          cfg->coreID);
    }

  return ESP_OK;
}

esp_err_t WriteDMX(sdmx_handle_t *dmx, uint8_t *data, uint16_t len)
{
  xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
  memcpy(dmx->data, data, len);
  xEventGroupSetBits(dmx->dmx_events_group, SEND_DMX_BIT);
  xSemaphoreGive(dmx->sync_dmx);
  return ESP_OK;
}

esp_err_t ReadDMX(sdmx_handle_t *dmx, uint8_t *data, uint16_t len)
{
  xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
  memcpy(data, dmx->data, len);
  xSemaphoreGive(dmx->sync_dmx);
  return ESP_OK;
}

esp_err_t PacketReady(sdmx_handle_t *dmx)
{
  EventBits_t uxBits;
  uxBits = xEventGroupWaitBits(dmx->dmx_events_group, READ_DMX_BIT, pdTRUE, pdFALSE,
      pdMS_TO_TICKS(1000));
  if (uxBits & READ_DMX_BIT)
    return ESP_OK;
  else
    return ESP_ERR_NOT_FINISHED;
}
