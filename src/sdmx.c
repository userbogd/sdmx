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
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include <stdint.h>
#include <string.h>

#define TAG "sDMX driver"

void dmx_service_timer_callback(void *arg)
{
  sdmx_handle_t *dmx;
  dmx = (sdmx_handle_t *)arg;

  // dmx->rx_cntr = 0;
  // dmx->state = DMX_IDLE;
  // uart_flush_input(dmx->cfg.uart);
  // xQueueReset(dmx->dmx_rx_queue);
  ESP_LOGW(TAG, "Packet receive watchdog reset state %d", dmx->state);
}

IRAM_ATTR static void uart_tx_task(void *arg)
{
  sdmx_handle_t *dmx;
  dmx = (sdmx_handle_t *)arg;

  while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(DMX_PACKET_RATE));
      uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
      uart_set_line_inverse(dmx->cfg.uart, UART_SIGNAL_TXD_INV);
      esp_rom_delay_us(DMX_BREAK_US);
      uart_set_line_inverse(dmx->cfg.uart, 0);
      esp_rom_delay_us(DMX_MAB_US);
      xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
      dmx->data[0] = DMX_START_BYTE;
      uart_write_bytes(dmx->cfg.uart, (const char *)dmx->data, 513);
      xSemaphoreGive(dmx->sync_dmx);
    }
}

static void on_packet_received(sdmx_handle_t *dmx)
{
  ESP_LOGI(TAG, "PACKET OK at %d", (int)dmx->last_dmx_packet);
  return;
  dmx->last_dmx_packet = xTaskGetTickCount();
  xSemaphoreGive(dmx->sync_dmx);
  for (int i = 0; i < DMX_PACKET_SIZE; i++)
    printf("%02X", *(dmx->data + i));
  printf("\r\n");
  xSemaphoreGive(dmx->sync_dmx);
}

void uart_rx_task(void *arg)
{
  sdmx_handle_t *dmx;
  dmx = (sdmx_handle_t *)arg;

  uart_event_t event;
  uint8_t *temp = (uint8_t *)malloc(dmx->cfg.circ_buff_size);
  while (1)
    {
      if (xQueueReceive(dmx->dmx_rx_queue, (void *)&event, portMAX_DELAY))
        {
          bzero(temp, dmx->cfg.circ_buff_size);
          uart_read_bytes(dmx->cfg.uart, temp, event.size, pdMS_TO_TICKS(1000));
          int it = 0;
          switch (event.type)
            {
              case UART_BREAK:
                dmx->state = DMX_BREAK;
                dmx->rx_cntr = 0;
                it = 1; // for skip brake byte

                break;
              case UART_DATA:
                dmx->state = DMX_DATA;
                it = 0;
                break;
              case UART_FRAME_ERR:
              case UART_PARITY_ERR:
              case UART_BUFFER_FULL:
              case UART_FIFO_OVF:
              default:
                // error recevied, going to idle mode
                dmx->state = DMX_IDLE;
                uart_flush_input(dmx->cfg.uart);
                xQueueReset(dmx->dmx_rx_queue);
                ESP_LOGW(TAG, "FRAME ERROR DETECTED");
                break;
            }
          xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
          for (int i = it; i < event.size && dmx->rx_cntr < DMX_PACKET_SIZE; i++)
            dmx->data[dmx->rx_cntr++] = temp[i];
          xSemaphoreGive(dmx->sync_dmx);
          if (dmx->rx_cntr >= DMX_PACKET_SIZE)
            {
              dmx->state = DMX_IDLE;
              dmx->rx_cntr = 0;
              uart_flush_input(dmx->cfg.uart);
              xQueueReset(dmx->dmx_rx_queue);
              on_packet_received(dmx);
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

  gpio_pad_select_gpio(cfg->dc_pin);
  gpio_set_direction(cfg->dc_pin, GPIO_MODE_OUTPUT);
  dmx->state = DMX_IDLE;
  if (cfg->dmx_direction == output)
    {
      gpio_set_level(cfg->dc_pin, 1);
      xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 4 * 1024, dmx, 24, NULL,
          cfg->coreID);
    }
  else
    {
      esp_timer_create_args_t dmx_service_timer_args
          = { .callback = &dmx_service_timer_callback, .name = "dmxTimer", .arg = dmx };
      ESP_ERROR_CHECK(esp_timer_create(&dmx_service_timer_args, &dmx->tmr));
      esp_timer_start_periodic(dmx->tmr, 10000);
      gpio_set_level(cfg->dc_pin, 0);
      xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4 * 1024, dmx, 24, NULL,
          cfg->coreID);
    }

  return ESP_OK;
}

esp_err_t WriteDMX(sdmx_handle_t *dmx, uint8_t *data, uint16_t len)
{
  xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
  memcpy(dmx->data + 1, data, len);
  xSemaphoreGive(dmx->sync_dmx);
  return ESP_OK;
}
