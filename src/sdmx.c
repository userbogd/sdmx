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
#include <stdint.h>
#include <string.h>
#include "freertos/idf_additions.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"

#define TAG "sDMX driver"

static const int SEND_DMX_BIT = BIT0;
static const int READ_DMX_BIT = BIT1;
static const int SENT_DMX_BIT = BIT3;
static const int PROG_DMX_BIT = BIT4;

const uint8_t param_data[] = {0x4F, 0x4F, 0x4F, 0x4F, 0x01, 0xC7, 0x40, 0x4B, 0x5A};
const uint8_t autoaddr_data[] = {0x00, 0xD2, 0x5A};
const uint8_t curr_data[] = {0x1E, 0x1E, 0x1E, 0x1E, 0x4B, 0x5A};

const uint8_t addr_inst[] = {0xA5, 0x5A, 0x4B, 0x5A};
const uint8_t param_inst[] = {0xC5, 0x3A, 0x4B, 0x5A};
const uint8_t curr_inst[] = {0x35, 0xCA, 0x4B, 0x5A};
const uint8_t autoaddr_inst[] = {0xC5, 0x69, 0x4B, 0x5A};

uint8_t inst[4];
uint8_t data[9];
uint8_t len;

const uint8_t bit_reverse_table[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 0x08, 0x88, 0x48,
    0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4,
    0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C,
    0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2,
    0x32, 0xB2, 0x72, 0xF2, 0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A,
    0xFA, 0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 0x0E, 0x8E,
    0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE, 0x01, 0x81, 0x41, 0xC1, 0x21,
    0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1, 0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9,
    0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55,
    0xD5, 0x35, 0xB5, 0x75, 0xF5, 0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD,
    0x7D, 0xFD, 0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 0x0B,
    0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB, 0x07, 0x87, 0x47, 0xC7,
    0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F,
    0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF};

static void change_order(uint8_t *dat, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) dat[i] = bit_reverse_table[dat[i]];
}

void SetDMXMode(sdmx_handle_t *dmx, dmx_util_mode_t mod)
{
    dmx->dmx_mode = mod;
}

const uint8_t zerobyte[] = {0x00};
IRAM_ATTR static void uart_tx_task(void *arg)
{
    sdmx_handle_t *dmx;
    dmx = (sdmx_handle_t *)arg;
    uint8_t start_byte = DMX_START_BYTE;
    EventBits_t uxBits;
    while (1) {
        if (dmx->dmx_mode == DMX_UTIL_MODE_NORMAL) {
            uxBits = xEventGroupWaitBits(dmx->dmx_events_group, SEND_DMX_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(500));
            if (uxBits & SEND_DMX_BIT) {
                uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
                uart_set_line_inverse(dmx->cfg.uart, UART_SIGNAL_TXD_INV);
                esp_rom_delay_us(DMX_BREAK_US);
                uart_set_line_inverse(dmx->cfg.uart, 0);
                esp_rom_delay_us(DMX_MAB_US);
                xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
                uart_write_bytes(dmx->cfg.uart, (const char *)&start_byte, 1);
                uart_write_bytes(dmx->cfg.uart, (const char *)dmx->data, DMX_PACKET_SIZE);
                xSemaphoreGive(dmx->sync_dmx);
                xEventGroupSetBits(dmx->dmx_events_group, SENT_DMX_BIT);
            }
        } else if (dmx->dmx_mode == DMX_UTIL_MODE_PROG) {
            uxBits = xEventGroupWaitBits(dmx->dmx_events_group, PROG_DMX_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(500));
            if (uxBits & PROG_DMX_BIT) {
                ESP_LOGI(TAG, "Programm dmx");
                ESP_LOG_BUFFER_HEX(TAG, dmx->dmx_util_data, 10);
                uart_set_line_inverse(dmx->cfg.uart, UART_SIGNAL_TXD_INV);
                vTaskDelay(pdMS_TO_TICKS(1500));
                uart_set_line_inverse(dmx->cfg.uart, 0);
                esp_rom_delay_us(75);
                uart_write_bytes(dmx->cfg.uart, zerobyte, 1);
                uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
                esp_rom_delay_us(45);

                switch (dmx->dmx_util_data[0]) {
                case DMX_UTIL_PROG_ADDRESS:
                    memcpy(inst, addr_inst, 4);
                    change_order(inst, 4);
                    uart_write_bytes(dmx->cfg.uart, inst, 4);
                    uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
                    uint16_t start_addr = dmx->dmx_util_data[1] * 256 + dmx->dmx_util_data[2];
                    uint16_t channels = dmx->dmx_util_data[3] * 256 + dmx->dmx_util_data[4];
                    uint16_t dev_num = dmx->dmx_util_data[5] * 256 + dmx->dmx_util_data[6];
                    for (int i = 0; i < dev_num; i++) {
                        int curaddr = (dmx->dmx_util_data[7] == 0x01) ? start_addr + channels * i : start_addr;
                        uint8_t high_addr = (uint8_t)(curaddr >> 8) & 0xF;
                        high_addr = bit_reverse_table[high_addr];
                        data[0] = 0x00;
                        data[0] |= (high_addr & 0xf) << 4;
                        data[0] |= ~high_addr & 0xf;
                        data[1] = bit_reverse_table[(uint8_t)(curaddr & 0x00FF)];
                        data[2] = 0x4B;
                        data[3] = 0x5A;
                        change_order(data, 4);
                        uart_write_bytes(dmx->cfg.uart, data, 4);
                        uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
                        esp_rom_delay_us(45);
                    }
                    break;

                case DMX_UTIL_PROG_PARAMS:
                    memcpy(inst, param_inst, 4);
                    change_order(inst, 4);
                    data[0] = bit_reverse_table[dmx->dmx_util_data[1]];
                    data[1] = bit_reverse_table[dmx->dmx_util_data[2]];
                    data[2] = bit_reverse_table[dmx->dmx_util_data[3]];
                    data[3] = bit_reverse_table[dmx->dmx_util_data[4]];
                    data[4] = bit_reverse_table[dmx->dmx_util_data[5]];
                    data[5] = dmx->dmx_util_data[6];
                    data[6] = dmx->dmx_util_data[7];

                    data[7] = 0x4B;
                    data[8] = 0x5A;
                    change_order(data, 9);
                    uart_write_bytes(dmx->cfg.uart, inst, 4);
                    uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
                    esp_rom_delay_us(45);
                    uart_write_bytes(dmx->cfg.uart, data, 9);

                    break;

                case DMX_UTIL_PROG_GAIN:
                    memcpy(inst, curr_inst, 4);
                    change_order(inst, 4);
                    data[0] = bit_reverse_table[dmx->dmx_util_data[1]] >> 1;
                    data[1] = bit_reverse_table[dmx->dmx_util_data[2]] >> 1;
                    data[2] = bit_reverse_table[dmx->dmx_util_data[3]] >> 1;
                    data[3] = bit_reverse_table[dmx->dmx_util_data[4]] >> 1;
                    data[4] = 0x4B;
                    data[5] = 0x5A;
                    change_order(data, 6);
                    uart_write_bytes(dmx->cfg.uart, inst, 4);
                    uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
                    esp_rom_delay_us(45);
                    uart_write_bytes(dmx->cfg.uart, data, 6);
                    break;

                case DMX_UTIL_PROG_AUTOMATIC:
                    memcpy(inst, autoaddr_inst, 4);
                    change_order(inst, 4);
                    data[0] = bit_reverse_table[dmx->dmx_util_data[1]];
                    data[4] = 0x4B;
                    data[5] = 0x5A;
                    change_order(data, 3);
                    uart_write_bytes(dmx->cfg.uart, inst, 4);
                    uart_wait_tx_done(dmx->cfg.uart, portMAX_DELAY);
                    esp_rom_delay_us(45);
                    uart_write_bytes(dmx->cfg.uart, data, 3);
                    break;
                }
            }
        } else if (dmx->dmx_mode == DMX_UTIL_MODE_REFLECT) {
            uart_set_line_inverse(dmx->cfg.uart, UART_SIGNAL_TXD_INV);
            esp_rom_delay_us(2);
            uart_set_line_inverse(dmx->cfg.uart, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
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
    while (1) {
        if (xQueueReceive(dmx->dmx_rx_queue, (void *)&event, portMAX_DELAY)) {
            bzero(tmp1, dmx->cfg.circ_buff_size);
            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(dmx->cfg.uart, tmp1, event.size, portMAX_DELAY);
                if (dmx->state == DMX_BREAK) {
                    xSemaphoreTake(dmx->sync_dmx, portMAX_DELAY);
                    memcpy(dmx->data, &tmp2[2], sizeof(dmx->data));
                    xSemaphoreGive(dmx->sync_dmx);
                    // on_packet_received(dmx);
                    xEventGroupSetBits(dmx->dmx_events_group, READ_DMX_BIT);
                    if (tmp1[1] == 0) {
                        dmx->state = DMX_DATA;
                        rxoffset = 0;
                        dmx->last_dmx_packet = xTaskGetTickCount();
                    }
                }
                if (dmx->state == DMX_DATA) {
                    for (int i = 0; i < event.size; i++) {
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

const uart_config_t uart_config = {.baud_rate = DMX_BAUD,
                                   .data_bits = UART_DATA_8_BITS,
                                   .parity = UART_PARITY_DISABLE,
                                   .stop_bits = UART_STOP_BITS_2,
                                   .rx_flow_ctrl_thresh = 0,
                                   .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

esp_err_t InitDMXchannel(sdmx_handle_t *dmx, sdmx_config_t *cfg)
{
    memcpy(&dmx->cfg, cfg, sizeof(sdmx_config_t));

    ESP_ERROR_CHECK(
        uart_driver_install(cfg->uart, cfg->circ_buff_size * 2, cfg->circ_buff_size * 2, 20, &dmx->dmx_rx_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(cfg->uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart, cfg->tx_pin, cfg->rx_pin, cfg->dc_pin, UART_PIN_NO_CHANGE));

    dmx->sync_dmx = xSemaphoreCreateMutex();
    dmx->dmx_events_group = xEventGroupCreate();

    gpio_pad_select_gpio(cfg->dc_pin);
    gpio_set_direction(cfg->dc_pin, GPIO_MODE_OUTPUT);
    dmx->state = DMX_IDLE;
    dmx->dmx_mode = DMX_UTIL_MODE_PROG;
    if (dmx->cfg.dmx_direction == DMX_DIRECT_OUTPUT) {
        gpio_set_level(cfg->dc_pin, 1);
        xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 4 * 1024, dmx, 8, NULL, cfg->coreID);
    } else {
        gpio_set_level(cfg->dc_pin, 0);
        xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4 * 1024, dmx, 8, NULL, cfg->coreID);
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
    uxBits = xEventGroupWaitBits(dmx->dmx_events_group, READ_DMX_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
    if (uxBits & READ_DMX_BIT)
        return ESP_OK;
    else
        return ESP_ERR_NOT_FINISHED;
}

esp_err_t PacketSent(sdmx_handle_t *dmx)
{
    EventBits_t uxBits;
    uxBits = xEventGroupWaitBits(dmx->dmx_events_group, SENT_DMX_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
    if (uxBits & SENT_DMX_BIT)
        return ESP_OK;
    else
        return ESP_ERR_NOT_FINISHED;
}

esp_err_t StartProg(sdmx_handle_t *dmx)
{
    xEventGroupSetBits(dmx->dmx_events_group, PROG_DMX_BIT);
    return ESP_OK;
}
