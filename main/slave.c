/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// FreeModbus Slave Example ESP32

#include <stdio.h>
#include <stdint.h>
#include "esp_err.h"
#include "mbcontroller.h"       // for mbcontroller defines and api
#include "esp_log.h"            // for log_write
#include "sdkconfig.h"


#define MY_REGISTERS_SIZE (20 * 100 + 20 + 0x4000)  //за да покрива до P20.20 16 и 32 бита

// преобразува номера на 16 битов параметър от DC ELL задвижване в адрес на modbus регистър
#define PARAM16_NUM_TO_REG_ADDRESS(group, parametr) (group * 100 + parametr - 1)

// This file defines structure of modbus parameters which reflect correspond modbus address space
// for each modbus register type (coils, discreet inputs, holding registers, input registers)
#pragma pack(push, 1)
typedef struct
{
    uint16_t my_regs[MY_REGISTERS_SIZE];
} holding_reg_params_t;
#pragma pack(pop)

holding_reg_params_t holding_reg_params = { 0 };

#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_SLAVE_ADDR   (CONFIG_MB_SLAVE_ADDR)      // The address of device in Modbus network
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.

// Defines below are used to define register start address for each type of Modbus registers
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) >> 1))

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_READ_MASK                        (MB_EVENT_HOLDING_REG_RD)
#define MB_WRITE_MASK                       (MB_EVENT_HOLDING_REG_WR)
#define MB_READ_WRITE_MASK                  (MB_READ_MASK | MB_WRITE_MASK)

static const char *TAG = "SLAVE_TEST";

static portMUX_TYPE param_lock = portMUX_INITIALIZER_UNLOCKED;

// Set register values into known state
static void setup_reg_data(void)
{
    // Define initial state of parameters
    holding_reg_params.my_regs[PARAM16_NUM_TO_REG_ADDRESS(1, 14)] = 4998;   //P01.14 [Hz] Line frequency = 49.98Hz
}

// An example application of Modbus slave. It is based on freemodbus stack.
// See deviceparams.h file for more information about assigned Modbus parameters.
// These parameters can be accessed from main application and also can be changed
// by external Modbus master host.
void app_main(void)
{
    mb_param_info_t reg_info; // keeps the Modbus registers access information
    mb_communication_info_t comm_info; // Modbus communication parameters
    mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    
    void* mbc_slave_handler = NULL;

    ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave_handler)); // Initialization of Modbus controller

    // Setup communication parameters and start stack
#if CONFIG_MB_COMM_MODE_ASCII
    comm_info.mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
    comm_info.mode = MB_MODE_RTU,
#endif
    comm_info.slave_addr = MB_SLAVE_ADDR;
    comm_info.port = MB_PORT_NUM;
    comm_info.baudrate = MB_DEV_SPEED;
    comm_info.parity = MB_PARITY_NONE;
    ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

    // The code below initializes Modbus register area descriptors
    // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
    // Initialization should be done for each supported Modbus register area according to register map.
    // When external master trying to access the register in the area that is not initialized
    // by mbc_slave_set_descriptor() API call then Modbus stack
    // will send exception response for this register area.
    reg_area.type = MB_PARAM_HOLDING; // Set type of register area
    reg_area.start_offset = HOLD_OFFSET(my_regs); // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_params.my_regs; // Set pointer to storage instance
    reg_area.size = (size_t)(MY_REGISTERS_SIZE << 1);
    // ESP_LOGE(TAG, "start_offset = %u INST_ADDR:0x%" PRIx32 ", SIZE:%u", reg_area.start_offset, (uint32_t)reg_area.address, (unsigned)reg_area.size);
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    setup_reg_data(); // Set values into known state

    // Starts of modbus controller and stack
    ESP_ERROR_CHECK(mbc_slave_start());

    // Set UART pin numbers
    ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD,
                            CONFIG_MB_UART_RXD, CONFIG_MB_UART_RTS,
                            UART_PIN_NO_CHANGE));

    // Set UART driver mode to Half Duplex
    ESP_ERROR_CHECK(uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "Modbus slave stack initialized.");
    ESP_LOGI(TAG, "Start modbus test...");

    while (1) {
        // Check for read/write events of Modbus master for certain events
        (void)mbc_slave_check_event(MB_READ_WRITE_MASK);
        ESP_ERROR_CHECK_WITHOUT_ABORT(mbc_slave_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
        const char* rw_str = (reg_info.type & MB_READ_MASK) ? "READ" : "WRITE";
        // Filter events and process them accordingly
        if(reg_info.type & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD)) {
            // Get parameter information from parameter queue
            ESP_LOGD(TAG, "HOLDING %s (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                            rw_str,
                            reg_info.time_stamp,
                            (unsigned)reg_info.mb_offset,
                            (unsigned)reg_info.type,
                            (uint32_t)reg_info.address,
                            (unsigned)reg_info.size);

            uint16_t addres = reg_info.mb_offset;
            for (int i = 0; i < reg_info.size; i++, addres++) {
                if (addres < MY_REGISTERS_SIZE) {
                    if (reg_info.type & MB_EVENT_HOLDING_REG_RD) {  // само при четене
                        switch (addres) {
                        case PARAM16_NUM_TO_REG_ADDRESS(1, 14) : //P01.14 [Hz] Line frequency ~ 50.00Hz
                            portENTER_CRITICAL(&param_lock);    // не чи ни трябват
                            if (holding_reg_params.my_regs[PARAM16_NUM_TO_REG_ADDRESS(1, 14)] < 5010)
                                holding_reg_params.my_regs[PARAM16_NUM_TO_REG_ADDRESS(1, 14)] ++;
                            else
                                holding_reg_params.my_regs[PARAM16_NUM_TO_REG_ADDRESS(1, 14)] = 4990;
                            portEXIT_CRITICAL(&param_lock);
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
        }
    }
    // Destroy of Modbus controller on alarm
    ESP_LOGI(TAG,"Modbus controller destroyed.");
    vTaskDelay(100);
    ESP_ERROR_CHECK(mbc_slave_destroy());
}