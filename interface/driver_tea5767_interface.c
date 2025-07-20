/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_tea5767_interface.c
 * @brief     driver tea5767 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-11-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/11/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_tea5767_interface.h"

#include <stdarg.h>
#include <driver/i2c_master.h>
#include "freertos/FreeRTOS.h"

#include "iic.h"

/**
 * @brief chip address definition
 */
#define TEA5767_ADDRESS             0x60        /**< iic device address, 这是左移前的地址 */

i2c_master_bus_handle_t i2c_master_bus_handle;
i2c_master_dev_handle_t i2c_master_dev_handle;

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t tea5767_interface_iic_init(void) {
    // 初始化i2c总线
    i2c_master_bus_handle = iic_init_master_bus(I2C_NUM_1);
    // 初始化iic设备
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TEA5767_ADDRESS, // esp32的i2c驱动会自动左移，所以填写左移前的地址
        .scl_speed_hz = 400'000, // tea5767最大频率400 khz
        .scl_wait_us = 0,
    };
    auto ret = i2c_master_bus_add_device(i2c_master_bus_handle, &device_config, &i2c_master_dev_handle);
    ESP_ERROR_CHECK(ret);
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t tea5767_interface_iic_deinit(void) {
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(i2c_master_dev_handle));
    return 0;
}

/**
 * @brief     interface iic bus write command
 * @param[in] addr iic device write address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t tea5767_interface_iic_write_cmd(uint8_t addr, uint8_t* buf, uint16_t len) {
    if (addr != TEA5767_ADDRESS) {
        return 1;
    }
    auto ret = i2c_master_transmit(i2c_master_dev_handle, buf, len, -1);
    ESP_ERROR_CHECK(ret);
    return 0;
}

/**
 * @brief      interface iic bus read command
 * @param[in]  addr iic device write address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t tea5767_interface_iic_read_cmd(uint8_t addr, uint8_t* buf, uint16_t len) {
    if (addr != TEA5767_ADDRESS) {
        return 1;
    }
    auto ret = i2c_master_receive(i2c_master_dev_handle, buf, len, -1);
    ESP_ERROR_CHECK(ret);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void tea5767_interface_delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void tea5767_interface_debug_print(const char* const fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}
