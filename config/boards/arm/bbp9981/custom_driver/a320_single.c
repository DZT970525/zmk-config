/*
 * a320_input_handler.c - Handle Avago A320 motion sensor over I2C
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

LOG_MODULE_REGISTER(a320_single, LOG_LEVEL_INF);

// === 配置 Motion GPIO ===
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 2

static const struct device *motion_gpio_dev;

/* ==== I2C Device ==== */
#define A320_NODE DT_INST(0, avago_a320)
static const struct i2c_dt_spec a320_i2c = I2C_DT_SPEC_GET(A320_NODE);

/* ==== Touch 状态标志 ==== */
static bool touched = false;

/* ==== 初始化 ==== */
static int a320_init(void) {
    LOG_INF("Initializing A320 input handler...");

    motion_gpio_dev = DEVICE_DT_GET(MOTION_GPIO_NODE);
    if (!device_is_ready(motion_gpio_dev)) {
        LOG_ERR("Motion GPIO device not ready");
        return -ENODEV;
    }
    gpio_pin_configure(motion_gpio_dev, MOTION_GPIO_PIN, GPIO_INPUT | GPIO_PULL_UP);

    if (!device_is_ready(a320_i2c.bus)) {
        LOG_ERR("I2C bus not ready for A320 sensor");
        return -ENODEV;
    }

    LOG_INF("A320 sensor initialized at addr=0x%02x", a320_i2c.addr);
    return 0;
}

/* ==== 读运动数据 ==== */
static int a320_read_motion(int16_t *dx, int16_t *dy) {

    uint8_t buf[7] = {0};

    /* 触发读取：先写寄存器地址 0x0A */
    uint8_t reg = 0x0A;
    int ret = i2c_write_dt(&a320_i2c, &reg, 1);
    if (ret < 0) {
        LOG_ERR("Failed to write register address 0x0A: %d", ret);
        return ret;
    }

    /* 再从 0x0A 连续读取 7 字节数据 */
    ret = i2c_burst_read_dt(&a320_i2c, 0x0A, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read from 0x0A: %d", ret);
        return ret;
    }

    /* 第二个和第四个字节分别是 dx/dy */
    *dx = (int8_t)buf[1];
    *dy = (int8_t)buf[3];

    return 0;
}

/* ==== 外部接口：是否触摸 ==== */
bool tp_is_touched(void) { return touched; }

/* ==== 驱动轮询线程 ==== */
void a320_polling_thread(void) {
    int16_t dx, dy;

    if (a320_init() < 0) {
        return;
    }

    while (1) {
        int pin_state = gpio_pin_get(motion_gpio_dev, MOTION_GPIO_PIN);
        uint32_t now = k_uptime_get_32();
        if (pin_state == 0) {
            if (a320_read_motion(&dx, &dy) == 0) {
                if (dx || dy) {
                    LOG_DBG("Motion dx=%d dy=%d", dx, dy);
                    /* 上报到 HID 鼠标事件 */
                    zmk_hid_mouse_scroll_set(0, 0);
                    zmk_hid_mouse_movement_set(0, 0);
                    zmk_hid_mouse_movement_update(dx, dy);
                    zmk_endpoints_send_mouse_report();
                    touched = true;
                }
            }
        } else {
            // === 触控板未被触摸，清空鼠标状态，避免后续误触 ===
            zmk_hid_mouse_scroll_set(0, 0);
            zmk_hid_mouse_movement_set(0, 0);
            zmk_endpoints_send_mouse_report();
            touched = false;
        }

        k_msleep(10); /* 10ms 轮询 */
    }
}

/* ==== 启动线程 ==== */
K_THREAD_DEFINE(a320_thread_id, 1024, a320_polling_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0,
                0);
