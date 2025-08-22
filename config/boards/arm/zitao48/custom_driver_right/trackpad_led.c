/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpad_led

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zmk/hid_indicators.h>

LOG_MODULE_REGISTER(trackpad_led, LOG_LEVEL_INF);

/* ==== 配置参数 ==== */
#define CYCLE_BRT_STEP 5
#define CYCLE_INTERVAL_MS 20
#define CYCLE_BRT_MAX 100
#define CYCLE_BRT_MIN 10

/* ==== 子节点 LED 数量宏 ==== */
#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define INDICATOR_LED_NUM_LEDS (DT_NUM_CHILD(DT_CHOSEN(zmk_trackpad_led)))

#define HID_INDICATORS_CAPS_LOCK (1 << 1)

/* ==== LED 设备 ==== */
static const struct device *const led_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_trackpad_led));

/* ==== 内部变量 ==== */
static struct k_work_delayable cycle_work;
static int cycle_brightness = CYCLE_BRT_MIN;
static bool cycle_direction_up = true;

/* LED 亮度设置 */
static void set_led_brightness(int brightness) {
    for (int i = 0; i < INDICATOR_LED_NUM_LEDS; i++) {
        led_set_brightness(led_dev, i, brightness);
    }
}

/* 循环任务处理函数：大写锁定时循环渐亮渐暗 */
static void cycle_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    bool current_capslock = (zmk_hid_indicators_get_current_profile() & HID_INDICATORS_CAPS_LOCK);

    if (current_capslock) {
        /* 渐亮渐暗效果 */
        set_led_brightness(cycle_brightness);

        if (cycle_direction_up) {
            cycle_brightness += CYCLE_BRT_STEP;
            if (cycle_brightness >= CYCLE_BRT_MAX) {
                cycle_brightness = CYCLE_BRT_MAX;
                cycle_direction_up = false;
            }
        } else {
            cycle_brightness -= CYCLE_BRT_STEP;
            if (cycle_brightness <= CYCLE_BRT_MIN) {
                cycle_brightness = CYCLE_BRT_MIN;
                cycle_direction_up = true;
            }
        }
    } else {
        /* 未开启大写锁定时熄灭 LED */
        set_led_brightness(0);
    }

    k_work_reschedule(&cycle_work, K_MSEC(CYCLE_INTERVAL_MS));
}

/* 初始化函数 */
static int trackpad_led_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED device not ready");
        return -ENODEV;
    }

    k_work_init_delayable(&cycle_work, cycle_work_handler);
    k_work_schedule(&cycle_work, K_NO_WAIT);

    LOG_INF("Trackball LED driver initialized with %d LEDs", INDICATOR_LED_NUM_LEDS);
    return 0;
}

/* 使用 SYS_INIT 注册初始化函数 */
SYS_INIT(trackpad_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
