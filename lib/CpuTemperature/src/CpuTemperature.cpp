// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Thomas Basler and others
 */

#include "CpuTemperature.h"
#include <Arduino.h>

#if CONFIG_IDF_TARGET_ESP32
// there is no official API available on the original ESP32
extern "C" {
uint8_t temprature_sens_read();
}
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#if ESP_IDF_VERSION_MAJOR < 5
#include "driver/temp_sensor.h"
#else
#include "driver/temperature_sensor.h"
#endif
#endif

CpuTemperatureClass CpuTemperature;

float CpuTemperatureClass::read()
{
    std::lock_guard<std::mutex> lock(_mutex);

    float temperature = NAN;
    bool success = false;

#if CONFIG_IDF_TARGET_ESP32
    uint8_t raw = temprature_sens_read();
    ESP_LOGV(TAG, "Raw temperature value: %d", raw);
    temperature = (raw - 32) / 1.8f;
    success = (raw != 128);
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#if ESP_IDF_VERSION_MAJOR < 5
    temp_sensor_config_t tsens = TSENS_CONFIG_DEFAULT();
    temp_sensor_set_config(tsens);
    temp_sensor_start();
    esp_err_t result = temp_sensor_read_celsius(&temperature);
    temp_sensor_stop();
    success = (result == ESP_OK);
#else
    static temperature_sensor_handle_t sensor_handle = nullptr;
    if (sensor_handle == nullptr) {
        // Range with 1°C accuracy
        temperature_sensor_config_t sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        ESP_ERROR_CHECK(temperature_sensor_install(&sensor_config, &sensor_handle));
    }
    ESP_ERROR_CHECK(temperature_sensor_enable(sensor_handle));
    success = (temperature_sensor_get_celsius(sensor_handle, &temperature) == ESP_OK);
    ESP_ERROR_CHECK(temperature_sensor_disable(sensor_handle));
#endif
#endif

    if (success && std::isfinite(temperature)) {
        return temperature;
    } else {
        ESP_LOGD(TAG, "Ignoring invalid temperature (success=%d, value=%.1f)", success, temperature);
        return NAN;
    }
}
