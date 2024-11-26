#pragma once

#include <functional>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-bt.h"
#endif

class BLE {
public:
    static esp_err_t init();
    static void done();
};

class BLEGAP : public BLE {
public:
    typedef std::function<void(const esp_ble_gap_cb_param_t::ble_scan_result_evt_param *scan_result)> scan_result_cb;

    static esp_err_t init();
    static void done();

    static esp_err_t setScanParams(const esp_ble_scan_params_t *scan_params);
    static esp_err_t startScan(uint32_t duration);
    static esp_err_t stopScan();
    static void onScan(scan_result_cb cb) {
        on_scan = cb;
    }

    static esp_err_t setAdvData(const esp_ble_adv_data_t *adv_data);
    static esp_err_t setAdvRawData(const uint8_t *raw_data, uint8_t raw_data_len);
    static esp_err_t startAdvert(const esp_ble_adv_params_t *adv_params);
    static esp_err_t stopAdvert();

    static esp_err_t lastError() {
        return ble_err;
    }

protected:
    static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

    static SemaphoreHandle_t ble_sem;
    static scan_result_cb on_scan;
    static volatile esp_err_t ble_err;
};

esp_err_t BLE::init() {
    esp_err_t err;

#ifdef ARDUINO_ARCH_ESP32
    if (btStart()) {
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
            if ((err = esp_bluedroid_init() != ESP_OK)) {
                ESP_LOGE("BLE", "esp_bluedroid_init() error %d!", err);
                btStop();
                return err;
            }
        }
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
            if ((err = esp_bluedroid_enable() != ESP_OK)) {
                ESP_LOGE("BLE", "esp_bluedroid_enable() error %d!", err);
                esp_bluedroid_deinit();
                btStop();
                return err;
            }
        }
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
            return ESP_OK;
        }
    }
#else
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        const esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        if ((err = esp_bt_controller_init((esp_bt_controller_config_t*)&bt_cfg)) != ESP_OK) {
            ESP_LOGE("BLE", "esp_bt_controller_init() error %d!", err);
            return err;
        }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        if ((err = esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK)) {
            ESP_LOGE("BLE", "esp_bt_controller_enable() error %d!", err);
            esp_bt_controller_deinit();
            return err;
        }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
            if ((err = esp_bluedroid_init() != ESP_OK)) {
                ESP_LOGE("BLE", "esp_bluedroid_init() error %d!", err);
                esp_bt_controller_disable();
                esp_bt_controller_deinit();
                return err;
            }
        }
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
            if ((err = esp_bluedroid_enable() != ESP_OK)) {
                ESP_LOGE("BLE", "esp_bluedroid_enable() error %d!", err);
                esp_bluedroid_deinit();
                esp_bt_controller_disable();
                esp_bt_controller_deinit();
                return err;
            }
        }
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
            return ESP_OK;
        }
    }
#endif
    return ESP_FAIL;
}

void BLE::done() {
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
#ifdef ARDUINO_ARCH_ESP32
    btStop();
#else
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
#endif
}

esp_err_t BLEGAP::init() {
    if ((ble_err = BLE::init()) == ESP_OK) {
        if ((ble_err = esp_ble_gap_register_callback(gap_event_handler)) == ESP_OK) {
            ble_sem = xSemaphoreCreateBinary();
            return ESP_OK;
        } else
            ESP_LOGE("BLEGAP", "esp_ble_gap_register_callback() error %d!", ble_err);
        BLE::done();
    }
    return ble_err;
}

void BLEGAP::done() {
    BLE::done();
    vSemaphoreDelete(ble_sem);
}

esp_err_t BLEGAP::setScanParams(const esp_ble_scan_params_t *scan_params) {
    if ((ble_err = esp_ble_gap_set_scan_params((esp_ble_scan_params_t*)scan_params)) == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
    } else
        ESP_LOGE("BLEGAP", "esp_ble_gap_set_scan_params() error %d!", ble_err);
    return ble_err;
}

esp_err_t BLEGAP::startScan(uint32_t duration) {
    if ((ble_err = esp_ble_gap_start_scanning(duration)) == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
    } else
        ESP_LOGE("BLEGAP", "esp_ble_gap_start_scanning() error %d!", ble_err);
    return ble_err;
}

esp_err_t BLEGAP::stopScan() {
    if ((ble_err = esp_ble_gap_stop_scanning()) == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
    } else
        ESP_LOGE("BLEGAP", "esp_ble_gap_stop_scanning() error %d!", ble_err);
    return ble_err;
}

esp_err_t BLEGAP::setAdvData(const esp_ble_adv_data_t *adv_data) {
    if ((ble_err = esp_ble_gap_config_adv_data((esp_ble_adv_data_t*)adv_data)) == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
    } else
        ESP_LOGE("BLEGAP", "esp_ble_gap_config_adv_data() error %d!", ble_err);
    return ble_err;
}

esp_err_t BLEGAP::setAdvRawData(const uint8_t *raw_data, uint8_t raw_data_len) {
    if ((ble_err = esp_ble_gap_config_adv_data_raw((uint8_t*)raw_data, raw_data_len)) == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
    } else
        ESP_LOGE("BLEGAP", "esp_ble_gap_config_adv_data_raw() error %d!", ble_err);
    return ble_err;
}

esp_err_t BLEGAP::startAdvert(const esp_ble_adv_params_t *adv_params) {
    if ((ble_err = esp_ble_gap_start_advertising((esp_ble_adv_params_t*)adv_params)) == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
    } else
        ESP_LOGE("BLEGAP", "esp_ble_gap_start_advertising() error %d!", ble_err);
    return ble_err;
}

esp_err_t BLEGAP::stopAdvert() {
    if ((ble_err = esp_ble_gap_stop_advertising()) == ESP_OK) {
        xSemaphoreTake(ble_sem, portMAX_DELAY);
    } else
        ESP_LOGE("BLEGAP", "esp_ble_gap_stop_advertising() error %d!", ble_err);
    return ble_err;
}

void BLEGAP::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ble_err = param->scan_param_cmpl.status;
            xSemaphoreGive(ble_sem);
            break;
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            ble_err = param->scan_start_cmpl.status;
            xSemaphoreGive(ble_sem);
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ble_err = param->scan_stop_cmpl.status;
            xSemaphoreGive(ble_sem);
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (on_scan)
                on_scan(&param->scan_rst);
            break;
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ble_err = param->adv_data_cmpl.status;
            xSemaphoreGive(ble_sem);
            break;
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ble_err = param->adv_data_raw_cmpl.status;
            xSemaphoreGive(ble_sem);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            ble_err = param->adv_start_cmpl.status;
            xSemaphoreGive(ble_sem);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ble_err = param->adv_stop_cmpl.status;
            xSemaphoreGive(ble_sem);
            break;
        default:
            break;
    }
}

SemaphoreHandle_t BLEGAP::ble_sem = NULL;
BLEGAP::scan_result_cb BLEGAP::on_scan = NULL;
volatile esp_err_t BLEGAP::ble_err = ESP_OK;
