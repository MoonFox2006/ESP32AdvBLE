#include <Arduino.h>
#include "BLE.h"

//#define SENDER

#define PMS_DATA_COUNT  9

constexpr uint16_t SIGN = 0xAA55;

struct __attribute__((__packed__)) payload_t {
    uint16_t sign;
    int16_t temp;
    uint16_t hum;
    uint16_t press;
    uint16_t co2;
    uint16_t pms[PMS_DATA_COUNT];
};
struct __attribute__((__packed__)) frame_t {
    uint8_t len;
    uint8_t type;
    payload_t payload;
};

#ifndef SENDER
static void halt(const char *msg) {
    Serial.println(msg);
    Serial.flush();
    esp_deep_sleep_start();
}

static uint8_t crc8(uint8_t data, uint8_t crc = 0xFF) {
    crc ^= data;
    for (uint8_t bit = 8; bit > 0; --bit) {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0x31;
        else
            crc <<= 1;
    }
    return crc;
}

static uint8_t crc8(const uint8_t *data, uint8_t len, uint8_t crc = 0xFF) {
    while (len--) {
        crc = crc8(*data++, crc);
    }
    return crc;
}
#endif

void setup() {
    Serial.begin(115200);

#ifdef SENDER
    if (BLEGAP::init() != ESP_OK) {
        Serial.println("BLE init error!");
    } else {
        const esp_ble_adv_params_t adv_params = {
            .adv_int_min = 160, // 100 ms.
            .adv_int_max = 800, // 500 ms.
            .adv_type = ADV_TYPE_NONCONN_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
        };

        frame_t frame;

        frame.len = sizeof(frame_t::payload) + sizeof(frame_t::type);
        frame.type = 0xFF;
        frame.payload.sign = SIGN;
        frame.payload.temp = 2500 + (int8_t)random(256); // 25.00 C +- 1.27
        frame.payload.hum = 3500 + (int8_t)random(256); // 35.00% +- 1.27
        frame.payload.press = 1000 + (int8_t)random(256); // 1000 hPa +- 127
        frame.payload.co2 = 800 + (int8_t)random(256); // 800 ppm +- 127
        memset(frame.payload.pms, 0, sizeof(payload_t::pms));
        if ((BLEGAP::setAdvRawData((const uint8_t*)&frame, sizeof(frame)) != ESP_OK) || (BLEGAP::startAdvert(&adv_params) != ESP_OK)) {
            Serial.println("BLE advert error!");
        } else {
            Serial.println("BLE data advertising...");
            delay(2000); // 2 sec.
            BLEGAP::stopAdvert();
        }
        BLEGAP::done();
    }
    Serial.flush();
    esp_sleep_enable_timer_wakeup(30000000); // 30 sec.
    esp_deep_sleep_disable_rom_logging();
    esp_deep_sleep_start();
#else
    const esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 16, // 10 ms.
        .scan_window = 16, // 10 ms.
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };

    if (BLEGAP::init() != ESP_OK)
        halt("BLE init error!");
    BLEGAP::onScan([](const esp_ble_gap_cb_param_t::ble_scan_result_evt_param *scan_result) {
        if ((scan_result->ble_evt_type == ESP_BLE_EVT_NON_CONN_ADV) && (scan_result->adv_data_len == sizeof(frame_t))) {
            const frame_t *frame = (frame_t*)scan_result->ble_adv;

            if ((frame->len == sizeof(frame_t::payload) + sizeof(frame_t::type)) && (frame->type == 0xFF) &&
                (frame->payload.sign == SIGN)) {
                static uint8_t last_crc = 0xFF;

                uint8_t crc = crc8((const uint8_t*)&frame->payload, sizeof(payload_t));

                if (crc != last_crc) {
                    Serial.printf("BLE[%02X:%02X:%02X:%02X:*:*]\t", scan_result->bda[0], scan_result->bda[1],
                        scan_result->bda[2], scan_result->bda[3]); // , scan_result->bda[4], scan_result->bda[5]
                    Serial.printf("%.2f C, %.2f %%, %u hPa, %u ppm, PMS [", frame->payload.temp / 100.0, frame->payload.hum / 100.0,
                        frame->payload.press, frame->payload.co2);
                    for (uint8_t i = 0; i < 9; ++i) {
                        if (i)
                            Serial.print(", ");
                        Serial.printf("%u", frame->payload.pms[i]);
                    }
                    Serial.println(']');
                    last_crc = crc;
                }
            }
        }
/*
        else {
            Serial.printf("BLE[%02X:%02X:%02X:%02X:%02X:%02X] skipped\r\n", scan_result->bda[0], scan_result->bda[1],
                scan_result->bda[2], scan_result->bda[3], scan_result->bda[4], scan_result->bda[5]);
        }
*/
    });
    if ((BLEGAP::setScanParams(&scan_params) != ESP_OK) || (BLEGAP::startScan(0) != ESP_OK)) {
        BLEGAP::done();
        halt("BLE scan error!");
    }
    Serial.println("*** BLE ready ***");
#endif
}

void loop() {}
