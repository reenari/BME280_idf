//
// Created by Ari Reen on 18/03/2021.
//

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include <esp_sleep.h>
#include <driver/i2c.h>
#include "include/bme280_idf.h"
#include "BME280_driver/bme280.h"

static const char *TAG = "BME280";

#define I2C_MASTER_FREQ_HZ 1000000


BME280_INTF_RET_TYPE bme280idf_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
BME280_INTF_RET_TYPE bme280idf_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bme280idf_delay_us(uint32_t period, void *intf_ptr);

struct idf_intf{
    i2c_port_t i2c_port;
    uint8_t dev_addr;
};

struct bme280_dev *bme280idf_init(struct bme280_dev *bme280, i2c_port_t i2c_port) {

    ESP_LOGI(TAG, "starting");

    struct idf_intf *intf_ptr= malloc(sizeof *intf_ptr);
    intf_ptr->dev_addr = BME280_I2C_ADDR_SEC;
    intf_ptr->i2c_port = i2c_port;
    bme280->intf_ptr = intf_ptr;
    bme280->intf = BME280_I2C_INTF;
    bme280->read = bme280idf_i2c_read;
    bme280->write = bme280idf_i2c_write;
    bme280->delay_us = bme280idf_delay_us;

    int8_t rslt = bme280_init(bme280);
    if(rslt != BME280_INTF_RET_SUCCESS) {
        ESP_LOGE(TAG, "init %d", rslt);
    }

    /* Recommended mode of operation: Indoor navigation */
    uint8_t settings_sel;
    bme280->settings.osr_h = BME280_OVERSAMPLING_1X;
    bme280->settings.osr_p = BME280_OVERSAMPLING_16X;
    bme280->settings.osr_t = BME280_OVERSAMPLING_2X;
    bme280->settings.filter = BME280_FILTER_COEFF_16;
    bme280->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, bme280);
    if(rslt != BME280_INTF_RET_SUCCESS) {
        ESP_LOGE(TAG,"settings %d", rslt);
    }
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, bme280);
    if(rslt != BME280_INTF_RET_SUCCESS) {
        ESP_LOGE(TAG, "mode %d", rslt);
    }

    ESP_LOGI(TAG, "rslt %d", rslt);

    return bme280;
}

BME280_INTF_RET_TYPE bme280idf_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    BME280_INTF_RET_TYPE status = BME280_INTF_RET_SUCCESS;
    const struct idf_intf *idf_intf = (struct idf_intf*)intf_ptr;
    const int8_t dev_addr = idf_intf->dev_addr;

    ESP_LOGV(TAG, "i2c read addr: %x, reg %x, len %d",dev_addr, reg_addr, len);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if ( len > 1) {
        i2c_master_read(cmd, data, len -1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len -1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(idf_intf->i2c_port, cmd, 100 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG,"i2c read fail %d", err);
        status = BME280_E_COMM_FAIL;
    }

    ESP_LOGV(TAG, "data %x from %x", data[0], *(uint8_t*)intf_ptr);
    return status;
}

BME280_INTF_RET_TYPE bme280idf_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    BME280_INTF_RET_TYPE status = BME280_INTF_RET_SUCCESS;
    const struct idf_intf *idf_intf = (struct idf_intf*)intf_ptr;
    const int8_t dev_addr = idf_intf->dev_addr;

    ESP_LOGV(TAG, "i2c write addr: %x, reg %x, len %d",dev_addr, reg_addr, len);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(idf_intf->i2c_port, cmd, 10 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG,"i2c write fail %d", err);
        status = BME280_E_COMM_FAIL;
    }

    ESP_LOGV(TAG, "wrote data %x to %x", data[0], *(uint8_t*)intf_ptr);

    return status;
}

void bme280idf_delay_us(uint32_t period, void *intf_ptr){
    if (period < 1000 * portTICK_PERIOD_MS ) {
        ets_delay_us(period); // busy wait
    } else {
        vTaskDelay(period / (1000 * portTICK_PERIOD_MS));
    }
}


int8_t bme280_idf_get_sensor(struct bme280_dev *dev, struct bme280_data *comp_data)
{
    int8_t rslt;

    /* Delay while the sensor completes a measurement */
    dev->delay_us(70 * 1000, dev->intf_ptr);

    rslt = bme280_get_sensor_data(BME280_ALL, comp_data, dev);

    if(rslt != BME280_INTF_RET_SUCCESS) {
        ESP_LOGE(TAG, "get sensor data %d", rslt);
    }
    return rslt;
}