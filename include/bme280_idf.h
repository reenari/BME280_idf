//
// Created by Ari Reen on 18/03/2021.
//

#ifndef BME280_IDF_H
#define ME280_IDF_H

#include "../BME280_driver/bme280.h"
struct bme280_dev *bme280idf_init(struct bme280_dev *bme280, i2c_port_t i2c_port);
int8_t bme280_idf_get_sensor(struct bme280_dev *dev, struct bme280_data *comp_data);

#endif //BME280_IDF_H
