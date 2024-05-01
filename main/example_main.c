#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_err.h>
#include "sensor_imu.h"
#include "esp_log.h"

static const char *TAG = "mpu6050 test";

AccelerationData accelerationMeasures;
GyroscopeData gyroscopeMeasures;

void app_main() {
  imu_init();
  while (true){
    get_acceleration_data(&accelerationMeasures);
    get_gyroscope_data(&gyroscopeMeasures);

    ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", accelerationMeasures.x, accelerationMeasures.y, accelerationMeasures.z);
    ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyroscopeMeasures.x, gyroscopeMeasures.y, gyroscopeMeasures.z);
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  
}
