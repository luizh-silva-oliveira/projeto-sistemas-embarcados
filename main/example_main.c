#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_err.h>
#include "esp_log.h"
#include "sensor_imu.h"
#include "imu_tools.h"

static const char *TAG_SENSOR_IMU = "mpu6050 sensor_imu test";
static const char *TAG_IMU_TOOLS = "mpu6050 imu_tools test";

AccelerationData accelerationMeasures;
GyroscopeData gyroscopeMeasures;
IMUData gyroAcellMeasures;

Quaternion quaternion;
Quaternion quaternionToGetTest;
EulerAngle eulerAngle;

void app_main() {
  imu_init();
  while (true){
    //exemplos sensor_imu
    get_acceleration_data(&accelerationMeasures);
    get_gyroscope_data(&gyroscopeMeasures);

    ESP_LOGI(TAG_SENSOR_IMU, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", accelerationMeasures.x, accelerationMeasures.y, accelerationMeasures.z);
    ESP_LOGI(TAG_SENSOR_IMU, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyroscopeMeasures.x, gyroscopeMeasures.y, gyroscopeMeasures.z);
    
    //exemplos imu_tools
    get_imu_data(&gyroAcellMeasures);
    calculate_quaternion(&gyroAcellMeasures, &quaternion);
    quaternion_to_euler(&quaternion, &eulerAngle);
    //get_quaternion(&quaternionToGetTest);

    ESP_LOGI(TAG_IMU_TOOLS, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", gyroAcellMeasures.accel_x, gyroAcellMeasures.accel_y, gyroAcellMeasures.accel_z);
    ESP_LOGI(TAG_IMU_TOOLS, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyroAcellMeasures.gyro_x, gyroAcellMeasures.gyro_y, gyroAcellMeasures.gyro_z);
    
    ESP_LOGI(TAG_IMU_TOOLS, "quat_w:%.2f, quat_x:%.2f, quat_y:%.2f, quat_z:%.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    //ESP_LOGI(TAG_IMU_TOOLS, "quat_get_w:%.2f, quat_get_x:%.2f, quat_get_y:%.2f, quat_get_z:%.2f\n", quaternionToGetTest.w, quaternionToGetTest.x, quaternionToGetTest.y, quaternionToGetTest.z);
    ESP_LOGI(TAG_IMU_TOOLS, "pitch:%.2f, yaw:%.2f, roll:%.2f\n", eulerAngle.pitch, eulerAngle.yaw, eulerAngle.roll);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  
}
