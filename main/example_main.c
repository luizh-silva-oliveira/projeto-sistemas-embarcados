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

/*Estados do sistema*/
typedef enum {
    GET_IMU_DATA = 0, CALCULATE_QUATERNIONS, CALCULATE_EULER_ANGLES
} States;

void app_main() {
  imu_init();

  static States state = GET_IMU_DATA;
  static const int quantStates = 3;//variavel que informa a quantidade de estados

  while (true){
    for (int i = 0; i < quantStates; i++){
      switch(state) {
        case GET_IMU_DATA:
          esp_err_t status = get_acceleration_data(&accelerationMeasures);
          if (status == ESP_OK) status = get_gyroscope_data(&gyroscopeMeasures);
          if (status == ESP_OK) status = get_imu_data(&gyroAcellMeasures);
          if (status == ESP_OK) state = CALCULATE_QUATERNIONS;
          //Saída dos dados capturados
          ESP_LOGI(TAG_SENSOR_IMU, "acce_:%.2f, acce_y:%.2f, acce_z:%.2f", accelerationMeasures.x, accelerationMeasures.y, accelerationMeasures.z);
          ESP_LOGI(TAG_SENSOR_IMU, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyroscopeMeasures.x, gyroscopeMeasures.y, gyroscopeMeasures.z);
          
          ESP_LOGI(TAG_IMU_TOOLS, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", gyroAcellMeasures.accel_x, gyroAcellMeasures.accel_y, gyroAcellMeasures.accel_z);
          ESP_LOGI(TAG_IMU_TOOLS, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f\n", gyroAcellMeasures.gyro_x, gyroAcellMeasures.gyro_y, gyroAcellMeasures.gyro_z);
          break;

        case CALCULATE_QUATERNIONS:
          status = calculate_quaternion(&gyroAcellMeasures, &quaternion);
          if (status == ESP_OK) status = get_quaternion(&quaternionToGetTest);
          if (status == ESP_OK) state = CALCULATE_EULER_ANGLES;

          //Saída dos dados capturados
          ESP_LOGI(TAG_IMU_TOOLS, "quat_w:%.2f, quat_x:%.2f, quat_y:%.2f, quat_z:%.2f", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
          ESP_LOGI(TAG_IMU_TOOLS, "quat_get_w:%.2f, quat_get_x:%.2f, quat_get_y:%.2f, quat_get_z:%.2f", quaternionToGetTest.w, quaternionToGetTest.x, quaternionToGetTest.y, quaternionToGetTest.z);
          break;
          
        case CALCULATE_EULER_ANGLES:
          status = quaternion_to_euler(&quaternion, &eulerAngle);
          if (status == ESP_OK) state = GET_IMU_DATA;

          //Saída dos dados capturados
          ESP_LOGI(TAG_IMU_TOOLS, "pitch:%.2f, yaw:%.2f, roll:%.2f\n", eulerAngle.pitch, eulerAngle.yaw, eulerAngle.roll);
          break;
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); //delay para evitar de poluir o terminal com muitos prints
  }
  
}
