#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ImuManager.hpp"

#define POLL_INTERVAL_MS 1000 // Poll every 1 second
static const char* TAG = "IMU_APP";

extern "C" void app_main(void) 
{
    ImuManager imu;
    imu.setup();

    printf("IMU initialized, starting data polling...");

    while (1) 
    {

        imu.loop();

        const RawAccelVec& lowAccel = imu.getRawLowAccelerationIng();
        // const RawAccelVec& highAccel = imu.getRawHighAccelerationIng();
        // const RawGyroVec& gyro = imu.getRawGyroInMdps();
        // const Temperature& temp = imu.getTemperatureInDegreeC();

        // Print sensor data
        printf("Low Accel [g]: %.3f, %.3f, %.3f", 
                 lowAccel[0] , lowAccel[1] , lowAccel[2] );
        // ESP_LOGI(TAG, "High Accel [g]: %.3f, %.3f, %.3f", 
        //          highAccel[0] , highAccel[1], highAccel[2] );
        // ESP_LOGI(TAG, "Gyro [dps]: %.2f, %.2f, %.2f", 
        //          gyro[0] , gyro[1] , gyro[2] );
        // ESP_LOGI(TAG, "Temperature [°C]: %.2f", temp);

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}