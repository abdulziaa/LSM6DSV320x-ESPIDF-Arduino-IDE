#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ImuManager.hpp"

//=============================================================================
// PIN CONFIGURATION - Change these values to match your hardware setup
//=============================================================================
#define IMU_CS_PIN      GPIO_NUM_5      // Chip Select pin
#define IMU_SCK_PIN     GPIO_NUM_18     // SPI Clock pin
#define IMU_MISO_PIN    GPIO_NUM_19     // SPI MISO (Master In Slave Out) pin
#define IMU_MOSI_PIN    GPIO_NUM_23     // SPI MOSI (Master Out Slave In) pin
#define IMU_SPI_MODE    0               // SPI Mode (0, 1, 2, or 3)

#define POLL_INTERVAL_MS 1000 // Poll every 1 second
static const char* TAG = "IMU_APP";

extern "C" void app_main(void) 
{
    // Initialize IMU with configured pins
    ImuManager imu(IMU_CS_PIN, IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_SPI_MODE);
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