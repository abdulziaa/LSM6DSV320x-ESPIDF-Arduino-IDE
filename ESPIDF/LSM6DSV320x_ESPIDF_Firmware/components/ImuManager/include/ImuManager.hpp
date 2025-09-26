#pragma once

#include <vector>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6dsv320x_reg.h"

using RawAccelVec   = std::vector<float>;
using RawGyroVec    = std::vector<float>;
using QuaternionVec = std::vector<float>;
using Temperature   = float;
using EulerAngles   = double;
using TimeInSeconds = double;
using SpiPins       = std::uint8_t;
using SpiMode       = std::uint8_t;
using Flag          = bool;
using u32           = std::uint32_t;
using u16           = std::uint16_t;
using u8           = std::uint8_t;

class ImuManager
{
private:
    const SpiPins cs_, sck_, miso_, mosi_;
    RawAccelVec accel_{3, 0.0f};   
    RawGyroVec  gyro_{3, 0.0f};    
    Temperature tempInC_{0.0f};
    QuaternionVec quat_{4, 0.0f};  
    EulerAngles pitch_{0.0}, roll_{0.0}, yaw_{0.0};

    Flag initSpi();
    Flag setSpiMode();
    Flag setSpiClockSpeed();

    static u32 spiRead(void *handle, u8 reg, u8 *bufp, u16 len);
    static u32 spiWrite(void *handle, u8 reg, const u8 *bufp, u16 len);
    static u32 spidelay(u32 ms);



public:
    ImuManager(SpiPins cs = 5, SpiPins sck = 18, SpiPins miso = 19, SpiPins mosi = 23, SpiMode mode = 0);
    ~ImuManager() = default;

    void setup();
    void loop();


    RawAccelVec getRawAccelerationInMg() const { return accel_; }
    RawGyroVec getRawGyroInMdps() const { return gyro_; }
    Temperature getTemperatureInDegreeC() const { return tempInC_; }
    QuaternionVec getQuaternions() const { return quat_; }
    EulerAngles getPitch() const { return pitch_; }
    EulerAngles getRoll()  const { return roll_; }
    EulerAngles getYaw()   const { return yaw_; }
};


ImuManager::ImuManager(SpiPins cs, SpiPins sck, SpiPins miso, SpiPins mosi, SpiMode mode)
    : cs_{cs}, sck_{sck}, miso_{miso}, mosi_{mosi}
{
    
}
