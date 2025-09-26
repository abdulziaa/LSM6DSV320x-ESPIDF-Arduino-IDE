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
using u8            = std::uint8_t;
using SpiWriteFunc     = u32(*)(void*, u8, const u8*, u16);
using SpiReadFunc      = u32(*)(void*, u8, u8*, u16);
using SpiDelayFunc     = void(*)(uint32_t);
using DataReadyStatus  = lsm6dsv320x_data_ready_t;

class ImuManager
{
private:
    const SpiPins cs_, sck_, miso_, mosi_;
    RawAccelVec accel_{3, 0.0f};   
    RawGyroVec  gyro_{3, 0.0f};    
    Temperature tempInC_{0.0f};
    QuaternionVec quat_{4, 0.0f};  
    EulerAngles pitch_{0.0}, roll_{0.0}, yaw_{0.0};
    DataReadyStatus status, lowAccelStatus_, highAccelStatus_, gyroStatus_, tempStatus_;

    Flag initSpi();
    Flag setSpiMode();
    Flag setSpiClockSpeed();

    static u32 spiRead(void *handle, u8 reg, u8 *bufp, u16 len);
    static u32 spiWrite(void *handle, u8 reg, const u8 *bufp, u16 len);
    static u32 spidelay(u32 ms);

    void initImu320x(stmdev_ctx_t& dev_ctx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void* handle);
    Flag whoAmI();
    Flag resetImu();
    void setupImuDataRatesAndScales();
    void setupImuFilter();
    DataReadyStatus getImuDataStatus();
    DataReadyStatus isLowAccelReady()    const { return lowAccelStatus_; }
    DataReadyStatus isHighAccelReady()   const { return highAccelStatus_; }
    DataReadyStatus isGyroReady()        const { return gyroStatus_; }
    DataReadyStatus isTemperatureReady() const { return tempStatus_; }

    void updateLowAccelVec();
    void updateHighAccelVec();
    void updateGyroVec();
    void updateTemperatureVar();
    void updateQuaternionVec();
    void updatePitchVar();
    void updateRollVar();
    void updateYawVar();




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

void ImuManager::setup()
{

}
void ImuManager::loop()
{

}


Flag ImuManager::initSpi()
{

}

Flag ImuManager::setSpiMode()
{

}

Flag ImuManager::setSpiClockSpeed()
{

}

static u32 ImuManager::spiRead(void *handle, u8 reg, u8 *bufp, u16 len)
{

}

static u32 ImuManager::spiWrite(void *handle, u8 reg, const u8 *bufp, u16 len)
{

}

static u32 ImuManager::spidelay(u32 ms)
{

}

void ImuManager::initImu320x(stmdev_ctx_t& dev_ctx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void* handle)
{
    // dev_ctx.write_reg = write;
    // dev_ctx.read_reg  = read;
    // dev_ctx.mdelay    = delay;
    // dev_ctx.handle    = handle;
}

Flag ImuManager::whoAmI()
{

}

Flag ImuManager::resetImu()
{

}
void setupImuDataRatesAndScales()
{

}

void ImuManager::setupImuFilter()
{

}

DataReadyStatus ImuManager::getImuDataStatus()
{
    // lsm6dsv320x_flag_data_ready_get(&dev_ctx, &status);
    // lowAccelStatus_  = status.drdy_xl;
    // highAccelStatus_ = status.drdy_hgxl;
    // gyroStatus_      = status.drdy_gy;
    // tempStatus_      = status.drdy_temp;
}