#pragma once
#include <vector>
#include <cstring>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6dsv320x_reg.h"

#define BOOT_TIME 10 // ms
#define POLL_INTERVAL 1000 // ms
#define CNT_FOR_OUTPUT 100

using RawAccelVec         = std::vector<float>;
using RawGyroVec          = std::vector<float>;
using QuaternionVec       = std::vector<float>;
using Temperature         = float;
using EulerAngles         = double;
using TimeInSeconds       = double;
using SpiMode             = std::uint8_t;
using Flag                = bool;
using u32                 = std::uint32_t;
using u16                 = std::uint16_t;
using RawMotion           = int16_t;
using RawTemperature      = int16_t;
using u8                  = std::uint8_t;
using SpiWriteFunc        = int32_t (*)(void*, u8, const u8*, u16);
using SpiReadFunc         = int32_t (*)(void*, u8, u8*, u16);
using SpiDelayFunc        = void (*)(uint32_t);
using DevCtx              = stmdev_ctx_t;
using SpiDeviceHandle     = spi_device_handle_t;
using FilterSettingMask   = lsm6dsv320x_filt_settling_mask_t;
using Character           = char;

class ImuManager 
{
private:
    static const Character* TAG; 
    const gpio_num_t cs_, sck_, miso_, mosi_;
    const SpiMode mode_;
    RawAccelVec lowAccel_{3, 0.0f};
    RawAccelVec highAccel_{3, 0.0f};
    RawGyroVec dpsGyro_{3, 0.0f};
    Temperature tempInC_{0.0f};
    QuaternionVec quat_{4, 0.0f};
    EulerAngles pitch_{0.0}, roll_{0.0}, yaw_{0.0};
    RawMotion dataRawMotionOne[3], dataRawMotionTwo[3] , dataRawMotionThree[3];
    RawTemperature dataRawTemperature;
    lsm6dsv320x_data_ready_t status;
    bool lowAccelStatus_, highAccelStatus_, gyroStatus_, tempStatus_;
    DevCtx devCtx;
    SpiDeviceHandle spiHandle;
    static FilterSettingMask filterSettingMask;
    u8 whoamI;
    Character txBuffer[256];

    Flag initSpi();
    static int32_t spiRead(void* handle, u8 reg, u8* bufp, u16 len);
    static int32_t spiWrite(void* handle, u8 reg, const u8* bufp, u16 len);
    static void spidelay(u32 ms);
    void initImu320x(stmdev_ctx_t& devCtx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void* handle);
    Flag whoAmI();
    Flag resetImu();
    void setupImuDataRatesAndScales();
    void setupImuFilter();
    void getImuDataStatus();
    bool isLowAccelReady() const { return lowAccelStatus_; }
    bool isHighAccelReady() const { return highAccelStatus_; }
    bool isGyroReady() const { return gyroStatus_; }
    bool isTemperatureReady() const { return tempStatus_; }
    void updateLowAccelVec();
    void updateHighAccelVec();
    void updateGyroVec();
    void updateTemperatureVar();
    void updateQuaternionVec();
    void updatePitchVar();
    void updateRollVar();
    void updateYawVar();

public:
    ImuManager(gpio_num_t cs = GPIO_NUM_5, gpio_num_t sck = GPIO_NUM_18, gpio_num_t miso = GPIO_NUM_19, gpio_num_t mosi = GPIO_NUM_23, SpiMode mode = 0);
    ~ImuManager() = default;
    void setup();
    void loop();
    const RawAccelVec& getRawLowAccelerationIng() const { return lowAccel_; }
    const RawAccelVec& getRawHighAccelerationIng() const { return highAccel_; }
    const RawGyroVec& getRawGyroInMdps() const { return dpsGyro_; }
    const Temperature& getTemperatureInDegreeC() const { return tempInC_; }
    QuaternionVec getQuaternions() const { return quat_; }
    EulerAngles getPitch() const { return pitch_; }
    EulerAngles getRoll() const { return roll_; }
    EulerAngles getYaw() const { return yaw_; }
};