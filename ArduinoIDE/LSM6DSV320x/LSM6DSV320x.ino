#include <Arduino.h>
#include <SPI.h>
#include "lsm6dsv320x_reg.h" // Ensure this header is available or define registers manually

// Pin definitions for ESP32
#define CS_PIN 5    // Chip Select
#define SCK_PIN 18  // SPI Clock
#define MISO_PIN 19 // SPI MISO
#define MOSI_PIN 23 // SPI MOSI
#define INT1_PIN 4  // Not used (kept for reference, can remove if desired)
#define INT2_PIN 5  // Not used (kept for reference, can remove if desired)

// SPI and sensor context
static stmdev_ctx_t dev_ctx;
static SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // 1MHz, Mode 0

// Sensor data variables
static int16_t data_raw_motion[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI;
static char tx_buffer[1000];
static lsm6dsv320x_filt_settling_mask_t filt_settling_mask;
static uint8_t lg_xl_data_valid = 0;
static uint8_t hg_xl_data_valid = 0;
static uint8_t gyro_data_valid = 0;
static uint8_t temp_data_valid = 0;

// Constants from original code
#define BOOT_TIME 10  // Sensor boot time in ms
#define CNT_FOR_OUTPUT 100  // Number of samples for output averaging
#define POLL_INTERVAL 10   // Polling interval in ms (adjust based on ODR)

void setup() {
  // Initialize Serial for output
  Serial.begin(115200);
  while (!Serial);
  delay(2000); // Wait for Serial to stabilize and boot messages to clear

  // Initialize SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect SPI initially

  // Initialize sensor context
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SPI;

  // Wait sensor boot time
  platform_delay(BOOT_TIME);

  // Check device ID
  lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);
  Serial.print("WHOAMI: 0x");
  Serial.println(whoamI, HEX); // Debug WHOAMI
  if (whoamI != LSM6DSV320X_ID) {
    Serial.println("Device ID mismatch!");
    // while (1);
  }

  // Reset sensor
  lsm6dsv320x_reset_t rst;
  lsm6dsv320x_reset_set(&dev_ctx, LSM6DSV320X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv320x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV320X_READY);

  // Configure sensor
  lsm6dsv320x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv320x_xl_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_60Hz);
  lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_960Hz, 1);
  lsm6dsv320x_gy_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_120Hz);
  lsm6dsv320x_xl_full_scale_set(&dev_ctx, LSM6DSV320X_2g);
  lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g);
  lsm6dsv320x_gy_full_scale_set(&dev_ctx, LSM6DSV320X_2000dps);

  // Configure filtering
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv320x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv320x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV320X_GY_ULTRA_LIGHT);
  lsm6dsv320x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv320x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV320X_XL_STRONG);

  // Disable interrupt routing (since no interrupts are used)
  lsm6dsv320x_pin_int_route_t pin_int = {0};
  lsm6dsv320x_pin_int1_route_hg_set(&dev_ctx, &pin_int); // Set to disable

  Serial.println("LSM6DSV32X initialized successfully!");
}

void loop() {
  
  // Poll data readiness every POLL_INTERVAL ms
  static unsigned long lastPollTime = 0;
  if (millis() - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = millis();

    lsm6dsv320x_data_ready_t status;
    lsm6dsv320x_flag_data_ready_get(&dev_ctx, &status);

    hg_xl_data_valid = status.drdy_hgxl;
    lg_xl_data_valid = status.drdy_xl;
    gyro_data_valid = status.drdy_gy;
    temp_data_valid = status.drdy_temp;

    // Serial.print("HG XL Data Valid: ");
    // Serial.println(hg_xl_data_valid);
    // Serial.print("LG XL Data Valid: ");
    // Serial.println(lg_xl_data_valid);
    // Serial.print("Gyro Data Valid: ");
    // Serial.println(gyro_data_valid);
    // Serial.print("Temp Data Valid: ");
    // Serial.println(temp_data_valid);

    if (lg_xl_data_valid) {
      lg_xl_data_valid = 0;
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      lsm6dsv320x_acceleration_raw_get(&dev_ctx, data_raw_motion);
      acceleration_mg[0] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[0]);
      acceleration_mg[1] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[1]);
      acceleration_mg[2] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[2]);

      Serial.print("LOW Accel : ");
      Serial.print(acceleration_mg[0]/1000);
      Serial.print("  "); Serial.print(acceleration_mg[1]/1000);
      Serial.print("  ");Serial.println(acceleration_mg[2]/1000);
    }

    if (hg_xl_data_valid) {
      hg_xl_data_valid = 0;
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      lsm6dsv320x_hg_acceleration_raw_get(&dev_ctx, data_raw_motion);
      Serial.print("Raw HG XL: ");
    // Serial.print(data_raw_motion[0]); Serial.print(":");
    // Serial.print(data_raw_motion[1]); Serial.print(":");
    // Serial.println(data_raw_motion[2]);
      acceleration_mg[0] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[0]);
      acceleration_mg[1] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[1]);
      acceleration_mg[2] = lsm6dsv320x_from_fs256_to_mg(data_raw_motion[2]);
      // Serial.printf("XL: %d:%d:%d\n", acceleration_mg[0],acceleration_mg[1],acceleration_mg[2]);
      Serial.print("HI Accel : ");
      Serial.print(acceleration_mg[0]/1000);
      Serial.print("  "); Serial.print(acceleration_mg[1]/1000);
      Serial.print("  ");Serial.println(acceleration_mg[2]/1000);
    }

    if (gyro_data_valid) {
      gyro_data_valid = 0;
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      lsm6dsv320x_angular_rate_raw_get(&dev_ctx, data_raw_motion);
      
      angular_rate_mdps[0] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[0]);
      angular_rate_mdps[1] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[1]);
      angular_rate_mdps[2] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[2]);
      // Serial.printf("GY: %d:%d:%d\n", angular_rate_mdps[0],angular_rate_mdps[1],angular_rate_mdps[2]);
      Serial.print("Gyro : ");
      Serial.print(angular_rate_mdps[0]);
      Serial.print(" "); Serial.print(angular_rate_mdps[1]);
      Serial.print(" ");Serial.println(angular_rate_mdps[2]);
    }

    if (temp_data_valid) {
      temp_data_valid = 0;
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm6dsv320x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lsm6dsv320x_from_lsb_to_celsius(data_raw_temperature);
      Serial.print("Temperature_degC :"); Serial.println(temperature_degC);
    }
  }

  static uint16_t lowg_xl_cnt = 0, hg_xl_cnt = 0, gyro_cnt = 0, temp_cnt = 0;
  static double lowg_xl_sum[3] = {0}, hg_xl_sum[3] = {0}, gyro_sum[3] = {0}, temp_sum = 0;

  if (lg_xl_data_valid) {
    lowg_xl_sum[0] += acceleration_mg[0];
    lowg_xl_sum[1] += acceleration_mg[1];
    lowg_xl_sum[2] += acceleration_mg[2];
    lowg_xl_cnt++;
  }

  if (hg_xl_data_valid) {
    hg_xl_sum[0] += acceleration_mg[0];
    hg_xl_sum[1] += acceleration_mg[1];
    hg_xl_sum[2] += acceleration_mg[2];
    hg_xl_cnt++;
  }

  if (gyro_data_valid) {
    gyro_sum[0] += angular_rate_mdps[0];
    gyro_sum[1] += angular_rate_mdps[1];
    gyro_sum[2] += angular_rate_mdps[2];
    gyro_cnt++;
  }

  if (temp_data_valid) {
    temp_sum += temperature_degC;
    temp_cnt++;
  }

  if (lowg_xl_cnt >= CNT_FOR_OUTPUT) {
    acceleration_mg[0] = lowg_xl_sum[0] / lowg_xl_cnt;
    acceleration_mg[1] = lowg_xl_sum[1] / lowg_xl_cnt;
    acceleration_mg[2] = lowg_xl_sum[2] / lowg_xl_cnt;
    snprintf(tx_buffer, sizeof(tx_buffer), "lg xl (media of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
             lowg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    // Serial.print(tx_buffer);
    lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
    lowg_xl_cnt = 0;

    acceleration_mg[0] = hg_xl_sum[0] / hg_xl_cnt;
    acceleration_mg[1] = hg_xl_sum[1] / hg_xl_cnt;
    acceleration_mg[2] = hg_xl_sum[2] / hg_xl_cnt;
    snprintf(tx_buffer, sizeof(tx_buffer), "hg xl (media of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
             hg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    // Serial.print(tx_buffer);
    hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
    hg_xl_cnt = 0;

    angular_rate_mdps[0] = gyro_sum[0] / gyro_cnt;
    angular_rate_mdps[1] = gyro_sum[1] / gyro_cnt;
    angular_rate_mdps[2] = gyro_sum[2] / gyro_cnt;
    snprintf(tx_buffer, sizeof(tx_buffer), "gyro (media of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
             gyro_cnt, angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
    // Serial.print(tx_buffer);
    gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
    gyro_cnt = 0;

    temperature_degC = temp_sum / temp_cnt;
    snprintf(tx_buffer, sizeof(tx_buffer), "Temperature (media of %d samples) [degC]:%6.2f\r\n\r\n",
             temp_cnt, temperature_degC);
    // Serial.print(tx_buffer);
    temp_cnt = 0;
    temp_sum = 0.0;
  }
  delay(300);
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F); // Write command (clear MSB)
  SPI.transfer((uint8_t*)bufp, len);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80); // Read command (set MSB)
  SPI.transfer(bufp, len);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return 0;
}

static void tx_com(uint8_t *tx_buffer, uint16_t len) {
  Serial.write(tx_buffer, len);
}

static void platform_delay(uint32_t ms) {
  delay(ms);
}

static void platform_init(void) {
  // No specific initialization needed for ESP32
}