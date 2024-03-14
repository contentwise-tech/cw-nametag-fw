#include "imu.h"
#include "drivers/imu/imu.h"
#include "drivers/i2cDriver/i2cDriver.h"
#include "lsm6dsm_reg.h"

#define IMU_ADDRESS               (LSM6DSM_I2C_ADD_L & 0xFE)//0xD4

gpioStr IMU_VCC_PIN = {.port = gpioPortB, .pin = 1};

int32_t lsm6dsm_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len){
    return i2cReadRegister(IMU_ADDRESS, reg, data, len);
}

int32_t lsm6dsm_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len){
    return i2cWriteRegister(IMU_ADDRESS, reg, data, len);
}

static void IMUpowerOn(void){
    GPIO_PinOutSet(IMU_VCC_PIN.port, IMU_VCC_PIN.pin);
}

static void initAllSensors(void){
    /*  Enable Block Data Update */
    lsm6dsm_block_data_update_set(NULL, PROPERTY_ENABLE);
    /* Set Output Data Rate for Acc and Gyro */
    lsm6dsm_xl_data_rate_set(NULL, LSM6DSM_XL_ODR_12Hz5);
    lsm6dsm_gy_data_rate_set(NULL, LSM6DSM_GY_ODR_12Hz5);
    /* Set full scale */
    lsm6dsm_xl_full_scale_set(NULL, LSM6DSM_2g);
    lsm6dsm_gy_full_scale_set(NULL, LSM6DSM_2000dps);
    /* Configure filtering chain(No aux interface)
     * Accelerometer - analog filter
    */
    lsm6dsm_xl_filter_analog_set(NULL, LSM6DSM_XL_ANA_BW_400Hz);
    /* Accelerometer - LPF1 path (LPF2 not used) */
    //lsm6dsm_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSM_XL_LP1_ODR_DIV_4);
    /* Accelerometer - LPF1 + LPF2 path */
    lsm6dsm_xl_lp2_bandwidth_set(NULL, LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_100);
    /* Accelerometer - High Pass / Slope path */
    //lsm6dsm_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6dsm_xl_hp_bandwidth_set(&dev_ctx, LSM6DSM_XL_HP_ODR_DIV_100);
    /* Gyroscope - filtering chain */
    lsm6dsm_gy_band_pass_set(NULL, LSM6DSM_HP_260mHz_LP1_STRONG);
}

static void initFreeFallDetection(void){
    lsm6dsm_int1_route_t int_1_reg;
    /*  Enable LIR. */
    lsm6dsm_int_notification_set(NULL, LSM6DSM_INT_LATCHED);
    /*  Set Free Fall duration to 3 and 6 samples event duration. */
    lsm6dsm_ff_dur_set(NULL, 0x06);
    lsm6dsm_ff_threshold_set(NULL, LSM6DSM_FF_TSH_312mg);
    /*  Enable interrupt generation on Free Fall INT1 pin. */
    lsm6dsm_pin_int1_route_get(NULL, &int_1_reg);
    int_1_reg.int1_ff = PROPERTY_ENABLE;
    lsm6dsm_pin_int1_route_set(NULL, int_1_reg);
}

static void initSingleTapDetection(void){
  lsm6dsm_int1_route_t int_1_reg;
    /* Enable Tap detection on X, Y, Z */
    lsm6dsm_tap_detection_on_z_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_tap_detection_on_y_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_tap_detection_on_x_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_4d_mode_set(NULL, PROPERTY_ENABLE);
    /* Set Tap threshold to 01001b, therefore the tap threshold is
     * 562.5 mg (= 9 * FS_XL / 2 5 )
     */
    lsm6dsm_tap_threshold_x_set(NULL, 0x09);
    /* Configure Single Tap parameter
     *
     * The SHOCK field of the INT_DUR2 register is set to 10b: an
     * interrupt is generated when the slope data exceeds the programmed
     * threshold, and returns below it within 38.5 ms (= 2 * 8 / ODR_XL)
     * corresponding to the Shock time window.
     *
     * The QUIET field of the INT_DUR2 register is set to 01b: since the
     * latch mode is disabled, the interrupt is kept high for the duration
     * of the Quiet window, therefore 9.6 ms (= 1 * 4 / ODR_XL.)
     *
     * DUR already set to 0 after reset
     */
    //lsm6dsm_tap_dur_set(&dev_ctx, 0x0);
    lsm6dsm_tap_quiet_set(NULL, 0x01);
    lsm6dsm_tap_shock_set(NULL, 0x02);
    /* Enable Single Tap detection only */
    lsm6dsm_tap_mode_set(NULL, LSM6DSM_ONLY_SINGLE);
    /* Enable interrupt generation on Single Tap INT1 pin */
    lsm6dsm_pin_int1_route_get(NULL, &int_1_reg);
    int_1_reg.int1_single_tap = PROPERTY_ENABLE;
    lsm6dsm_pin_int1_route_set(NULL, int_1_reg);
}

static void initDoubleTapDetection(void){
    lsm6dsm_int1_route_t int_1_reg;

    /* Enable Tap detection on X, Y, Z */
    lsm6dsm_tap_detection_on_z_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_tap_detection_on_y_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_tap_detection_on_x_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_4d_mode_set(NULL, PROPERTY_ENABLE);
    /* Set Tap threshold to 01100b, therefore the tap threshold
     * is 750 mg (= 12 * FS_XL / 2 5 )
     */
    lsm6dsm_tap_threshold_x_set(NULL, 0x0c);
    /* Configure Double Tap parameter
     *
     * The SHOCK field of the INT_DUR2 register is set to 11b, therefore
     * the Shock time is 57.7 ms (= 3 * 8 / ODR_XL)
     *
     * The QUIET field of the INT_DUR2 register is set to 11b, therefore
     * the Quiet time is 28.8 ms (= 3 * 4 / ODR_XL)
     *
     * For the maximum time between two consecutive detected taps, the DUR
     * field of the INT_DUR2 register is set to 0111b, therefore the Duration
     * time is 538.5 ms (= 7 * 32 / ODR_XL)
     */
    lsm6dsm_tap_dur_set(NULL, 0x07);
    lsm6dsm_tap_quiet_set(NULL, 0x03);
    lsm6dsm_tap_shock_set(NULL, 0x03);
    /* Enable Double Tap detection */
    lsm6dsm_tap_mode_set(NULL, LSM6DSM_BOTH_SINGLE_DOUBLE);
    /* Enable interrupt generation on Double Tap INT1 pin */
    lsm6dsm_pin_int1_route_get(NULL, &int_1_reg);
    int_1_reg.int1_double_tap = PROPERTY_ENABLE;
    lsm6dsm_pin_int1_route_set(NULL, int_1_reg);
}

static void initPedometer(void){

    lsm6dsm_pedo_step_reset_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_pedo_threshold_set(NULL, 0x10);
    //lsm6dsm_fifo_pedo_and_timestamp_batch_set(NULL, PROPERTY_ENABLE);
    lsm6dsm_pedo_sens_set(NULL, PROPERTY_ENABLE);
}

//  https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm6dsm_STdC
int8_t initIMU(void){
    uint8_t rxBuffer[10];
    uint8_t rst;

    GPIO_PinModeSet(IMU_VCC_PIN.port, IMU_VCC_PIN.pin, gpioModePushPull, 0);
    initI2c();
    IMUpowerOn();

    delayMs(10);
    lsm6dsm_read_reg(NULL, LSM6DSM_WHO_AM_I, rxBuffer, 1);
    if (rxBuffer[0] != LSM6DSM_ID){
        return -1;
    }

    /* Restore default configuration */
    lsm6dsm_reset_set(NULL, PROPERTY_ENABLE);

    do {
        lsm6dsm_reset_get(NULL, &rst);
    } while (rst);

    /* Set XL Output Data Rate to 416 Hz */
    lsm6dsm_xl_data_rate_set(NULL, LSM6DSM_XL_ODR_416Hz);
    /* Set 2g full XL scale */
    lsm6dsm_xl_full_scale_set(NULL, LSM6DSM_2g);

    //initAllSensors();
    //initSingleTapDetection();
    initDoubleTapDetection();
    initFreeFallDetection();
    initPedometer();
    return 0;
}

float inuReadTemperature(void){
  int16_t temperature;

  volatile lsm6dsm_reg_t reg;

  lsm6dsm_status_reg_get(NULL, &reg.status_reg);
  if (reg.status_reg.tda){
      if (lsm6dsm_temperature_raw_get(NULL, &temperature) != 0){
          return -128;
      }
      return lsm6dsm_from_lsb_to_celsius(temperature);
  }

  return -128;
}

int8_t readFreeFall(void){
    lsm6dsm_all_sources_t all_source;
    /* Check if Free Fall events. */
    if (lsm6dsm_all_sources_get(NULL, &all_source) == 0){
        if (all_source.wake_up_src.ff_ia) {
          return 1;
        }
        return 0;
    }

    return -1;
}

int8_t readSingleTap(void){
    lsm6dsm_all_sources_t all_source;
    /* Check if Single Tap events */
    if (lsm6dsm_all_sources_get(NULL, &all_source) == 0){
        if (all_source.tap_src.single_tap){
            return 1;
        }
        return 0;
    }
    return -1;
}

int8_t readDoubleTap(void){
    lsm6dsm_all_sources_t all_source;
    /* Check if Single Tap events */
    if (lsm6dsm_all_sources_get(NULL, &all_source) == 0){
        if (all_source.tap_src.single_tap){
            return 1;
        }
        if (all_source.tap_src.double_tap){
            return 2;
        }
        return 0;
    }
    return -1;
}

uint8_t readPedometer(void){
  lsm6dsm_all_sources_t all_source;
  /* Check if Single Tap events */
  if (lsm6dsm_all_sources_get(NULL, &all_source) == 0){
      if (all_source.func_src1.step_detected){
          return all_source.func_src1.step_count_delta_ia;
      }
  }
  return 0;
}
