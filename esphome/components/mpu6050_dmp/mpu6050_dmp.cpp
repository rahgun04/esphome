#include "mpu6050_dmp.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MPU6050

namespace esphome {
namespace mpu6050_dmp {

static const char *const TAG = "mpu6050";

#define MPU6050_DMP_ERROR_CHECK(func, error_msg) \
  if ((func) != 0) { \
    this->mark_failed(error_msg); \
    return; \
  }

int MPU6050_DMP::i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, const uint8_t *data) {
  if (data == nullptr || length == 0) {
    ESP_LOGE("MPU6050", "i2c write: invalid args (len=%d, data=%p)", length, data);
    return -1;
  }

  auto err = this->write_register(reg_addr, data, length);

  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE("MPU6050", "i2c write failed at reg 0x%02X", reg_addr);
    return -1;
  }

  return 0;
}
void esphome::mpu6050_dmp::MPU6050_DMP::delay_ms(unsigned long ms) { delay(ms); }

void MPU6050_DMP::get_ms(unsigned long *ms) { ms[0] = millis(); }

int MPU6050_DMP::i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data) {
  if (data == nullptr || length == 0) {
    ESP_LOGE("MPU6050", "i2c read: invalid args (len=%d, data=%p)", length, data);
    return -1;
  }

  auto err = this->read_register(reg_addr, data, length);

  if (err != esphome::i2c::ERROR_OK) {
    ESP_LOGE("MPU6050", "i2c read failed at reg 0x%02X", reg_addr);
    return -1;
  }

  return 0;
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_init(struct int_param_s *int_param) {
  unsigned char data[6];

  /* Reset device. */
  data[0] = BIT_RESET;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
    return -1;
  delay_ms(100);

  /* Wake up chip. */
  data[0] = 0x00;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
    return -1;

  st.chip_cfg.accel_half = 0;

  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg.sensors = 0xFF;
  st.chip_cfg.gyro_fsr = 0xFF;
  st.chip_cfg.accel_fsr = 0xFF;
  st.chip_cfg.lpf = 0xFF;
  st.chip_cfg.sample_rate = 0xFFFF;
  st.chip_cfg.fifo_enable = 0xFF;
  st.chip_cfg.bypass_mode = 0xFF;

  /* mpu_set_sensors always preserves this setting. */
  st.chip_cfg.clk_src = INV_CLK_PLL;
  /* Handled in next call to mpu_set_bypass. */
  st.chip_cfg.active_low_int = 1;
  st.chip_cfg.latched_int = 0;
  st.chip_cfg.int_motion_only = 0;
  st.chip_cfg.lp_accel_mode = 0;
  memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
  st.chip_cfg.dmp_on = 0;
  st.chip_cfg.dmp_loaded = 0;
  st.chip_cfg.dmp_sample_rate = 0;

  if (mpu_set_gyro_fsr(2000))
    return -1;
  if (mpu_set_accel_fsr(2))
    return -1;
  if (mpu_set_lpf(42))
    return -1;
  if (mpu_set_sample_rate(50))
    return -1;
  if (mpu_configure_fifo(0))
    return -1;

  if (mpu_set_bypass(0))
    return -1;
  mpu_set_sensors(0);
  ESP_LOGD(TAG, "MPU6050 initialized successfully");
  return 0;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int MPU6050_DMP::mpu_reset_fifo(void) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  data = 0;
  if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
    return -1;

  if (st.chip_cfg.dmp_on) {
    data = BIT_FIFO_RST | BIT_DMP_RST;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
      return -1;
    delay_ms(50);
    data = BIT_DMP_EN | BIT_FIFO_EN;
    if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
      data |= BIT_AUX_IF_EN;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
      return -1;
    if (st.chip_cfg.int_enable)
      data = BIT_DMP_INT_EN;
    else
      data = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
      return -1;
    data = 0;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
      return -1;
  } else {
    data = BIT_FIFO_RST;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
      return -1;
    if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors & INV_XYZ_COMPASS))
      data = BIT_FIFO_EN;
    else
      data = BIT_FIFO_EN | BIT_AUX_IF_EN;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
      return -1;
    delay_ms(50);
    if (st.chip_cfg.int_enable)
      data = BIT_DATA_RDY_EN;
    else
      data = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
      return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable))
      return -1;
  }
  return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_get_gyro_fsr(unsigned short *fsr) {
  switch (st.chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS:
      fsr[0] = 250;
      break;
    case INV_FSR_500DPS:
      fsr[0] = 500;
      break;
    case INV_FSR_1000DPS:
      fsr[0] = 1000;
      break;
    case INV_FSR_2000DPS:
      fsr[0] = 2000;
      break;
    default:
      fsr[0] = 0;
      break;
  }
  return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_gyro_fsr(unsigned short fsr) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  switch (fsr) {
    case 250:
      data = INV_FSR_250DPS << 3;
      break;
    case 500:
      data = INV_FSR_500DPS << 3;
      break;
    case 1000:
      data = INV_FSR_1000DPS << 3;
      break;
    case 2000:
      data = INV_FSR_2000DPS << 3;
      break;
    default:
      return -1;
  }

  if (st.chip_cfg.gyro_fsr == (data >> 3))
    return 0;
  if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data))
    return -1;
  st.chip_cfg.gyro_fsr = data >> 3;
  return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_get_accel_fsr(unsigned char *fsr) {
  switch (st.chip_cfg.accel_fsr) {
    case INV_FSR_2G:
      fsr[0] = 2;
      break;
    case INV_FSR_4G:
      fsr[0] = 4;
      break;
    case INV_FSR_8G:
      fsr[0] = 8;
      break;
    case INV_FSR_16G:
      fsr[0] = 16;
      break;
    default:
      return -1;
  }
  if (st.chip_cfg.accel_half)
    fsr[0] <<= 1;
  return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_accel_fsr(unsigned char fsr) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  switch (fsr) {
    case 2:
      data = INV_FSR_2G << 3;
      break;
    case 4:
      data = INV_FSR_4G << 3;
      break;
    case 8:
      data = INV_FSR_8G << 3;
      break;
    case 16:
      data = INV_FSR_16G << 3;
      break;
    default:
      return -1;
  }

  if (st.chip_cfg.accel_fsr == (data >> 3))
    return 0;
  if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data))
    return -1;
  st.chip_cfg.accel_fsr = data >> 3;
  return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int MPU6050_DMP::mpu_get_lpf(unsigned short *lpf) {
  switch (st.chip_cfg.lpf) {
    case INV_FILTER_188HZ:
      lpf[0] = 188;
      break;
    case INV_FILTER_98HZ:
      lpf[0] = 98;
      break;
    case INV_FILTER_42HZ:
      lpf[0] = 42;
      break;
    case INV_FILTER_20HZ:
      lpf[0] = 20;
      break;
    case INV_FILTER_10HZ:
      lpf[0] = 10;
      break;
    case INV_FILTER_5HZ:
      lpf[0] = 5;
      break;
    case INV_FILTER_256HZ_NOLPF2:
    case INV_FILTER_2100HZ_NOLPF:
    default:
      lpf[0] = 0;
      break;
  }
  return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_lpf(unsigned short lpf) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (lpf >= 188)
    data = INV_FILTER_188HZ;
  else if (lpf >= 98)
    data = INV_FILTER_98HZ;
  else if (lpf >= 42)
    data = INV_FILTER_42HZ;
  else if (lpf >= 20)
    data = INV_FILTER_20HZ;
  else if (lpf >= 10)
    data = INV_FILTER_10HZ;
  else
    data = INV_FILTER_5HZ;

  if (st.chip_cfg.lpf == data)
    return 0;

  if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data))
    return -1;

  st.chip_cfg.lpf = data;
  return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_get_sample_rate(unsigned short *rate) {
  if (st.chip_cfg.dmp_on)
    return -1;
  else
    rate[0] = st.chip_cfg.sample_rate;
  return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_int_latched(unsigned char enable) {
  unsigned char tmp;
  if (st.chip_cfg.latched_int == enable)
    return 0;

  if (enable)
    tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
  else
    tmp = 0;
  if (st.chip_cfg.bypass_mode)
    tmp |= BIT_BYPASS_EN;
  if (st.chip_cfg.active_low_int)
    tmp |= BIT_ACTL;
  if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
    return -1;
  st.chip_cfg.latched_int = enable;
  return 0;
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  \n MPU6500: 0.24Hz, 0.49Hz, 0.98Hz, 1.95Hz, 3.91Hz, 7.81Hz, 15.63Hz, 31.25Hz, 62.5Hz, 125Hz, 250Hz, 500Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_lp_accel_mode(unsigned short rate) {
  unsigned char tmp[2];
  if (!rate) {
    mpu_set_int_latched(0);
    tmp[0] = 0;
    tmp[1] = BIT_STBY_XYZG;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
      return -1;
    st.chip_cfg.lp_accel_mode = 0;
    return 0;
  }
  /* For LP accel, we automatically configure the hardware to produce latched
   * interrupts. In LP accel mode, the hardware cycles into sleep mode before
   * it gets a chance to deassert the interrupt pin; therefore, we shift this
   * responsibility over to the MCU.
   *
   * Any register read will clear the interrupt.
   */
  mpu_set_int_latched(1);
  tmp[0] = BIT_LPA_CYCLE;
  if (rate == 1) {
    tmp[1] = INV_LPA_1_25HZ;
    mpu_set_lpf(5);
  } else if (rate <= 5) {
    tmp[1] = INV_LPA_5HZ;
    mpu_set_lpf(5);
  } else if (rate <= 20) {
    tmp[1] = INV_LPA_20HZ;
    mpu_set_lpf(10);
  } else {
    tmp[1] = INV_LPA_40HZ;
    mpu_set_lpf(20);
  }
  tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
    return -1;

  st.chip_cfg.sensors = INV_XYZ_ACCEL;
  st.chip_cfg.clk_src = 0;
  st.chip_cfg.lp_accel_mode = 1;
  mpu_configure_fifo(0);

  return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_sample_rate(unsigned short rate) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (st.chip_cfg.dmp_on)
    return -1;
  else {
    if (st.chip_cfg.lp_accel_mode) {
      if (rate && (rate <= 40)) {
        /* Just stay in low-power accel mode. */
        mpu_lp_accel_mode(rate);
        return 0;
      }
      /* Requested rate exceeds the allowed frequencies in LP accel mode,
       * switch back to full-power mode.
       */
      mpu_lp_accel_mode(0);
    }
    if (rate < 4)
      rate = 4;
    else if (rate > 1000)
      rate = 1000;

    data = 1000 / rate - 1;
    if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data))
      return -1;

    st.chip_cfg.sample_rate = 1000 / (1 + data);

    /* Automatically set LPF to 1/2 sampling rate. */
    mpu_set_lpf(st.chip_cfg.sample_rate >> 1);
    return 0;
  }
}

/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_get_accel_sens(unsigned short *sens) {
  switch (st.chip_cfg.accel_fsr) {
    case INV_FSR_2G:
      sens[0] = 16384;
      break;
    case INV_FSR_4G:
      sens[0] = 8092;
      break;
    case INV_FSR_8G:
      sens[0] = 4096;
      break;
    case INV_FSR_16G:
      sens[0] = 2048;
      break;
    default:
      return -1;
  }
  if (st.chip_cfg.accel_half)
    sens[0] >>= 1;
  return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_get_fifo_config(unsigned char *sensors) {
  sensors[0] = st.chip_cfg.fifo_enable;
  return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_configure_fifo(unsigned char sensors) {
  unsigned char prev;
  int result = 0;

  /* Compass data isn't going into the FIFO. Stop trying. */
  sensors &= ~INV_XYZ_COMPASS;

  if (st.chip_cfg.dmp_on)
    return 0;
  else {
    if (!(st.chip_cfg.sensors))
      return -1;
    prev = st.chip_cfg.fifo_enable;
    st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
    if (st.chip_cfg.fifo_enable != sensors)
      /* You're not getting what you asked for. Some sensors are
       * asleep.
       */
      result = -1;
    else
      result = 0;
    if (sensors || st.chip_cfg.lp_accel_mode)
      set_int_enable(1);
    else
      set_int_enable(0);
    if (sensors) {
      if (mpu_reset_fifo()) {
        st.chip_cfg.fifo_enable = prev;
        return -1;
      }
    }
  }

  return result;
}

/**
 *  @brief      Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_get_power_state(unsigned char *power_on) {
  if (st.chip_cfg.sensors)
    power_on[0] = 1;
  else
    power_on[0] = 0;
  return 0;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_sensors(unsigned char sensors) {
  unsigned char data;

  if (sensors & INV_XYZ_GYRO)
    data = INV_CLK_PLL;
  else if (sensors)
    data = 0;
  else
    data = BIT_SLEEP;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data)) {
    st.chip_cfg.sensors = 0;
    return -1;
  }
  st.chip_cfg.clk_src = data & ~BIT_SLEEP;

  data = 0;
  if (!(sensors & INV_X_GYRO))
    data |= BIT_STBY_XG;
  if (!(sensors & INV_Y_GYRO))
    data |= BIT_STBY_YG;
  if (!(sensors & INV_Z_GYRO))
    data |= BIT_STBY_ZG;
  if (!(sensors & INV_XYZ_ACCEL))
    data |= BIT_STBY_XYZA;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data)) {
    st.chip_cfg.sensors = 0;
    return -1;
  }

  if (sensors && (sensors != INV_XYZ_ACCEL))
    /* Latched interrupts only used in LP accel mode. */
    mpu_set_int_latched(0);

  st.chip_cfg.sensors = sensors;
  st.chip_cfg.lp_accel_mode = 0;
  delay_ms(50);
  return 0;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp, unsigned char *sensors,
                               unsigned char *more) {
  /* Assumes maximum packet size is gyro (6) + accel (6). */
  unsigned char data[MAX_PACKET_LENGTH];
  unsigned char packet_size = 0;
  unsigned short fifo_count, index = 0;

  if (st.chip_cfg.dmp_on)
    return -1;

  sensors[0] = 0;
  if (!st.chip_cfg.sensors)
    return -1;
  if (!st.chip_cfg.fifo_enable)
    return -1;

  if (st.chip_cfg.fifo_enable & INV_X_GYRO)
    packet_size += 2;
  if (st.chip_cfg.fifo_enable & INV_Y_GYRO)
    packet_size += 2;
  if (st.chip_cfg.fifo_enable & INV_Z_GYRO)
    packet_size += 2;
  if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL)
    packet_size += 6;

  if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
    return -1;
  fifo_count = (data[0] << 8) | data[1];
  if (fifo_count < packet_size)
    return 0;
  //    log_i("FIFO count: %hd\n", fifo_count);
  if (fifo_count > (st.hw->max_fifo >> 1)) {
    /* FIFO is 50% full, better check overflow bit. */
    if (i2c_read(st.hw->addr, st.reg->int_status, 1, data))
      return -1;
    if (data[0] & BIT_FIFO_OVERFLOW) {
      mpu_reset_fifo();
      return -2;
    }
  }
  get_ms((unsigned long *) timestamp);

  if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data))
    return -1;
  more[0] = fifo_count / packet_size - 1;
  sensors[0] = 0;

  if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) {
    accel[0] = (data[index + 0] << 8) | data[index + 1];
    accel[1] = (data[index + 2] << 8) | data[index + 3];
    accel[2] = (data[index + 4] << 8) | data[index + 5];
    sensors[0] |= INV_XYZ_ACCEL;
    index += 6;
  }
  if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_X_GYRO) {
    gyro[0] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_X_GYRO;
    index += 2;
  }
  if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Y_GYRO) {
    gyro[1] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_Y_GYRO;
    index += 2;
  }
  if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Z_GYRO) {
    gyro[2] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_Z_GYRO;
    index += 2;
  }

  return 0;
}

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int MPU6050_DMP::mpu_read_fifo_stream(unsigned short length, unsigned char *data, unsigned char *more) {
  unsigned char tmp[2];
  unsigned short fifo_count;
  if (!st.chip_cfg.dmp_on)
    return -1;
  if (!st.chip_cfg.sensors)
    return -1;

  if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, tmp))
    return -1;
  fifo_count = (tmp[0] << 8) | tmp[1];
  if (fifo_count < length) {
    more[0] = 0;
    return -1;
  }
  if (fifo_count > (st.hw->max_fifo >> 1)) {
    /* FIFO is 50% full, better check overflow bit. */
    if (i2c_read(st.hw->addr, st.reg->int_status, 1, tmp))
      return -1;
    if (tmp[0] & BIT_FIFO_OVERFLOW) {
      mpu_reset_fifo();
      return -2;
    }
  }

  if (i2c_read(st.hw->addr, st.reg->fifo_r_w, length, data))
    return -1;
  more[0] = fifo_count / length - 1;
  return 0;
}

int MPU6050_DMP::get_accel_prod_shift(float *st_shift) {
  unsigned char tmp[4], shift_code[3], ii;

  if (i2c_read(st.hw->addr, 0x0D, 4, tmp))
    return 0x07;

  shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
  shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
  shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
  for (ii = 0; ii < 3; ii++) {
    if (!shift_code[ii]) {
      st_shift[ii] = 0.f;
      continue;
    }
    /* Equivalent to..
     * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
     */
    st_shift[ii] = 0.34f;
    while (--shift_code[ii])
      st_shift[ii] *= 1.034f;
  }
  return 0;
}

int MPU6050_DMP::accel_self_test(long *bias_regular, long *bias_st) {
  int jj, result = 0;
  float st_shift[3], st_shift_cust, st_shift_var;

  get_accel_prod_shift(st_shift);
  for (jj = 0; jj < 3; jj++) {
    st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
    if (st_shift[jj]) {
      st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
      if (fabs(st_shift_var) > test.max_accel_var)
        result |= 1 << jj;
    } else if ((st_shift_cust < test.min_g) || (st_shift_cust > test.max_g))
      result |= 1 << jj;
  }

  return result;
}

int MPU6050_DMP::gyro_self_test(long *bias_regular, long *bias_st) {
  int jj, result = 0;
  unsigned char tmp[3];
  float st_shift, st_shift_cust, st_shift_var;

  if (i2c_read(st.hw->addr, 0x0D, 3, tmp))
    return 0x07;

  tmp[0] &= 0x1F;
  tmp[1] &= 0x1F;
  tmp[2] &= 0x1F;

  for (jj = 0; jj < 3; jj++) {
    st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
    if (tmp[jj]) {
      st_shift = 3275.f / test.gyro_sens;
      while (--tmp[jj])
        st_shift *= 1.046f;
      st_shift_var = st_shift_cust / st_shift - 1.f;
      if (fabs(st_shift_var) > test.max_gyro_var)
        result |= 1 << jj;
    } else if ((st_shift_cust < test.min_dps) || (st_shift_cust > test.max_dps))
      result |= 1 << jj;
  }
  return result;
}

int MPU6050_DMP::get_st_biases(long *gyro, long *accel, unsigned char hw_test) {
  unsigned char data[MAX_PACKET_LENGTH];
  unsigned char packet_count, ii;
  unsigned short fifo_count;

  data[0] = 0x01;
  data[1] = 0;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
    return -1;
  delay_ms(200);
  data[0] = 0;
  if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;
  data[0] = BIT_FIFO_RST | BIT_DMP_RST;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;
  delay_ms(15);
  data[0] = st.test->reg_lpf;
  if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
    return -1;
  data[0] = st.test->reg_rate_div;
  if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
    return -1;
  if (hw_test)
    data[0] = st.test->reg_gyro_fsr | 0xE0;
  else
    data[0] = st.test->reg_gyro_fsr;
  if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
    return -1;

  if (hw_test)
    data[0] = st.test->reg_accel_fsr | 0xE0;
  else
    data[0] = test.reg_accel_fsr;
  if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
    return -1;
  if (hw_test)
    delay_ms(200);

  /* Fill FIFO for test.wait_ms milliseconds. */
  data[0] = BIT_FIFO_EN;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;

  data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;
  delay_ms(test.wait_ms);
  data[0] = 0;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;

  if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
    return -1;

  fifo_count = (data[0] << 8) | data[1];
  packet_count = fifo_count / MAX_PACKET_LENGTH;
  gyro[0] = gyro[1] = gyro[2] = 0;
  accel[0] = accel[1] = accel[2] = 0;

  for (ii = 0; ii < packet_count; ii++) {
    short accel_cur[3], gyro_cur[3];
    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, MAX_PACKET_LENGTH, data))
      return -1;
    accel_cur[0] = ((short) data[0] << 8) | data[1];
    accel_cur[1] = ((short) data[2] << 8) | data[3];
    accel_cur[2] = ((short) data[4] << 8) | data[5];
    accel[0] += (long) accel_cur[0];
    accel[1] += (long) accel_cur[1];
    accel[2] += (long) accel_cur[2];
    gyro_cur[0] = (((short) data[6] << 8) | data[7]);
    gyro_cur[1] = (((short) data[8] << 8) | data[9]);
    gyro_cur[2] = (((short) data[10] << 8) | data[11]);
    gyro[0] += (long) gyro_cur[0];
    gyro[1] += (long) gyro_cur[1];
    gyro[2] += (long) gyro_cur[2];
  }
#ifdef EMPL_NO_64BIT
  gyro[0] = (long) (((float) gyro[0] * 65536.f) / test.gyro_sens / packet_count);
  gyro[1] = (long) (((float) gyro[1] * 65536.f) / test.gyro_sens / packet_count);
  gyro[2] = (long) (((float) gyro[2] * 65536.f) / test.gyro_sens / packet_count);
  if (has_accel) {
    accel[0] = (long) (((float) accel[0] * 65536.f) / test.accel_sens / packet_count);
    accel[1] = (long) (((float) accel[1] * 65536.f) / test.accel_sens / packet_count);
    accel[2] = (long) (((float) accel[2] * 65536.f) / test.accel_sens / packet_count);
    /* Don't remove gravity! */
    accel[2] -= 65536L;
  }
#else
  gyro[0] = (long) (((long long) gyro[0] << 16) / test.gyro_sens / packet_count);
  gyro[1] = (long) (((long long) gyro[1] << 16) / test.gyro_sens / packet_count);
  gyro[2] = (long) (((long long) gyro[2] << 16) / test.gyro_sens / packet_count);
  accel[0] = (long) (((long long) accel[0] << 16) / test.accel_sens / packet_count);
  accel[1] = (long) (((long long) accel[1] << 16) / test.accel_sens / packet_count);
  accel[2] = (long) (((long long) accel[2] << 16) / test.accel_sens / packet_count);
  /* Don't remove gravity! */
  if (accel[2] > 0L)
    accel[2] -= 65536L;
  else
    accel[2] += 65536L;
#endif

  return 0;
}

/*
 *  \n This function must be called with the device either face-up or face-down
 *  (z-axis is parallel to gravity).
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @return     Result mask (see above).
 */
int MPU6050_DMP::mpu_run_self_test(long *gyro, long *accel) {
  const unsigned char tries = 2;
  long gyro_st[3], accel_st[3];
  unsigned char accel_result, gyro_result;
  int ii;

  int result;
  unsigned char accel_fsr, fifo_sensors, sensors_on;
  unsigned short gyro_fsr, sample_rate, lpf;
  unsigned char dmp_was_on;

  if (st.chip_cfg.dmp_on) {
    mpu_set_dmp_state(0);
    dmp_was_on = 1;
  } else
    dmp_was_on = 0;

  /* Get initial settings. */
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  mpu_get_lpf(&lpf);
  mpu_get_sample_rate(&sample_rate);
  sensors_on = st.chip_cfg.sensors;
  mpu_get_fifo_config(&fifo_sensors);

  for (ii = 0; ii < tries; ii++)
    if (!get_st_biases(gyro, accel, 0))
      break;
  if (ii == tries) {
    /* If we reach this point, we most likely encountered an I2C error.
     * We'll just report an error for all three sensors.
     */
    result = 0;
    goto restore;
  }
  for (ii = 0; ii < tries; ii++)
    if (!get_st_biases(gyro_st, accel_st, 1))
      break;
  if (ii == tries) {
    /* Again, probably an I2C error. */
    result = 0;
    goto restore;
  }
  accel_result = accel_self_test(accel, accel_st);
  gyro_result = gyro_self_test(gyro, gyro_st);

  result = 0;
  if (!gyro_result)
    result |= 0x01;
  if (!accel_result)
    result |= 0x02;

  result |= 0x04;

restore:
  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg.gyro_fsr = 0xFF;
  st.chip_cfg.accel_fsr = 0xFF;
  st.chip_cfg.lpf = 0xFF;
  st.chip_cfg.sample_rate = 0xFFFF;
  st.chip_cfg.sensors = 0xFF;
  st.chip_cfg.fifo_enable = 0xFF;
  st.chip_cfg.clk_src = INV_CLK_PLL;
  mpu_set_gyro_fsr(gyro_fsr);
  mpu_set_accel_fsr(accel_fsr);
  mpu_set_lpf(lpf);
  mpu_set_sample_rate(sample_rate);
  mpu_set_sensors(sensors_on);
  mpu_configure_fifo(fifo_sensors);

  if (dmp_was_on)
    mpu_set_dmp_state(1);

  return result;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data) {
  unsigned char tmp[2];

  if (!data)
    return -1;
  if (!st.chip_cfg.sensors)
    return -1;

  tmp[0] = (unsigned char) (mem_addr >> 8);
  tmp[1] = (unsigned char) (mem_addr & 0xFF);

  /* Check bank boundaries. */
  if (tmp[1] + length > st.hw->bank_size)
    return -1;

  if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->mem_r_w, length, data))
    return -1;
  return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data) {
  unsigned char tmp[2];

  if (!data)
    return -1;
  if (!st.chip_cfg.sensors)
    return -1;

  tmp[0] = (unsigned char) (mem_addr >> 8);
  tmp[1] = (unsigned char) (mem_addr & 0xFF);

  /* Check bank boundaries. */
  if (tmp[1] + length > st.hw->bank_size)
    return -1;

  if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->mem_r_w, length, data))
    return -1;
  return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_load_firmware(unsigned short length, const unsigned char *firmware, unsigned short start_addr,
                                   unsigned short sample_rate) {
  unsigned short ii;
  unsigned short this_write;
  /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK (16)
  unsigned char cur[LOAD_CHUNK], tmp[2];

  if (st.chip_cfg.dmp_loaded)
    /* DMP should only be loaded once. */
    return -1;

  if (!firmware)
    return -1;
  for (ii = 0; ii < length; ii += this_write) {
    this_write = fmin(LOAD_CHUNK, length - ii);
    if (mpu_write_mem(ii, this_write, (unsigned char *) &firmware[ii]))
      return -1;
    if (mpu_read_mem(ii, this_write, cur))
      return -1;
    if (memcmp(firmware + ii, cur, this_write))
      return -2;
  }

  /* Set program start address. */
  tmp[0] = start_addr >> 8;
  tmp[1] = start_addr & 0xFF;
  if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp))
    return -1;

  st.chip_cfg.dmp_loaded = 1;
  st.chip_cfg.dmp_sample_rate = sample_rate;
  return 0;
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_bypass(unsigned char bypass_on) {
  unsigned char tmp;

  if (st.chip_cfg.bypass_mode == bypass_on)
    return 0;

  if (bypass_on) {
    if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    tmp &= ~BIT_AUX_IF_EN;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    delay_ms(3);
    tmp = BIT_BYPASS_EN;
    if (st.chip_cfg.active_low_int)
      tmp |= BIT_ACTL;
    if (st.chip_cfg.latched_int)
      tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
      return -1;
  } else {
    /* Enable I2C master mode if compass is being used. */
    if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
      tmp |= BIT_AUX_IF_EN;
    else
      tmp &= ~BIT_AUX_IF_EN;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    delay_ms(3);
    if (st.chip_cfg.active_low_int)
      tmp = BIT_ACTL;
    else
      tmp = 0;
    if (st.chip_cfg.latched_int)
      tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
      return -1;
  }
  st.chip_cfg.bypass_mode = bypass_on;
  return 0;
}

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
int MPU6050_DMP::set_int_enable(unsigned char enable) {
  unsigned char tmp;

  if (st.chip_cfg.dmp_on) {
    if (enable)
      tmp = BIT_DMP_INT_EN;
    else
      tmp = 0x00;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
      return -1;
    st.chip_cfg.int_enable = tmp;
  } else {
    if (!st.chip_cfg.sensors)
      return -1;
    if (enable && st.chip_cfg.int_enable)
      return 0;
    if (enable)
      tmp = BIT_DATA_RDY_EN;
    else
      tmp = 0x00;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
      return -1;
    st.chip_cfg.int_enable = tmp;
  }
  return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int MPU6050_DMP::mpu_set_dmp_state(unsigned char enable) {
  unsigned char tmp;
  if (st.chip_cfg.dmp_on == enable)
    return 0;

  if (enable) {
    if (!st.chip_cfg.dmp_loaded)
      return -1;
    /* Disable data ready interrupt. */
    set_int_enable(0);
    /* Disable bypass mode. */
    mpu_set_bypass(0);
    /* Keep constant sample rate, FIFO rate controlled by DMP. */
    mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);
    /* Remove FIFO elements. */
    tmp = 0;
    i2c_write(st.hw->addr, 0x23, 1, &tmp);
    st.chip_cfg.dmp_on = 1;
    /* Enable DMP interrupt. */
    set_int_enable(1);
    mpu_reset_fifo();
  } else {
    /* Disable DMP interrupt. */
    set_int_enable(0);
    /* Restore FIFO settings. */
    tmp = st.chip_cfg.fifo_enable;
    i2c_write(st.hw->addr, 0x23, 1, &tmp);
    st.chip_cfg.dmp_on = 0;
    mpu_reset_fifo();
  }
  return 0;
}

/**
 *  @brief  Load the DMP with this image.
 *  @return 0 if successful.
 */
int MPU6050_DMP::dmp_load_motion_driver_firmware(void) {
  return mpu_load_firmware(DMP_CODE_SIZE, dmp_memory, sStartAddress, DMP_SAMPLE_RATE);
}

/**
 *  @brief      Set DMP output rate.
 *  Only used when DMP is on.
 *  @param[in]  rate    Desired fifo rate (Hz).
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_fifo_rate(unsigned short rate) {
  const unsigned char regs_end[12] = {DINAFE, DINAF2, DINAAB, 0xc4, DINAAA, DINAF1,
                                      DINADF, DINADF, 0xBB,   0xAF, DINADF, DINADF};
  unsigned short div;
  unsigned char tmp[8];

  if (rate > DMP_SAMPLE_RATE)
    return -1;
  div = DMP_SAMPLE_RATE / rate - 1;
  tmp[0] = (unsigned char) ((div >> 8) & 0xFF);
  tmp[1] = (unsigned char) (div & 0xFF);
  if (mpu_write_mem(D_0_22, 2, tmp))
    return -1;
  if (mpu_write_mem(CFG_6, 12, (unsigned char *) regs_end))
    return -1;

  dmp.fifo_rate = rate;
  return 0;
}

/**
 *  @brief      Get DMP output rate.
 *  @param[out] rate    Current fifo rate (Hz).
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_get_fifo_rate(unsigned short *rate) {
  rate[0] = dmp.fifo_rate;
  return 0;
}

/**
 *  @brief      Calibrate the gyro data in the DMP.
 *  After eight seconds of no motion, the DMP will compute gyro biases and
 *  subtract them from the quaternion output. If @e dmp_enable_feature is
 *  called with @e DMP_FEATURE_SEND_CAL_GYRO, the biases will also be
 *  subtracted from the gyro output.
 *  @param[in]  enable  1 to enable gyro calibration.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_enable_gyro_cal(unsigned char enable) {
  if (enable) {
    unsigned char regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
    return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
  } else {
    unsigned char regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
    return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
  }
}

/**
 *  @brief      Generate 3-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]  enable  1 to enable 3-axis quaternion.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_enable_lp_quat(unsigned char enable) {
  unsigned char regs[4];
  if (enable) {
    regs[0] = DINBC0;
    regs[1] = DINBC2;
    regs[2] = DINBC4;
    regs[3] = DINBC6;
  } else
    memset(regs, 0x8B, 4);

  mpu_write_mem(CFG_LP_QUAT, 4, regs);

  return mpu_reset_fifo();
}

/**
 *  @brief       Generate 6-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]   enable  1 to enable 6-axis quaternion.
 *  @return      0 if successful.
 */
int MPU6050_DMP::dmp_enable_6x_lp_quat(unsigned char enable) {
  unsigned char regs[4];
  if (enable) {
    regs[0] = DINA20;
    regs[1] = DINA28;
    regs[2] = DINA30;
    regs[3] = DINA38;
  } else
    memset(regs, 0xA3, 4);

  mpu_write_mem(CFG_8, 4, regs);

  return mpu_reset_fifo();
}

/**
 *  @brief      Enable DMP features.
 *  The following \#define's are used in the input mask:
 *  \n DMP_FEATURE_TAP
 *  \n DMP_FEATURE_ANDROID_ORIENT
 *  \n DMP_FEATURE_LP_QUAT
 *  \n DMP_FEATURE_6X_LP_QUAT
 *  \n DMP_FEATURE_GYRO_CAL
 *  \n DMP_FEATURE_SEND_RAW_ACCEL
 *  \n DMP_FEATURE_SEND_RAW_GYRO
 *  \n NOTE: DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually
 *  exclusive.
 *  \n NOTE: DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 *  mutually exclusive.
 *  @param[in]  mask    Mask of features to enable.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_enable_feature(unsigned short mask) {
  unsigned char tmp[10];

  /* TODO: All of these settings can probably be integrated into the default
   * DMP image.
   */
  /* Set integration scale factor. */
  tmp[0] = (unsigned char) ((GYRO_SF >> 24) & 0xFF);
  tmp[1] = (unsigned char) ((GYRO_SF >> 16) & 0xFF);
  tmp[2] = (unsigned char) ((GYRO_SF >> 8) & 0xFF);
  tmp[3] = (unsigned char) (GYRO_SF & 0xFF);
  mpu_write_mem(D_0_104, 4, tmp);

  /* Send sensor data to the FIFO. */
  tmp[0] = 0xA3;
  if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
    tmp[1] = 0xC0;
    tmp[2] = 0xC8;
    tmp[3] = 0xC2;
  } else {
    tmp[1] = 0xA3;
    tmp[2] = 0xA3;
    tmp[3] = 0xA3;
  }
  if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
    tmp[4] = 0xC4;
    tmp[5] = 0xCC;
    tmp[6] = 0xC6;
  } else {
    tmp[4] = 0xA3;
    tmp[5] = 0xA3;
    tmp[6] = 0xA3;
  }
  tmp[7] = 0xA3;
  tmp[8] = 0xA3;
  tmp[9] = 0xA3;
  mpu_write_mem(CFG_15, 10, tmp);

  /* Send gesture data to the FIFO. */
  if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
    tmp[0] = DINA20;
    printf("tap on");
  } else
    tmp[0] = 0xD8;
  mpu_write_mem(CFG_27, 1, tmp);

  if (mask & DMP_FEATURE_GYRO_CAL)
    dmp_enable_gyro_cal(1);
  else
    dmp_enable_gyro_cal(0);

  if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
    if (mask & DMP_FEATURE_SEND_CAL_GYRO) {
      tmp[0] = 0xB2;
      tmp[1] = 0x8B;
      tmp[2] = 0xB6;
      tmp[3] = 0x9B;
    } else {
      tmp[0] = DINAC0;
      tmp[1] = DINA80;
      tmp[2] = DINAC2;
      tmp[3] = DINA90;
    }
    mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
  }

  if (mask & DMP_FEATURE_TAP) {
    /* Enable tap. */
    tmp[0] = 0xF8;
    mpu_write_mem(CFG_20, 1, tmp);
    dmp_set_tap_thresh(TAP_Z, 5);
    dmp_set_tap_axes(TAP_Z);
    dmp_set_tap_count(1);
    dmp_set_tap_time(100);
    dmp_set_tap_time_multi(500);

    dmp_set_shake_reject_thresh(GYRO_SF, 200);
    dmp_set_shake_reject_time(40);
    dmp_set_shake_reject_timeout(10);
  } else {
    tmp[0] = 0xD8;
    mpu_write_mem(CFG_20, 1, tmp);
  }

  // Android orient is diabled
  tmp[0] = 0xD8;
  mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);

  if (mask & DMP_FEATURE_LP_QUAT)
    dmp_enable_lp_quat(1);
  else
    dmp_enable_lp_quat(0);

  if (mask & DMP_FEATURE_6X_LP_QUAT)
    dmp_enable_6x_lp_quat(1);
  else
    dmp_enable_6x_lp_quat(0);

  /* Pedometer is always enabled. But no functionality implemented */
  dmp.feature_mask = mask | DMP_FEATURE_PEDOMETER;
  mpu_reset_fifo();

  dmp.packet_length = 0;
  if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
    dmp.packet_length += 6;
  if (mask & DMP_FEATURE_SEND_ANY_GYRO)
    dmp.packet_length += 6;
  if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
    dmp.packet_length += 16;
  if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
    dmp.packet_length += 4;

  return 0;
}

/**
 *  @brief      Get list of currently enabled DMP features.
 *  @param[out] Mask of enabled features.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_get_enabled_features(unsigned short *mask) {
  mask[0] = dmp.feature_mask;
  return 0;
}

/**
 *  @brief      Push gyro and accel orientation to the DMP.
 *  The orientation is represented here as the output of
 *  @e inv_orientation_matrix_to_scalar.
 *  @param[in]  orient  Gyro and accel orientation in body frame.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_orientation(unsigned short orient) {
  unsigned char gyro_regs[3], accel_regs[3];
  const unsigned char gyro_axes[3] = {DINA4C, DINACD, DINA6C};
  const unsigned char accel_axes[3] = {DINA0C, DINAC9, DINA2C};
  const unsigned char gyro_sign[3] = {DINA36, DINA56, DINA76};
  const unsigned char accel_sign[3] = {DINA26, DINA46, DINA66};

  gyro_regs[0] = gyro_axes[orient & 3];
  gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
  gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
  accel_regs[0] = accel_axes[orient & 3];
  accel_regs[1] = accel_axes[(orient >> 3) & 3];
  accel_regs[2] = accel_axes[(orient >> 6) & 3];

  /* Chip-to-body, axes only. */
  if (mpu_write_mem(FCFG_1, 3, gyro_regs))
    return -1;
  if (mpu_write_mem(FCFG_2, 3, accel_regs))
    return -1;

  memcpy(gyro_regs, gyro_sign, 3);
  memcpy(accel_regs, accel_sign, 3);
  if (orient & 4) {
    gyro_regs[0] |= 1;
    accel_regs[0] |= 1;
  }
  if (orient & 0x20) {
    gyro_regs[1] |= 1;
    accel_regs[1] |= 1;
  }
  if (orient & 0x100) {
    gyro_regs[2] |= 1;
    accel_regs[2] |= 1;
  }

  /* Chip-to-body, sign only. */
  if (mpu_write_mem(FCFG_3, 3, gyro_regs))
    return -1;
  if (mpu_write_mem(FCFG_7, 3, accel_regs))
    return -1;
  dmp.orient = orient;
  return 0;
}

/**
 *  @brief      Push gyro biases to the DMP.
 *  Because the gyro integration is handled in the DMP, any gyro biases
 *  calculated by the MPL should be pushed down to DMP memory to remove
 *  3-axis quaternion drift.
 *  \n NOTE: If the DMP-based gyro calibration is enabled, the DMP will
 *  overwrite the biases written to this location once a new one is computed.
 *  @param[in]  bias    Gyro biases in q16.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_gyro_bias(long *bias) {
  long gyro_bias_body[3];
  unsigned char regs[4];

  gyro_bias_body[0] = bias[dmp.orient & 3];
  if (dmp.orient & 4)
    gyro_bias_body[0] *= -1;
  gyro_bias_body[1] = bias[(dmp.orient >> 3) & 3];
  if (dmp.orient & 0x20)
    gyro_bias_body[1] *= -1;
  gyro_bias_body[2] = bias[(dmp.orient >> 6) & 3];
  if (dmp.orient & 0x100)
    gyro_bias_body[2] *= -1;

#ifdef EMPL_NO_64BIT
  gyro_bias_body[0] = (long) (((float) gyro_bias_body[0] * GYRO_SF) / 1073741824.f);
  gyro_bias_body[1] = (long) (((float) gyro_bias_body[1] * GYRO_SF) / 1073741824.f);
  gyro_bias_body[2] = (long) (((float) gyro_bias_body[2] * GYRO_SF) / 1073741824.f);
#else
  gyro_bias_body[0] = (long) (((long long) gyro_bias_body[0] * GYRO_SF) >> 30);
  gyro_bias_body[1] = (long) (((long long) gyro_bias_body[1] * GYRO_SF) >> 30);
  gyro_bias_body[2] = (long) (((long long) gyro_bias_body[2] * GYRO_SF) >> 30);
#endif

  regs[0] = (unsigned char) ((gyro_bias_body[0] >> 24) & 0xFF);
  regs[1] = (unsigned char) ((gyro_bias_body[0] >> 16) & 0xFF);
  regs[2] = (unsigned char) ((gyro_bias_body[0] >> 8) & 0xFF);
  regs[3] = (unsigned char) (gyro_bias_body[0] & 0xFF);
  if (mpu_write_mem(D_EXT_GYRO_BIAS_X, 4, regs))
    return -1;

  regs[0] = (unsigned char) ((gyro_bias_body[1] >> 24) & 0xFF);
  regs[1] = (unsigned char) ((gyro_bias_body[1] >> 16) & 0xFF);
  regs[2] = (unsigned char) ((gyro_bias_body[1] >> 8) & 0xFF);
  regs[3] = (unsigned char) (gyro_bias_body[1] & 0xFF);
  if (mpu_write_mem(D_EXT_GYRO_BIAS_Y, 4, regs))
    return -1;

  regs[0] = (unsigned char) ((gyro_bias_body[2] >> 24) & 0xFF);
  regs[1] = (unsigned char) ((gyro_bias_body[2] >> 16) & 0xFF);
  regs[2] = (unsigned char) ((gyro_bias_body[2] >> 8) & 0xFF);
  regs[3] = (unsigned char) (gyro_bias_body[2] & 0xFF);
  return mpu_write_mem(D_EXT_GYRO_BIAS_Z, 4, regs);
}

/**
 *  @brief      Push accel biases to the DMP.
 *  These biases will be removed from the DMP 6-axis quaternion.
 *  @param[in]  bias    Accel biases in q16.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_accel_bias(long *bias) {
  long accel_bias_body[3];
  unsigned char regs[12];
  long long accel_sf;
  unsigned short accel_sens;

  mpu_get_accel_sens(&accel_sens);
  accel_sf = (long long) accel_sens << 15;
  // vTaskDelay(1);

  accel_bias_body[0] = bias[dmp.orient & 3];
  if (dmp.orient & 4)
    accel_bias_body[0] *= -1;
  accel_bias_body[1] = bias[(dmp.orient >> 3) & 3];
  if (dmp.orient & 0x20)
    accel_bias_body[1] *= -1;
  accel_bias_body[2] = bias[(dmp.orient >> 6) & 3];
  if (dmp.orient & 0x100)
    accel_bias_body[2] *= -1;

#ifdef EMPL_NO_64BIT
  accel_bias_body[0] = (long) (((float) accel_bias_body[0] * accel_sf) / 1073741824.f);
  accel_bias_body[1] = (long) (((float) accel_bias_body[1] * accel_sf) / 1073741824.f);
  accel_bias_body[2] = (long) (((float) accel_bias_body[2] * accel_sf) / 1073741824.f);
#else
  accel_bias_body[0] = (long) (((long long) accel_bias_body[0] * accel_sf) >> 30);
  accel_bias_body[1] = (long) (((long long) accel_bias_body[1] * accel_sf) >> 30);
  accel_bias_body[2] = (long) (((long long) accel_bias_body[2] * accel_sf) >> 30);
#endif

  regs[0] = (unsigned char) ((accel_bias_body[0] >> 24) & 0xFF);
  regs[1] = (unsigned char) ((accel_bias_body[0] >> 16) & 0xFF);
  regs[2] = (unsigned char) ((accel_bias_body[0] >> 8) & 0xFF);
  regs[3] = (unsigned char) (accel_bias_body[0] & 0xFF);
  regs[4] = (unsigned char) ((accel_bias_body[1] >> 24) & 0xFF);
  regs[5] = (unsigned char) ((accel_bias_body[1] >> 16) & 0xFF);
  regs[6] = (unsigned char) ((accel_bias_body[1] >> 8) & 0xFF);
  regs[7] = (unsigned char) (accel_bias_body[1] & 0xFF);
  regs[8] = (unsigned char) ((accel_bias_body[2] >> 24) & 0xFF);
  regs[9] = (unsigned char) ((accel_bias_body[2] >> 16) & 0xFF);
  regs[10] = (unsigned char) ((accel_bias_body[2] >> 8) & 0xFF);
  regs[11] = (unsigned char) (accel_bias_body[2] & 0xFF);
  return mpu_write_mem(D_ACCEL_BIAS, 12, regs);
}

/**
 *  @brief      Set tap threshold for a specific axis.
 *  @param[in]  axis    1, 2, and 4 for XYZ accel, respectively.
 *  @param[in]  thresh  Tap threshold, in mg/ms.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_tap_thresh(unsigned char axis, unsigned short thresh) {
  unsigned char tmp[4], accel_fsr;
  float scaled_thresh;
  unsigned short dmp_thresh, dmp_thresh_2;
  if (!(axis & TAP_XYZ) || thresh > 1600)
    return -1;

  scaled_thresh = (float) thresh / DMP_SAMPLE_RATE;

  mpu_get_accel_fsr(&accel_fsr);
  switch (accel_fsr) {
    case 2:
      dmp_thresh = (unsigned short) (scaled_thresh * 16384);
      /* dmp_thresh * 0.75 */
      dmp_thresh_2 = (unsigned short) (scaled_thresh * 12288);
      break;
    case 4:
      dmp_thresh = (unsigned short) (scaled_thresh * 8192);
      /* dmp_thresh * 0.75 */
      dmp_thresh_2 = (unsigned short) (scaled_thresh * 6144);
      break;
    case 8:
      dmp_thresh = (unsigned short) (scaled_thresh * 4096);
      /* dmp_thresh * 0.75 */
      dmp_thresh_2 = (unsigned short) (scaled_thresh * 3072);
      break;
    case 16:
      dmp_thresh = (unsigned short) (scaled_thresh * 2048);
      /* dmp_thresh * 0.75 */
      dmp_thresh_2 = (unsigned short) (scaled_thresh * 1536);
      break;
    default:
      return -1;
  }
  tmp[0] = (unsigned char) (dmp_thresh >> 8);
  tmp[1] = (unsigned char) (dmp_thresh & 0xFF);
  tmp[2] = (unsigned char) (dmp_thresh_2 >> 8);
  tmp[3] = (unsigned char) (dmp_thresh_2 & 0xFF);

  if (axis & TAP_X) {
    if (mpu_write_mem(DMP_TAP_THX, 2, tmp))
      return -1;
    if (mpu_write_mem(D_1_36, 2, tmp + 2))
      return -1;
  }
  if (axis & TAP_Y) {
    if (mpu_write_mem(DMP_TAP_THY, 2, tmp))
      return -1;
    if (mpu_write_mem(D_1_40, 2, tmp + 2))
      return -1;
  }
  if (axis & TAP_Z) {
    if (mpu_write_mem(DMP_TAP_THZ, 2, tmp))
      return -1;
    if (mpu_write_mem(D_1_44, 2, tmp + 2))
      return -1;
  }
  return 0;
}

/**
 *  @brief      Set which axes will register a tap.
 *  @param[in]  axis    1, 2, and 4 for XYZ, respectively.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_tap_axes(unsigned char axis) {
  unsigned char tmp = 0;

  if (axis & TAP_X)
    tmp |= 0x30;
  if (axis & TAP_Y)
    tmp |= 0x0C;
  if (axis & TAP_Z)
    tmp |= 0x03;
  return mpu_write_mem(D_1_72, 1, &tmp);
}

/**
 *  @brief      Set minimum number of taps needed for an interrupt.
 *  @param[in]  min_taps    Minimum consecutive taps (1-4).
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_tap_count(unsigned char min_taps) {
  unsigned char tmp;

  if (min_taps < 1)
    min_taps = 1;
  else if (min_taps > 4)
    min_taps = 4;

  tmp = min_taps - 1;
  return mpu_write_mem(D_1_79, 1, &tmp);
}

/**
 *  @brief      Set length between valid taps.
 *  @param[in]  time    Milliseconds between taps.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_tap_time(unsigned short time) {
  unsigned short dmp_time;
  unsigned char tmp[2];

  dmp_time = time / (1000 / DMP_SAMPLE_RATE);
  tmp[0] = (unsigned char) (dmp_time >> 8);
  tmp[1] = (unsigned char) (dmp_time & 0xFF);
  return mpu_write_mem(DMP_TAPW_MIN, 2, tmp);
}

/**
 *  @brief      Set max time between taps to register as a multi-tap.
 *  @param[in]  time    Max milliseconds between taps.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_tap_time_multi(unsigned short time) {
  unsigned short dmp_time;
  unsigned char tmp[2];

  dmp_time = time / (1000 / DMP_SAMPLE_RATE);
  tmp[0] = (unsigned char) (dmp_time >> 8);
  tmp[1] = (unsigned char) (dmp_time & 0xFF);
  return mpu_write_mem(D_1_218, 2, tmp);
}

/**
 *  @brief      Set shake rejection threshold.
 *  If the DMP detects a gyro sample larger than @e thresh, taps are rejected.
 *  @param[in]  sf      Gyro scale factor.
 *  @param[in]  thresh  Gyro threshold in dps.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_shake_reject_thresh(long sf, unsigned short thresh) {
  unsigned char tmp[4];
  long thresh_scaled = sf / 1000 * thresh;
  tmp[0] = (unsigned char) (((long) thresh_scaled >> 24) & 0xFF);
  tmp[1] = (unsigned char) (((long) thresh_scaled >> 16) & 0xFF);
  tmp[2] = (unsigned char) (((long) thresh_scaled >> 8) & 0xFF);
  tmp[3] = (unsigned char) ((long) thresh_scaled & 0xFF);
  return mpu_write_mem(D_1_92, 4, tmp);
}

/**
 *  @brief      Set shake rejection time.
 *  Sets the length of time that the gyro must be outside of the threshold set
 *  by @e gyro_set_shake_reject_thresh before taps are rejected. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_shake_reject_time(unsigned short time) {
  unsigned char tmp[2];

  time /= (1000 / DMP_SAMPLE_RATE);
  tmp[0] = time >> 8;
  tmp[1] = time & 0xFF;
  return mpu_write_mem(D_1_90, 2, tmp);
}

/**
 *  @brief      Set shake rejection timeout.
 *  Sets the length of time after a shake rejection that the gyro must stay
 *  inside of the threshold before taps can be detected again. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_set_shake_reject_timeout(unsigned short time) {
  unsigned char tmp[2];

  time /= (1000 / DMP_SAMPLE_RATE);
  tmp[0] = time >> 8;
  tmp[1] = time & 0xFF;
  return mpu_write_mem(D_1_88, 2, tmp);
}

/**
 *  @brief      Decode the four-byte gesture data and execute any callbacks.
 *  @param[in]  gesture Gesture data from DMP packet.
 *  @return     0 if successful.
 */
int MPU6050_DMP::decode_gesture(unsigned char *gesture) {
  // printf("decoding");
  unsigned char tap;
  tap = 0x3F & gesture[3];

  if (gesture[1] & INT_SRC_TAP) {
    unsigned char direction, count;
    direction = tap >> 3;
    count = (tap % 8) + 1;
    tap_cb(count, direction);
  }
  return 0;
}

void MPU6050_DMP::tap_cb(unsigned char count, unsigned char direction) {
  ESP_LOGD(TAG, "tap count: %d, direction: %d\n", count, direction);
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_WXYZ_QUAT
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] quat        3-axis quaternion data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int MPU6050_DMP::dmp_read_fifo(short *gyro, short *accel, long *quat, unsigned long *timestamp, short *sensors,
                               unsigned char *more) {
  unsigned char fifo_data[32];
  unsigned char ii = 0;

  /* TODO: sensors[0] only changes when dmp_enable_feature is called. We can
   * cache this value and save some cycles.
   */
  sensors[0] = 0;

  /* Get a packet. */
  if (mpu_read_fifo_stream(dmp.packet_length, fifo_data, more))
    return -1;

  /* Parse DMP packet. */
  if (dmp.feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
#ifdef FIFO_CORRUPTION_CHECK
    long quat_q14[4], quat_mag_sq;
#endif
    quat[0] = ((long) fifo_data[0] << 24) | ((long) fifo_data[1] << 16) | ((long) fifo_data[2] << 8) | fifo_data[3];
    quat[1] = ((long) fifo_data[4] << 24) | ((long) fifo_data[5] << 16) | ((long) fifo_data[6] << 8) | fifo_data[7];
    quat[2] = ((long) fifo_data[8] << 24) | ((long) fifo_data[9] << 16) | ((long) fifo_data[10] << 8) | fifo_data[11];
    quat[3] = ((long) fifo_data[12] << 24) | ((long) fifo_data[13] << 16) | ((long) fifo_data[14] << 8) | fifo_data[15];
    ii += 16;
#ifdef FIFO_CORRUPTION_CHECK
    /* We can detect a corrupted FIFO by monitoring the quaternion data and
     * ensuring that the magnitude is always normalized to one. This
     * shouldn't happen in normal operation, but if an I2C error occurs,
     * the FIFO reads might become misaligned.
     *
     * Let's start by scaling down the quaternion data to avoid long long
     * math.
     */
    quat_q14[0] = quat[0] >> 16;
    quat_q14[1] = quat[1] >> 16;
    quat_q14[2] = quat[2] >> 16;
    quat_q14[3] = quat[3] >> 16;
    quat_mag_sq =
        quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] + quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
    if ((quat_mag_sq < QUAT_MAG_SQ_MIN) || (quat_mag_sq > QUAT_MAG_SQ_MAX)) {
      /* Quaternion is outside of the acceptable threshold. */
      mpu_reset_fifo();
      sensors[0] = 0;
      return -1;
    }
    sensors[0] |= INV_WXYZ_QUAT;
#endif
  }

  if (dmp.feature_mask & DMP_FEATURE_SEND_RAW_ACCEL) {
    accel[0] = ((short) fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
    accel[1] = ((short) fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
    accel[2] = ((short) fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
    ii += 6;
    sensors[0] |= INV_XYZ_ACCEL;
  }

  if (dmp.feature_mask & DMP_FEATURE_SEND_ANY_GYRO) {
    gyro[0] = ((short) fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
    gyro[1] = ((short) fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
    gyro[2] = ((short) fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
    ii += 6;
    sensors[0] |= INV_XYZ_GYRO;
  }

  /* Gesture data is at the end of the DMP packet. Parse it and call
   * the gesture callbacks (if registered).
   */

  if (dmp.feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT)) {
    decode_gesture(fifo_data + ii);
  }
  get_ms(timestamp);
  return 0;
}

void MPU6050_DMP::setup() {
  MPU6050_DMP_ERROR_CHECK(mpu_init(NULL), "init");
  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  MPU6050_DMP_ERROR_CHECK(dmp_load_motion_driver_firmware(), "load firmware");
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                     DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  MPU6050_DMP_ERROR_CHECK(mpu_set_dmp_state(1), "set DMP state");
}

void MPU6050_DMP::loop() {
  short sensors;
  unsigned char more;
  unsigned long timestamp;
  dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
}

void MPU6050_DMP::dump_config() {}

void MPU6050_DMP::update() { ESP_LOGD(TAG, "accel: %d %d %d", accel[0], accel[1], accel[2]); }

}  // namespace mpu6050_dmp
}  // namespace esphome
