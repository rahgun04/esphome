#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

namespace esphome {
namespace mpu6050_dmp {



class MPU6050_DMP : public PollingComponent, public i2c::I2CDevice {
#ifdef USE_SENSOR
    SUB_SENSOR(temperature)
    SUB_SENSOR(acceleration_x)
    SUB_SENSOR(acceleration_y)
    SUB_SENSOR(acceleration_z)
    SUB_SENSOR(gyro_x)
    SUB_SENSOR(gyro_y)
    SUB_SENSOR(gyro_z)
#endif

#ifdef USE_BINARY_SENSOR
    SUB_BINARY_SENSOR(tap)
#endif

public:
    void setup() override;
    void dump_config() override;
    void update() override;
    void loop() override;

protected:
    //State vars
    struct gyro_reg_s reg = {
        .who_am_i       = 0x75,
        .rate_div       = 0x19,
        .lpf            = 0x1A,
        .prod_id        = 0x0C,
        .user_ctrl      = 0x6A,
        .fifo_en        = 0x23,
        .gyro_cfg       = 0x1B,
        .accel_cfg      = 0x1C,
        .motion_thr     = 0x1F,
        .motion_dur     = 0x20,
        .fifo_count_h   = 0x72,
        .fifo_r_w       = 0x74,
        .raw_gyro       = 0x43,
        .raw_accel      = 0x3B,
        .temp           = 0x41,
        .int_enable     = 0x38,
        .dmp_int_status = 0x39,
        .int_status     = 0x3A,
        .pwr_mgmt_1     = 0x6B,
        .pwr_mgmt_2     = 0x6C,
        .int_pin_cfg    = 0x37,
        .mem_r_w        = 0x6F,
        .accel_offs     = 0x06,
        .i2c_mst        = 0x24,
        .bank_sel       = 0x6D,
        .mem_start_addr = 0x6E,
        .prgm_start_h   = 0x70
    };
    struct hw_s hw = {
        .addr           = 0x68,
        .max_fifo       = 1024,
        .num_reg        = 118,
        .temp_sens      = 340,
        .temp_offset    = -521,
        .bank_size      = 256
    };

    struct test_s test = {
        .gyro_sens      = 32768/250,
        .accel_sens     = 32768/16,
        .reg_rate_div   = 0,    /* 1kHz. */
        .reg_lpf        = 1,    /* 188Hz. */
        .reg_gyro_fsr   = 0,    /* 250dps. */
        .reg_accel_fsr  = 0x18, /* 16g. */
        .wait_ms        = 50,
        .packet_thresh  = 5,    /* 5% */
        .min_dps        = 10.f,
        .max_dps        = 105.f,
        .max_gyro_var   = 0.14f,
        .min_g          = 0.3f,
        .max_g          = 0.95f,
        .max_accel_var  = 0.14f
    };

    struct gyro_state_s st = {
        .reg = &reg,
        .hw = &hw,
        .test = &test
    };
    // From inv_mpu.h
    int mpu_init(struct int_param_s *int_param);

    int mpu_set_dmp_state(unsigned char enable);

    int mpu_get_lpf(unsigned short *lpf);
    int mpu_set_lpf(unsigned short lpf);

    int mpu_get_gyro_fsr(unsigned short *fsr);
    int mpu_set_gyro_fsr(unsigned short fsr);

    int mpu_get_accel_fsr(unsigned char *fsr);
    int mpu_set_accel_fsr(unsigned char fsr);

    int mpu_get_accel_sens(unsigned short *sens);

    int mpu_get_sample_rate(unsigned short *rate);
    int mpu_set_sample_rate(unsigned short rate);

    int mpu_get_fifo_config(unsigned char *sensors);
    int mpu_configure_fifo(unsigned char sensors);

    int mpu_get_power_state(unsigned char *power_on);
    int mpu_set_sensors(unsigned char sensors);

    int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
        unsigned char *sensors, unsigned char *more);
    int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
        unsigned char *more);
    int mpu_reset_fifo(void);

    int mpu_write_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data);
    int mpu_read_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data);
    int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
        unsigned short start_addr, unsigned short sample_rate);
    int set_int_enable(unsigned char enable);
    int mpu_set_bypass(unsigned char bypass_on);
    int mpu_lp_accel_mode(unsigned short rate);
    int mpu_set_int_latched(unsigned char enable);
    int accel_self_test(long *bias_regular, long *bias_st);
    int gyro_self_test(long *bias_regular, long *bias_st);
    int get_st_biases(long *gyro, long *accel, unsigned char hw_test);
    int get_accel_prod_shift(float *shift);
    struct dmp_s dmp = {
        .tap_cb = NULL,
        .android_orient_cb = NULL,
        .orient = 0,
        .feature_mask = 0,
        .fifo_rate = 0,
        .packet_length = 0
    };

    int dmp_load_motion_driver_firmware(void);
    int dmp_set_fifo_rate(unsigned short rate);
    int dmp_get_fifo_rate(unsigned short *rate);
    int dmp_enable_feature(unsigned short mask);
    int dmp_get_enabled_features(unsigned short *mask);
    int dmp_set_orientation(unsigned short orient);
    int dmp_set_gyro_bias(long *bias);
    int dmp_enable_gyro_cal(unsigned char enable);
    int dmp_set_accel_bias(long *bias);
    int mpu_run_self_test(long *gyro, long *accel);

    int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char));
    int dmp_set_tap_thresh(unsigned char axis, unsigned short thresh);
    int dmp_set_tap_axes(unsigned char axis);
    int dmp_set_tap_count(unsigned char min_taps);
    int dmp_set_tap_time(unsigned short time);
    int dmp_set_tap_time_multi(unsigned short time);
    int dmp_set_shake_reject_thresh(long sf, unsigned short thresh);
    int dmp_set_shake_reject_time(unsigned short time);
    int dmp_set_shake_reject_timeout(unsigned short time);

    int decode_gesture(unsigned char *gesture);

    int dmp_read_fifo(short *gyro, short *accel, long *quat,
    unsigned long *timestamp, short *sensors, unsigned char *more);

    int dmp_enable_lp_quat(unsigned char enable);
    int dmp_enable_6x_lp_quat(unsigned char enable);

    int i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
    int i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, const uint8_t *data);
    void delay_ms(unsigned long ms);
    void get_ms(unsigned long *ms);
};

}  // namespace mpu6050_dmp
}  // namespace esphome
