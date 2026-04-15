import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID


DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_MPU6050_DMP_ID = "mpu6050_dmp_id"
CONF_TAP_THRESHOLD = "tap_threshold"
CONF_TAP_AXIS = "tap_axis"


TAP_DIRECTIONS = {"TAP_X": 0x01, "TAP_Y": 0x02, "TAP_Z": 0x04, "TAP_XYZ": 0x07}