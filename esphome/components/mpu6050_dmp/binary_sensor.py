import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_MOVING

from . import CONF_MPU6050_DMP_ID, MPU6050_DMP

DEPENDENCIES = ["mpu6050_dmp", "i2c"]

CONF_TAP_THRESHOLD = "tap_threshold"
CONF_TAP_AXIS = "tap_axis"


TAP_DIRECTIONS = {"tap_x": 0x01, "tap_y": 0x02, "tap_z": 0x04, "tap_xyz": 0x07}

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_MOVING
).extend(
    {
        cv.GenerateID(CONF_MPU6050_DMP_ID): cv.use_id(MPU6050_DMP),
        cv.Required(CONF_TAP_AXIS): cv.enum(TAP_DIRECTIONS, lower=True),
        cv.Optional(CONF_TAP_THRESHOLD, 5): cv.int_range(1, 200),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MPU6050_DMP_ID])
    var = await binary_sensor.new_binary_sensor(config)
    func = getattr(hub, "set_tap_binary_sensor")
    cg.add(func(var))
