import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_MPU6050_DMP_ID = "mpu6050_dmp_id"


mpu6050_dmp_nds = cg.esphome_ns.namespace("mpu6050_dmp")
MPU6050_DMP = mpu6050_dmp_nds.class_("MPU6050_DMP", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPU6050_DMP),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x68))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
