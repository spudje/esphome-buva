import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import fan, spi
from esphome.const import CONF_ID
from esphome import pins

CODEOWNERS = ["@yourusername"]
DEPENDENCIES = ["spi"]

zehnder_fan_ns = cg.esphome_ns.namespace("zehnder_fan")
ZehnderFanComponent = zehnder_fan_ns.class_(
    "ZehnderFanComponent", fan.Fan, cg.PollingComponent, spi.SPIDevice
)

CONF_CE_PIN = "ce_pin"
CONF_PWR_PIN = "pwr_pin"
CONF_TXEN_PIN = "txen_pin"
CONF_DR_PIN = "dr_pin"

CONFIG_SCHEMA = (
    fan.FAN_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(ZehnderFanComponent),
            cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PWR_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_TXEN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DR_PIN): pins.gpio_input_pin_schema,
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await fan.register_fan(var, config)
    await spi.register_spi_device(var, config)

    ce_pin = await cg.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))

    pwr_pin = await cg.gpio_pin_expression(config[CONF_PWR_PIN])
    cg.add(var.set_pwr_pin(pwr_pin))

    txen_pin = await cg.gpio_pin_expression(config[CONF_TXEN_PIN])
    cg.add(var.set_txen_pin(txen_pin))

    dr_pin = await cg.gpio_pin_expression(config[CONF_DR_PIN])
    cg.add(var.set_dr_pin(dr_pin))
