Here is a detailed plan to refactor the codebase into a clean, minimal ESPHome external component for the ESP32-C6.

### High-Level Strategy

Our goal is to distill the provided codebase into its core logic—communicating with the fan via the nRF905 radio—and wrap it within the ESPHome component architecture. This means we will aggressively remove all code related to networking, web servers, and multi-platform hardware support, as ESPHome provides these features for us.

We will create two main C++ classes:
1.  A low-level `NRF905Controller` class to handle the SPI communication and register manipulation of the radio.
2.  A high-level `ZehnderFan` class to implement the fan-specific communication protocol (pairing, setting speed, etc.) using the `NRF905Controller`.

Finally, we will create an ESPHome component that uses these classes and exposes the fan functionality (like speed presets and pairing) as services and a standard `fan` entity to ESPHome and Home Assistant.

---

### **Phase 1: Project Scaffolding and Code Pruning**

This phase is about setting up the right structure and deleting everything we don't need.

**1. Create the ESPHome External Component Directory Structure:**
In your ESPHome configuration directory, create the following structure:

```
esphome_config/
|-- my_fan_controller.yaml
|-- custom_components/
|   |-- zehnder_fan/
|   |   |-- __init__.py
|   |   |-- zehnder_fan.h
|   |   |-- zehnder_fan.cpp
```

**2. Identify and Isolate Essential Code:**
Copy the contents of the following files from the original project into your `zehnder_fan` directory. These are the only files that contain logic we need to preserve and refactor.

*   **Keepers (to be refactored):**
    *   `fan.h` / `fan.cpp`: Contains the core fan communication protocol logic.
    *   `nrf905.h` / `nrf905.cpp`: Contains the nRF905 hardware driver.
    *   `config.h`: We will extract PIN definitions and radio constants from here.
    *   `utils.h` / `utils.cpp`: We might keep `binToHexstring` for logging/debugging.

*   **Aggressively Discard:**
    *   **All API files:** `json_api*.*`, `apitest*.*` - ESPHome provides its own API and services.
    *   **All HTML/Web files:** `certificate.h`, `ssdp.h`, `html*.*`, `javascript.h`, `stylesheet.h`, `icons.h` - The web server is not needed.
    *   **Hardware Abstraction Layers:** `board.h`, `esp8266.*`, `esp32.*`, `bcm2835.*` - We will use the standard ESP-IDF/Arduino functions for the ESP32-C6 directly. This is a major simplification.
    *   **NVRAM Management:** `nvram.h`, `nvram.cpp` - ESPHome handles configuration persistence. Fan pairing info will be stored in the YAML.
    *   **Main Application Files:** `nRF905API.ino.cpp`, `nRF905API.h` - The `setup()` and `loop()` from this file contain logic we don't need (WiFi connect, DNS, OTA, etc.).

### **Phase 2: Refactoring the Core Logic**

Now, we'll rewrite and modernize the code we decided to keep. We'll work inside the `zehnder_fan.cpp` and `zehnder_fan.h` files.

**1. Create the `NRF905Controller` Class (from `nrf905.h`/`.cpp`):**
This class will be a clean, modern C++ driver for the nRF905.

*   **Header (`zehnder_fan.h`):**
    ```cpp
    #include "esphome/core/component.h"
    #include "esphome/components/spi/spi.h"

    class NRF905Controller : public esphome::spi::SPIDevice<esphome::spi::BIT_ORDER_MSB_FIRST,
                                                            esphome::spi::CLOCK_POLARITY_LOW,
                                                            esphome::spi::CLOCK_PHASE_LEADING> {
    public:
        // Methods for setting frequency, power, addresses, etc.
        void setup(InternalGPIOPin *ce_pin, InternalGPIOPin *pwr_pin, InternalGPIOPin *txen_pin, InternalGPIOPin *dr_pin);
        void set_mode_idle();
        void set_mode_receive();
        bool transmit_payload(const uint8_t *payload, size_t size);
        bool read_payload(uint8_t *buffer, size_t size);
        bool is_data_ready();
        
        // ... other public methods from the original nrf905.cpp ...
    private:
        // Remove all `board->` calls and replace with direct GPIO/SPI calls
        InternalGPIOPin *ce_pin_;
        // ... other pins ...
    };
    ```

*   **Implementation (`zehnder_fan.cpp`):**
    *   Replace all `board->writePin(...)` with `this->ce_pin_->digital_write(...)`.
    *   Replace all `board->readPin(...)` with `this->dr_pin_->digital_read(...)`.
    *   Replace all `board->SPITransfern(...)` with `this->spi_transfer(...)`.
    *   Remove all interrupt service routines (`ISR`). We will use a simpler polling model in the component's `loop()`, which is more stable and easier to manage within ESPHome. Check the `DR` pin state in a loop instead.
    *   Hardcode the radio configuration constants from `config.h` (e.g., `NRF905_XTAL_FREQUENCY`).

**2. Refactor the `Fan` Logic into a `ZehnderFan` Class:**
This class will handle the application-level logic.

*   **Header (`zehnder_fan.h`):**
    ```cpp
    #include <optional>

    struct FanPairingInfo {
        uint32_t network_id;
        uint8_t main_unit_id;
        uint8_t main_unit_type;
    };
    
    class ZehnderFan {
    public:
        ZehnderFan(NRF905Controller *radio);
        std::optional<FanPairingInfo> discover();
        bool set_speed(uint8_t speed, uint8_t timer);
        // ... other methods ...
    private:
        NRF905Controller *radio_;
    };
    ```

*   **Implementation (`zehnder_fan.cpp`):**
    *   Modify the `Fan` class constructor and methods to belong to `ZehnderFan`.
    *   Replace all `Serial.printf` with ESPHome's logging framework (e.g., `ESP_LOGD("zehnder_fan", "Starting discovery...")`). This integrates perfectly with ESPHome's logging.
    *   Refactor the `discover()` method. Instead of printing to serial, it should return an `std::optional<FanPairingInfo>`. If pairing is successful, it returns the struct; otherwise, it returns `std::nullopt`.
    *   Refactor `setSpeed()` and other commands to be clean, blocking functions that return `true` on success and `false` on failure.

### **Phase 3: Creating the ESPHome Component**

This is where we tie our refactored C++ code into the ESPHome framework.

**1. Define the Component and Fan Output (`zehnder_fan.h`):**

```cpp
#include "esphome/components/fan/fan.h"

class ZehnderFanComponent : public esphome::fan::Fan, public esphome::PollingComponent {
public:
    void setup() override;
    void loop() override;
    void dump_config() override;

    // Expose a service for pairing
    void pair_device();

    // Fan control
    esphome::fan::FanTraits get_traits() override;
    void control(const esphome::fan::FanCall &call) override;

protected:
    NRF905Controller nrf_radio_;
    std::unique_ptr<ZehnderFan> fan_protocol_;
    
    // Pins from YAML
    esphome::GPIOPin *ce_pin_;
    // ... other pins ...
};
```

**2. Implement the Component Logic (`zehnder_fan.cpp`):**

*   **`setup()`:** Initialize the GPIO pins, initialize the SPI bus, and instantiate your `NRF905Controller` and `ZehnderFan` objects. Configure the radio with the default parameters from the original `fan.h` (e.g., the Zehnder profile).
*   **`loop()`:** This will be called periodically. Here, you can check `fan_protocol_->is_data_ready()`. This replaces the need for interrupts.
*   **`dump_config()`:** Log the component's configuration (pins, etc.) for easy debugging.
*   **`get_traits()`:** Define the fan's capabilities. It supports `LOW`, `MEDIUM`, `HIGH` presets.
*   **`control(const esphome::fan::FanCall &call)`:** This is the heart of the ESPHome integration.
    *   If `call.get_state()` is changed, turn the fan on/off.
    *   If `call.get_preset()` is changed, call your `fan_protocol_->set_speed()` method with the corresponding speed constant (`FAN_SPEED_LOW`, etc.).
*   **`pair_device()`:** This custom method will call `fan_protocol_->discover()` and log the results. We will expose this as a service in the YAML.

**3. Define the Configuration Schema (`__init__.py`):**

This file tells ESPHome how to validate the YAML configuration and generate the C++ code.

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import fan, spi
from esphome.const import CONF_ID

zehnder_fan_ns = cg.esphome_ns.namespace('zehnder_fan')
ZehnderFanComponent = zehnder_fan_ns.class_('ZehnderFanComponent', fan.Fan, cg.PollingComponent, spi.SPIDevice)

# Define pins
CONF_CE_PIN = 'ce_pin'
# ... other pins ...

CONFIG_SCHEMA = fan.FAN_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(ZehnderFanComponent),
    cv.Required(CONF_CE_PIN): cv.pinned_ 思っoutput,
    # ... schemas for pwr_pin, txen_pin, dr_pin ...
}).extend(spi.spi_device_schema(cs_pin_schema=True))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await fan.register_fan(var, config)
    await spi.register_spi_device(var, config)

    # Set up pins
    ce_pin = await cg.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))
    # ... set other pins ...
```

### **Phase 4: Final YAML Configuration**

Your final `my_fan_controller.yaml` will be beautifully simple.

```yaml
# 1. Enable the SPI bus for the ESP32-C6
spi:
  clk_pin: GPIO7  # Use the correct default pins for ESP32-C6
  mosi_pin: GPIO6
  miso_pin: GPIO2

# 2. Define the external component
external_components:
  - source:
      type: local
      path: custom_components/zehnder_fan
    components: [ fan ]
    
# 3. Configure the fan
fan:
  - platform: zehnder_fan
    name: "Mechanical Ventilation"
    # Pin configuration copied from original config.h for ESP32
    cs_pin: GPIO18
    ce_pin: GPIO5
    pwr_pin: GPIO15
    txen_pin: GPIO7
    dr_pin: GPIO23

    # Expose a service to start the pairing process
    services:
      - service: pair_device
        then:
          - lambda: |-
              id(my_fan).pair_device();

# ... standard wifi, api, logger configuration ...
wifi:
  ssid: "nRF905"
  password: "nrf905api"
```

This plan provides a clear path from a complex, multi-featured project to a lean, maintainable, and modern ESPHome component. It honors the core logic of the original author while leveraging the power of ESPHome to handle all the boilerplate.