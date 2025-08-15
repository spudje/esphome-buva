#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan.h"
#include "esphome/core/preferences.h"

#include <optional>

namespace esphome {
namespace zehnder_fan {

// Constants extracted from the original fan.h and config.h
static const uint32_t NRF905_XTAL_FREQUENCY = 16000000;
static const uint8_t FAN_FRAMESIZE = 16;
static const uint8_t FAN_TX_FRAMES = 4;
static const uint8_t FAN_TX_RETRIES = 10;
static const uint32_t FAN_REPLY_TIMEOUT_MS = 500;
static const uint32_t NETWORK_LINK_ID = 0xA55A5AA5;

// Fan device types and commands
enum {
    FAN_TYPE_BROADCAST = 0x00,
    FAN_TYPE_MAIN_UNIT = 0x01,
    FAN_TYPE_REMOTE_CONTROL = 0x03,
};
enum {
    FAN_FRAME_SETSPEED = 0x02,
    FAN_FRAME_SETTIMER = 0x03,
    FAN_NETWORK_JOIN_REQUEST = 0x04,
    FAN_FRAME_SETSPEED_REPLY = 0x05,
    FAN_NETWORK_JOIN_OPEN = 0x06,
    FAN_TYPE_FAN_SETTINGS = 0x07,
    FAN_FRAME_0B = 0x0B,
    FAN_NETWORK_JOIN_ACK = 0x0C,
};
enum {
    FAN_SPEED_AUTO = 0x00,
    FAN_SPEED_LOW = 0x01,
    FAN_SPEED_MEDIUM = 0x02,
    FAN_SPEED_HIGH = 0x03,
};


struct FanPairingInfo {
    uint32_t network_id;
    uint8_t main_unit_id;
    uint8_t main_unit_type;
    uint8_t my_device_id;
};

// =========================================================================
// 1. Low-Level nRF905 Radio Controller
// =========================================================================
class NRF905Controller : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                              spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_LEADING> {
public:
    void setup_pins(GPIOPin *pwr_pin, GPIOPin *ce_pin, GPIOPin *txen_pin, GPIOPin *dr_pin);
    bool init();
    void set_mode_idle();
    void set_mode_receive();
    void set_mode_transmit();

    void set_tx_address(uint32_t address);
    void set_rx_address(uint32_t address);

    void write_tx_payload(const uint8_t *payload, size_t size);
    bool read_rx_payload(uint8_t *buffer, size_t size);

    bool is_data_ready() { return this->dr_pin_->digital_read(); }

private:
    void write_config_registers(const uint8_t *config, size_t size);
    
    GPIOPin *pwr_pin_{nullptr};
    GPIOPin *ce_pin_{nullptr};
    GPIOPin *txen_pin_{nullptr};
    GPIOPin *dr_pin_{nullptr};
};


// =========================================================================
// 2. High-Level Fan Communication Protocol
// =========================================================================
class ZehnderFanProtocol {
public:
    ZehnderFanProtocol(NRF905Controller *radio);

    std::optional<FanPairingInfo> pair();
    bool set_speed(const FanPairingInfo &pairing_info, uint8_t speed, uint8_t timer_minutes);

private:
    bool transmit_and_wait(size_t retries);
    
    NRF905Controller *radio_;
    uint8_t rx_buffer_[FAN_FRAMESIZE]{0};
};


// =========================================================================
// 3. ESPHome Component
// =========================================================================
class ZehnderFanComponent : public fan::Fan, public PollingComponent {
public:
    void setup() override;
    void loop() override;
    void dump_config() override;

    fan::FanTraits get_traits() override;
    void control(const fan::FanCall &call) override;

    // Service function to initiate pairing
    void start_pairing();

    // Pin Setters from YAML
    void set_pwr_pin(GPIOPin *pin) { this->pwr_pin_ = pin; }
    void set_ce_pin(GPIOPin *pin) { this->ce_pin_ = pin; }
    void set_txen_pin(GPIOPin *pin) { this->txen_pin_ = pin; }
    void set_dr_pin(GPIOPin *pin) { this->dr_pin_ = pin; }

protected:
    void save_pairing_info(const FanPairingInfo &info);
    bool load_pairing_info();
    void clear_pairing_info();

    NRF905Controller nrf_radio_;
    std::unique_ptr<ZehnderFanProtocol> fan_protocol_;
    
    // Pins from YAML
    GPIOPin *pwr_pin_;
    GPIOPin *ce_pin_;
    GPIOPin *txen_pin_;
    GPIOPin *dr_pin_;

    std::optional<FanPairingInfo> pairing_info_;
    ESPHomePreferences preferences_;
};

} // namespace zehnder_fan
} // namespace esphome