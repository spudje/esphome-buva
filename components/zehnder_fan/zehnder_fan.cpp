#include "zehnder_fan.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace zehnder_fan {

static const char *const TAG = "zehnder_fan";

// =========================================================================
// 1. NRF905Controller Implementation
// =========================================================================

void NRF905Controller::setup_pins(GPIOPin *pwr_pin, GPIOPin *ce_pin, GPIOPin *txen_pin, GPIOPin *dr_pin) {
    this->pwr_pin_ = pwr_pin;
    this->ce_pin_ = ce_pin;
    this->txen_pin_ = txen_pin;
    this->dr_pin_ = dr_pin;
}

bool NRF905Controller::init() {
    // Initialize SPI device
    this->spi_setup();
    
    this->pwr_pin_->setup();
    this->ce_pin_->setup();
    this->txen_pin_->setup();
    this->dr_pin_->setup();

    this->set_mode_idle();

    // Zehnder nRF905 configuration profile from fan.h
    const uint8_t zehnder_config[] = {0x76, 0x2E, 0x44, 0x10, 0x10, 0xA5, 0x5A, 0x5A, 0xA5, 0xDB};
    this->write_config_registers(zehnder_config, sizeof(zehnder_config));

    ESP_LOGD(TAG, "NRF905 initialized.");
    return true;
}

void NRF905Controller::set_mode_idle() {
    this->pwr_pin_->digital_write(true);
    this->ce_pin_->digital_write(false);
    this->txen_pin_->digital_write(false);
}

void NRF905Controller::set_mode_receive() {
    this->pwr_pin_->digital_write(true);
    this->txen_pin_->digital_write(false);
    this->ce_pin_->digital_write(true);
}

void NRF905Controller::set_mode_transmit() {
    this->pwr_pin_->digital_write(true);
    this->ce_pin_->digital_write(false); // Drop to standby before enabling TX
    delayMicroseconds(100);
    this->txen_pin_->digital_write(true);
    this->ce_pin_->digital_write(true);
}

void NRF905Controller::set_tx_address(uint32_t address) {
    this->enable();
    this->write_byte(0x22); // W_TX_ADDRESS
    this->write_byte((address >> 0) & 0xFF);
    this->write_byte((address >> 8) & 0xFF);
    this->write_byte((address >> 16) & 0xFF);
    this->write_byte((address >> 24) & 0xFF);
    this->disable();
}

void NRF905Controller::set_rx_address(uint32_t address) {
    // RX address is set via config registers, not a direct command
    const uint8_t config_update[] = {
        0x76, 0x2E, 0x44, 0x10, 0x10, // Keep first 5 bytes
        (uint8_t)((address >> 0) & 0xFF),
        (uint8_t)((address >> 8) & 0xFF),
        (uint8_t)((address >> 16) & 0xFF),
        (uint8_t)((address >> 24) & 0xFF),
        0xDB, // Keep last byte
    };
    this->write_config_registers(config_update, sizeof(config_update));
}


void NRF905Controller::write_tx_payload(const uint8_t *payload, size_t size) {
    this->enable();
    this->write_byte(0x20); // W_TX_PAYLOAD
    this->write_array(payload, size);
    this->disable();
}

bool NRF905Controller::read_rx_payload(uint8_t *buffer, size_t size) {
    if (!this->is_data_ready()) {
        return false;
    }
    this->enable();
    this->write_byte(0x24); // R_RX_PAYLOAD
    this->read_array(buffer, size);
    this->disable();
    return true;
}

void NRF905Controller::write_config_registers(const uint8_t *config, size_t size) {
    this->set_mode_idle();
    this->enable();
    this->write_byte(0x00); // W_CONFIG
    this->write_array(config, size);
    this->disable();
}


// =========================================================================
// 2. ZehnderFanProtocol Implementation
// =========================================================================

ZehnderFanProtocol::ZehnderFanProtocol(NRF905Controller *radio) : radio_(radio) {}

bool ZehnderFanProtocol::transmit_and_wait(size_t retries) {
    for (int i = 0; i < retries; ++i) {
        // Transmit burst
        this->radio_->set_mode_transmit();
        delay(2); // Short delay to ensure transmission starts
        this->radio_->set_mode_receive();

        // Wait for a reply
        uint32_t start_time = millis();
        while (millis() - start_time < FAN_REPLY_TIMEOUT_MS) {
            if (this->radio_->read_rx_payload(this->rx_buffer_, FAN_FRAMESIZE)) {
                return true; // Got a reply
            }
            yield();
        }
    }
    return false; // Timed out
}

std::optional<FanPairingInfo> ZehnderFanProtocol::pair() {
    ESP_LOGD(TAG, "Starting fan pairing discovery...");
    this->radio_->set_mode_idle();
    this->radio_->set_tx_address(NETWORK_LINK_ID);
    this->radio_->set_rx_address(NETWORK_LINK_ID);
    this->radio_->set_mode_receive();

    uint8_t my_device_id = random_uint32() & 0xFE; // Avoid 0xFF
    if (my_device_id == 0x00) my_device_id = 1;

    // Frame 1: Send broadcast discovery
    uint8_t payload_discover[FAN_FRAMESIZE] = {0x04, 0x00, FAN_TYPE_REMOTE_CONTROL, my_device_id, 0xFA, FAN_NETWORK_JOIN_ACK, 0x04, 0xa5, 0x5a, 0x5a, 0xa5, 0x00, 0x00, 0x00, 0x00, 0x00};
    this->radio_->write_tx_payload(payload_discover, sizeof(payload_discover));

    if (!this->transmit_and_wait(FAN_TX_RETRIES)) {
        ESP_LOGW(TAG, "Pairing failed: No response from main unit.");
        return std::nullopt;
    }

    if (this->rx_buffer_[5] != FAN_NETWORK_JOIN_OPEN) {
        ESP_LOGW(TAG, "Pairing failed: Received unexpected frame type 0x%02X.", this->rx_buffer_[5]);
        return std::nullopt;
    }

    FanPairingInfo result;
    result.main_unit_type = this->rx_buffer_[2];
    result.main_unit_id = this->rx_buffer_[3];
    result.network_id = (uint32_t)this->rx_buffer_[7] | ((uint32_t)this->rx_buffer_[8] << 8) | ((uint32_t)this->rx_buffer_[9] << 16) | ((uint32_t)this->rx_buffer_[10] << 24);
    result.my_device_id = my_device_id;
    
    ESP_LOGD(TAG, "Found fan unit ID 0x%02X on network 0x%08X. Requesting to join...", result.main_unit_id, result.network_id);

    // Frame 2: Request to join the discovered network
    this->radio_->set_tx_address(result.network_id);
    this->radio_->set_rx_address(result.network_id);
    
    uint8_t payload_join[FAN_FRAMESIZE] = {0};
    payload_join[0] = FAN_TYPE_MAIN_UNIT;
    payload_join[1] = result.main_unit_id;
    payload_join[2] = FAN_TYPE_REMOTE_CONTROL;
    payload_join[3] = my_device_id;
    payload_join[4] = 0xFA;
    payload_join[5] = FAN_NETWORK_JOIN_REQUEST;
    payload_join[7] = (result.network_id >> 0) & 0xFF;
    payload_join[8] = (result.network_id >> 8) & 0xFF;
    payload_join[9] = (result.network_id >> 16) & 0xFF;
    payload_join[10] = (result.network_id >> 24) & 0xFF;

    this->radio_->write_tx_payload(payload_join, sizeof(payload_join));

    if (!this->transmit_and_wait(FAN_TX_RETRIES)) {
        ESP_LOGW(TAG, "Pairing failed: No response to join request.");
        return std::nullopt;
    }

    // Frame 3: Acknowledge successful link
    uint8_t payload_ack[FAN_FRAMESIZE] = {0};
    payload_ack[0] = FAN_TYPE_MAIN_UNIT;
    payload_ack[1] = result.main_unit_id;
    payload_ack[2] = FAN_TYPE_REMOTE_CONTROL;
    payload_ack[3] = my_device_id;
    payload_ack[4] = 0xFA;
    payload_ack[5] = FAN_FRAME_0B;

    this->radio_->write_tx_payload(payload_ack, sizeof(payload_ack));
    this->transmit_and_wait(1); // Fire and forget is OK here

    ESP_LOGI(TAG, "Pairing successful! Network ID: 0x%08X, Fan ID: 0x%02X, My Device ID: 0x%02X",
             result.network_id, result.main_unit_id, result.my_device_id);

    return result;
}

bool ZehnderFanProtocol::set_speed(const FanPairingInfo &pairing_info, uint8_t speed, uint8_t timer_minutes) {
    this->radio_->set_mode_idle();
    this->radio_->set_tx_address(pairing_info.network_id);
    this->radio_->set_rx_address(pairing_info.network_id);

    uint8_t payload[FAN_FRAMESIZE] = {0};
    payload[0] = FAN_TYPE_MAIN_UNIT;
    payload[1] = pairing_info.main_unit_id;
    payload[2] = FAN_TYPE_REMOTE_CONTROL;
    payload[3] = pairing_info.my_device_id;
    payload[4] = 0xFA; // TTL
    payload[5] = (timer_minutes > 0) ? FAN_FRAME_SETTIMER : FAN_FRAME_SETSPEED;
    payload[6] = (timer_minutes > 0) ? 0x02 : 0x01; // Number of parameters
    payload[7] = speed;
    payload[8] = timer_minutes;

    this->radio_->write_tx_payload(payload, sizeof(payload));
    
    if (this->transmit_and_wait(FAN_TX_RETRIES)) {
        ESP_LOGD(TAG, "Set speed command acknowledged.");
        return true;
    }
    
    ESP_LOGW(TAG, "Set speed command was not acknowledged by the fan.");
    return false;
}


// =========================================================================
// 3. ZehnderFanComponent Implementation
// =========================================================================

void ZehnderFanComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Zehnder Fan...");

    // Initialize nRF905 SPI device
    this->nrf_radio_.set_spi_parent(this->spi_parent_);
    this->nrf_radio_.set_cs_pin(this->cs_pin_);
    this->nrf_radio_.setup_pins(this->pwr_pin_, this->ce_pin_, this->txen_pin_, this->dr_pin_);
    this->nrf_radio_.init();

    this->fan_protocol_ = make_unique<ZehnderFanProtocol>(&this->nrf_radio_);
    
    // Setup preferences for storing pairing data
    this->preferences_ = global_preferences->make_preference<FanPairingInfo>(this->get_object_id_hash());

    if (this->load_pairing_info()) {
        ESP_LOGI(TAG, "Loaded pairing info from flash.");
    } else {
        ESP_LOGW(TAG, "No pairing info found. Fan needs to be paired.");
    }
}

void ZehnderFanComponent::loop() {
    // Polling logic can be added here if needed, but for now we are command-driven.
}

void ZehnderFanComponent::update() {
    // Required by PollingComponent, but not needed for this implementation
}

void ZehnderFanComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Zehnder Fan Component:");
    LOG_PIN("  PWR Pin: ", this->pwr_pin_);
    LOG_PIN("  CE Pin: ", this->ce_pin_);
    LOG_PIN("  TXEN Pin: ", this->txen_pin_);
    LOG_PIN("  DR Pin: ", this->dr_pin_);
    if (this->pairing_info_.has_value()) {
        ESP_LOGCONFIG(TAG, "  Paired Network ID: 0x%08X", this->pairing_info_->network_id);
        ESP_LOGCONFIG(TAG, "  Paired Fan ID: 0x%02X", this->pairing_info_->main_unit_id);
    } else {
        ESP_LOGCONFIG(TAG, "  Device is not paired.");
    }
}

fan::FanTraits ZehnderFanComponent::get_traits() {
    // The fan supports Off, Low, Medium, High speeds.
    return fan::FanTraits(false, true, false, 3);
}

void ZehnderFanComponent::control(const fan::FanCall &call) {
    if (!this->pairing_info_.has_value()) {
        ESP_LOGE(TAG, "Cannot control fan: Not paired.");
        return;
    }

    if (call.get_state().has_value()) {
        this->state = *call.get_state();
    }
    if (call.get_speed().has_value()) {
        this->speed = *call.get_speed();
    }

    uint8_t fan_speed = FAN_SPEED_AUTO; // Off
    if (this->state) {
        switch (this->speed) {
            case 1: fan_speed = FAN_SPEED_LOW; break;
            case 2: fan_speed = FAN_SPEED_MEDIUM; break;
            case 3: fan_speed = FAN_SPEED_HIGH; break;
        }
    }
    
    // For now, timer is not implemented via Home Assistant fan model. Could be a separate service.
    uint8_t timer = 0; 
    
    ESP_LOGD(TAG, "Setting fan speed to level %d", this->speed);
    this->fan_protocol_->set_speed(this->pairing_info_.value(), fan_speed, timer);
    
    this->publish_state();
}

void ZehnderFanComponent::start_pairing() {
    ESP_LOGI(TAG, "Pairing service called. Attempting to discover and pair with fan...");
    auto result = this->fan_protocol_->pair();
    if (result.has_value()) {
        this->save_pairing_info(result.value());
        this->load_pairing_info(); // Reload into component state
        ESP_LOGI(TAG, "Pairing successful and info saved to flash.");
    } else {
        ESP_LOGE(TAG, "Pairing failed.");
    }
}

void ZehnderFanComponent::save_pairing_info(const FanPairingInfo &info) {
    this->preferences_.save(&info);
}

bool ZehnderFanComponent::load_pairing_info() {
    FanPairingInfo loaded_info;
    if (this->preferences_.load(&loaded_info)) {
        ESP_LOGI(TAG, "Loaded pairing info: Network ID 0x%08X, Fan ID 0x%02X, My Device ID 0x%02X",
                 loaded_info.network_id, loaded_info.main_unit_id, loaded_info.my_device_id);
        this->pairing_info_ = loaded_info;
        return true;
    } else {
        ESP_LOGW(TAG, "No pairing info found in flash. Device is not paired.");
        this->pairing_info_ = std::nullopt;
        return false;
    }
}

void ZehnderFanComponent::clear_pairing_info() {
    this->preferences_.reset();
    this->pairing_info_ = std::nullopt;
}

} // namespace zehnder_fan
} // namespace esphome