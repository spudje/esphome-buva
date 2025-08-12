#include "zehnder_fan.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace zehnder_fan {

void NRF905Controller::setup(spi::SPIComponent *parent, InternalGPIOPin *cs_pin, InternalGPIOPin *ce_pin, InternalGPIOPin *pwr_pin, InternalGPIOPin *txen_pin, InternalGPIOPin *dr_pin) {
  parent_ = parent;
  cs_pin_ = cs_pin;
  ce_pin_ = ce_pin;
  pwr_pin_ = pwr_pin;
  txen_pin_ = txen_pin;
  dr_pin_ = dr_pin;
  
  ce_pin_->setup();
  pwr_pin_->setup();
  txen_pin_->setup();
  dr_pin_->setup();
  
  ce_pin_->pin_mode(gpio::FLAG_OUTPUT);
  pwr_pin_->pin_mode(gpio::FLAG_OUTPUT);
  txen_pin_->pin_mode(gpio::FLAG_OUTPUT);
  dr_pin_->pin_mode(gpio::FLAG_INPUT);
}

bool NRF905Controller::init() {
  set_mode_idle();
  
  if (!test_spi()) {
    ESP_LOGE(TAG, "NRF905 SPI test failed");
    return false;
  }
  
  memcpy(config_registers_, ZEHNDER_PROFILE, NRF905_REGISTER_COUNT);
  encode_config_registers();
  write_config_registers();
  
  ESP_LOGD(TAG, "NRF905 initialized successfully");
  return true;
}

void NRF905Controller::set_mode_idle() {
  ce_pin_->digital_write(false);
  txen_pin_->digital_write(false);
  pwr_pin_->digital_write(true);
}

void NRF905Controller::set_mode_receive() {
  ce_pin_->digital_write(true);
  txen_pin_->digital_write(false);
  pwr_pin_->digital_write(true);
}

void NRF905Controller::set_mode_transmit() {
  ce_pin_->digital_write(true);
  txen_pin_->digital_write(true);
  pwr_pin_->digital_write(true);
}

bool NRF905Controller::transmit_payload(const uint8_t *payload, size_t size) {
  if (size > NRF905_MAX_FRAMESIZE) {
    ESP_LOGE(TAG, "Payload size %d exceeds maximum %d", size, NRF905_MAX_FRAMESIZE);
    return false;
  }
  
  memcpy(tx_payload_, payload, size);
  write_tx_payload(payload);
  return start_tx(FAN_TX_FRAMES);
}

bool NRF905Controller::read_payload(uint8_t *buffer, size_t size) {
  if (!is_data_ready()) {
    return false;
  }
  
  this->enable();
  this->write_byte(NRF905_COMMAND_R_RX_PAYLOAD);
  this->read_array(buffer, size);
  this->disable();
  
  return true;
}

bool NRF905Controller::is_data_ready() {
  return dr_pin_->digital_read();
}

void NRF905Controller::set_tx_address(uint32_t address) {
  tx_address_ = address;
  write_tx_address();
}

void NRF905Controller::set_rx_address(uint32_t address) {
  rx_address_ = address;
  
  config_registers_[5] = (address & 0x000000FF);
  config_registers_[6] = (address & 0x0000FF00) >> 8;
  config_registers_[7] = (address & 0x00FF0000) >> 16;
  config_registers_[8] = (address & 0xFF000000) >> 24;
  
  encode_config_registers();
  write_config_registers();
}

void NRF905Controller::write_config_registers() {
  this->enable();
  this->write_byte(NRF905_COMMAND_W_CONFIG);
  this->write_array(config_registers_, NRF905_REGISTER_COUNT);
  this->disable();
  delay(1);
}

void NRF905Controller::write_tx_payload(const uint8_t *payload) {
  this->enable();
  this->write_byte(NRF905_COMMAND_W_TX_PAYLOAD);
  this->write_array(payload, FAN_FRAMESIZE);
  this->disable();
}

void NRF905Controller::write_tx_address() {
  uint8_t addr_bytes[4];
  addr_bytes[0] = (tx_address_ & 0x000000FF);
  addr_bytes[1] = (tx_address_ & 0x0000FF00) >> 8;
  addr_bytes[2] = (tx_address_ & 0x00FF0000) >> 16;
  addr_bytes[3] = (tx_address_ & 0xFF000000) >> 24;
  
  this->enable();
  this->write_byte(NRF905_COMMAND_W_TX_ADDRESS);
  this->write_array(addr_bytes, 4);
  this->disable();
}

bool NRF905Controller::start_tx(uint32_t retransmit_count) {
  set_mode_transmit();
  
  uint32_t start_time = millis();
  bool tx_done = false;
  
  for (uint32_t i = 0; i < retransmit_count && !tx_done; i++) {
    set_mode_transmit();
    delay(1);
    
    while (!dr_pin_->digital_read() && (millis() - start_time) < 2000) {
      delay(1);
    }
    
    if (dr_pin_->digital_read()) {
      tx_done = true;
    }
    
    set_mode_receive();
    delay(10);
  }
  
  return tx_done;
}

uint8_t NRF905Controller::get_status() {
  this->enable();
  uint8_t status = this->read_byte();
  this->disable();
  return status;
}

void NRF905Controller::encode_config_registers() {
}

bool NRF905Controller::test_spi() {
  uint32_t original_addr = tx_address_;
  
  tx_address_ = 0x55555555;
  write_tx_address();
  
  this->enable();
  this->write_byte(NRF905_COMMAND_R_TX_ADDRESS);
  uint8_t test_addr[4];
  this->read_array(test_addr, 4);
  this->disable();
  
  uint32_t read_addr = test_addr[0] | (test_addr[1] << 8) | (test_addr[2] << 16) | (test_addr[3] << 24);
  
  tx_address_ = original_addr;
  write_tx_address();
  
  return (read_addr == 0x55555555);
}

ZehnderFan::ZehnderFan(NRF905Controller *radio) : radio_(radio), rx_count_(0) {
}

std::optional<FanPairingInfo> ZehnderFan::discover(uint8_t device_id, uint32_t timeout) {
  ESP_LOGD(TAG, "Starting discovery with device ID 0x%02X", device_id);
  
  uint8_t payload[FAN_FRAMESIZE] = {
    FAN_TYPE_BROADCAST, 0x00, FAN_TYPE_REMOTE_CONTROL, device_id, 
    FAN_TTL, FAN_NETWORK_JOIN_ACK, 0x04, 0xa5, 0x5a, 0x5a, 0xa5, 
    0x00, 0x00, 0x00, 0x00, 0x00
  };
  
  radio_->set_mode_idle();
  radio_->set_rx_address(NETWORK_LINK_ID);
  radio_->set_tx_address(NETWORK_LINK_ID);
  
  rx_count_ = 0;
  radio_->write_tx_payload(payload);
  
  if (transmit_data(FAN_TX_RETRIES) == FAN_RESULT_SUCCESS) {
    for (size_t i = 0; i < rx_count_; i++) {
      uint8_t frametype = rx_buffer_[i][5];
      if (frametype == FAN_NETWORK_JOIN_OPEN) {
        uint32_t network_id = rx_buffer_[i][7] | 
                             (rx_buffer_[i][8] << 8) | 
                             (rx_buffer_[i][9] << 16) | 
                             (rx_buffer_[i][10] << 24);
        
        uint8_t main_unit_type = rx_buffer_[i][2];
        uint8_t main_unit_id = rx_buffer_[i][3];
        
        ESP_LOGD(TAG, "Found unit type 0x%02X with ID 0x%02X on network 0x%08X", 
                 main_unit_type, main_unit_id, network_id);
        
        payload[0] = FAN_TYPE_MAIN_UNIT;
        payload[1] = main_unit_id;
        payload[5] = FAN_NETWORK_JOIN_REQUEST;
        payload[7] = (network_id & 0x000000FF);
        payload[8] = (network_id & 0x0000FF00) >> 8;
        payload[9] = (network_id & 0x00FF0000) >> 16;
        payload[10] = (network_id & 0xFF000000) >> 24;
        
        radio_->set_rx_address(network_id);
        radio_->set_tx_address(network_id);
        radio_->write_tx_payload(payload);
        
        rx_count_ = 0;
        if (transmit_data(FAN_TX_RETRIES) == FAN_RESULT_SUCCESS) {
          for (size_t j = 0; j < rx_count_; j++) {
            uint8_t reply_frametype = rx_buffer_[j][5];
            if (reply_frametype == FAN_FRAME_0B || reply_frametype == FAN_FRAME_04) {
              ESP_LOGD(TAG, "Link successful to unit with ID 0x%02X on network 0x%08X", 
                       main_unit_id, network_id);
              
              return FanPairingInfo{network_id, main_unit_id, main_unit_type};
            }
          }
        }
      }
    }
  }
  
  ESP_LOGD(TAG, "No networks found during discovery");
  return std::nullopt;
}

bool ZehnderFan::set_speed(uint8_t speed, uint8_t timer) {
  if (!radio_) {
    ESP_LOGE(TAG, "Radio not initialized");
    return false;
  }
  
  uint8_t payload[FAN_FRAMESIZE] = {
    FAN_TYPE_MAIN_UNIT, 0x00, FAN_TYPE_REMOTE_CONTROL, create_device_id(),
    FAN_TTL, FAN_FRAME_SETSPEED, 0x01, speed, timer, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  
  if (timer != 0) {
    payload[5] = FAN_FRAME_SETTIMER;
    payload[6] = 0x02;
  }
  
  radio_->write_tx_payload(payload);
  FanResult result = transmit_data(FAN_TX_RETRIES);
  
  if (result == FAN_RESULT_SUCCESS) {
    for (size_t i = 0; i < rx_count_; i++) {
      uint8_t frametype = rx_buffer_[i][5];
      if (frametype == FAN_TYPE_FAN_SETTINGS) {
        uint8_t reply_payload[FAN_FRAMESIZE] = {
          FAN_TYPE_MAIN_UNIT, 0x00, FAN_TYPE_REMOTE_CONTROL, create_device_id(),
          FAN_TTL, FAN_FRAME_SETSPEED_REPLY, 0x03, 0x54, 0x03, 0x20,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };
        
        radio_->write_tx_payload(reply_payload);
        return (transmit_data(FAN_TX_RETRIES) == FAN_RESULT_SUCCESS);
      }
    }
  }
  
  return (result == FAN_RESULT_SUCCESS);
}

bool ZehnderFan::set_voltage(uint8_t voltage) {
  if (!radio_) {
    ESP_LOGE(TAG, "Radio not initialized");
    return false;
  }
  
  uint8_t payload[FAN_FRAMESIZE] = {
    FAN_TYPE_MAIN_UNIT, 0x00, FAN_TYPE_REMOTE_CONTROL, create_device_id(),
    FAN_TTL, FAN_FRAME_SETVOLTAGE, 0x01, voltage, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  
  radio_->write_tx_payload(payload);
  FanResult result = transmit_data(FAN_TX_RETRIES);
  
  return (result == FAN_RESULT_SUCCESS);
}

uint8_t ZehnderFan::create_device_id() {
  // Use ESPHome's helper to get a random number between 1 and 254 inclusive.
  return (random_uint32() % 254) + 1;
}

FanResult ZehnderFan::transmit_data(size_t retries) {
  for (size_t attempt = 0; attempt < retries; attempt++) {
    if (radio_->start_tx(FAN_TX_FRAMES)) {
      size_t rx_before = rx_count_;
      
      uint32_t start_time = millis();
      while ((millis() - start_time) < FAN_REPLY_TIMEOUT) {
        if (radio_->is_data_ready() && rx_count_ < 32) {
          radio_->read_payload(rx_buffer_[rx_count_].data(), FAN_FRAMESIZE);
          rx_count_++;
        }
        delay(10);
      }
      
      if (rx_count_ > rx_before) {
        return FAN_RESULT_SUCCESS;
      }
    }
  }
  
  return FAN_ERROR_TX_FAILED;
}

void ZehnderFanComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Zehnder Fan...");
  
  // nrf_radio_.set_spi_parent(this->parent_);
  // nrf_radio_.set_cs_pin(this->cs_);
  nrf_radio_.setup(this->parent_, this->cs_, ce_pin_, pwr_pin_, txen_pin_, dr_pin_);
  
  if (!nrf_radio_.init()) {
    ESP_LOGE(TAG, "Failed to initialize NRF905 radio");
    this->mark_failed();
    return;
  }
  
  fan_protocol_ = std::make_unique<ZehnderFan>(&nrf_radio_);
  device_id_ = fan_protocol_->create_device_id();
  
  ESP_LOGD(TAG, "Generated device ID: 0x%02X", device_id_);
  ESP_LOGCONFIG(TAG, "Zehnder Fan setup complete");
}

void ZehnderFanComponent::update() {
  nrf_radio_.set_mode_receive();
}

void ZehnderFanComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Zehnder Fan:");
  ESP_LOGCONFIG(TAG, "  Device ID: 0x%02X", device_id_);
  if (pairing_info_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Network ID: 0x%08X", pairing_info_->network_id);
    ESP_LOGCONFIG(TAG, "  Main Unit ID: 0x%02X", pairing_info_->main_unit_id);
    ESP_LOGCONFIG(TAG, "  Main Unit Type: 0x%02X", pairing_info_->main_unit_type);
  } else {
    ESP_LOGCONFIG(TAG, "  Not paired");
  }
}

void ZehnderFanComponent::pair_device() {
  ESP_LOGI(TAG, "Starting pairing process...");
  
  auto result = fan_protocol_->discover(device_id_, FAN_JOIN_DEFAULT_TIMEOUT);
  if (result.has_value()) {
    pairing_info_ = result;
    ESP_LOGI(TAG, "Pairing successful! Network: 0x%08X, Unit: 0x%02X", 
             pairing_info_->network_id, pairing_info_->main_unit_id);
  } else {
    ESP_LOGE(TAG, "Pairing failed - no main unit found");
  }
}

fan::FanTraits ZehnderFanComponent::get_traits() {
  auto traits = fan::FanTraits();
  // REMOVE this line: traits.set_speed_count(4);
  traits.set_supported_preset_modes({
    fan::FAN_PRESET_LOW, fan::FAN_PRESET_MEDIUM, fan::FAN_PRESET_HIGH
  });
  return traits;
}

void ZehnderFanComponent::control(const fan::FanCall &call) {
  if (!pairing_info_.has_value()) {
    ESP_LOGW(TAG, "Cannot control fan - not paired");
    return;
  }

  if (call.get_preset_mode().has_value()) { // Changed from get_preset()
    auto preset = *call.get_preset_mode(); // Changed from get_preset()
    uint8_t speed = FAN_SPEED_AUTO;

    if (preset == fan::FAN_PRESET_LOW) { // Use standard preset names
      speed = FAN_SPEED_LOW;
    } else if (preset == fan::FAN_PRESET_MEDIUM) {
      speed = FAN_SPEED_MEDIUM;
    } else if (preset == fan::FAN_PRESET_HIGH) {
      speed = FAN_SPEED_HIGH;
    }

    // Update the internal state for Home Assistant UI
    this->state = true;
    this->preset_mode = preset; // Changed from this->preset

    ESP_LOGD(TAG, "Setting fan speed to %s (%d)", preset.c_str(), speed);
    fan_protocol_->set_speed(speed, 0);
  }
  
  this->publish_state(); // This will now publish the correct state to HA
}

}  // namespace zehnder_fan
}  // namespace esphome