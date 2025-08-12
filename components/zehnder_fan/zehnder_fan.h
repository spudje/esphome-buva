#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan.h"
#include "esphome/core/log.h"
#include <optional>

namespace esphome {
namespace zehnder_fan {

static const char *const TAG = "zehnder_fan";

#define FAN_FRAMESIZE 16
#define FAN_TX_FRAMES 4
#define FAN_TX_RETRIES 10
#define FAN_TTL 250
#define FAN_REPLY_TIMEOUT 500

#define NRF905_REGISTER_COUNT 10
#define NRF905_MAX_FRAMESIZE 32
#define NRF905_XTAL_FREQUENCY 16000000
#define NRF905_SPI_FREQUENCY 1000000

#define NRF905_COMMAND_W_CONFIG 0x00
#define NRF905_COMMAND_R_CONFIG 0x10
#define NRF905_COMMAND_W_TX_PAYLOAD 0x20
#define NRF905_COMMAND_R_TX_PAYLOAD 0x21
#define NRF905_COMMAND_W_TX_ADDRESS 0x22
#define NRF905_COMMAND_R_TX_ADDRESS 0x23
#define NRF905_COMMAND_R_RX_PAYLOAD 0x24
#define NRF905_COMMAND_CHANNEL_CONFIG 0x80

enum FanDeviceType {
  FAN_TYPE_BROADCAST = 0x00,
  FAN_TYPE_MAIN_UNIT = 0x01,
  FAN_TYPE_REMOTE_CONTROL = 0x03,
  FAN_TYPE_CO2_SENSOR = 0x18
};

enum FanCommand {
  FAN_FRAME_SETVOLTAGE = 0x01,
  FAN_FRAME_SETSPEED = 0x02,
  FAN_FRAME_SETTIMER = 0x03,
  FAN_NETWORK_JOIN_REQUEST = 0x04,
  FAN_FRAME_SETSPEED_REPLY = 0x05,
  FAN_NETWORK_JOIN_OPEN = 0x06,
  FAN_TYPE_FAN_SETTINGS = 0x07,
  FAN_FRAME_0B = 0x0B,
  FAN_FRAME_04 = 0x04,
  FAN_NETWORK_JOIN_ACK = 0x0C,
  FAN_TYPE_QUERY_NETWORK = 0x0D,
  FAN_TYPE_QUERY_DEVICE = 0x10,
  FAN_FRAME_SETVOLTAGE_REPLY = 0x1D
};

enum FanSpeed {
  FAN_SPEED_AUTO = 0x00,
  FAN_SPEED_LOW = 0x01,
  FAN_SPEED_MEDIUM = 0x02,
  FAN_SPEED_HIGH = 0x03,
  FAN_SPEED_MAX = 0x04
};

enum FanResult {
  FAN_RESULT_SUCCESS = 0x00,
  FAN_ERROR_NOT_FOUND,
  FAN_ERROR_NOT_COMPLETED,
  FAN_ERROR_TX_FAILED,
  FAN_ERROR_NO_REPLY,
  FAN_ERROR_NO_ACKNOWLEDGE,
  FAN_ERROR_CONFIG_FAILED = 0xFF
};

enum NRFMode {
  NRF_POWER_DOWN = 0,
  NRF_IDLE,
  NRF_RECEIVE,
  NRF_TRANSMIT
};

const uint32_t NETWORK_LINK_ID = 0xA55A5AA5;
const uint32_t NETWORK_DEFAULT_ID = 0xE7E7E7E7;
const uint32_t FAN_JOIN_DEFAULT_TIMEOUT = 10000;

const uint8_t ZEHNDER_PROFILE[NRF905_REGISTER_COUNT] = {
  0x76, 0x2E, 0x44, 0x10, 0x10, 0xA5, 0x5A, 0x5A, 0xA5, 0xDB
};

struct FanPairingInfo {
  uint32_t network_id;
  uint8_t main_unit_id;
  uint8_t main_unit_type;
};

class NRF905Controller {
 public:
  void setup(spi::SPIComponent *parent, InternalGPIOPin *cs_pin, InternalGPIOPin *ce_pin, InternalGPIOPin *pwr_pin, InternalGPIOPin *txen_pin, InternalGPIOPin *dr_pin);
  bool init();
  void set_mode_idle();
  void set_mode_receive();
  void set_mode_transmit();
  bool transmit_payload(const uint8_t *payload, size_t size);
  bool read_payload(uint8_t *buffer, size_t size);
  bool is_data_ready();
  void set_tx_address(uint32_t address);
  void set_rx_address(uint32_t address);
  uint32_t get_rx_address() const { return rx_address_; }
  void write_config_registers();
  void write_tx_payload(const uint8_t *payload);
  bool start_tx(uint32_t retransmit_count);
  uint8_t get_status();

 private:
  spi::SPIComponent *parent_; // Store the SPI bus
  InternalGPIOPin *cs_pin_;   // Store the CS pin
  InternalGPIOPin *ce_pin_;
  InternalGPIOPin *pwr_pin_;
  InternalGPIOPin *txen_pin_;
  InternalGPIOPin *dr_pin_;
  
  uint32_t tx_address_;
  uint32_t rx_address_;
  uint8_t config_registers_[NRF905_REGISTER_COUNT];
  uint8_t tx_payload_[NRF905_MAX_FRAMESIZE];
  
  void encode_config_registers();
  void write_tx_address();
  bool test_spi();
};

class ZehnderFan {
 public:
  ZehnderFan(NRF905Controller *radio);
  std::optional<FanPairingInfo> discover(uint8_t device_id, uint32_t timeout);
  bool set_speed(uint8_t speed, uint8_t timer);
  bool set_voltage(uint8_t voltage);
  uint8_t create_device_id();

 private:
  NRF905Controller *radio_;
  std::vector<uint8_t> rx_buffer_[32];
  size_t rx_count_;
  
  FanResult transmit_data(size_t retries);
};

class ZehnderFanComponent : public fan::Fan, public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  
  void set_ce_pin(InternalGPIOPin *pin) { ce_pin_ = pin; }
  void set_pwr_pin(InternalGPIOPin *pin) { pwr_pin_ = pin; }
  void set_txen_pin(InternalGPIOPin *pin) { txen_pin_ = pin; }
  void set_dr_pin(InternalGPIOPin *pin) { dr_pin_ = pin; }
  
  void pair_device();
  
  fan::FanTraits get_traits() override;
  void control(const fan::FanCall &call) override;

 protected:
  InternalGPIOPin *ce_pin_;
  InternalGPIOPin *pwr_pin_;
  InternalGPIOPin *txen_pin_;
  InternalGPIOPin *dr_pin_;
  
  NRF905Controller nrf_radio_;
  std::unique_ptr<ZehnderFan> fan_protocol_;
  
  uint8_t device_id_;
  std::optional<FanPairingInfo> pairing_info_;
};

}  // namespace zehnder_fan
}  // namespace esphome