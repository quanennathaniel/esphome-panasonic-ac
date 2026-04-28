#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {

namespace panasonic_ac {

static const char *const VERSION = "2.5.0";

static const uint8_t BUFFER_SIZE = 128;
static const uint8_t READ_TIMEOUT = 20;

static const uint8_t MIN_TEMPERATURE = 16;
static const uint8_t MAX_TEMPERATURE = 30;
static const float TEMPERATURE_STEP = 0.5;
static const float TEMPERATURE_TOLERANCE = 2;
static const uint8_t TEMPERATURE_THRESHOLD = 100;

enum class CommandType { Normal, Response, Resend };

enum class ACType {
  DNSKP11,
  CZTACG1
};

class PanasonicAC : public Component, public uart::UARTDevice, public climate::Climate {
 public:
  void set_outside_temperature_sensor(sensor::Sensor *outside_temperature_sensor);
  void set_outside_temperature_offset(int8_t outside_temperature_offset);
  void set_vertical_swing_select(select::Select *vertical_swing_select);
  void set_horizontal_swing_select(select::Select *horizontal_swing_select);
  void set_nanoex_switch(switch_::Switch *nanoex_switch);
  void set_nanoeg_switch(switch_::Switch *nanoeg_switch);
  void set_eco_switch(switch_::Switch *eco_switch);
  void set_econavi_switch(switch_::Switch *econavi_switch);
  void set_mild_dry_switch(switch_::Switch *mild_dry_switch);
  void set_current_power_consumption_sensor(sensor::Sensor *current_power_consumption_sensor);

  void set_current_temperature_sensor(sensor::Sensor *current_temperature_sensor);
  void set_current_temperature_offset(int8_t current_temperature_offset);

  void setup() override;
  void loop() override;

 protected:
  sensor::Sensor *outside_temperature_sensor_ = nullptr;
  select::Select *vertical_swing_select_ = nullptr;
  select::Select *horizontal_swing_select_ = nullptr;
  switch_::Switch *nanoex_switch_ = nullptr;                    // Switch to toggle nanoeX on/off
  switch_::Switch *nanoeg_switch_ = nullptr;                    // Switch to toggle nanoe-G on/off
  switch_::Switch *eco_switch_ = nullptr;
  switch_::Switch *econavi_switch_ = nullptr;
  switch_::Switch *mild_dry_switch_ = nullptr;
  sensor::Sensor *current_temperature_sensor_ = nullptr;
  sensor::Sensor *current_power_consumption_sensor_ = nullptr;

  size_t vertical_swing_state_;
  size_t horizontal_swing_state_;

  int8_t current_temperature_offset_ = 0;
  int8_t outside_temperature_offset_ = 0;
  bool nanoex_state_ = false;
  bool nanoeg_state_ = false;                                   // Stores the state of nanoe-G to prevent duplicate packets
  bool eco_state_ = false;
  bool econavi_state_ = false;
  bool mild_dry_state_ = false;

  bool waiting_for_response_ = false;

  std::vector<uint8_t> rx_buffer_;

  uint32_t init_time_;
  uint32_t last_read_;
  uint32_t last_packet_sent_;
  uint32_t last_packet_received_;

  climate::ClimateTraits traits() override;

  void read_data();

  void update_outside_temperature(int8_t temperature);
  void update_current_temperature(int8_t temperature);
  void update_target_temperature(uint8_t raw_value);
  void update_swing_horizontal(const StringRef &swing);
  void update_swing_vertical(const StringRef &swing);
  void update_nanoex(bool nanoex);
  void update_nanoeg(bool nanoeg);
  void update_eco(bool eco);
  void update_econavi(bool econavi);
  void update_mild_dry(bool mild_dry);
  void update_current_power_consumption(int16_t power);

  virtual void on_horizontal_swing_change(const StringRef &swing) = 0;
  virtual void on_vertical_swing_change(const StringRef &swing) = 0;
  virtual void on_nanoex_change(bool nanoex) = 0;
  virtual void on_nanoeg_change(bool nanoeg) = 0;
  virtual void on_eco_change(bool eco) = 0;
  virtual void on_econavi_change(bool econavi) = 0;
  virtual void on_mild_dry_change(bool mild_dry) = 0;

  climate::ClimateAction determine_action();

  void log_packet(std::vector<uint8_t> data, bool outgoing = false);
};

}  // namespace panasonic_ac
}  // namespace esphome
