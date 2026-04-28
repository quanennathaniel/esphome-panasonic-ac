#include "esppac_cnt.h"
#include "esppac_commands_cnt.h"
#include "esphome/core/log.h"

namespace esphome {
namespace panasonic_ac {
namespace CNT {

static const char *const TAG = "panasonic_ac.cz_tacg1";

static climate::ClimateMode determine_mode(uint8_t mode) {
  uint8_t nib1 = (mode >> 4) & 0x0F;
  uint8_t nib2 = (mode >> 0) & 0x0F;
  if (nib2 == 0x00) return climate::CLIMATE_MODE_OFF;
  switch (nib1) {
    case 0x00: return climate::CLIMATE_MODE_HEAT_COOL;
    case 0x03: return climate::CLIMATE_MODE_COOL;
    case 0x04: return climate::CLIMATE_MODE_HEAT;
    case 0x02: return climate::CLIMATE_MODE_DRY;
    case 0x06: return climate::CLIMATE_MODE_FAN_ONLY;
    default: return climate::CLIMATE_MODE_OFF;
  }
}

static const char *determine_fan_speed(uint8_t speed) {
  switch (speed) {
    case 0xA0: return "Automatic";
    case 0x30: return "1";
    case 0x40: return "2";
    case 0x50: return "3";
    case 0x60: return "4";
    case 0x70: return "5";
    default: return "Unknown";
  }
}

static const char *determine_vertical_swing(uint8_t swing) {
  uint8_t nib = (swing >> 4) & 0x0F;
  switch (nib) {
    case 0x0E: return "swing";
    case 0x0F: return "auto";
    case 0x01: return "up";
    case 0x02: return "up_center";
    case 0x03: return "center";
    case 0x04: return "down_center";
    case 0x05: return "down";
    default: return "Unknown";
  }
}

static const char *determine_horizontal_swing(uint8_t swing) {
  uint8_t nib = (swing >> 0) & 0x0F;
  switch (nib) {
    case 0x0D: return "auto";
    case 0x09: return "left";
    case 0x0A: return "left_center";
    case 0x06: return "center";
    case 0x0B: return "right_center";
    case 0x0C: return "right";
    default: return "Unknown";
  }
}

static const char *determine_preset(uint8_t preset) {
  uint8_t nib = (preset >> 0) & 0x0F;
  return (nib == 0x02) ? "Powerful" : (nib == 0x04) ? "Quiet" : "Normal";
}

static bool determine_preset_nanoex(uint8_t preset) {
  return (preset >> 4) & 0x04;
}

// Logic based on your logs: Byte 28 changes from 0x61 to 0x9F (bit 7 and others)
static bool determine_nanoeg(uint8_t value) {
  return (value & 0x80) == 0x80; 
}

static bool determine_eco(uint8_t value) {
  return value == 0x40;
}

static bool determine_econavi(uint8_t value) {
  return value & 0x10;
}

static bool determine_mild_dry(uint8_t value) {
  return value == 0x7F;
}

uint16_t determine_power_consumption(uint8_t byte_28, uint8_t byte_29) {
  return (uint16_t) (byte_28 + (byte_29 * 256));
}

void PanasonicACCNT::setup() {
  PanasonicAC::setup();
  ESP_LOGD(TAG, "Using modified 35-byte protocol for Nanoe-G");
}

void PanasonicACCNT::loop() {
  PanasonicAC::read_data();
  if (millis() - this->last_read_ > READ_TIMEOUT && !this->rx_buffer_.empty()) {
    if (!verify_packet()) return;
    this->waiting_for_response_ = false;
    this->last_packet_received_ = millis();
    handle_packet();
    this->rx_buffer_.clear();
  }
  handle_cmd();
  handle_poll();
}

void PanasonicACCNT::control(const climate::ClimateCall &call) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;

  if (call.get_mode().has_value()) {
    switch (*call.get_mode()) {
      case climate::CLIMATE_MODE_COOL: this->cmd[0] = 0x34; break;
      case climate::CLIMATE_MODE_HEAT: this->cmd[0] = 0x44; break;
      case climate::CLIMATE_MODE_DRY: this->cmd[0] = 0x24; break;
      case climate::CLIMATE_MODE_HEAT_COOL: this->cmd[0] = 0x04; break;
      case climate::CLIMATE_MODE_FAN_ONLY: this->cmd[0] = 0x64; break;
      case climate::CLIMATE_MODE_OFF: this->cmd[0] &= 0xF0; break;
      default: break;
    }
  }

  if (call.get_target_temperature().has_value()) {
    this->cmd[1] = (*call.get_target_temperature() - this->current_temperature_offset_) / TEMPERATURE_STEP;
  }

  if (call.has_custom_fan_mode()) {
    const auto fanMode = call.get_custom_fan_mode();
    if (fanMode == "Automatic") this->cmd[3] = 0xA0;
    else if (fanMode == "1") this->cmd[3] = 0x30;
    else if (fanMode == "2") this->cmd[3] = 0x40;
    else if (fanMode == "3") this->cmd[3] = 0x50;
    else if (fanMode == "4") this->cmd[3] = 0x60;
    else if (fanMode == "5") this->cmd[3] = 0x70;
  }
}

void PanasonicACCNT::set_data(bool set) {
  this->mode = determine_mode(this->data[0]);
  this->set_custom_fan_mode_(determine_fan_speed(this->data[3]));

  StringRef verticalSwing(determine_vertical_swing(this->data[4]));
  StringRef horizontalSwing(determine_horizontal_swing(this->data[4]));

  const char *preset = determine_preset(this->data[5]);
  bool nanoex = determine_preset_nanoex(this->data[5]);
  
  // Pointing to the correct byte in the expanded data vector
  bool nanoeg = determine_nanoeg(this->data[28]); 
  bool eco = determine_eco(this->data[8]);
  bool econavi = determine_econavi(this->data[5]);
  bool mildDry = determine_mild_dry(this->data[2]);

  this->update_target_temperature((int8_t) this->data[1]);

  if (set) {
    if (this->rx_buffer_[18] != 0x80) this->update_current_temperature((int8_t) this->rx_buffer_[18]);
    if (this->rx_buffer_[19] != 0x80) this->update_outside_temperature((int8_t) this->rx_buffer_[19]);
    
    if (this->current_power_consumption_sensor_ != nullptr) {
      uint16_t power = determine_power_consumption(this->rx_buffer_[28], this->rx_buffer_[29]);
      this->update_current_power_consumption(power);
    }
  }

  this->update_swing_vertical(verticalSwing);
  this->update_swing_horizontal(horizontalSwing);
  this->set_custom_preset_(preset);
  this->update_nanoex(nanoex);
  this->update_nanoeg(nanoeg);
  this->update_eco(eco);
  this->update_econavi(econavi);
  this->update_mild_dry(mildDry);
}

void PanasonicACCNT::handle_packet() {
  if (this->rx_buffer_[0] == POLL_HEADER) {
    // CAPTURE ALL 33 BYTES of data payload
    this->data = std::vector<uint8_t>(this->rx_buffer_.begin() + 2, this->rx_buffer_.end() - 1);
    this->set_data(true);
    this->publish_state();
    if (this->state_ != ACState::Ready) this->state_ = ACState::Ready;
  }
}

void PanasonicACCNT::on_nanoeg_change(bool state) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;

  this->nanoeg_state_ = state;
  if (state) {
    ESP_LOGV(TAG, "Turning nanoe-G ON via Byte 28 Bitmask");
    this->cmd[28] |= 0x80; // Set high bit
    // Force the rest of the byte to match log activity if mask isn't enough
    this->cmd[28] |= 0x1F; 
  } else {
    ESP_LOGV(TAG, "Turning nanoe-G OFF via Byte 28 Bitmask");
    this->cmd[28] &= ~0x80; // Clear high bit
    this->cmd[28] &= 0x61;
  }
}

void PanasonicACCNT::on_vertical_swing_change(const StringRef &swing) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;
  if (swing == "down") this->cmd[4] = (this->cmd[4] & 0x0F) + 0x50;
  else if (swing == "auto") this->cmd[4] = (this->cmd[4] & 0x0F) + 0xF0;
}

void PanasonicACCNT::on_horizontal_swing_change(const StringRef &swing) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;
  if (swing == "left") this->cmd[4] = (this->cmd[4] & 0xF0) + 0x09;
  else if (swing == "auto") this->cmd[4] = (this->cmd[4] & 0xF0) + 0x0D;
}

void PanasonicACCNT::on_nanoex_change(bool state) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;
  this->nanoex_state_ = state;
  if (state) this->cmd[5] |= 0x40;
  else this->cmd[5] &= ~0x40;
}

void PanasonicACCNT::on_eco_change(bool state) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;
  this->eco_state_ = state;
  this->cmd[8] = state ? 0x40 : 0x00;
}

void PanasonicACCNT::on_econavi_change(bool state) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;
  this->econavi_state_ = state;
  if (state) this->cmd[5] |= 0x10;
  else this->cmd[5] &= ~0x10;
}

void PanasonicACCNT::on_mild_dry_change(bool state) {
  if (this->state_ != ACState::Ready) return;
  if (this->cmd.empty()) this->cmd = this->data;
  this->mild_dry_state_ = state;
  this->cmd[2] = state ? 0x7F : 0x80;
}

void PanasonicACCNT::send_command(std::vector<uint8_t> command, CommandType type, uint8_t header) {
  uint8_t length = command.size();
  command.insert(command.begin(), header);
  command.insert(command.begin() + 1, length);
  uint8_t checksum = 0;
  for (uint8_t i : command) checksum -= i;
  command.push_back(checksum);
  send_packet(command, type);
}

void PanasonicACCNT::send_packet(const std::vector<uint8_t> &packet, CommandType type) {
  this->last_packet_sent_ = millis();
  if (type != CommandType::Response) this->waiting_for_response_ = true;
  write_array(packet);
  log_packet(packet, true);
}

void PanasonicACCNT::handle_poll() {
  if (millis() - this->last_packet_sent_ > POLL_INTERVAL) {
    send_command(CMD_POLL, CommandType::Normal, POLL_HEADER);
  }
}

void PanasonicACCNT::handle_cmd() {
  if (!this->cmd.empty() && millis() - this->last_packet_sent_ > CMD_INTERVAL) {
    send_command(this->cmd, CommandType::Normal, CTRL_HEADER);
    this->cmd.clear();
  }
}

bool PanasonicACCNT::verify_packet() {
  if (this->rx_buffer_.size() < 12) return false;
  if (this->rx_buffer_[0] != CTRL_HEADER && this->rx_buffer_[0] != POLL_HEADER) return false;
  if (this->rx_buffer_[1] != this->rx_buffer_.size() - 3) return false;
  uint8_t checksum = 0;
  for (uint8_t b : this->rx_buffer_) checksum += b;
  return checksum == 0;
}

} // namespace CNT
} // namespace panasonic_ac
} // namespace esphome
