//original code from https://github.com/esphome/esphome/blob/dev/esphome/components/alpha3/
//I just figured out the new request "command" bytes and response type bytes

#include "alpha2.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <lwip/sockets.h>  //gives ntohl
#include <cstdio>

#ifdef USE_ESP32

namespace esphome {
namespace alpha2 {

static const char *const TAG = "alpha2";

static void log_geni_packet(const char *direction, const char *address, const uint8_t *data, size_t length) {
  constexpr size_t MAX_LOGGED_BYTES = 96;
  char hex[3 * MAX_LOGGED_BYTES + 4];
  size_t offset = 0;
  const size_t bytes_to_log = std::min(length, MAX_LOGGED_BYTES);

  for (size_t i = 0; i < bytes_to_log; i++) {
    const int written = std::snprintf(hex + offset, sizeof(hex) - offset, i == 0 ? "%02X" : " %02X",
                                      static_cast<unsigned>(data[i]));
    if (written < 0 || static_cast<size_t>(written) >= sizeof(hex) - offset)
      return;
    offset += written;
  }
  if (length > bytes_to_log)
    std::snprintf(hex + offset, sizeof(hex) - offset, " ...");
  else
    hex[offset] = '\0';

  ESP_LOGD(TAG, "[%s] %s GENI (%u bytes): %s", address, direction, static_cast<unsigned>(length), hex);
}

void Alpha2::dump_config() {
  ESP_LOGCONFIG(TAG, "ALPHA2");
  LOG_SENSOR(" ", "Flow", this->flow_sensor_);
  LOG_SENSOR(" ", "Head", this->head_sensor_);
  LOG_SENSOR(" ", "Power", this->power_sensor_);
  LOG_SENSOR(" ", "Current", this->current_sensor_);
  LOG_SENSOR(" ", "Speed", this->speed_sensor_);
  LOG_SENSOR(" ", "Voltage", this->voltage_sensor_);
}

void Alpha2::setup() {}

void Alpha2::extract_publish_sensor_value_(const uint8_t *response, int16_t length, int16_t response_offset,
                                           int16_t value_offset, sensor::Sensor *sensor, float factor) {
  if (sensor == nullptr)
    return;
  // we need to handle cases where a value is split over two packets
  const int16_t value_length = 4;  // 32bit float
  // offset inside current response packet
  auto rel_offset = value_offset - response_offset;
  if (rel_offset <= -value_length)
    return;  // aready passed the value completly
  if (rel_offset >= length)
    return;  // value not in this packet

  auto start_offset = std::max(0, rel_offset);
  auto end_offset = std::min((int16_t) (rel_offset + value_length), length);
  auto copy_length = end_offset - start_offset;
  auto buffer_offset = std::max(-rel_offset, 0);
  std::memcpy(this->buffer_ + buffer_offset, response + start_offset, copy_length);

  if (rel_offset + value_length <= length) {
    // we have the whole value
    void *buffer = this->buffer_;                          // to prevent warnings when casting the pointer
    *((int32_t *) buffer) = ntohl(*((int32_t *) buffer));  // values are big endian
    float fvalue = *((float *) buffer);
    sensor->publish_state(fvalue * factor);
  }
}

bool Alpha2::is_current_response_type_(const uint8_t *response_type) {
  return !std::memcmp(this->response_type_, response_type, GENI_RESPONSE_TYPE_LENGTH);
}

void Alpha2::handle_geni_response_(const uint8_t *response, uint16_t length) {
  if (this->response_offset_ >= this->response_length_) {
    ESP_LOGD(TAG, "[%s] GENI response begin", this->parent_->address_str());
    if (length < GENI_RESPONSE_HEADER_LENGTH) {
      ESP_LOGW(TAG, "[%s] response too short", this->parent_->address_str());
      return;
    }
    if (response[0] != 36 || response[2] != 248 || response[3] != 231 || response[4] != 10) {
      ESP_LOGW(TAG, "[%s] response bytes %d %d %d %d %d don't match GENI HEADER", this->parent_->address_str(),
               response[0], response[1], response[2], response[3], response[4]);
      return;
    }
    this->response_length_ = response[1] - GENI_RESPONSE_HEADER_LENGTH + 2;  // maybe 2 byte checksum
    this->response_offset_ = -GENI_RESPONSE_HEADER_LENGTH;
    std::memcpy(this->response_type_, response + 5, GENI_RESPONSE_TYPE_LENGTH);
  }

  auto extract_publish_sensor_value = [response, length, this](int16_t value_offset, sensor::Sensor *sensor,
                                                               float factor) {
    this->extract_publish_sensor_value_(response, length, this->response_offset_, value_offset, sensor, factor);
  };

  if (this->is_current_response_type_(GENI_RESPONSE_TYPE_FLOW_HEAD)) {
    ESP_LOGD(TAG, "[%s] FLOW HEAD Response", this->parent_->address_str());
    extract_publish_sensor_value(GENI_RESPONSE_FLOW_OFFSET, this->flow_sensor_, 3600.0F);
    extract_publish_sensor_value(GENI_RESPONSE_HEAD_OFFSET, this->head_sensor_, .0001F);
  } else if (this->is_current_response_type_(GENI_RESPONSE_TYPE_POWER)) {
    ESP_LOGD(TAG, "[%s] POWER Response", this->parent_->address_str());
    extract_publish_sensor_value(GENI_RESPONSE_POWER_OFFSET, this->power_sensor_, 1.0F);
    extract_publish_sensor_value(GENI_RESPONSE_CURRENT_OFFSET, this->current_sensor_, 1.0F);
    extract_publish_sensor_value(GENI_RESPONSE_MOTOR_SPEED_OFFSET, this->speed_sensor_, 1.0F);
    extract_publish_sensor_value(GENI_RESPONSE_VOLTAGE_AC_OFFSET, this->voltage_sensor_, 1.0F);
  } else {
    ESP_LOGW(TAG, "unkown GENI response Type %d %d %d %d %d %d %d %d", this->response_type_[0], this->response_type_[1],
             this->response_type_[2], this->response_type_[3], this->response_type_[4], this->response_type_[5],
             this->response_type_[6], this->response_type_[7]);
  }

  this->response_offset_ += length;
}

void Alpha2::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        this->response_offset_ = 0;
        this->response_length_ = 0;
        this->extended_stage_ = ExtendedStage::IDLE;
        this->extended_initialized_ = false;
        this->extended_response_expected_ = 0;
        this->extended_response_received_ = 0;
        ESP_LOGI(TAG, "[%s] connection open", this->parent_->address_str());
      }
      break;
    }
    case ESP_GATTC_CONNECT_EVT: {
      if (std::memcmp(param->connect.remote_bda, this->parent_->get_remote_bda(), 6) != 0)
        return;
      if (this->extended_connection_) {
        ESP_LOGI(TAG, "[%s] Extended connection: skipping link encryption to match Android capture",
                 this->parent_->address_str());
        break;
      }
      auto ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      if (ret) {
        ESP_LOGW(TAG, "esp_ble_set_encryption failed, status=%x", ret);
      }
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] BLE disconnected, reason=0x%02X", this->parent_->address_str(), param->disconnect.reason);
      this->node_state = espbt::ClientState::IDLE;
      this->extended_stage_ = ExtendedStage::IDLE;
      this->extended_initialized_ = false;
      this->extended_response_expected_ = 0;
      this->extended_response_received_ = 0;
      if (this->flow_sensor_ != nullptr)
        this->flow_sensor_->publish_state(NAN);
      if (this->head_sensor_ != nullptr)
        this->head_sensor_->publish_state(NAN);
      if (this->power_sensor_ != nullptr)
        this->power_sensor_->publish_state(NAN);
      if (this->current_sensor_ != nullptr)
        this->current_sensor_->publish_state(NAN);
      if (this->speed_sensor_ != nullptr)
        this->speed_sensor_->publish_state(NAN);
      if (this->speed_sensor_ != nullptr)
        this->voltage_sensor_->publish_state(NAN);
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *chr = this->parent_->get_characteristic(ALPHA2_GENI_SERVICE_UUID, ALPHA2_GENI_CHARACTERISTIC_UUID);
      if (chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No GENI service found at device, not an Alpha2..?", this->parent_->address_str());
        break;
      }
      this->geni_handle_ = chr->handle;
      ESP_LOGD(TAG, "[%s] Registering for GENI notifications, characteristic=0x%04X",
               this->parent_->address_str(), this->geni_handle_);
      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                      chr->handle);
      if (status) {
        ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d", this->parent_->address_str(),
                 status);
      }
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "[%s] GENI notification registration failed, status=%d", this->parent_->address_str(),
                 param->reg_for_notify.status);
        break;
      }
      ESP_LOGI(TAG, "[%s] GENI notifications registered", this->parent_->address_str());
      this->node_state = espbt::ClientState::ESTABLISHED;
      if (this->extended_connection_)
        ESP_LOGI(TAG, "[%s] Starting extended Android-style GENI initialization", this->parent_->address_str());
      this->update();
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle == this->geni_handle_) {
        log_geni_packet("RX", this->parent_->address_str(), param->notify.value, param->notify.value_len);
        if (param->notify.value_len >= 5 && param->notify.value[0] == 36 && param->notify.value[2] == 248 &&
            param->notify.value[3] == 231 && param->notify.value[4] == 10) {
          this->handle_geni_response_(param->notify.value, param->notify.value_len);
        }
        if (this->extended_connection_ && this->extended_stage_ != ExtendedStage::IDLE &&
            this->is_extended_response_complete_(param->notify.value, param->notify.value_len)) {
          switch (this->extended_stage_) {
            case ExtendedStage::WAITING_FOR_INITIALIZATION:
              this->send_extended_request_(ExtendedStage::WAITING_FOR_VERSION);
              break;
            case ExtendedStage::WAITING_FOR_VERSION:
              this->send_extended_request_(ExtendedStage::WAITING_FOR_NAME);
              break;
            case ExtendedStage::WAITING_FOR_NAME:
              this->extended_initialized_ = true;
              if (this->flow_sensor_ != nullptr || this->head_sensor_ != nullptr) {
                this->send_extended_request_(ExtendedStage::WAITING_FOR_FLOW_HEAD);
              } else if (this->power_sensor_ != nullptr || this->current_sensor_ != nullptr ||
                         this->speed_sensor_ != nullptr || this->voltage_sensor_ != nullptr) {
                uint8_t geni_request_power[] = {39, 7, 231, 248, 10, 3, 87, 0, 69, 138, 205};
                this->send_request_(geni_request_power, sizeof(geni_request_power));
                this->extended_stage_ = ExtendedStage::WAITING_FOR_POWER;
              } else {
                this->extended_stage_ = ExtendedStage::IDLE;
              }
              break;
            case ExtendedStage::WAITING_FOR_FLOW_HEAD: {
              if (this->power_sensor_ != nullptr || this->current_sensor_ != nullptr ||
                  this->speed_sensor_ != nullptr || this->voltage_sensor_ != nullptr) {
                uint8_t geni_request_power[] = {39, 7, 231, 248, 10, 3, 87, 0, 69, 138, 205};
                this->extended_stage_ = ExtendedStage::WAITING_FOR_POWER;
                this->send_request_(geni_request_power, sizeof(geni_request_power));
              } else {
                this->extended_stage_ = ExtendedStage::IDLE;
              }
              break;
            }
            case ExtendedStage::WAITING_FOR_POWER:
              this->extended_stage_ = ExtendedStage::IDLE;
              break;
            case ExtendedStage::IDLE:
              break;
          }
        }
      }
      break;
    }
    default:
      break;
  }
}

void Alpha2::send_request_(uint8_t *request, size_t len) {
  log_geni_packet("TX", this->parent_->address_str(), request, len);
  auto status =
      esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->geni_handle_, len,
                               request, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status)
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char failed, status=%d", this->parent_->address_str(), status);
}

void Alpha2::send_extended_request_(ExtendedStage stage) {
  static uint8_t geni_request_initialize[] = {39, 7, 255, 248, 2, 3, 148, 149, 150, 79, 145};
  static uint8_t geni_request_version[] = {39, 5, 231, 248, 7, 1, 17, 64, 9};
  static uint8_t geni_request_name[] = {39, 5, 231, 248, 7, 1, 1, 82, 56};
  static uint8_t geni_request_flow_head[] = {39, 7, 231, 248, 10, 3, 93, 1, 34, 98, 124};

  this->extended_stage_ = stage;
  this->extended_response_expected_ = 0;
  this->extended_response_received_ = 0;

  switch (stage) {
    case ExtendedStage::WAITING_FOR_INITIALIZATION:
      this->send_request_(geni_request_initialize, sizeof(geni_request_initialize));
      break;
    case ExtendedStage::WAITING_FOR_VERSION:
      this->send_request_(geni_request_version, sizeof(geni_request_version));
      break;
    case ExtendedStage::WAITING_FOR_NAME:
      this->send_request_(geni_request_name, sizeof(geni_request_name));
      break;
    case ExtendedStage::WAITING_FOR_FLOW_HEAD:
      this->send_request_(geni_request_flow_head, sizeof(geni_request_flow_head));
      break;
    case ExtendedStage::WAITING_FOR_POWER:
    case ExtendedStage::IDLE:
      this->extended_stage_ = ExtendedStage::IDLE;
      break;
  }
}

bool Alpha2::is_extended_response_complete_(const uint8_t *response, uint16_t length) {
  if (this->extended_response_received_ == 0) {
    if (length < 5 || response[0] != 36 || response[2] != 248 || response[3] != 231) {
      ESP_LOGW(TAG, "[%s] Extended GENI response has an invalid header", this->parent_->address_str());
      this->extended_stage_ = ExtendedStage::IDLE;
      return false;
    }
    this->extended_response_expected_ = static_cast<size_t>(response[1]) + 4;
  }

  // The response length counts bytes following the four-byte GENI prefix.
  this->extended_response_received_ += length;
  if (this->extended_response_received_ < this->extended_response_expected_)
    return false;

  if (this->extended_response_received_ != this->extended_response_expected_) {
    ESP_LOGW(TAG, "[%s] Extended GENI response length mismatch: expected %u bytes, received %u",
             this->parent_->address_str(), static_cast<unsigned>(this->extended_response_expected_),
             static_cast<unsigned>(this->extended_response_received_));
    this->extended_stage_ = ExtendedStage::IDLE;
    this->extended_response_expected_ = 0;
    this->extended_response_received_ = 0;
    return false;
  }

  this->extended_response_expected_ = 0;
  this->extended_response_received_ = 0;
  return true;
}

void Alpha2::update() {
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "[%s] Cannot poll, not connected", this->parent_->address_str());
    return;
  }

  if (this->extended_connection_) {
    if (this->extended_stage_ != ExtendedStage::IDLE) {
      ESP_LOGD(TAG, "[%s] Waiting for extended GENI response", this->parent_->address_str());
      return;
    }
    if (!this->extended_initialized_) {
      this->send_extended_request_(ExtendedStage::WAITING_FOR_INITIALIZATION);
      return;
    }
    if (this->flow_sensor_ != nullptr || this->head_sensor_ != nullptr) {
      this->send_extended_request_(ExtendedStage::WAITING_FOR_FLOW_HEAD);
    } else if (this->power_sensor_ != nullptr || this->current_sensor_ != nullptr || this->speed_sensor_ != nullptr ||
               this->voltage_sensor_ != nullptr) {
      uint8_t geni_request_power[] = {39, 7, 231, 248, 10, 3, 87, 0, 69, 138, 205};
      this->extended_stage_ = ExtendedStage::WAITING_FOR_POWER;
      this->send_request_(geni_request_power, sizeof(geni_request_power));
    }
    return;
  }

  if (this->flow_sensor_ != nullptr || this->head_sensor_ != nullptr) {
    uint8_t geni_request_flow_head[] =   {39, 7, 231, 248, 10, 3, 93, 1, 34, 98, 124};
    this->send_request_(geni_request_flow_head, sizeof(geni_request_flow_head));
    delay(25);  // need to wait between requests
  }
  if (this->power_sensor_ != nullptr || this->current_sensor_ != nullptr || this->speed_sensor_ != nullptr ||
      this->voltage_sensor_ != nullptr) {
    uint8_t geni_request_power[] = {39, 7, 231, 248, 10, 3, 87, 0, 69, 138, 205};
    this->send_request_(geni_request_power, sizeof(geni_request_power));
    delay(25);  // need to wait between requests
  }
}
}  // namespace alpha2
}  // namespace esphome

#endif
