#include "pvvx_mithermometer.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace pvvx_mithermometer {

static const char *TAG = "pvvx_mithermometer";

typedef struct __attribute__ ((packed)) data_t {
//uint8_t     size;   // = 15
//uint8_t     uid;    // = 0x16, 16-bit UUID
//uint16_t    UUID;   // = 0x151A, GATT Service 0x181A Environmental Sensing
uint8_t     MAC[6]; // [0] - lo, .. [6] - hi digits
int16_t     temperature;    // x 0.01 degree
uint16_t    humidity;       // x 0.01 %
uint16_t    battery_mv;     // mV
uint8_t     battery_level;  // 0..100 %
uint8_t     counter;        // measurement count
uint8_t     flags;  // GPIO_TRG pin (marking "reset" on circuit board) flags: 
                    // bit0: GPIO_TRG pin input value (real level)
                    // bit1: GPIO_TRG pin output value (pull Up/Down)
                    // bit2: Output GPIO_TRG pin is controlled according to the set parameters
                    // bit3: Temperature trigger event
                    // bit4: Humidity trigger event
} data_t;

void PVVXMiThermometer::dump_config() {
  ESP_LOGCONFIG(TAG, "PVVX MiThermometer");
  LOG_SENSOR("  ", "Temperature", this->temperature_);
  LOG_SENSOR("  ", "Humidity", this->humidity_);
  LOG_SENSOR("  ", "Battery Level", this->battery_level_);
  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_);
}

bool PVVXMiThermometer::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  if (device.address_uint64() != this->address_) {
    ESP_LOGVV(TAG, "parse_device(): address %llx - expecting %llx", device.address_uint64(), this->address_);
    return false;
  }
  ESP_LOGVV(TAG, "parse_device(): MAC address %s found.", device.address_str().c_str());
  ESP_LOGD(TAG, "parse_device(): MAC address %s found.", device.address_str().c_str());

  bool success = false;
  for (auto &service_data : device.get_service_datas()) {
    auto res = parse_header(service_data);
    if (res->is_duplicate) {
      continue;
    }
    if (!(parse_message(service_data.data, *res))) {
      continue;
    }
    if (!(report_results(res, device.address_str()))) {
      continue;
    }
    if (res->temperature.has_value() && this->temperature_ != nullptr)
      this->temperature_->publish_state(*res->temperature);
    if (res->humidity.has_value() && this->humidity_ != nullptr)
      this->humidity_->publish_state(*res->humidity);
    if (res->battery_level.has_value() && this->battery_level_ != nullptr)
      this->battery_level_->publish_state(*res->battery_level);
    if (res->battery_voltage.has_value() && this->battery_voltage_ != nullptr)
      this->battery_voltage_->publish_state(*res->battery_voltage);
    success = true;
  }

  if (!success) {
    return false;
  }

  return true;
}

optional<ParseResult> PVVXMiThermometer::parse_header(const esp32_ble_tracker::ServiceData &service_data) {
  ParseResult result;
  if (!service_data.uuid.contains(0x1A, 0x18)) {
    ESP_LOGD(TAG, "parse_header(): no service data UUID magic bytes.");
    return {};
  }

  auto raw = service_data.data;
  const  data_t *pvvx_data = reinterpret_cast<const data_t*>(raw.data());

  uint8_t last_frame_count = this->counter;
  if (last_frame_count == pvvx_data->counter) {
    ESP_LOGD(TAG, "parse_header(): duplicate data packet received (%d).", static_cast<int>(last_frame_count));
    result.is_duplicate = true;
    return {};
  }
  this->counter = pvvx_data->counter;
  result.is_duplicate = false;

  return result;
}

bool PVVXMiThermometer::parse_message(const std::vector<uint8_t> &message, ParseResult &result) {
  const uint8_t *data = message.data();
  const int data_length = sizeof(data_t);

  if (message.size() < data_length) {
    ESP_LOGD(TAG, "parse_message(): payload has wrong size (%d - expected at least %d)!", message.size(), data_length);
    return false;
  }

  const  data_t *pvvx_data = reinterpret_cast<const data_t*>(data);
  result.temperature = pvvx_data->temperature / 100.0f;

  result.humidity = pvvx_data->humidity / 100.0f;

  result.battery_level = pvvx_data->battery_level;

  result.battery_voltage = pvvx_data->battery_mv / 1.0e3f;

  return true;
}

bool PVVXMiThermometer::report_results(const optional<ParseResult> &result, const std::string &address) {
  if (!result.has_value()) {
    ESP_LOGD(TAG, "report_results(): no results available.");
    return false;
  }

  ESP_LOGD(TAG, "Got PVVX MiThermometer (%s):", address.c_str());

  if (result->temperature.has_value()) {
    ESP_LOGD(TAG, "  Temperature: %.1f °C", *result->temperature);
  }
  if (result->humidity.has_value()) {
    ESP_LOGD(TAG, "  Humidity: %.0f %%", *result->humidity);
  }
  if (result->battery_level.has_value()) {
    ESP_LOGD(TAG, "  Battery Level: %.0f %%", *result->battery_level);
  }
  if (result->battery_voltage.has_value()) {
    ESP_LOGD(TAG, "  Battery Voltage: %.3f V", *result->battery_voltage);
  }

  return true;
}

}  // namespace pvvx_mithermometer
}  // namespace esphome

#endif
