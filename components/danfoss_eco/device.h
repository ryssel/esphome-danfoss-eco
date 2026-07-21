#pragma once

#include <set>
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/climate/climate.h"

#include "esphome/core/preferences.h"

#include "helpers.h"
#include "command.h"
#include "properties.h"
#include "my_component.h"
#include "xxtea.h"

#ifdef USE_ESP32

#include <esp_gattc_api.h>

namespace esphome
{
  namespace danfoss_eco
  {
    using namespace std;
    using namespace climate;

    class Device : public MyComponent, public esphome::ble_client::BLEClientNode
    {
    public:
      Device() : xxtea(make_shared<Xxtea>()){};

      void dump_config() override
      {
        LOG_CLIMATE("", "Danfoss Eco eTRV", this);
        ESP_LOGCONFIG(TAG, "  MAC Address: %s", this->parent()->address_str());
        LOG_SENSOR("", "Battery Level", this->battery_level_);
        LOG_SENSOR("", "Room Temperature", this->temperature_);
        LOG_BINARY_SENSOR("", "Problems", this->problems_);
      }

      void setup() override;
      void loop() override;
      void update() override;
      void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) override;

      void set_secret_key(uint8_t *, bool) override;

      void set_secret_key(const string &);
      void set_pin_code(const string &);
      void set_request_timeout_ms(uint32_t timeout_ms);

    protected:
      void control(const ClimateCall &call) override;

      void connect();
      void disconnect();

      void write_pin();
      void on_write_pin(esp_ble_gattc_cb_param_t::gattc_write_evt_param);

      void on_read(esp_ble_gattc_cb_param_t::gattc_read_char_evt_param);
      void on_write(esp_ble_gattc_cb_param_t::gattc_write_evt_param);
      void request_device_state_();

      shared_ptr<Xxtea> xxtea;

      shared_ptr<WritableProperty> p_pin{nullptr};
      shared_ptr<BatteryProperty> p_battery{nullptr};
      shared_ptr<TemperatureProperty> p_temperature{nullptr};
      shared_ptr<SettingsProperty> p_settings{nullptr};
      shared_ptr<ErrorsProperty> p_errors{nullptr};
      shared_ptr<SecretKeyProperty> p_secret_key{nullptr};

      set<shared_ptr<DeviceProperty>> properties{nullptr};

    private:
      void teardown_connection_(bool clear_queue, bool reset_backoff, const char *reason);

      ESPPreferenceObject secret_pref_;
      uint32_t pin_code_ = 0;

      uint8_t request_counter_ = 0;
      bool request_watchdog_active_ = false;
      uint32_t request_watchdog_started_ms_ = 0;
      uint32_t request_watchdog_timeout_ms_ = 15000;
      uint32_t request_timeout_ms_ = 15000;
      uint8_t timeout_backoff_level_ = 0;
      bool preserve_backoff_on_next_disconnect_ = false;
      bool scheduled_poll_pending_ = false;
      uint32_t scheduled_poll_due_ms_ = 0;
      uint16_t poll_spread_ms_ = 0;
      static constexpr uint32_t REQUEST_TIMEOUT_MIN_MS = 1000;
      static constexpr uint32_t REQUEST_TIMEOUT_MAX_MS = 60000;
      CommandQueue commands_;
    };

  } // namespace danfoss_eco
} // namespace esphome

#endif // USE_ESP32
