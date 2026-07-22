#include "device.h"
#include <algorithm>
#include <cmath>
#include <esp_timer.h>

#ifdef USE_ESP32

namespace esphome
{
  namespace danfoss_eco
  {
    static bool startup_marker_logged = false;
    static Device *connect_slot_owner = nullptr;
    static uint32_t connect_slot_available_at_ms = 0;

    static uint32_t now_ms()
    {
      return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    }

    void Device::setup()
    {
      if (!startup_marker_logged)
      {
        startup_marker_logged = true;
        ESP_LOGI(TAG, "[BLE_FLOW][BUILD_MARKER] danfoss_eco_ble_hardening_v4 compiled %s %s", __DATE__, __TIME__);
      }

      shared_ptr<MyComponent> sp_this(this);

      this->p_pin = make_shared<WritableProperty>(sp_this, xxtea, SERVICE_SETTINGS, CHARACTERISTIC_PIN);
      this->p_battery = make_shared<BatteryProperty>(sp_this, xxtea);
      this->p_temperature = make_shared<TemperatureProperty>(sp_this, xxtea);
      this->p_settings = make_shared<SettingsProperty>(sp_this, xxtea);
      this->p_errors = make_shared<ErrorsProperty>(sp_this, xxtea);
      this->p_secret_key = make_shared<SecretKeyProperty>(sp_this, xxtea);

      this->properties = {this->p_pin, this->p_battery, this->p_temperature, this->p_settings, this->p_errors, this->p_secret_key};
      // pretend, we have already discovered the device
      copy_address(this->parent()->get_address(), this->parent()->get_remote_bda());

      const uint8_t *bda = this->parent()->get_remote_bda();
      const uint16_t seed = (static_cast<uint16_t>(bda[4]) << 8) | bda[5];
      this->poll_spread_ms_ = 200 + (seed % 3000);
      ESP_LOGD(TAG, "[%s][BLE_FLOW] periodic poll spread configured: %u ms", this->get_name().c_str(), static_cast<unsigned>(this->poll_spread_ms_));
    }

    void Device::loop()
    {
      if (this->status_has_error())
      {
        this->teardown_connection_(true, true, "status error");
        this->status_clear_error();
      }

      if (this->scheduled_poll_pending_ && static_cast<int32_t>(now_ms() - this->scheduled_poll_due_ms_) >= 0)
      {
        this->scheduled_poll_pending_ = false;
        this->connect();
        this->request_device_state_();
      }

      if (this->node_state != ClientState::ESTABLISHED)
      {
        if (this->node_state == ClientState::IDLE && !this->commands_.empty())
        {
          this->connect();
        }
        return;
      }

      Command *cmd = this->commands_.pop();
      while (cmd != nullptr)
      {
        const char *command_type = cmd->type == CommandType::WRITE ? "WRITE" : "READ";
        uint16_t handle = cmd->property != nullptr ? cmd->property->handle : INVALID_HANDLE;

        const bool request_sent = cmd->execute(this->parent());
        ESP_LOGD(TAG, "[%s][BLE_FLOW] queued %s request: handle=%#04x, sent=%d", this->get_name().c_str(), command_type, handle, request_sent);

        if (request_sent)
        {
          if (this->request_counter_ == 0xFF)
          {
            ESP_LOGW(TAG, "[%s] request counter overflow guard hit, forcing disconnect", this->get_name().c_str());
            delete cmd;
            this->teardown_connection_(true, true, "request counter overflow");
            return;
          }

          this->request_counter_++;
          ESP_LOGD(TAG, "[%s][BLE_FLOW] pending requests incremented: %u", this->get_name().c_str(), static_cast<unsigned>(this->request_counter_));

          if (!this->request_watchdog_active_)
          {
            this->request_watchdog_active_ = true;
            this->request_watchdog_started_ms_ = now_ms();
            const uint32_t timeout_factor = 1U << this->timeout_backoff_level_;
            this->request_watchdog_timeout_ms_ = std::min(this->request_timeout_ms_ * timeout_factor, REQUEST_TIMEOUT_MAX_MS);
            ESP_LOGV(TAG, "[%s][BLE_FLOW] request watchdog started (%u ms, backoff level=%u)", this->get_name().c_str(), this->request_watchdog_timeout_ms_, static_cast<unsigned>(this->timeout_backoff_level_));
          }
        }

        delete cmd;
        cmd = this->commands_.pop();
      }

      if (this->request_watchdog_active_ && this->request_counter_ > 0)
      {
        const uint32_t elapsed_ms = now_ms() - this->request_watchdog_started_ms_;
        if (elapsed_ms >= this->request_watchdog_timeout_ms_)
        {
          ESP_LOGW(TAG, "[%s][BLE_FLOW] request watchdog timeout: elapsed=%u ms, pending=%u - forcing disconnect", this->get_name().c_str(), static_cast<unsigned>(elapsed_ms), static_cast<unsigned>(this->request_counter_));
          if (this->timeout_backoff_level_ < 5)
            this->timeout_backoff_level_++;

          this->preserve_backoff_on_next_disconnect_ = true;

          this->teardown_connection_(true, false, "request watchdog timeout");
          return;
        }
      }

      // once we are done with pending commands - check to see if there are any pending requests
      // if there are no pending requests - we are done with the device for now and should disconnect
      if (this->request_counter_ == 0)
        this->disconnect();
    }

    void Device::update()
    {
      if (!this->scheduled_poll_pending_)
      {
        this->scheduled_poll_pending_ = true;
        this->scheduled_poll_due_ms_ = now_ms() + this->poll_spread_ms_;
        ESP_LOGD(TAG, "[%s][BLE_FLOW] periodic poll scheduled in %u ms", this->get_name().c_str(), static_cast<unsigned>(this->poll_spread_ms_));
      }
    }

    void Device::control(const ClimateCall &call)
    {
      // CRITICAL SAFETY: Prevent simultaneous mode and temperature changes
      if (call.get_mode().has_value() && call.get_target_temperature().has_value())
      {
        ESP_LOGW(TAG, "[%s] Handling mode first, temp deferred", this->get_name().c_str());
        ClimateCall mode_call(this);
        mode_call.set_mode(*call.get_mode());
        this->control(mode_call);
        return;
      }

      if (call.get_target_temperature().has_value())
      {
        if (!this->p_temperature->data)
        {
          ESP_LOGE(TAG, "[%s] No temperature data - read first", this->get_name().c_str());
          return;
        }

        TemperatureData &t_data = (TemperatureData &)(*this->p_temperature->data);
        float new_temp = *call.get_target_temperature();
        
        if (new_temp < 5.0f || new_temp > 30.0f)
        {
          ESP_LOGE(TAG, "[%s] INVALID NEW TEMP: %.1f (rejecting)", this->get_name().c_str(), new_temp);
          return;
        }
        
        if (std::abs(t_data.target_temperature - new_temp) >= 0.1f)
        {
          t_data.target_temperature = new_temp;
          this->commands_.push(new Command(CommandType::WRITE, this->p_temperature));
          this->connect();
        }
      }

      if (call.get_mode().has_value())
      {
        if (!this->p_settings->data)
        {
          ESP_LOGE(TAG, "[%s] No settings data - read first", this->get_name().c_str());
          return;
        }

        SettingsData &s_data = (SettingsData &)(*this->p_settings->data);
        ClimateMode new_mode = *call.get_mode();
        ClimateMode current_mode = s_data.device_mode;
        
        if (new_mode != current_mode)
        {
          ESP_LOGD(TAG, "[%s] Mode change: %d -> %d", this->get_name().c_str(), (int)current_mode, (int)new_mode);
          
          s_data.device_mode = new_mode;
          this->mode = s_data.device_mode;
          this->publish_state();
          this->commands_.push(new Command(CommandType::WRITE, this->p_settings));
          this->connect();
        }
      }
    }

    void Device::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
    {
      switch (event)
      {
      case ESP_GATTC_CONNECT_EVT:
        if (memcmp(param->connect.remote_bda, this->parent()->get_remote_bda(), 6) != 0)
          return; // event does not belong to this client, exit gattc_event_handler

        ESP_LOGD(TAG, "[%s] connect, conn_id=%d", this->get_name().c_str(), param->connect.conn_id);
        break;

      case ESP_GATTC_OPEN_EVT:
        if (param->open.status == ESP_GATT_OK)
          ESP_LOGV(TAG, "[%s] open, conn_id=%d", this->get_name().c_str(), param->open.conn_id);
        else
        {
          this->open_fail_cooldown_until_ms_ = now_ms() + OPEN_FAIL_RETRY_COOLDOWN_MS;
          ESP_LOGW(TAG, "[%s] failed to open, conn_id=%d, status=%#04x", this->get_name().c_str(), param->open.conn_id, param->open.status);
          ESP_LOGW(TAG, "[%s][BLE_FLOW] applying open-fail cooldown: %u ms", this->get_name().c_str(), static_cast<unsigned>(OPEN_FAIL_RETRY_COOLDOWN_MS));
        }
        break;

      case ESP_GATTC_CLOSE_EVT:
        if (param->close.status == ESP_GATT_OK)
          ESP_LOGV(TAG, "[%s] close, conn_id=%d, reason=%d", this->get_name().c_str(), param->close.conn_id, param->close.reason);
        else
          ESP_LOGW(TAG, "[%s] failed to close, conn_id=%d, status=%#04x", this->get_name().c_str(), param->close.conn_id, param->close.status);
        break;

      case ESP_GATTC_DISCONNECT_EVT:
      {
        const bool reset_backoff = !this->preserve_backoff_on_next_disconnect_;
        ESP_LOGD(TAG, "[%s][BLE_FLOW] disconnect, conn_id=%d, reason=%#04x, reset_backoff=%d, backoff_level=%u", this->get_name().c_str(), param->disconnect.conn_id, (int)param->disconnect.reason, reset_backoff, static_cast<unsigned>(this->timeout_backoff_level_));
        this->preserve_backoff_on_next_disconnect_ = false;
        this->teardown_connection_(false, reset_backoff, "gatt disconnect event");
      }
        break;

      case ESP_GATTC_SEARCH_CMPL_EVT:
        for (auto p : this->properties)
          p->init_handle(this->parent());

        write_pin();
        break;

      case ESP_GATTC_WRITE_CHAR_EVT:
        if (param->write.handle == this->p_pin->handle)
          this->on_write_pin(param->write);
        else
          this->on_write(param->write);
        break;

      case ESP_GATTC_READ_CHAR_EVT:
        this->on_read(param->read);
        break;

      default:
        ESP_LOGV(TAG, "[%s] unhandled event: event=%d, gattc_if=%d", this->get_name().c_str(), (int)event, gattc_if);
        break;
      }
    }

    void Device::write_pin()
    {
      ESP_LOGD(TAG, "[%s] writing pin", this->get_name().c_str());

      uint8_t pin_bytes[sizeof(uint32_t)];
      write_int(pin_bytes, 0, this->pin_code_);

      if (!this->p_pin->write_request(this->parent(), pin_bytes, sizeof(pin_bytes)))
        this->status_set_error();
    }

    void Device::on_read(esp_ble_gattc_cb_param_t::gattc_read_char_evt_param param)
    {
      ESP_LOGD(TAG, "[%s][BLE_FLOW] read response: handle=%#04x, status=%#04x", this->get_name().c_str(), param.handle, param.status);
      if (this->request_counter_ == 0)
      {
        ESP_LOGW(TAG, "[%s][BLE_FLOW] read response with empty pending counter", this->get_name().c_str());
        return;
      }

      this->request_counter_--;
      ESP_LOGD(TAG, "[%s][BLE_FLOW] pending requests decremented after read: %u", this->get_name().c_str(), static_cast<unsigned>(this->request_counter_));

      if (this->request_counter_ == 0)
      {
        this->request_watchdog_active_ = false;
        this->request_watchdog_timeout_ms_ = this->request_timeout_ms_;
        this->timeout_backoff_level_ = 0;
      }

      if (param.status != ESP_GATT_OK)
      {
        ESP_LOGW(TAG, "[%s] failed to read characteristic: handle=%#04x, status=%#04x", this->get_name().c_str(), param.handle, param.status);
        return;
      }

      auto device_property = find_if(properties.begin(), properties.end(),
                                     [&param](shared_ptr<DeviceProperty> p)
                                     { return p->handle == param.handle; });

      if (device_property != properties.end())
        (*device_property)->update_state(param.value, param.value_len);
      else
        ESP_LOGW(TAG, "[%s] unknown property with handle=%#04x", this->get_name().c_str(), param.handle);
    }

    void Device::on_write(esp_ble_gattc_cb_param_t::gattc_write_evt_param param)
    {
      ESP_LOGD(TAG, "[%s][BLE_FLOW] write response: handle=%#04x, status=%#04x", this->get_name().c_str(), param.handle, param.status);
      if (this->request_counter_ == 0)
      {
        ESP_LOGW(TAG, "[%s][BLE_FLOW] write response with empty pending counter", this->get_name().c_str());
        return;
      }

      this->request_counter_--;
      ESP_LOGD(TAG, "[%s][BLE_FLOW] pending requests decremented after write: %u", this->get_name().c_str(), static_cast<unsigned>(this->request_counter_));

      if (this->request_counter_ == 0)
      {
        this->request_watchdog_active_ = false;
        this->request_watchdog_timeout_ms_ = this->request_timeout_ms_;
        this->timeout_backoff_level_ = 0;
      }

      if (param.status != ESP_GATT_OK)
        ESP_LOGW(TAG, "[%s] failed to write characteristic: handle=%#04x, status=%#04x", this->get_name().c_str(), param.handle, param.status);
      else
        this->request_device_state_();
    }

    void Device::request_device_state_()
    {
      if (this->xxtea->status() != XXTEA_STATUS_SUCCESS)
        return;

      ESP_LOGI(TAG, "[%s] requesting device state", this->get_name().c_str());

      this->commands_.push(new Command(CommandType::READ, this->p_battery));
      this->commands_.push(new Command(CommandType::READ, this->p_temperature));
      this->commands_.push(new Command(CommandType::READ, this->p_settings));
      this->commands_.push(new Command(CommandType::READ, this->p_errors));
    }

    void Device::on_write_pin(esp_ble_gattc_cb_param_t::gattc_write_evt_param param)
    {
      if (param.status != ESP_GATT_OK)
      {
        ESP_LOGE(TAG, "[%s] pin FAILED, status=%#04x", this->get_name().c_str(), param.status);
        this->disconnect();
        this->mark_failed();
        return;
      }

      ESP_LOGD(TAG, "[%s] pin OK", this->get_name().c_str());
      this->node_state = ClientState::ESTABLISHED;

      // after PIN is written, we might need to read the secret_key from the device
      if (this->xxtea->status() == XXTEA_STATUS_NOT_INITIALIZED && this->p_secret_key->handle != INVALID_HANDLE)
      {
        ESP_LOGD(TAG, "[%s] attempting to read the device secret_key", this->get_name().c_str());
        this->commands_.push(new Command(CommandType::READ, this->p_secret_key));
      }
    }

    void Device::connect()
    {
      const uint32_t now = now_ms();

      if (this->node_state == ClientState::ESTABLISHED)
      {
        return;
      }

      if (static_cast<int32_t>(now - this->open_fail_cooldown_until_ms_) < 0)
      {
        if (static_cast<int32_t>(now - this->last_connect_attempt_ms_) >= static_cast<int32_t>(CONNECT_ATTEMPT_INTERVAL_MS))
        {
          this->last_connect_attempt_ms_ = now;
          const uint32_t remaining_ms = this->open_fail_cooldown_until_ms_ - now;
          ESP_LOGV(TAG, "[%s][BLE_FLOW] open-fail cooldown active: %u ms remaining", this->get_name().c_str(), static_cast<unsigned>(remaining_ms));
        }
        return;
      }

      if (static_cast<int32_t>(now - this->last_connect_attempt_ms_) < static_cast<int32_t>(CONNECT_ATTEMPT_INTERVAL_MS))
      {
        return;
      }

      if (connect_slot_owner != nullptr && connect_slot_owner != this)
      {
        this->last_connect_attempt_ms_ = now;
        ESP_LOGV(TAG, "[%s][BLE_FLOW] connect slot busy, waiting", this->get_name().c_str());
        return;
      }

      if (connect_slot_owner == nullptr)
      {
        if (static_cast<int32_t>(now - connect_slot_available_at_ms) < 0)
        {
          this->last_connect_attempt_ms_ = now;
          ESP_LOGV(TAG, "[%s][BLE_FLOW] connect slot cooling down", this->get_name().c_str());
          return;
        }

        connect_slot_owner = this;
        ESP_LOGD(TAG, "[%s][BLE_FLOW] acquired connect slot", this->get_name().c_str());
      }

      this->last_connect_attempt_ms_ = now;

      if (this->xxtea->status() == XXTEA_STATUS_NOT_INITIALIZED)
        ESP_LOGI(TAG, "[%s] Short press Danfoss Eco hardware button NOW in order to allow reading the secret key", this->get_name().c_str());

      if (!parent()->enabled)
      {
        ESP_LOGD(TAG, "[%s] re-enabling ble_client", this->get_name().c_str());
        parent()->set_enabled(true);
      }
      
      this->parent()->connect(); // trigger BLE connection attempt
    }

    void Device::disconnect()
    {
      this->teardown_connection_(false, true, "normal disconnect");
    }

    void Device::set_request_timeout_ms(uint32_t timeout_ms)
    {
      this->request_timeout_ms_ = std::max(REQUEST_TIMEOUT_MIN_MS, std::min(timeout_ms, REQUEST_TIMEOUT_MAX_MS));
      this->request_watchdog_timeout_ms_ = this->request_timeout_ms_;

      ESP_LOGD(TAG, "[%s][BLE_FLOW] request timeout configured: %u ms", this->get_name().c_str(), this->request_timeout_ms_);
    }

    void Device::teardown_connection_(bool clear_queue, bool reset_backoff, const char *reason)
    {
      if (this->request_counter_ > 0 || !this->commands_.empty())
      {
        ESP_LOGW(TAG, "[%s][BLE_FLOW] teardown (%s): pending=%u, queued=%u, clear_queue=%d, reset_backoff=%d", this->get_name().c_str(), reason, static_cast<unsigned>(this->request_counter_), static_cast<unsigned>(this->commands_.size()), clear_queue, reset_backoff);
      }

      if (connect_slot_owner == this)
      {
        connect_slot_owner = nullptr;
        connect_slot_available_at_ms = now_ms() + CONNECT_SLOT_COOLDOWN_MS;
        ESP_LOGD(TAG, "[%s][BLE_FLOW] released connect slot, cooldown=%u ms", this->get_name().c_str(), static_cast<unsigned>(CONNECT_SLOT_COOLDOWN_MS));
      }

      this->parent()->set_enabled(false);
      this->request_counter_ = 0;
      this->request_watchdog_active_ = false;

      if (reset_backoff)
      {
        this->request_watchdog_timeout_ms_ = this->request_timeout_ms_;
        this->timeout_backoff_level_ = 0;
      }

      if (clear_queue)
        this->commands_.clear();

      this->node_state = ClientState::IDLE;
    }

    void Device::set_pin_code(const string &str)
    {
      if (str.length() > 0)
        this->pin_code_ = atoi((const char *)str.c_str());

      ESP_LOGD(TAG, "[%s] PIN: %04d", this->get_name().c_str(), this->pin_code_);
    }

    void Device::set_secret_key(const string &str)
    {
      // initialize the preference object
      uint32_t hash = fnv1_hash("danfoss_eco_secret__" + this->get_name());
      this->secret_pref_ = global_preferences->make_preference<SecretKeyValue>(hash, true);

      if (str.length() > 0)
      {
        uint8_t buff[SECRET_KEY_LENGTH];
        ESP_LOGD(TAG, "[%s] secret_key was passed via config", this->get_name().c_str());
        parse_hex_str(str.c_str(), 32, buff);
        this->set_secret_key(buff, false);
      }
      else
      {
        auto key_buff = SecretKeyValue();
        if (this->secret_pref_.load(&key_buff))
        {
          // use persisted secret value
          ESP_LOGD(TAG, "[%s] secret_key was loaded from flash", this->get_name().c_str());
          this->set_secret_key(key_buff.value, false);
        }
      }
    }

    void Device::set_secret_key(uint8_t *key, bool persist)
    {
      ESP_LOGD(TAG, "[%s] secret_key bytes: %s", this->get_name().c_str(), format_hex_pretty(key, SECRET_KEY_LENGTH).c_str());

      int status = this->xxtea->set_key(key, SECRET_KEY_LENGTH);
      if (status != XXTEA_STATUS_SUCCESS)
      {
        ESP_LOGE(TAG, "xxtea initialization failed, status: %d", status);
        this->mark_failed();
      }
      else if (persist)
      {
        // if xxtea was initialized successfully and secret_key should be persisted
        auto key_buff = SecretKeyValue(key);
        this->secret_pref_.save(&key_buff);
        global_preferences->sync();

        ESP_LOGI(TAG, "[%s] secret_key was saved to flash", this->get_name().c_str());
      }
    }

  } // namespace danfoss_eco
} // namespace esphome

#endif
