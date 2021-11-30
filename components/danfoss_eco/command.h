#pragma once

#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

#include "properties.h"

namespace esphome
{
    namespace danfoss_eco
    {

        enum class CommandType
        {
            READ,
            WRITE
        };

        struct Command
        {
            Command(CommandType t, std::shared_ptr<DeviceProperty> const &p) : type(t), property(p) {}

            CommandType type; // 0 - read, 1 - write
            std::shared_ptr<DeviceProperty> property;
        };

        class CommandQueue : public esphome::esp32_ble_tracker::Queue<Command>
        {
        public:
            bool is_empty() { return this->q_.empty(); }
        };

    } // namespace danfoss_eco
} // namespace esphome