#pragma once

#include "gpio.hpp"
#include "debug.hpp"
#include "hw.hpp"
#include "rfid.hpp"
#include "imu.hpp"


namespace card {

    class SmartCard {
        RFID rfid{};
        TIM_HandleTypeDef* card_read_timer{};
        bool initialized{};

        auto end_card_read() const -> void {
            // Reset For Next Poll
            debug("Reset For Next Poll");
            rfid.halt();

            // Add Newline To Debug Output
            debug("\n\r");
        }

    public:
        SmartCard() = default;
        SmartCard(SPI_HandleTypeDef* hspi, TIM_HandleTypeDef* tim, const GPIOPin select_pin, const GPIOPin reset_pin)
        : rfid{hspi, select_pin, reset_pin}, card_read_timer{tim}, initialized{false} {
        }

        auto init() -> void {
            debug("\n");
            debug("Initializing...");

            // Start Timer Interrupts
            check(HAL_TIM_Base_Start_IT(card_read_timer));

            // Initialize RFID Module
            debug("Initializing RFID...");
            rfid.init();

            debug("Initialized\n\r");
            initialized = true;
        }

        auto card_read() const -> void {
            if (!initialized) return;

            debug("Entering Card Read Callback");
            std::uint8_t str[MAX_LEN];

            // Find Cards Present
            auto status = rfid.request(PICC_REQIDL, str);
            if (status == MI_OK) {
                debug("Card Found");
                debug("\tType " + std::to_string(str[0]));
            } else {
                debug("No Card Found");
                end_card_read();
                return;
            }

            // Get Card UID
            status = rfid.anticoll(str);
            if (status == MI_OK) {
                debugf("\tCard UID: %x:%x:%x:%x\n\r", str[0], str[1], str[2], str[3]);
                const auto capacity = rfid.select_tag(str);
                debug("\tCard Capacity: " + std::to_string(capacity));
            } else {
                debug("\tIssue Resolving Card UID");
                end_card_read();
                return;
            }

            // Reset For Next Poll
            end_card_read();
        }

    };

}
