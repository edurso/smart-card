#pragma once

#include <optional>

#include "card.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "hw.hpp"
#include "imu.hpp"
#include "rfid.hpp"
#include "speaker.hpp"


namespace card {

    class SmartCard {
        RFID rfid{};
        IMU imu{};
        Speaker speaker{};
        TIM_HandleTypeDef* timer{};
        bool initialized{};

        // For Testing Speaker
        std::size_t cnt{};
        std::uint32_t freq = 0;
        bool going_up = true;

    public:
        SmartCard() = default;
        SmartCard(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* int_tim, TIM_HandleTypeDef* sp_tim, const std::uint32_t tim_ch, const GPIOPin select_pin, const GPIOPin reset_pin)
        : rfid{hspi, select_pin, reset_pin}, imu{hi2c}, speaker{sp_tim, tim_ch}, timer{int_tim}, initialized{false} {
        }

        /**
         * Initialize the SmartCard
         */
        auto init() -> void {
            debug("\n");
            debug("Initializing...");

            // Start Timer Interrupts
            check(HAL_TIM_Base_Start_IT(timer));

            // Initialize RFID Module
            debug("Initializing RFID...");
            rfid.init();

            // Initialize IMU
            debug("Initializing IMU...");
            imu.init();

            // Initialize Speaker
            debug("Initializing Speaker...");
            speaker.init();

            debug("Initialized\n\r");
            initialized = true;
        }

        /**
         * Write data to the memory on the MIFARE RFID Card
         * @param data Data to be written to the card (not to exceed 47 blocks x 16 bytes/block)
         * @return An optional card transaction. std::nullopt indicates write was not attempted.
         */
        [[nodiscard]] auto write_card(const std::string& data) -> std::optional<CardTransaction> {
            return rfid.write_card(data);
        }

        /**
         * Indicates an external interrupt has been fired.
         */
        auto fired() -> void {
            imu.fired();

            if (const auto payload = rfid.read_card()) {
                if (const auto& [transaction, data] = *payload; transaction == SUCCESS) {
                    debug("Card Found: \"" + data + "\"");
                    speaker.start(PLAY_SUCCESS);
                } else {
                    debug("Card Found Defective");
                    speaker.start(PLAY_ERROR);
                }
            } else {
                debug("No Card Found");
                speaker.start(SILENT);
            }
        }

        auto update_speaker() -> void {
            speaker.update();
        }

    };

}
