#pragma once

#include <optional>

#include "gpio.hpp"
#include "debug.hpp"
#include "hw.hpp"
#include "rfid.hpp"
#include "imu.hpp"


namespace card {

    class SmartCard {
        RFID rfid{};
        IMU imu{};
        TIM_HandleTypeDef* timer{};
        bool initialized{};

    public:
        SmartCard() = default;
        SmartCard(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* tim, const GPIOPin select_pin, const GPIOPin reset_pin)
        : rfid{hspi, select_pin, reset_pin}, imu{hi2c}, timer{tim}, initialized{false} {
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

            debug("Initialized\n\r");
            initialized = true;
        }

        /**
         * Read data from the memory on the MIFARE RFID Card
         * @return The data in the 47 readable blocks of the card as a string.
         */
        [[nodiscard]] auto read_card() -> std::optional<std::string> {
            return rfid.read_card();
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
        }

    };

}
