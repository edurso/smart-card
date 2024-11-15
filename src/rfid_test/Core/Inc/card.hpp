#pragma once

#include <array>
#include <vector>

#include "gpio.hpp"
#include "debug.hpp"
#include "hw.hpp"
#include "rfid.hpp"
#include "imu.hpp"


namespace card {
    constexpr std::uint8_t key_a[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    constexpr std::array<std::uint8_t, 47> blocks = {1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 16, 17, 18, 20, 21, 22,
                                                    24, 25, 26, 28, 29, 30, 32, 33, 34, 36, 37, 38, 40, 41, 42,
                                                    44, 45, 46, 48, 49, 50, 52, 53, 54, 56, 57, 58, 60, 61, 62};

    class SmartCard {
        RFID rfid{};
        IMU imu{};
        TIM_HandleTypeDef* timer{};
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
        SmartCard(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* tim, const GPIOPin select_pin, const GPIOPin reset_pin)
        : rfid{hspi, select_pin, reset_pin}, imu{hi2c}, timer{tim}, initialized{false} {
        }

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

                std::vector<std::string> block_values{};
                std::string data;

                for (const auto& block : blocks) {
                    std::string block_value = "Block " + std::to_string(block) + ": ";
                    std::uint8_t process = rfid.auth(0x60, block, key_a, str);
                    if (process == MI_OK) {
                        std::uint8_t block_data[16];
                        process = rfid.read_data(block, block_data);
                        if (process == MI_OK) {
                            std::string line;
                            for (const unsigned char i : block_data) line += (i == 0x00) ? ' ' : i;
                            line = reduce(line);
                            block_value += line;
                            data += line;
                        } else {
                            block_value += "Error Reading Data Packet";
                        }
                    } else {
                        block_value += "Could Not Authenticate To Block";
                    }
                    block_values.push_back(block_value);
                }

                debug("\tCard Data: " + data);

            } else {
                debug("\tIssue Resolving Card UID");
                end_card_read();
                return;
            }

            // Reset For Next Poll
            end_card_read();
        }

        auto fired() -> void {
            imu.fired();
        }

    };

}
