#pragma once

#include <algorithm>
#include <array>
#include <optional>
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

    enum CardTransaction {
        SUCCESS,
        FAILURE,
    };

    class SmartCard {
        RFID rfid{};
        IMU imu{};
        TIM_HandleTypeDef* timer{};
        bool initialized{};

        auto end_card_read() const -> void {
            // Reset For Next Poll
            // debug("Reset For Next Poll");
            rfid.halt();

            // Add Newline To Debug Output
            // debug("\n\r");
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

        [[nodiscard]] auto card_read() const -> std::optional<std::string> {
            if (!initialized) return std::nullopt;

            std::string data;
            std::uint8_t str[MAX_LEN];

            // Re-Initialize RFID Module
            rfid.init();

            // Find Cards Present
            auto status = rfid.request(PICC_REQIDL, str);
            if (status != MI_OK) {
                end_card_read();
                return std::nullopt;
            }

            // Get Card UID
            status = rfid.anticoll(str);
            if (status == MI_OK) {
                const auto capacity = rfid.select_tag(str);
                std::vector<std::string> block_values{};

                for (const auto& block : blocks) {
                    std::string block_value = "Block " + std::to_string(block) + ": ";
                    std::uint8_t process = rfid.auth(0x60, block, key_a, str);
                    if (process == MI_OK) {
                        std::uint8_t block_data[16];
                        process = rfid.read_data(block, block_data);
                        if (process == MI_OK) {
                            std::string line;
                            for (const unsigned char i : block_data) {
                                if (i != 0x00) {
                                    line += i;
                                }
                            }
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

            } else {
                end_card_read();
                return std::nullopt;
            }

            // Reset For Next Poll
            end_card_read();
            return data;
        }

        [[nodiscard]] auto card_write(const std::string& data) const -> std::optional<CardTransaction> {
            if (!initialized) return std::nullopt;

            constexpr std::uint8_t block_size = 16;
            if (data.length() > (blocks.size() * block_size)) return std::nullopt;

            std::uint8_t str[MAX_LEN];

            // Re-Initialize RFID Module
            rfid.init();

            // Find Cards Present
            auto status = rfid.request(PICC_REQIDL, str);
            if (status != MI_OK) {
                end_card_read();
                return std::nullopt;
            }

            // Get Card UID
            status = rfid.anticoll(str);
            if (status == MI_OK) {
                const auto capacity = rfid.select_tag(str);
                std::vector<std::string> block_values{};
                std::uint8_t block_data[block_size];
                std::size_t block_index = 0;

                for (const auto& block : blocks) {
                    std::string block_value = "Block " + std::to_string(block) + ": ";
                    std::uint8_t process = rfid.auth(0x60, block, key_a, str);
                    if (process == MI_OK) {
                        std::fill(std::begin(block_data), std::end(block_data), 0x00);

                        const std::size_t start_idx = block_index * block_size;
                        const std::size_t current_chunk_size = std::min(static_cast<std::size_t>(block_size), data.length() - start_idx);

                        if (start_idx < data.length()) {
                            std::copy_n(data.begin() + static_cast<std::uint8_t>(start_idx), current_chunk_size, block_data);
                        }

                        // try {
                        //     debug("Writing data \"" + data.substr(start_idx, current_chunk_size) + "\"");
                        // } catch (...) {}
                        process = rfid.write_data(block, block_data);
                        if (process == MI_OK) {
                            block_value += "Success Writing Data Packet";
                        } else {
                            block_value += "Error Writing Data Packet";
                        }
                    } else {
                        block_value += "Could Not Authenticate To Block";
                    }
                    block_values.push_back(block_value);
                    block_index++;
                }

            } else {
                end_card_read();
                return FAILURE;
            }

            // Reset For Next Poll
            end_card_read();
            return SUCCESS;
        }

        auto fired() -> void {
            imu.fired();
        }

    };

}
