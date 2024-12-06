#pragma once

#include <optional>
#include <vector>

#include "card.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "hw.hpp"
#include "imu.hpp"
#include "rfid.hpp"
#include "speaker.hpp"
#include "contact.hpp"
#include "contact.h"


namespace card {

    class SmartCard {
        RFID rfid{};
        IMU imu{};
        Speaker speaker{};
        TIM_HandleTypeDef* timer{};
        GPIOPin red_led{};
        GPIOPin green_led{};
        GPIOPin lcd_cs_pin{};
        GPIOPin ts_cs_pin{};
        std::size_t fired_counter{};
        bool read_valid{};
        bool initialized{};

        std::vector<Contact> contacts{};
        std::size_t current_contact_idx{};
        Contact current_contact{};
        Contact me{};

    public:
        SmartCard() = default;
        SmartCard(
            const std::string& me,
            SPI_HandleTypeDef* hspi,
            I2C_HandleTypeDef* hi2c,
            TIM_HandleTypeDef* int_tim,
            TIM_HandleTypeDef* sp_tim,
            const std::uint32_t tim_ch,
            const GPIOPin select_pin,
            const GPIOPin reset_pin,
            const GPIOPin led_error_pin,
            const GPIOPin led_success_pin,
            const GPIOPin lcd_cs_pin,
            const GPIOPin ts_cs_pin
            ) :
        rfid{hspi, select_pin, reset_pin},
        imu{hi2c},
        speaker{sp_tim, tim_ch},
        timer{int_tim},
        red_led{led_error_pin},
        green_led{led_success_pin},
        lcd_cs_pin{lcd_cs_pin},
        ts_cs_pin{ts_cs_pin},
        initialized{false},
        me{me}
        {
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
        auto motion_detected() -> void {
            imu.fired();
            const auto lcd_cs_state = lcd_cs_pin.read();
            const auto ts_cs_state = ts_cs_pin.read();
            lcd_cs_pin.write(GPIO_PIN_SET);
            ts_cs_pin.write(GPIO_PIN_SET);

            // Limits Over-Frequent Reading
            if (!read_valid) {

                green_led.write(GPIO_PIN_RESET);
                red_led.write(GPIO_PIN_RESET);

                if (const auto payload = rfid.read_card()) {
                    if (const auto& [transaction, data] = *payload; transaction == SUCCESS) {
                        speaker.start(PLAY_SUCCESS);
                        green_led.write(GPIO_PIN_SET);
                        red_led.write(GPIO_PIN_RESET);
                        if (const auto contact = Contact(data); contact.is_valid()) {
                            auto exists = false;
                            for (const auto& c : contacts) {
                                if (contact.same_as(c)) {
                                    exists = true;
                                    debug("Contact " + contact.get_name() + " Already Added");
                                }
                            }
                            if (!exists) {
                                contacts.push_back(contact);
                                debug("Contact " + contact.get_name() + " Added");
                            }
                            debug("Card Storing " + std::to_string(contacts.size()) + " Contacts");
                        }
                    } else {
                        debug("Contact Found, Could Not Read");
                        speaker.start(PLAY_ERROR);
                        green_led.write(GPIO_PIN_RESET);
                        red_led.write(GPIO_PIN_SET);
                    }
                    read_valid = true;
                } else {
                    debug("No Card Found");
                    green_led.write(GPIO_PIN_RESET);
                    red_led.write(GPIO_PIN_RESET);
                    speaker.start(SILENT);
                }

            }

            lcd_cs_pin.write(lcd_cs_state);
            ts_cs_pin.write(ts_cs_state);
        }

        auto update_speaker() -> void {
            speaker.update();
        }

        auto update_card_read_state() -> void {
            if (fired_counter++ > 100) {
                read_valid = false;
            }
        }

        auto get_data(const req_t req) -> contact_t {
            Contact contact{}; // default contact

            switch (req) {
            case MY_CARD:
                contact = me;
                break;
            case NEXT_CARD:
                if (!contacts.empty()) {
                    if (++current_contact_idx == contacts.size()) current_contact_idx = 0;
                    current_contact = contacts[current_contact_idx];
                    contact = current_contact;
                }
                break;
            case PREV_CARD:
                if (!contacts.empty()) {
                    if (current_contact_idx == 0) {
                        current_contact_idx = contacts.size() - 1;
                    } else {
                        current_contact_idx--;
                    }
                    current_contact = contacts[current_contact_idx];
                    contact = current_contact;
                }
                break;
            case BACK:
                contact = current_contact;
                break;
            case RESET_CONTACTS:
                contacts.clear();
                break;
            default:
                break;
            }

            return contact.get_contact_t();
        }

    };

}
