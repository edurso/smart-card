#pragma once

#include <cstdint>

#include "debug.hpp"
#include "hw.hpp"

#include "main.h"


namespace card {

    enum SpeakerStatus : uint8_t {
        SILENT = 0,
        PLAY_ERROR,
        PLAY_SUCCESS,
    };

    enum SpeakerTone : uint16_t {
        NO_TONE = 0,
        LOW = 150,
        MEDIUM = 350,
        HIGH = 550,
    };

    class Speaker {
        static constexpr std::uint32_t TIM_FREQ = 32000000;
        static constexpr std::uint32_t TONE_DURATION = 200;

        TIM_HandleTypeDef* htim{};
        std::size_t counter{};
        std::uint32_t channel{};
        volatile SpeakerStatus status{};
        SpeakerTone tone{};

        static auto calculate_psc(const std::uint32_t freq) -> std::uint32_t {
            if (freq == 0) return 0;
            return ((TIM_FREQ / ((1000 * freq) + 1)) - 1);
        }

        auto set_frequency(const std::uint32_t freq) const -> void {
            __HAL_TIM_SET_PRESCALER(htim, calculate_psc(freq));
        }

    public:
        Speaker() = default;
        explicit Speaker(TIM_HandleTypeDef* htim, const std::uint32_t channel)
        : htim(htim), channel(channel), status(SILENT), tone(NO_TONE) {
        }

        auto init() const -> void {
            check(HAL_TIM_PWM_Start(htim, channel));
            set_frequency(0);
        }

        auto start(const SpeakerStatus status) -> void {
            if (this->status == SILENT) {
                this->status = status;
            }
        }

        auto update() -> void {

            // debugf("Status: %u\n\r", status);
            if (counter++ % TONE_DURATION != 0) return;

            switch (status) {
                case SILENT: {
                    status = SILENT;
                    tone = NO_TONE;
                    break;
                }
                case PLAY_ERROR: {
                    status = PLAY_ERROR;

                    if (tone == NO_TONE) tone = HIGH;
                    else if (tone == HIGH) tone = MEDIUM;
                    else if (tone == MEDIUM) tone = LOW;
                    else if (tone == LOW) tone = NO_TONE;
                    else debug("Invalid Tone Asserted");

                    if (tone == NO_TONE) status = SILENT;

                    break;
                }
                case PLAY_SUCCESS: {
                    status = PLAY_SUCCESS;

                    if (tone == NO_TONE) tone = LOW;
                    else if (tone == LOW) tone = MEDIUM;
                    else if (tone == MEDIUM) tone = HIGH;
                    else if (tone == HIGH) tone = NO_TONE;
                    else debug("Invalid Tone Asserted");

                    if (tone == NO_TONE) status = SILENT;
                    break;
                }
            }

            set_frequency(tone);
        }

    };

}
