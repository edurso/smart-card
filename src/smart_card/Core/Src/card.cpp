#include "card.hpp"

#include "main.h"

// Handles defined in IOC
extern TIM_HandleTypeDef htim1; // Variable Hz
extern TIM_HandleTypeDef htim7; // 100 Hz

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;

// Rename handles
#define SPEAKER_TIMER &htim1
#define INT_TIMER &htim7
#define LCD_SPI_H &hspi1
#define SPI_H &hspi3
#define I2C_H &hi2c1


namespace card {

    SmartCard smart_card;
    bool initialized{};
    bool write{};
    contact_t current_contact{};

    TS_StateTypeDef ts{};
    uint16_t x_boxsize{}, y_boxsize{};
    uint16_t oldcolor{};
    uint16_t currentcolor{};
    int touched{};

    // Format: "John Doe|john@doe.com|+1 (123) 567-1234|These are some notes|~"
    // const std::string data = "John Doe|john@doe.com|+1 (123) 567-1234|These are some notes about John|~";
    // const std::string data = "Rick Astley|Never gonna give you up|867-5309|Never gonna let you down...|~";
    const std::string data = "Eric D'Urso|edurso@umich.edu|+1 (734) 560-3417|EECS373 Student, edurso.com|~";
    // const std::string data = "Alanson Sample|apsample@umich.edu|+1 (734) 763-1669|alansonsample.com|~";
    // const std::string data = "Ethan McKean|emckean@umich.edu|+1 (373) 373-4823|Website Coming Soon|~";
    // const std::string data = "Luke Nelson|lukenels@umich.edu|+1 (734) 892-6993|Quandale Dingle's #1 Fan!|~";

    const bool WRITE = false;

    auto get_data(const req_t req) -> contact_t {
        if (!initialized)
            return Contact().get_contact_t();
        return smart_card.get_data(req);
    }

    auto draw_main_page(uint16_t x_boxsize, uint16_t y_boxsize, uint16_t color) -> void {
        BSP_LCD_SetTextColor(color);
        BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, 0, x_boxsize, y_boxsize);
        BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2, (uint8_t*)"MY INFO",
                                CENTER_MODE);
        BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, y_boxsize, x_boxsize, y_boxsize);
        BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize + y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2,
                                (uint8_t*)"NEXT", CENTER_MODE);
        BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, y_boxsize * 2, y_boxsize, y_boxsize);
        BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize * 2 + y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2,
                                (uint8_t*)"PREVIOUS", CENTER_MODE);

        std::string text = "RESET";
        uint16_t xpos = (y_boxsize - text.length() * BSP_LCD_GetFont()->Width) / 2;

        BSP_LCD_DrawRect(0, y_boxsize * 2, y_boxsize, y_boxsize);
        BSP_LCD_DisplayStringAt(xpos, y_boxsize * 2 + y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2,
                                (uint8_t*)text.data(), LEFT_MODE);
    }

    auto draw_my_page(uint16_t x_boxsize, uint16_t y_boxsize, uint16_t color) -> void {
        BSP_LCD_SetTextColor(color);
        BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, 0, x_boxsize, y_boxsize);
        BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2, (uint8_t*)"BACK",
                                CENTER_MODE);
    }

    auto draw_contact(contact_t c, int erase) -> void {
        if (erase == 0) {
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        }
        else {
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        }
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height, (uint8_t*)c.name, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 2, (uint8_t*)c.email, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 3, (uint8_t*)c.phone, CENTER_MODE);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height * 4, (uint8_t*)c.notes, CENTER_MODE);
    }

    auto draw_contact_page(contact_t& current_contact, page_t current_page, uint16_t x_boxsize, uint16_t y_boxsize, uint16_t color) -> void {
        if (previous_page == MAIN_PAGE) {
            draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_BLACK);
        }
        else if (previous_page == MY_PAGE) {
            draw_my_page(x_boxsize, y_boxsize, LCD_COLOR_BLACK);
        }
        draw_contact(current_contact, 1);
        BSP_LCD_SetTextColor(color);
        BSP_LCD_DisplayStringAt(0, BSP_LCD_GetFont()->Height, (uint8_t*)"Add new contact?", CENTER_MODE);
        std::string text = "NO";
        uint16_t xpos = (y_boxsize - text.length() * BSP_LCD_GetFont()->Width) / 2;
        BSP_LCD_DrawRect(0, y_boxsize * 2, y_boxsize, y_boxsize);
        BSP_LCD_DisplayStringAt(xpos, y_boxsize * 2 + y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2,
                                (uint8_t*)text.data(), LEFT_MODE);

        BSP_LCD_DrawRect(BSP_LCD_GetXSize() - x_boxsize, y_boxsize * 2, y_boxsize, y_boxsize);
        BSP_LCD_DisplayStringAt(x_boxsize * 1.75, y_boxsize * 2 + y_boxsize / 2 - BSP_LCD_GetFont()->Height / 2,
                                (uint8_t*)"YES", CENTER_MODE);
    }

    auto init_callback() -> void {
        debug("\n");
        debug("Initializing...");
        debug("Initializing LCD...");
        BSP_LCD_Init();
        BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
        BSP_LCD_Clear(LCD_COLOR_BLACK);

        // ts_calib();

        x_boxsize = BSP_LCD_GetYSize() / 3;
        y_boxsize = BSP_LCD_GetYSize() / 3;
        draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);

        currentcolor = LCD_COLOR_RED;

        current_page = MAIN_PAGE;
        touched = 0;
        contact_page_drawn = 1;

        initialized = true;
        current_contact = Contact().get_contact_t();

        const auto rfid_cs_pin = GPIOPin(GPIOA, GPIO_PIN_0);
        const auto rfid_rst_pin = GPIOPin(GPIOA, GPIO_PIN_11);

        smart_card = SmartCard(data, SPI_H, I2C_H, INT_TIMER, SPEAKER_TIMER, TIM_CHANNEL_1, rfid_cs_pin, rfid_rst_pin
                               // GPIOPin(GPIOA, GPIO_PIN_11)
        );
        smart_card.init();
    }

    auto main_loop() -> void {
        if (!initialized) {
            debug("Not Initialized, Exiting");
            Error_Handler();
        }

        current_contact = get_data(RESET_CONTACTS);
        draw_contact(current_contact, 0);

        while (true) {
            if (contact_page_drawn == 0) {
                draw_contact_page(current_contact, current_page, x_boxsize, y_boxsize, LCD_COLOR_WHITE);
                contact_page_drawn = 1;
            }
            BSP_TS_GetState(&ts);
            if (ts.TouchDetected && touched == 0) {
                switch (current_page) {
                case MAIN_PAGE:
                    y_boxsize = BSP_LCD_GetYSize() / 3;
                    if (ts.X > BSP_LCD_GetXSize() - x_boxsize) {
                        // touching a button on the right
                        if (ts.Y >= 0 && ts.Y < y_boxsize) {
                            // my page
                            draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_BLACK);
                            current_page = MY_PAGE;
                            draw_my_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);
                            draw_contact(current_contact, 1);
                            current_contact = get_data(MY_CARD);
                            draw_contact(current_contact, 0);
                        }
                        else if (ts.Y >= y_boxsize && ts.Y < y_boxsize * 2) {
                            // next
                            draw_contact(current_contact, 1);
                            current_contact = get_data(NEXT_CARD);
                            draw_contact(current_contact, 0);
                        }
                        else if (ts.Y >= y_boxsize * 2 && ts.Y < y_boxsize * 3) {
                            // previous
                            draw_contact(current_contact, 1);
                            current_contact = get_data(PREV_CARD);
                            draw_contact(current_contact, 0);
                        }
                    }
                    else if (ts.X < x_boxsize && ts.Y >= y_boxsize * 2 && ts.Y < y_boxsize * 3) {
                        // touching reset button
                        draw_contact(current_contact, 1);
                        current_contact = get_data(RESET_CONTACTS);
                        draw_contact(current_contact, 0);
                    }
                    break;
                case MY_PAGE:
                    if (ts.X > BSP_LCD_GetXSize() - x_boxsize) {
                        if (ts.Y >= 0 && ts.Y < y_boxsize) {
                            // back
                            draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_BLACK);
                            draw_contact(current_contact, 1);
                            current_contact = get_data(BACK);
                            draw_contact(current_contact, 0);
                            current_page = MAIN_PAGE;
                            draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);
                        }
                    }
                    break;
                case NEW_CONTACT:
                    debug("new contact page");
                    if (ts.X < x_boxsize && ts.Y >= y_boxsize * 2 && ts.Y < y_boxsize * 3) {
                        // touching no
                        get_data(REJECT_CONTACT);
                        draw_contact_page(current_contact, current_page, x_boxsize, y_boxsize, LCD_COLOR_BLACK);
                        current_page = MAIN_PAGE;
                        draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);
                        draw_contact(current_contact, 0);
                    }
                    else if (ts.X > BSP_LCD_GetXSize() - x_boxsize && ts.Y >= y_boxsize * 2 && ts.Y < y_boxsize * 3) {
                        // touching yes
                        draw_contact_page(current_contact, current_page, x_boxsize, y_boxsize, LCD_COLOR_BLACK);
                        current_page = MAIN_PAGE;
                        draw_main_page(x_boxsize, y_boxsize, LCD_COLOR_WHITE);
                        current_contact = get_data(ACCEPT_CONTACT);
                        draw_contact(current_contact, 0);
                    }
                    current_page = MAIN_PAGE;
                }
                touched = 1;
            }
            if (!ts.TouchDetected) {
                touched = 0;
            }
            HAL_Delay(1);
        }
    }

    auto write_card() -> void {
        if (!write) {
            write = true;

            // const auto data = "";
            // const auto data = "This is some data on the card.";
            // const auto data = "This is some different data on a different card.";
            // const auto data = "This is some different data on a keychain tag.";
            // const auto data = "This data is on the card. The card can Store 1KB of Data!";
            // const auto data = "\n\rNever gonna give you up\n\rNever gonna let you down\n\rNever gonna run around and
            // desert you\n\rNever gonna make you cry\n\rNever gonna say goodbye\n\rNever gonna tell a lie and hurt
            // you\n\r"; const auto data = "According to all known laws of aviation, there is no way a bee should be
            // able to fly. Its wings are too small to get its fat little body off the ground. The bee, of course, flies
            // anyway because bees don't care what humans think is impossible. Yellow, black. Yellow, black. Yellow,
            // black. Yellow, black. Ooh, black and yellow! Let's shake it up a little. Barry! Breakfast is ready!
            // Coming! Hang on a second. Hello? Barry? Adam? Can you believe this is happening? I can't. I'll pick you
            // up. Looking sharp. Use the stairs, Your father paid good money for those. Sorry. I'm excited. Here's the
            // graduate. We're very proud of you, son. A perfect report card, all B's. Very proud. Ma! I got a thing
            // going here. You got lint on your fuzz. Ow! That's me!";

            if (const auto result = smart_card.write_card(data)) {
                if (*result == SUCCESS) {
                    debug("Card Found: Write Successful");
                }
                else {
                    debug("Failed to write to card");
                }
            }
            else {
                debug("No Card Found");
                write = false;
            }
        }
    }

    auto imu_interrupt_callback() -> void {
        if (!initialized)
            return;

        if (WRITE) {
            write_card();
        } else {
            smart_card.motion_detected();
        }
    }

    auto noise_callback() -> void {
        if (!initialized)
            return;

        // NOTE disable on write_card() call
        if (!WRITE) {
            smart_card.update_speaker();
            smart_card.update_card_read_state();
        }
    }

} // namespace card


extern "C"
{

    void init() { card::init_callback(); }

    void loop() { card::main_loop(); }

    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
        if (htim == INT_TIMER) {
            card::noise_callback();
        }
    }

    // ReSharper disable once CppParameterMayBeConst
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        if (GPIO_Pin == GPIO_PIN_1) {
            card::imu_interrupt_callback();
        }
    }
}
