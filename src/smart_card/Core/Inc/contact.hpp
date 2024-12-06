#pragma once

#include <cstring>

#include "debug.hpp"


#define MAX_SIZE 128


namespace card {

    // "John Doe|john@doe.com|+1 (123) 567-1234|These are some notes|~"

    struct contact_t {
        char name[MAX_SIZE];
        char email[MAX_SIZE];
        char phone[MAX_SIZE];
        char notes[MAX_SIZE];
    };

    class Contact {
        std::string name{};
        std::string email{};
        std::string phone{};
        std::string notes{};
        bool valid = false;

        bool parse(const std::string& input) {
            size_t start = 0;
            size_t end = 0;
            bool success = true;
            // debug("Parsing contact");

            // Extract name
            end = input.find('|', start);
            if (end == std::string::npos) {
                debug("Failed to Extract Name");
                success = false;
            }
            name = input.substr(start, end - start);
            // debug("\tname: " + name);

            // Extract email
            start = end + 1;
            end = input.find('|', start);
            if (end == std::string::npos) {
                debug("Failed to Extract Email");
                success = false;
            }
            email = input.substr(start, end - start);
            // debug("\temail: " + email);

            // Extract phone
            start = end + 1;
            end = input.find('|', start);
            if (end == std::string::npos) {
                debug("Failed to Extract Phone Number");
                success = false;
            }
            phone = input.substr(start, end - start);
            // debug("\tphone: " + phone);

            // Extract notes
            start = end + 1;
            end = input.find('|', start);
            if (end == std::string::npos) {
                debug("Failed to Extract Notes");
                success = false;
            }
            notes = input.substr(start, end - start);
            // debug("\tnotes: " + notes);

            // Optional: Validate the remaining `~` ending
            if (input.find('|', end + 1) != std::string::npos || input[end + 1] != '~') {
                debug("Failed to Extract Name");
                success = false;
            }

            return success;
        }

    public:
        Contact() :
            name("No Contacts Stored"),
            email("-----------------"),
            phone("Move Smart Card Near"),
            notes("Another To Connect!")
        {}

        explicit Contact(const std::string& contact_str) {
            valid = parse(contact_str);
        }

        [[nodiscard]] bool is_valid() const {
            return valid;
        }

        [[nodiscard]] std::string get_name() const {
            return name;
        }

        [[nodiscard]] contact_t get_contact_t() const {
            contact_t result{};

            if (name.size() >= MAX_SIZE ||
                email.size() >= MAX_SIZE ||
                phone.size() >= MAX_SIZE ||
                notes.size() >= MAX_SIZE) {
                debug("Invalid Data Recieved");
                result = Contact().get_contact_t();
                return result;
            }

            std::strncpy(result.name, name.c_str(), sizeof(result.name) - 1);
            result.name[sizeof(result.name) - 1] = '\0';

            std::strncpy(result.email, email.c_str(), sizeof(result.email) - 1);
            result.email[sizeof(result.email) - 1] = '\0';

            std::strncpy(result.phone, phone.c_str(), sizeof(result.phone) - 1);
            result.phone[sizeof(result.phone) - 1] = '\0';

            std::strncpy(result.notes, notes.c_str(), sizeof(result.notes) - 1);
            result.notes[sizeof(result.notes) - 1] = '\0';

            return result;
        }

        [[nodiscard]] bool same_as(const Contact& other) const {
            return other.valid == this->valid &&
                    other.name == this->name &&
                    other.email == this->email &&
                    other.phone == this->phone &&
                    other.notes == this->notes;
        }
        
    };
}
