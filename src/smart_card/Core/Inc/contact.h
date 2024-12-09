
#pragma once

// TODO this is also stored in the HPP
#define MAX_SIZE 50

struct contact_t {
    char name[MAX_SIZE];
    char email[MAX_SIZE];
    char phone[MAX_SIZE];
    char notes[MAX_SIZE];
};

enum req_t {
    MY_CARD,
    NEXT_CARD,
    PREV_CARD,
    BACK,
    RESET_CONTACTS,
    REJECT_CONTACT,
    ACCEPT_CONTACT
};
