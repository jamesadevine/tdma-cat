#ifndef TDMA_CAT_H
#define TDMA_CAT_H

#include "MicroBitConfig.h"

#define TDMA_CAT_WINDOW_SIZE_MS         1000
#define TDMA_CAT_SLOT_COUNT             4
#define TDMA_CAT_SLOT_SIZE_MS           (TDMA_CAT_WINDOW_SIZE_MS/TDMA_CAT_SLOT_COUNT)
#define TDMA_CAT_SLOT_SIZE_US           (TDMA_CAT_SLOT_SIZE_MS * 1000)
#define TDMA_CAT_TABLE_SIZE             TDMA_CAT_SLOT_COUNT
#define TDMA_CAT_DEFAULT_EXPIRATION     5
#define TDMA_CAT_DEFAULT_ADVERT_TTL     6
#define TDMA_CAT_ADVERTISEMENT_SLOT     0
#define TDMA_CAT_UNITIALISED_SLOT       -1

#define TDMA_SLOT_FLAGS_ADVERTISED      0x1

struct TDMACATSuperFrame;

struct TDMA_CAT_Slot {
    uint64_t device_identifier;
    uint16_t slot_identifier;
    uint8_t expiration;
    uint8_t ttl:4; uint8_t flags:4;
};

struct TDMA_CAT_Advertisement {
    uint64_t device_identifier;
    uint16_t slot_identifier;
};

int tdma_init(uint64_t device_identifier);

int tdma_set_slot(TDMA_CAT_Slot slot);

TDMA_CAT_Slot tdma_get_current_slot();

int tdma_synchronise(TDMA_CAT_Slot slot);

int tdma_is_synchronised();

int tdma_is_advertising_slot();

int tdma_advert_required();

int tdma_advance_slot();

TDMA_CAT_Slot tdma_get_slot(uint32_t slot_identifier);

int tdma_slots_till_next_tx();

int tdma_fill_advertising_frame(TDMACATSuperFrame*);

#endif