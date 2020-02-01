#ifndef TDMA_CAT_H
#define TDMA_CAT_H

#include "MicroBitConfig.h"

#define TDMA_CAT_WINDOW_SIZE_MS         1000
#define TDMA_CAT_SLOT_COUNT             50
#define TDMA_CAT_SLOT_SIZE_MS           (TDMA_CAT_WINDOW_SIZE_MS/TDMA_CAT_SLOT_COUNT)
#define TDMA_CAT_SLOT_SIZE_US           (TDMA_CAT_SLOT_SIZE_MS * 1000)
#define TDMA_CAT_TABLE_SIZE             TDMA_CAT_SLOT_COUNT
#define TDMA_CAT_DEFAULT_EXPIRATION     5
#define TDMA_CAT_DEFAULT_ADVERT_TTL     6
#define TDMA_CAT_ADVERTISEMENT_SLOT     0
#define TDMA_CAT_UNITIALISED_SLOT       -1
#define TDMA_CAT_ADV_SLOT_MATCH_MAX     6
#define TDMA_CAT_NEVER_EXPIRE           0xff

#define TDMA_SLOT_FLAGS_ADVERTISE       0x1
#define TDMA_SLOT_FLAGS_UNITIALISED     0x2
#define TDMA_SLOT_FLAGS_OWNER           0x4
#define TDMA_SLOT_FLAGS_ERROR           0x8

struct TDMACATSuperFrame;

struct TDMA_CAT_Slot {
    uint64_t device_identifier;
    uint8_t slot_identifier;
    uint8_t errors;
    uint8_t expiration;
    uint8_t distance:4; uint8_t flags:4;
};

struct TDMA_CAT_Advertisement {
    uint64_t device_identifier;
    uint16_t slot_identifier;
};

/**
 * Initialises the TDMA table and stores the identifier for this device.
 *
 * @param device_identifier the number that uniquely identifies this device.
 *                          Used in all look ups.
 **/
int tdma_init(uint64_t device_identifier);

/**
 * Sets the current slot to id.
 *
 * @param id the new slot id to move to.
 **/
void tdma_set_current_slot(int id);

/**
 * Sets a slot to the provided information using the slot identifier provided in slot.
 *
 * @param slot the new slot metadata to store.
 **/
int tdma_set_slot(TDMA_CAT_Slot slot);

/**
 * Clears a slot using the provided slot_identifier.
 *
 * @param slot_identifier the slot metadata to clear.
 **/
int tdma_clear_slot(uint32_t slot_identifier);

/**
 * Returns a slot from the table using the provided slot_identifier.
 *
 * Mostly used for debugging.
 *
 * @param slot_identifier the slot metadata to fetch. If the slot is unused or
 *                        not found, the slot flags will be set to uninitialised.
 **/
TDMA_CAT_Slot tdma_get_slot(uint32_t slot_identifier);

/**
 * Retrieves the current slot index used to index the tdma schedule
 *
 * @returns the current slot index.
 **/
int tdma_current_slot_idx();

/**
 * Retrieves the current slot metadata from the tdma schedule
 *
 * @returns the currently indexed slot from the table
 **/
TDMA_CAT_Slot tdma_get_current_slot();

/**
 * Determines if the current_slot_index variable has been initialised.
 *
 * @returns 1 if it has, 0 otherwise.
 **/
int tdma_is_synchronised();

/**
 * Gets the distance for the current slot.
 *
 * @returns the detected distance stored in the table, or if the current slot is uninitialised
 *          the biggest distance expected by the tdma driver (TDMA_CAT_DEFAULT_ADVERT_TTL)
 **/
int tdma_slot_distance();

/**
 * Determines if this device owns the current slot.
 *
 * @returns 1 if it does, 0 otherwise.
 **/
int tdma_is_owner();

/**
 * Determines if the current slot is an advertising slot.
 *
 * @returns 1 if it is, 0 otherwise.
 **/
int tdma_is_advertising_slot();

/**
 * Determines if the current slot is being used by a device.
 *
 * @returns 1 if it is, 0 otherwise.
 **/
int tdma_slot_is_occupied();

/**
 * Determines if this device needs to send an advertisement frame
 *
 * @returns 1 if it does, 0 otherwise.
 **/
int tdma_advert_required();

/**
 * Determines if this device needs to send an error frame.
 *
 * An error frame is used to tell devices that their schedules are colliding
 * indicated by an unusually high presence of errors (compared to the average).
 *
 * @returns 1 if it does, 0 otherwise.
 **/
int tdma_renegotiation_required();

/**
 * Advances the schedule to the next slot.
 *
 * @returns whether this device is the owner of the upcoming slot.
 **/
int tdma_advance_slot();

/**
 * Counts the number of slots owned by this device.
 *
 * @returns the number of slots owned by this device.
 **/
int tdma_count_slots();

/**
 * Obtains a slot to be used by this device.
 *
 * @note this only flags a slot for advertisement, a slot cannot be used
 * until it has been advertised
 **/
void tdma_obtain_slot();

/**
 * Returns the biggest distance vector recorded by this device.
 *
 * A distance vector is simply the initial_ttl of a packet minus the
 * received ttl of the packet (i.e. the number of hops).
 **/
int tdma_get_distance();

/**
 * Records a reception error for the current slot.
 **/
void tdma_rx_error();

/**
 * Fills the given frame with the appropriate advertising metadata.
 **/
int tdma_fill_advertising_frame(TDMACATSuperFrame*);

/**
 * Fills the given frame with the appropriate error metadata.
 **/
int tdma_fill_renogotiation_frame(TDMACATSuperFrame*);

/**
 * Performs clean up every window delta. This includes flagging errors for slots
 * and expiring slots that have not seen activity.
 **/
void tdma_window_tick();

#endif