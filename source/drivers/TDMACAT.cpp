#include "TDMACAT.h"
#include "ErrorNo.h"
#include "TDMACATRadio.h"

static TDMA_CAT_Slot table[TDMA_CAT_TABLE_SIZE];
static int current_slot = TDMA_CAT_UNITIALISED_SLOT;

int tdma_init(uint64_t device_identifier)
{
    memset(&table, 0, sizeof(TDMA_CAT_Slot) * TDMA_CAT_TABLE_SIZE);

    TDMA_CAT_Slot adv;

    adv.device_identifier = device_identifier;
    adv.slot_identifier = TDMA_CAT_ADVERTISEMENT_SLOT;
    adv.expiration = 0xff;
    adv.ttl = 0;
    adv.flags = 0;

    table[TDMA_CAT_ADVERTISEMENT_SLOT] = adv;

    return MICROBIT_OK;
}

int tdma_set_slot(TDMA_CAT_Slot slot)
{
    uint16_t index = slot.slot_identifier;
    if (index > TDMA_CAT_TABLE_SIZE - 1 || index == TDMA_CAT_ADVERTISEMENT_SLOT)
        return MICROBIT_NO_RESOURCES;

    if (slot.device_identifier != table[index].device_identifier && table[index].expiration)
        return MICROBIT_NO_RESOURCES;

    table[index] = slot;
    return MICROBIT_OK;
}

TDMA_CAT_Slot tdma_get_slot(uint32_t slot_identifier)
{
    return table[slot_identifier];
}

TDMA_CAT_Slot tdma_get_current_slot()
{
    TDMA_CAT_Slot blank;
    blank.expiration = 255;

    if (current_slot == TDMA_CAT_UNITIALISED_SLOT)
        return blank;

    return table[current_slot];
}

void tdma_set_current_slot(int slot_id)
{
    current_slot = slot_id;
}

/**
 * Returns 1 if transmission is required in the next time slot.
 **/
int tdma_advance_slot()
{
    current_slot = (current_slot + 1) % TDMA_CAT_TABLE_SIZE;
    return (table[current_slot].ttl == 0) ? 1 : 0;
}

int tdma_is_synchronised()
{
    return current_slot != TDMA_CAT_UNITIALISED_SLOT;
}

int tdma_is_advertising_slot()
{
    return current_slot == TDMA_CAT_ADVERTISEMENT_SLOT;
}

int tdma_advert_required()
{
    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].ttl ==  0 && !(table[i].flags & TDMA_SLOT_FLAGS_ADVERTISED))
            return 1;

    return 0;
}

/**
 * Returns the number of slots until this device must transmit.
 **/
int tdma_slots_till_next_tx()
{
    int i = (current_slot + 1) % TDMA_CAT_TABLE_SIZE;
    int slot_counter = 0;

    while (i != current_slot)
    {
        slot_counter++;

        if (table[i].ttl == 0)
            return slot_counter;

        i = (i + 1) % TDMA_CAT_TABLE_SIZE;
    }

    return MICROBIT_NO_RESOURCES;
}

int tdma_fill_advertising_frame(TDMACATSuperFrame* frame)
{
    uint8_t* slots = frame->payload;
    int advertIndex = 0;

    TDMA_CAT_Slot meta = table[TDMA_CAT_ADVERTISEMENT_SLOT];

    frame->ttl = TDMA_CAT_DEFAULT_ADVERT_TTL;
    frame->initial_ttl = TDMA_CAT_DEFAULT_ADVERT_TTL;
    frame->device_id = meta.device_identifier;
    frame->slot_id = TDMA_CAT_ADVERTISEMENT_SLOT;
    frame->frame_id = 0;
    frame->flags = TMDMA_CAT_FRAME_FLAGS_ADVERT;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].ttl ==  0 && !(table[i].flags & TDMA_SLOT_FLAGS_ADVERTISED))
        {
            table[i].flags &= ~(TDMA_SLOT_FLAGS_ADVERTISED);
            slots[advertIndex++] = i;
        }

    frame->length = advertIndex + TDMA_CAT_HEADER_SIZE - 1;

    return MICROBIT_OK;
}

