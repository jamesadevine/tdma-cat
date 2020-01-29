#include "TDMACAT.h"
#include "ErrorNo.h"
#include "TDMACATRadio.h"

static TDMA_CAT_Slot table[TDMA_CAT_TABLE_SIZE];
static volatile int current_slot = TDMA_CAT_UNITIALISED_SLOT;

static volatile int adv_slot_match = 0;
static volatile int adv_slot_counter = 0;

extern void log_int(const char*, int);

int tdma_init(uint64_t device_identifier)
{
    current_slot = TDMA_CAT_UNITIALISED_SLOT;
    memset(&table, 0, sizeof(TDMA_CAT_Slot) * TDMA_CAT_TABLE_SIZE);

    TDMA_CAT_Slot adv;

    adv.device_identifier = device_identifier;
    adv.slot_identifier = TDMA_CAT_ADVERTISEMENT_SLOT;
    adv.expiration = 0xff;
    adv.ttl = 0;
    adv.flags = TDMA_SLOT_FLAGS_OWNER;

    table[TDMA_CAT_ADVERTISEMENT_SLOT] = adv;

    TDMA_CAT_Slot init;
    init.expiration = 0;
    init.ttl = 0;
    init.flags = TDMA_SLOT_FLAGS_UNITIALISED;
    init.slot_identifier = 0;
    init.device_identifier = 0;

    adv_slot_match = microbit_random(TDMA_CAT_ADV_SLOT_MATCH_MAX);

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        table[i] = init;

    return MICROBIT_OK;
}

int tdma_clear_slot(uint32_t slot_identifier)
{
    if (slot_identifier > TDMA_CAT_TABLE_SIZE - 1 || slot_identifier == TDMA_CAT_ADVERTISEMENT_SLOT)
        return MICROBIT_INVALID_PARAMETER;

    table[slot_identifier].flags = TDMA_SLOT_FLAGS_UNITIALISED;
}

int tdma_set_slot(TDMA_CAT_Slot slot)
{
    uint16_t index = slot.slot_identifier;

    if (index > TDMA_CAT_TABLE_SIZE - 1 || index == TDMA_CAT_ADVERTISEMENT_SLOT)
        return MICROBIT_INVALID_PARAMETER;

    if ((table[index].flags & TDMA_SLOT_FLAGS_UNITIALISED) || (table[index].flags & TDMA_SLOT_FLAGS_ADVERTISE) || slot.device_identifier == table[index].device_identifier)
    {
        table[index] = slot;
        return MICROBIT_OK;
    }

    return MICROBIT_NO_RESOURCES;
}

TDMA_CAT_Slot tdma_get_slot(uint32_t slot_identifier)
{
    return table[slot_identifier];
}

int tdma_current_slot_idx()
{
    return current_slot;
}

TDMA_CAT_Slot tdma_get_current_slot()
{
    TDMA_CAT_Slot blank;
    blank.flags = TDMA_SLOT_FLAGS_UNITIALISED;

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

    if (current_slot == TDMA_CAT_ADVERTISEMENT_SLOT)
    {
        adv_slot_counter++;

        if (current_slot % adv_slot_match == 0)
        {
            adv_slot_match = microbit_random(TDMA_CAT_ADV_SLOT_MATCH_MAX);
            return 1;
        }
    }
    else if (table[current_slot].flags & TDMA_SLOT_FLAGS_OWNER && !(table[current_slot].flags & TDMA_SLOT_FLAGS_ADVERTISE))
        return 1;

    return 0;
}

int tdma_is_synchronised()
{
    log_int("IS",current_slot);
    return !(current_slot == TDMA_CAT_UNITIALISED_SLOT);
}

int tdma_is_advertising_slot()
{
    return current_slot == TDMA_CAT_ADVERTISEMENT_SLOT;
}

int tdma_rx_error()
{
    if (current_slot == TDMA_CAT_ADVERTISEMENT_SLOT)
        return MICROBIT_INVALID_PARAMETER;

    table[current_slot].errors++;
}

int tdma_count_slots()
{
    int count = 0;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].flags & TDMA_SLOT_FLAGS_OWNER)
            count++;

    return count;
}

int tdma_advert_required()
{
    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if ((table[i].flags & TDMA_SLOT_FLAGS_OWNER) && (table[i].flags & TDMA_SLOT_FLAGS_ADVERTISE))
            return 1;

    return 0;
}

int tdma_renegotiation_required()
{
    int required = 0;
    int total_errors = 0;
    int error_avg = 0;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        total_errors += table[i].errors;

    error_avg /= TDMA_CAT_TABLE_SIZE - 1;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
    {
        if (table[i].errors > error_avg)
        {
            table[i].flags |= TDMA_SLOT_FLAGS_ERROR;
            required = 1;
        }

        table[i].errors = 0;
    }


    return required;
}

void tdma_obtain_slot()
{
    int idx = 0;
    int reallocCount = 0;

    TDMA_CAT_Slot newSlot;

    newSlot.device_identifier = table[TDMA_CAT_ADVERTISEMENT_SLOT].device_identifier;
    newSlot.expiration = 0xff;
    newSlot.ttl = 0;
    newSlot.flags = TDMA_SLOT_FLAGS_ADVERTISE | TDMA_SLOT_FLAGS_OWNER;

    log_int("OBTT",0);

    while (reallocCount < 10)
    {
        idx = 1 + microbit_random(TDMA_CAT_TABLE_SIZE);
        if (table[idx].flags & TDMA_SLOT_FLAGS_UNITIALISED)
        {
            newSlot.slot_identifier = idx;
            table[idx] = newSlot;
            log_int("OBT",newSlot.slot_identifier);
            return;
        }

        reallocCount++;
    }
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
        if (table[i].flags & TDMA_SLOT_FLAGS_OWNER && (table[i].flags & TDMA_SLOT_FLAGS_ADVERTISE))
        {
            table[i].flags &= ~(TDMA_SLOT_FLAGS_ADVERTISE);
            slots[advertIndex++] = i;
            log_int("TDMA", i);
        }

    frame->length = advertIndex + TDMA_CAT_HEADER_SIZE - 1;

    return MICROBIT_OK;
}

int tdma_fill_renogotiation_frame(TDMACATSuperFrame* frame)
{
    uint8_t* slots = frame->payload;
    int advertIndex = 0;

    TDMA_CAT_Slot meta = table[TDMA_CAT_ADVERTISEMENT_SLOT];

    frame->ttl = TDMA_CAT_DEFAULT_ADVERT_TTL;
    frame->initial_ttl = TDMA_CAT_DEFAULT_ADVERT_TTL;
    frame->device_id = meta.device_identifier;
    frame->slot_id = TDMA_CAT_ADVERTISEMENT_SLOT;
    frame->frame_id = 0;
    frame->flags = TMDMA_CAT_FRAME_FLAGS_ERROR | TMDMA_CAT_FRAME_FLAGS_ADVERT;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].flags & TDMA_SLOT_FLAGS_ERROR)
        {
            table[i].flags &= ~(TDMA_SLOT_FLAGS_ERROR);
            slots[advertIndex++] = i;
            log_int("TDMA", i);
        }

    frame->length = advertIndex + TDMA_CAT_HEADER_SIZE - 1;

    return MICROBIT_OK;
}

