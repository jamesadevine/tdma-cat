#include "TDMACAT.h"
#include "ErrorNo.h"
#include "TDMACATRadio.h"

static TDMA_CAT_Slot table[TDMA_CAT_TABLE_SIZE];
static TDMA_CAT_Slot init;

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
    adv.expiration = TDMA_CAT_NEVER_EXPIRE;
    adv.distance = 0;
    adv.flags = TDMA_SLOT_FLAGS_OWNER;

    table[TDMA_CAT_ADVERTISEMENT_SLOT] = adv;

    init.expiration = 0;
    init.distance = 0xf;
    init.flags = TDMA_SLOT_FLAGS_UNITIALISED;
    init.slot_identifier = 0;
    init.device_identifier = 0;

    adv_slot_match = 1 + microbit_random(TDMA_CAT_ADV_SLOT_MATCH_MAX);
    adv_slot_counter = 0;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        table[i] = init;

    return MICROBIT_OK;
}

int tdma_clear_slot(uint32_t slot_identifier)
{
    if (slot_identifier > TDMA_CAT_TABLE_SIZE - 1 || slot_identifier == TDMA_CAT_ADVERTISEMENT_SLOT)
        return MICROBIT_INVALID_PARAMETER;

    table[slot_identifier] = init;

    return MICROBIT_OK;
}

int tdma_set_slot(TDMA_CAT_Slot slot, bool maintainDistance)
{
    uint16_t index = slot.slot_identifier;

    if (index > TDMA_CAT_TABLE_SIZE - 1 || index == TDMA_CAT_ADVERTISEMENT_SLOT)
        return MICROBIT_INVALID_PARAMETER;

    if ((table[index].flags & TDMA_SLOT_FLAGS_UNITIALISED) || (table[index].flags & TDMA_SLOT_FLAGS_ADVERTISE) || slot.device_identifier == table[index].device_identifier)
    {

        // the radio code is generalised such that we may receive our own advertisement packets
        // if we do, then we do not want to lose important metadata.
        if (table[index].expiration == TDMA_CAT_NEVER_EXPIRE)
            slot.expiration = TDMA_CAT_NEVER_EXPIRE;

        if (maintainDistance)
            slot.distance = min(slot.distance, table[index].distance);

        slot.flags |= (table[index].flags & TDMA_SLOT_FLAGS_OWNER);
        table[index] = slot;
        // SERIAL_DEBUG->printf("SS: %d %d %d\r\n", index, (int)slot.device_identifier, slot.flags);
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
    if (current_slot == TDMA_CAT_UNITIALISED_SLOT)
        return init;

    return table[current_slot];
}

void tdma_set_current_slot(int slot_id)
{
    current_slot = slot_id;
}

int tdma_advance_slot()
{
    current_slot = (current_slot + 1) % TDMA_CAT_TABLE_SIZE;

    if (current_slot == TDMA_CAT_ADVERTISEMENT_SLOT)
    {
        adv_slot_counter++;

        if (adv_slot_counter % adv_slot_match == 0)
        {
            adv_slot_match = 1 + microbit_random(TDMA_CAT_ADV_SLOT_MATCH_MAX);
            return 1;
        }
    }
    else if (table[current_slot].flags & TDMA_SLOT_FLAGS_OWNER && !(table[current_slot].flags & TDMA_SLOT_FLAGS_ADVERTISE))
        return 1;

    return 0;
}

int tdma_is_synchronised()
{
    return !(current_slot == TDMA_CAT_UNITIALISED_SLOT);
}

int tdma_is_advertising_slot()
{
    return current_slot == TDMA_CAT_ADVERTISEMENT_SLOT;
}

int tdma_is_owner()
{
    return (table[current_slot].flags & TDMA_SLOT_FLAGS_OWNER ? 1 : 0);
}

int tdma_slot_is_occupied()
{
    return (table[current_slot].flags & TDMA_SLOT_FLAGS_UNITIALISED) ? 0 : 1;
}

void tdma_rx_error()
{
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
    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].flags & TDMA_SLOT_FLAGS_ERROR)
            return 1;
    return 0;
}

void tdma_obtain_slot()
{
    int idx = 0;
    int reallocCount = 0;

    TDMA_CAT_Slot newSlot;

    newSlot.device_identifier = table[TDMA_CAT_ADVERTISEMENT_SLOT].device_identifier;
    newSlot.expiration = TDMA_CAT_NEVER_EXPIRE;
    newSlot.distance = 0;
    newSlot.flags = TDMA_SLOT_FLAGS_ADVERTISE | TDMA_SLOT_FLAGS_OWNER;

    while (reallocCount < 10)
    {
        idx = 1 + microbit_random(TDMA_CAT_TABLE_SIZE);
        if (table[idx].flags & TDMA_SLOT_FLAGS_UNITIALISED)
        {
            newSlot.slot_identifier = idx;
            table[idx] = newSlot;
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
        }

    frame->length = advertIndex + TDMA_CAT_HEADER_SIZE - 1;

    return MICROBIT_OK;
}

static void tdma_flag_errors()
{
    int total_errors = 0;
    int active_slots = 0;
    int error_avg = 0;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
    {
        if (table[i].flags & TDMA_SLOT_FLAGS_UNITIALISED || table[i].flags & TDMA_SLOT_FLAGS_OWNER)
            continue;

        active_slots++;
        total_errors += table[i].errors;
    }

    error_avg = total_errors / active_slots;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
    {
        if (table[i].errors > error_avg)
            table[i].flags |= TDMA_SLOT_FLAGS_ERROR;

        table[i].errors = 0;
    }
}

static void tdma_expire()
{
    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
    {
        if (table[i].flags & TDMA_SLOT_FLAGS_UNITIALISED || table[i].expiration == TDMA_CAT_NEVER_EXPIRE)
            continue;

        table[i].expiration--;

        if (table[i].expiration == 0)
            table[i] = init;
    }
}

void tdma_window_tick()
{
    tdma_expire();
    tdma_flag_errors();
}

int tdma_slot_distance()
{
    return table[current_slot].distance;
}

int tdma_get_distance()
{
    int biggest_distance = 0;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
    {
        if (table[i].flags & TDMA_SLOT_FLAGS_UNITIALISED || table[i].expiration == TDMA_CAT_NEVER_EXPIRE)
            continue;

        if (table[i].distance > biggest_distance)
            biggest_distance = table[i].distance;
    }

    return biggest_distance;
}

