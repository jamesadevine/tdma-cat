#include "TDMACAT.h"
#include "ErrorNo.h"
#include "TDMACATRadio.h"

static TDMA_CAT_Slot table[TDMA_CAT_TABLE_SIZE];
static volatile int current_slot = TDMA_CAT_UNITIALISED_SLOT;

extern void log_int(const char*, int);

int tdma_init(uint64_t device_identifier)
{
    current_slot = TDMA_CAT_UNITIALISED_SLOT;
    log_int("BF", current_slot);
    memset(&table, 0, sizeof(TDMA_CAT_Slot) * TDMA_CAT_TABLE_SIZE);

    TDMA_CAT_Slot adv;

    adv.device_identifier = device_identifier;
    adv.slot_identifier = TDMA_CAT_ADVERTISEMENT_SLOT;
    adv.expiration = 0xff;
    adv.ttl = 0;
    adv.flags = 0;

    table[TDMA_CAT_ADVERTISEMENT_SLOT] = adv;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        table[i].expiration = 0xff;

    log_int("AF", current_slot);

    return MICROBIT_OK;
}

int tdma_set_slot(TDMA_CAT_Slot slot)
{
    log_int("SS", current_slot);
    uint16_t index = slot.slot_identifier;
    if (index > TDMA_CAT_TABLE_SIZE - 1 || index == TDMA_CAT_ADVERTISEMENT_SLOT)
        return MICROBIT_NO_RESOURCES;

    if (slot.device_identifier != table[index].device_identifier && table[index].expiration)
        return MICROBIT_NO_RESOURCES;

    table[index] = slot;
    log_int("SSO", current_slot);
    return MICROBIT_OK;
}

TDMA_CAT_Slot tdma_get_slot(uint32_t slot_identifier)
{
    log_int("GS", current_slot);
    return table[slot_identifier];
}

TDMA_CAT_Slot tdma_get_current_slot()
{
    log_int("GCS", current_slot);
    TDMA_CAT_Slot blank;
    blank.expiration = 255;

    if (current_slot == TDMA_CAT_UNITIALISED_SLOT)
        return blank;

    return table[current_slot];
}

void tdma_set_current_slot(int slot_id)
{
    current_slot = slot_id;
    log_int("SCS", current_slot);
}

/**
 * Returns 1 if transmission is required in the next time slot.
 **/
int tdma_advance_slot()
{
    log_int("ADVA", current_slot);
    current_slot = (current_slot + 1) % TDMA_CAT_TABLE_SIZE;
    return (table[current_slot].ttl == 0 && !(table[current_slot].flags &TDMA_SLOT_FLAGS_ADVERTISE)) ? 1 : 0;
}

int tdma_is_synchronised()
{
    log_int("IS", current_slot);
    return !(current_slot == TDMA_CAT_UNITIALISED_SLOT);
}

int tdma_is_advertising_slot()
{
    log_int("ADVS", current_slot);
    return current_slot == TDMA_CAT_ADVERTISEMENT_SLOT;
}

int tdma_count_slots()
{
    log_int("COU", current_slot);
    int count = 0;

    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].ttl ==  0)
            count++;

    return count;
}

int tdma_advert_required()
{
    log_int("ADVR", current_slot);
    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].ttl ==  0 && table[i].flags & TDMA_SLOT_FLAGS_ADVERTISE)
            return 1;

    return 0;
}

void tdma_obtain_slot()
{
    log_int("obt", current_slot);
    for (int i = 1; i < TDMA_CAT_TABLE_SIZE; i++)
        if (table[i].expiration ==  0xff)
        {
            table[i].device_identifier = table[TDMA_CAT_ADVERTISEMENT_SLOT].device_identifier;
            table[i].slot_identifier = i;
            table[i].expiration = 0xff;
            table[i].ttl = 0;
            table[i].flags = TDMA_SLOT_FLAGS_ADVERTISE;
            return;
        }
}

int tdma_fill_advertising_frame(TDMACATSuperFrame* frame)
{
    log_int("ADVF", current_slot);
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
        if (table[i].ttl ==  0 && (table[i].flags & TDMA_SLOT_FLAGS_ADVERTISE))
        {
            table[i].flags &= ~(TDMA_SLOT_FLAGS_ADVERTISE);
            slots[advertIndex++] = i;
        }

    frame->length = advertIndex + TDMA_CAT_HEADER_SIZE - 1;

    return MICROBIT_OK;
}

