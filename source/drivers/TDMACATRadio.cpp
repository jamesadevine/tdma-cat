/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "MicroBitConfig.h"

#include "TDMACATRadio.h"

#if (TDMA_CAT_DIRECT_DEBUG == 0 && MICROBIT_RADIO_VERSION == MICROBIT_RADIO_PERIDO)

#include "TDMACAT.h"
#include "MicroBitComponent.h"
#include "EventModel.h"
#include "MicroBitDevice.h"
#include "ErrorNo.h"
#include "MicroBitFiber.h"
#include "MicroBitBLEManager.h"
#include "MicroBitHeapAllocator.h"
#include "nrf.h"

extern void log_int(const char*, int);
TDMACATRadio* TDMACATRadio::instance = NULL;

#define TIME_TO_TRANSMIT_BYTE_1MB           8
#define TX_PACKETS_SIZE                     (2 * TDMA_CAT_MAXIMUM_TX_BUFFERS)
#define FRAME_TRACKER_BUFFER_SIZE           20
#define FRAME_TRACKER_UNINITIALISED_VALUE     255

#define RADIO_DISABLE_TIME                  3
#define RADIO_TURNAROUND_TIME_US            145
#define RADIO_SHORTS_TURNAROUND_TIME_US     132
#define RADIO_DISABLE_TOLERANCE             300

// 4 bytes address, 1 prefix, 1 pre amble, 2 crc
#define RADIO_ON_AIR_BYTES                  8
#define RADIO_MAX_PACKET_SIZE               (TDMA_CAT_MAX_PACKET_SIZE + TDMA_CAT_HEADER_SIZE + RADIO_ON_AIR_BYTES)
#define RADIO_MIN_PACKET_SIZE               (TDMA_CAT_HEADER_SIZE + RADIO_ON_AIR_BYTES)
#define RADIO_MAX_PACKET_TIME_US            (RADIO_MAX_PACKET_SIZE * TIME_TO_TRANSMIT_BYTE_1MB)
#define RADIO_MIN_PACKET_TIME_US            (RADIO_MIN_PACKET_SIZE * TIME_TO_TRANSMIT_BYTE_1MB)

#define TDMA_DISCOVER_BACK_OFF              2000000
#define TDMA_PREPARATION_OFFSET_US          2000
#define TDMA_GRACE_PERIOD_US                500
#define TDMA_SYNC_TOLERANCE                 (TDMA_GRACE_PERIOD_US / 2)
#define TDMA_CAT_LISTEN_RATE                10

volatile uint32_t packets_received = 0;
volatile uint32_t packets_error = 0;
volatile uint32_t packets_transmitted = 0;
volatile uint32_t packets_forwarded = 0;

volatile int sync_drift = 0;
volatile int window_count = 0;

volatile uint8_t frame_tracker[FRAME_TRACKER_BUFFER_SIZE] = { 0 };


// the device will always wake and disable and fixed intervals
#define STATIC_WAKE_DISABLE         1
// the device will wake and disable based on the computed distance
// from the device that owns the slot.
#define DYNAMIC_WAKE_DISABLE        2

#define RADIO_DISABLE_NEVER             1
#define RADIO_DISABLE_WHERE_POSSIBLE    2

#define RADIO_DYNAMIC_TTL               100
#define RADIO_FIXED_TTL                 6

/**
  * Driver configuration flags
  **/
// assert the hardware is in the correct state
#define TDMA_CAT_ASSERT 1
// send error frames to slots that see a high number of errors
#define TDMA_CONFIGURATION_SUPPORT_RENEGOTIATION 0
// use fixed wake up and sleep times, or dynamic ones computed
// from the table
#define RADIO_WAKE_DISABLE_CONFIGURATION    STATIC_WAKE_DISABLE
#define RADIO_DISABLE_CONFIGURATION         RADIO_DISABLE_NEVER
#define RADIO_TTL_CALCULATION               RADIO_DYNAMIC_TTL

// set a custom serial number for testing.
#define TDMA_CUSTOM_SERIAL_NUMBER       5

#if TDMA_CAT_TEST_MODE == 1
    TestRole testRole;
#endif

// 20 [txen][CC0], 21[rxen][CC0], 22[dis][CC1], 27[END][CC2]
#define PPI_CHAN_TX_EN      20
#define PPI_CHAN_RX_EN      21
#define PPI_CHAN_DIS        22
#define PPI_CHAN_ADDRESS    26
#define PPI_CHAN_END        27

#define TIMER_CC_TX_RX              0
#define TIMER_CC_DISABLE_RADIO      1
#define TIMER_CC_TIMESTAMP          2
#define TIMER_CC_TDMA               3

#define RADIO_STATE_RECEIVE     (1)
#define RADIO_STATE_TRANSMIT    (2)
#define RADIO_STATE_FORWARD     (3)
#define RADIO_STATE_DISCOVER    (4)

#define TDMA_STATE_INITIALISATION       (1)
#define TDMA_STATE_DISCOVER             (2)
#define TDMA_STATE_NORMAL               (3)
#define TDMA_STATE_LISTEN               (4)

volatile uint8_t radioState = RADIO_STATE_RECEIVE;
volatile uint8_t tdmaState = TDMA_STATE_INITIALISATION;
volatile int16_t discover_schedule_start = -1;

uint32_t serial_number = 0;

extern void set_transmission_reception_gpio(int);
extern void process_packet(TDMACATSuperFrame* p, bool, int);

volatile int hw_state = 0;
#define HW_ASSERT(expected_state, panic_num) {\
                                        hw_state = NRF_RADIO->STATE;\
                                        if (hw_state != expected_state) \
                                            microbit_panic(__LINE__);\
                                        }\

inline void TIMER_SET_CC(uint8_t channel, int value)
{
    NRF_TIMER0->CC[channel] = value;
}

inline void TIMER_ENABLE_CLEAR_ON_COMPARE(uint8_t channel)
{
    NRF_TIMER0->SHORTS |= 1 << channel;
}

inline void TIMER_DISABLE_CLEAR_ON_COMPARE(uint8_t channel)
{
    NRF_TIMER0->SHORTS &= ~(1 << channel);
}

inline void TIMER_OFFSET_CC(uint8_t channel, int value)
{
    NRF_TIMER0->CC[channel] += value;
}

inline uint32_t TIMER_GET_CC(uint8_t channel)
{
    return NRF_TIMER0->CC[channel];
}

inline void TIMER_ENABLE_CC_IRQ(uint8_t channel)
{
    NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + channel));
}

inline void TIMER_ENABLE_IRQ()
{
    NVIC_EnableIRQ(TIMER0_IRQn);
}

inline void TIMER_DISABLE_IRQ()
{
    NVIC_DisableIRQ(TIMER0_IRQn);
}

inline void TIMER_SET_PRIORITY(int priority)
{
    NVIC_SetPriority(TIMER0_IRQn, priority);
}

inline void TIMER_START()
{
    NRF_TIMER0->TASKS_START = 1;
}

inline void TIMER_STOP()
{
    NRF_TIMER0->TASKS_STOP = 1;
}

inline void TIMER_CLEAR()
{
    NRF_TIMER0->TASKS_CLEAR = 1;
}

inline void RADIO_ENABLE_READY_START_SHORT()
{
    NRF_RADIO->SHORTS |= RADIO_SHORTS_READY_START_Msk;
}

inline void RADIO_DISABLE_READY_START_SHORT()
{
    NRF_RADIO->SHORTS &= ~(RADIO_SHORTS_READY_START_Msk);
}

inline void RADIO_DISABLE()
{
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;
}

inline void PPI_ENABLE_CHAN(uint8_t channel)
{
    NRF_PPI->CHENSET = 1 << channel;
}

inline void PPI_DISABLE_CHAN(uint8_t channel)
{
    NRF_PPI->CHENCLR = 1 << channel;
}

inline int FRAME_SEEN(uint16_t frame_id)
{
    for (int i = 0; i < FRAME_TRACKER_BUFFER_SIZE; i++)
    {
        if (frame_tracker[i] == FRAME_TRACKER_UNINITIALISED_VALUE)
            return 0;

        if (frame_id == frame_tracker[i])
            return 1;
    }

    return 0;
}

inline void TRACK_FRAME(uint16_t frame_id)
{
    for (int i = 0; i < FRAME_TRACKER_BUFFER_SIZE; i++)
    {
        if (frame_tracker[i] == FRAME_TRACKER_UNINITIALISED_VALUE)
        {
            frame_tracker[i] = frame_id;
            return;
        }
    }
}

inline void SYNC_TO_FRAME(uint32_t t_end, int ttl, int initial_ttl, int tx_time)
{
    // as packets have a predictable transmission time we can compute
    // when the transmitting device triggered a transmission.
    uint8_t hops = initial_ttl - ttl;
    uint32_t time_to_arrive = tx_time + (hops * (tx_time + RADIO_DISABLE_TIME + RADIO_TURNAROUND_TIME_US));
    uint32_t t_start = t_end - time_to_arrive;

    // this is the first packet received after power on; snap to the schedule
    if (tdmaState == TDMA_STATE_INITIALISATION)
    {
        TIMER_SET_CC(TIMER_CC_TDMA, t_start + (TDMA_CAT_SLOT_SIZE_US - TDMA_PREPARATION_OFFSET_US));
        tdmaState = TDMA_STATE_DISCOVER;
    }
    else
    {
        // we can then calculate the difference between schedules for
        // gentle drift compensation.
        int error = t_start - TDMA_PREPARATION_OFFSET_US;
        sync_drift = (sync_drift + error) / 2;

        // we're outside of the tolerance bounds, a gentle compensation can no longer help
        // proper resync on the next frame
        if (error > TDMA_SYNC_TOLERANCE || error < -(TDMA_SYNC_TOLERANCE))
            tdmaState = TDMA_STATE_INITIALISATION;
    }
}

inline void SET_RADIO_DISABLE(uint32_t t_end, int ttl, uint32_t tx_time)
{
    // again, because packet transmissions are predictable we can schedule
    // the radio to disable itself in the future.
    // (although the radio automatically powers off upon an END event
    // we may miss retransmissions).
    TIMER_SET_CC(TIMER_CC_DISABLE_RADIO, t_end + (ttl * (tx_time + RADIO_DISABLE_TIME + RADIO_TURNAROUND_TIME_US)) + RADIO_DISABLE_TOLERANCE);
    PPI_ENABLE_CHAN(PPI_CHAN_DIS);
}

void timer_callback(uint8_t)
{
    // t = 0 is this timer interrupt!
    // the first packet should be at t = 2 000 us.
    TIMER_CLEAR();
    TIMER_SET_CC(TIMER_CC_TDMA, TDMA_CAT_SLOT_SIZE_US);

    PPI_DISABLE_CHAN(PPI_CHAN_RX_EN);
    PPI_DISABLE_CHAN(PPI_CHAN_TX_EN);
    PPI_DISABLE_CHAN(PPI_CHAN_DIS);

    RADIO_DISABLE();

    // This indicates if we are able to transmit in a slot. Assume we can, unless
    // the tdma schedule tells us otherwise (it usually will).
    int owner = 1;

    // this variable will be set to one if we do not want to sleep through empty slots.
    // This is useful for discovering / re-discovering the schedule.
    bool needToReceive = false;

    // reset the frame tracker each window.
    memset((void*)frame_tracker, FRAME_TRACKER_UNINITIALISED_VALUE, FRAME_TRACKER_BUFFER_SIZE);
    memset(&TDMACATRadio::instance->staticFrame, 0, sizeof(TDMACATSuperFrame));

    // if we're about to ask for a slot, don't obtain more slots...
    if (!tdma_advert_required() && (tdma_count_slots() == 0 || TDMACATRadio::instance->queueSize(&TDMACATRadio::instance->txTail, &TDMACATRadio::instance->txHead) > TDMA_CAT_NEW_SLOT_THRESHOLD))
        tdma_obtain_slot();

    // a schedule exists
    if (tdma_is_synchronised())
    {
        owner = tdma_advance_slot();
    }
    else
    {
        // a schedule doesn't exist, create one
        tdmaState = TDMA_STATE_NORMAL;
        tdma_set_current_slot(TDMA_CAT_ADVERTISEMENT_SLOT);
    }

    // tracks a complete window from the starting slot + table size
    // helps to ensure that the schedule is discovered / re-discovered.
    if (tdmaState == TDMA_STATE_DISCOVER || tdmaState == TDMA_STATE_LISTEN)
    {
        needToReceive = true;
        int cs = tdma_current_slot_index();
        if (discover_schedule_start == -1)
            discover_schedule_start = cs;
        else if (discover_schedule_start == cs)
        {
            discover_schedule_start = -1;
            tdmaState = TDMA_STATE_NORMAL;
        }
    }

    // the time we power up the transmitter is: 2000 - the time it takes to power
    // up the transmitter.
    int wakeTime = TDMA_PREPARATION_OFFSET_US - RADIO_SHORTS_TURNAROUND_TIME_US;
    int disableTime = wakeTime;

    if (!(tdmaState == TDMA_STATE_DISCOVER) && owner)
    {
        bool needToTransmit = false;
#if RADIO_TTL_CALCULATION == RADIO_DYNAMIC_TTL
        int ttl = 1 + tdma_get_distance();
#else
        int ttl = RADIO_FIXED_TTL;
#endif

        if (tdma_is_advertising_slot())
        {
            // it is possible to miss new advertisements, so every
            // modulo (TDMA_CAT_LISTEN_RATE) we listen for a complete window.
            window_count = ((window_count + 1) % TDMA_CAT_LISTEN_RATE);

            if (window_count == 0 && tdmaState == TDMA_STATE_NORMAL)
                tdmaState = TDMA_STATE_LISTEN;

            // crystals generally have a drift. This has implications on the accuracy of timers
            // which may drift by up to 50 us every second.
            // we perform drift compensation based upon an average over time
            int compensation = -(sync_drift / 10);
            sync_drift = 0;
            TIMER_SET_CC(TIMER_CC_TDMA, TDMA_CAT_SLOT_SIZE_US - (compensation * 5));

            // update table expiration every window.
            tdma_window_tick();

            // do we need to send an advert?
            if (tdma_advert_required() && tdma_able_to_advertise())
            {
                // random backoff
                needToTransmit = true;
                wakeTime += microbit_random(TDMA_CAT_SLOT_SIZE_US - 5000);
                tdma_fill_advertising_frame(&TDMACATRadio::instance->staticFrame);
            }
#if TDMA_CONFIGURATION_SUPPORT_RENEGOTIATION == 1
            else if (tdma_renegotiation_required())
            {
                needToTransmit = true;
                wakeTime += microbit_random(TDMA_CAT_SLOT_SIZE_US - 5000);
                tdma_fill_renogotiation_frame(&TDMACATRadio::instance->staticFrame);
            }
#endif
        }
        // anything to send?
        else if (TDMACATRadio::instance->peakTxQueue())
        {
            needToTransmit = true;

            TDMACATSuperFrame* p = TDMACATRadio::instance->popTxQueue();
            memcpy(&TDMACATRadio::instance->staticFrame, p, sizeof(TDMACATSuperFrame));
            // return the buffer to the pool of buffers
            TDMACATRadio::instance->addBufferToPool(p);
            TDMACATRadio::instance->staticFrame.ttl = ttl;
            TDMACATRadio::instance->staticFrame.initial_ttl = ttl;
        }
        else
        {
            // We have nothing to send in our slot, send a keep alive message
            // to ensure that our slot is not lost.
            needToTransmit = true;
            TDMACATRadio::instance->staticFrame.length = TDMA_CAT_HEADER_SIZE - 1;
            TDMACATRadio::instance->staticFrame.device_id = serial_number;
            TDMACATRadio::instance->staticFrame.ttl = ttl;
            TDMACATRadio::instance->staticFrame.initial_ttl = ttl;
        }

        if (needToTransmit)
        {
            radioState = RADIO_STATE_TRANSMIT;

            // set reasonable frame defaults
            TDMACATRadio::instance->staticFrame.frame_id = 0;
            TDMACATRadio::instance->staticFrame.flags |= TMDMA_CAT_FRAME_FLAGS_DONE;
            TDMACATRadio::instance->staticFrame.slot_id = tdma_current_slot_index();

            NRF_RADIO->PACKETPTR = (uint32_t)&TDMACATRadio::instance->staticFrame;

            // set to power up the transmitter at t = 2 000 - time to power up transmitter us.
            TIMER_SET_CC(TIMER_CC_TX_RX, wakeTime);
            RADIO_ENABLE_READY_START_SHORT();

            PPI_ENABLE_CHAN(PPI_CHAN_TX_EN);
            PPI_ENABLE_CHAN(PPI_CHAN_END);
            return;
        }
    }

    if (tdma_is_advertising_slot())
    {
        needToReceive = true;
        wakeTime -= TDMA_GRACE_PERIOD_US;
        disableTime = TDMA_CAT_SLOT_SIZE_US;
    }
    else if (tdma_slot_is_occupied())
    {
        needToReceive = true;

#if RADIO_WAKE_DISABLE_CONFIGURATION == DYNAMIC_WAKE_DISABLE
        // if the slot is occupied, then compute the wake sleep based upon the distance to the node
        // (if config is set)
        // also wake up a little bit early and sleep a little late
        int distance = tdma_slot_distance();
        wakeTime += (distance * (RADIO_MIN_PACKET_SIZE + RADIO_TURNAROUND_TIME_US)) - TDMA_GRACE_PERIOD_US;
        disableTime += ((distance + 1) * (RADIO_MAX_PACKET_TIME_US + RADIO_TURNAROUND_TIME_US)) + TDMA_GRACE_PERIOD_US;
#else
        // wake up early
        wakeTime -= TDMA_GRACE_PERIOD_US;
        // expect to receive a packet within the first 2 ms of tx window.
        // set to disable at t = 4 000 us unless computation is dependent on distance
        // (this is set by configuring dynamic wake mode)
        disableTime += TDMA_PREPARATION_OFFSET_US;
#endif
    }
    else
    {
        // slot is not occupied but we may be listening / discovering and further
        // than 1 hop from a node in the network.
        disableTime += TDMA_CAT_DEFAULT_ADVERT_TTL * ((RADIO_MAX_PACKET_TIME_US + RADIO_TURNAROUND_TIME_US)) + TDMA_GRACE_PERIOD_US;
    }

    // if we don't need to transmit, we drop through into receive mode.
    // We only receive if the slot is occupied (according to the schedule)
    // or if we're discovering / listening.
    if (needToReceive)
    {
        radioState = RADIO_STATE_RECEIVE;

        NRF_RADIO->PACKETPTR = (uint32_t)&TDMACATRadio::instance->staticFrame;
#if RADIO_DISABLE_CONFIGURATION == RADIO_DISABLE_WHERE_POSSIBLE
        TIMER_SET_CC(TIMER_CC_DISABLE_RADIO, disableTime);
        PPI_ENABLE_CHAN(PPI_CHAN_DIS);
#endif
        TIMER_SET_CC(TIMER_CC_TX_RX, wakeTime);

        RADIO_ENABLE_READY_START_SHORT();

        PPI_ENABLE_CHAN(PPI_CHAN_RX_EN);
        PPI_ENABLE_CHAN(PPI_CHAN_END);

    }
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

extern "C" void RADIO_IRQHandler(void)
{
    NRF_RADIO->EVENTS_END = 0;
    TDMACATSuperFrame *p = &TDMACATRadio::instance->staticFrame;

    if (radioState == RADIO_STATE_FORWARD)
    {
        radioState = RADIO_STATE_RECEIVE;
        NRF_RADIO->PACKETPTR = (uint32_t)p;
        while(NRF_RADIO->EVENTS_DISABLED == 0);
#if TDMA_CAT_ASSERT == 1
        HW_ASSERT(0,0);
#endif
        RADIO_ENABLE_READY_START_SHORT();
        NRF_RADIO->TASKS_RXEN = 1;
        packets_forwarded++;
        return;
    }

    if (radioState == RADIO_STATE_RECEIVE)
    {
        if(NRF_RADIO->CRCSTATUS == 1)
        {
            int correction = 0;
            if(p->ttl != 0)
            {
                p->ttl--;
                radioState = RADIO_STATE_FORWARD;
                NRF_RADIO->PACKETPTR = (uint32_t)p;

#if TDMA_CAT_ASSERT == 1
                HW_ASSERT(0,0);
#endif
                NRF_RADIO->EVENTS_DISABLED = 0;
                RADIO_DISABLE_READY_START_SHORT();

                // this needs to happen at exactly the same time on all re-transmitters
                // using shorts and timers result in different timings, soooo SPIIIIN!
                NRF_RADIO->TASKS_TXEN = 1;
                volatile int i = 250;
                while(i-- > 0);
                NRF_RADIO->TASKS_START = 1;
                correction = 1;
            }

            tdma_frame_success();
            packets_received++;

            // we have a little bit of a back porch now whilst the device is retransmitting.
            bool advert = p->flags & TMDMA_CAT_FRAME_FLAGS_ADVERT;

            process_packet(&TDMACATRadio::instance->staticFrame, true, correction);


            TDMACATSlot slot;
            slot.device_identifier = p->device_id;
            // if we decrement the ttl, we need to apply a correction.
            // Otherwise the distance calculation will be incorrect
            slot.distance = p->initial_ttl - (p->ttl + correction);
            slot.expiration = TDMA_CAT_DEFAULT_EXPIRATION;
            slot.flags = 0;

            if (advert)
            {
                // if we're not in an advertising window ignore.
                if (tdma_is_advertising_slot())
                {
                    bool error = p->flags & TMDMA_CAT_FRAME_FLAGS_ERROR;

                    for (int i = 0; i < p->length - (TDMA_CAT_HEADER_SIZE - 1); i++)
                    {
                        slot.slot_identifier = p->payload[i];

                        // error frame? clear the slots in the payload
                        if (error)
                            tdma_clear_slot(p->payload[i]);
                        else
                            // otherwise update our table
                            tdma_set_slot(slot, true);
                    }
                }
            }
            // frame_ids are unique within a slot, check to see if seen before.
            else if (FRAME_SEEN(p->frame_id) == 0)
            {
                // track frame_id to prevent duplication.
                TRACK_FRAME(p->frame_id);

                // the timer has usefully captured the time of the END event (via PPI)
                // used in future calculations
                uint32_t t_end = TIMER_GET_CC(TIMER_CC_TIMESTAMP);

                // transmission time of the packet
                // i've computed the packet length to be packet_length + 8:
                // 1 pre-amble, 2 crc, 4 device address, 1 prefix
                uint32_t tx_time = (p->length + RADIO_ON_AIR_BYTES) * TIME_TO_TRANSMIT_BYTE_1MB;

                // this is the last frame in the slot
                // compute disable time.

                // (note for now there is only one frame per slot)
// #if RADIO_DISABLE_CONFIGURATION == RADIO_DISABLE_WHERE_POSSIBLE
                if (p->flags & TMDMA_CAT_FRAME_FLAGS_DONE)
                    SET_RADIO_DISABLE(t_end, p->ttl + correction, tx_time);
// #endif

                // only sync to the first frame in a slot.
                // (also ensure the slot isn't our own, syncing yourself is
                // usually sad and a little bit depressing)
                if (!tdma_is_owner() && p->frame_id == 0)
                {
                    SYNC_TO_FRAME(t_end, p->ttl + correction, p->initial_ttl, tx_time);
                    tdma_set_current_slot(p->slot_id);
                    slot.slot_identifier = p->slot_id;
                    tdma_set_slot(slot, false);
                }

// #if TDMA_CAT_TEST_MODE == 1
//                 process_packet(&TDMACATRadio::instance->staticFrame, true, correction);
// #else
//                 // queue the received frame from our pool of pre-allocated buffers.
//                 TDMACATRadio::instance->queueRxFrame(&TDMACATRadio::instance->staticFrame);
// #endif
            }
            return;
        }
        else
        {
            // increments the error counter for current_slot
            tdma_frame_error();
            packets_error++;
        }

        // eek! an error
        // note, we should probably stop the radio from automatically disabling itself
        // (the tx callback will set a default power off time)
        packets_received++;
        NRF_RADIO->PACKETPTR = (uint32_t)p;
#if TDMA_CAT_ASSERT == 1
        HW_ASSERT(0,0);
#endif
        NRF_RADIO->EVENTS_DISABLED = 0;
        RADIO_ENABLE_READY_START_SHORT();
        NRF_RADIO->TASKS_RXEN = 1;
        return;
    }

    // end event after transmitting
    // enter receive mode to forward our own packet!
    // flooooood!
    if (radioState == RADIO_STATE_TRANSMIT)
    {
        radioState = RADIO_STATE_RECEIVE;
        NRF_RADIO->PACKETPTR = (uint32_t)&TDMACATRadio::instance->staticFrame;
        while(NRF_RADIO->EVENTS_DISABLED == 0);
#if TDMA_CAT_ASSERT == 1
        HW_ASSERT(0,0);
#endif
        NRF_RADIO->EVENTS_DISABLED = 0;
        RADIO_ENABLE_READY_START_SHORT();
        NRF_RADIO->TASKS_RXEN = 1;
        packets_transmitted++;
        return;
    }
}

#pragma GCC pop_options

/**
  * Constructor.
  *
  * Initialise the TDMACATRadio.
  *
  * @note This class is demand activated, as a result most resources are only
  *       committed if send/recv or event registrations calls are made.
  */
TDMACATRadio::TDMACATRadio(LowLevelTimer& timer, uint16_t id) : timer(timer)
{
    this->id = id;
    this->status = 0;

    memset(&this->staticFrame, 0, sizeof(TDMACATSuperFrame));
    memset(this->txQueue, 0, sizeof(TDMACATSuperFrame*) * TDMA_CAT_QUEUE_SIZE);
    memset(this->rxQueue, 0, sizeof(TDMACATSuperFrame*) * TDMA_CAT_QUEUE_SIZE);
    memset(this->bufferPool, 0, sizeof(TDMACATSuperFrame*) * TDMA_CAT_BUFFER_POOL_SIZE);
    memset((void*)frame_tracker, FRAME_TRACKER_UNINITIALISED_VALUE, FRAME_TRACKER_BUFFER_SIZE);

    this->rxHead = 0;
    this->rxTail = 0;
    this->txHead = 0;
    this->txTail = 0;

    timer.disable();

    timer.setIRQ(timer_callback);

    // timer mode
    timer.setMode(TimerModeTimer);

    // 32-bit
    timer.setBitMode(BitMode32);

    // 16 Mhz / 2^4 = 1 Mhz
    timer.setPrescaler(4);

    NRF_TIMER0->TASKS_CLEAR = 1;

    NRF_TIMER0->CC[0] = 0;
    NRF_TIMER0->CC[1] = 0;
    NRF_TIMER0->CC[2] = 0;
    NRF_TIMER0->CC[3] = 0;

    microbit_seed_random();

#ifdef TDMA_CUSTOM_SERIAL_NUMBER
    serial_number = TDMA_CUSTOM_SERIAL_NUMBER;
#else
    serial_number = microbit_serial_number();
#endif

    SERIAL_DEBUG->printf("DEVICE ID: %d\r\n",serial_number);
    tdma_init(serial_number);

    instance = this;
}

/**
  * Change the output power level of the transmitter to the given value.
  *
  * @param power a value in the range 0..7, where 0 is the lowest power and 7 is the highest.
  *
  * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the value is out of range.
  */
int TDMACATRadio::setTransmitPower(int power)
{
    if (power < 0 || power >= MICROBIT_BLE_POWER_LEVELS)
        return MICROBIT_INVALID_PARAMETER;

    NRF_RADIO->TXPOWER = (uint32_t)MICROBIT_BLE_POWER_LEVEL[power];

    return MICROBIT_OK;
}

/**
  * Change the transmission and reception band of the radio to the given channel
  *
  * @param band a frequency band in the range 0 - 100. Each step is 1MHz wide, based at 2400MHz.
  *
  * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the value is out of range,
  *         or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int TDMACATRadio::setFrequencyBand(int band)
{
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    if (band < 0 || band > 100)
        return MICROBIT_INVALID_PARAMETER;

    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);

    NRF_RADIO->FREQUENCY = (uint32_t)band;
    NRF_RADIO->DATAWHITEIV = band;

    radioState = RADIO_STATE_RECEIVE;

    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_RXEN = 1;

    return MICROBIT_OK;
}

void TDMACATRadio::addBufferToPool(TDMACATSuperFrame* q)
{
    for (int i = 0; i < TDMA_CAT_BUFFER_POOL_SIZE; i++)
        if (this->bufferPool[i] == NULL)
        {
            this->bufferPool[i] = q;
            return;
        }
}

TDMACATSuperFrame* TDMACATRadio::getBufferFromPool()
{
    for (int i = 0; i < TDMA_CAT_BUFFER_POOL_SIZE; i++)
        if (this->bufferPool[i])
        {
            TDMACATSuperFrame* p = this->bufferPool[i];
            this->bufferPool[i] = NULL;
            return p;
        }

    return NULL;
}

int TDMACATRadio::addBufferToQueue(TDMACATSuperFrame** q, TDMACATSuperFrame* p, uint8_t* tail, uint8_t* head)
{
    int next_head = (*head + 1) % TDMA_CAT_QUEUE_SIZE;

    if (next_head == *tail)
        return MICROBIT_NO_RESOURCES;

    q[*head] = p;
    *head = next_head;

    return MICROBIT_OK;
}

TDMACATSuperFrame* TDMACATRadio::getBufferFromQueue(TDMACATSuperFrame** q, uint8_t* tail, uint8_t* head)
{
    if (*head == *tail)
        return NULL;

    TDMACATSuperFrame* p = q[*tail];
    q[*tail] = NULL;
    *tail = (*tail + 1) % TDMA_CAT_QUEUE_SIZE;

    return p;
}

int TDMACATRadio::queueRxFrame(TDMACATSuperFrame* s)
{
    TDMACATSuperFrame *p = TDMACATRadio::instance->getBufferFromPool();

    if (p)
    {
        memcpy(p, s, sizeof(TDMACATSuperFrame));
        addBufferToQueue(this->rxQueue, p, &this->rxTail, &this->rxHead);
    }

    return MICROBIT_OK;
}

int TDMACATRadio::queueTxFrame(TDMACATSuperFrame* s)
{
    TDMACATSuperFrame* p = getBufferFromPool();

    if (p == NULL || queueSize(&this->txTail, &this->txHead) == TDMA_CAT_EFFECTIVE_QUEUE_SIZE)
        return MICROBIT_NO_RESOURCES;

    memcpy(p, s, sizeof(TDMACATSuperFrame));

    int res = addBufferToQueue(this->txQueue, p, &this->txTail, &this->txHead);

    if (res != MICROBIT_OK)
        addBufferToPool(p);

    return res;
}

int TDMACATRadio::queueSize(uint8_t* tail, uint8_t* head)
{
    if (*head == *tail)
        return 0;

    return (*head > *tail) ? (*head - *tail) : ((*head + TDMA_CAT_QUEUE_SIZE) - *tail);
}

TDMACATSuperFrame* TDMACATRadio::popTxQueue()
{
    return getBufferFromQueue(this->txQueue, &this->txTail, &this->txHead);
}

/**
  * Initialises the radio for use as a multipoint sender/receiver
  *
  * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int TDMACATRadio::enable()
{
    // If the device is already initialised, then there's nothing to do.
    if (status & MICROBIT_RADIO_STATUS_INITIALISED)
        return MICROBIT_OK;

    // Only attempt to enable this radio mode if BLE is disabled.
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    // pre allocate all buffers
    for (int i = 0; i < TDMA_CAT_BUFFER_POOL_SIZE; i++)
        this->bufferPool[i] = new TDMACATSuperFrame;

    sync_drift = 0;

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->POWER = 0;
    NRF_RADIO->POWER = 1;

    // NRF_RADIO->TXPOWER = (uint32_t)MICROBIT_BLE_POWER_LEVEL[0];
    NRF_RADIO->TXPOWER = (uint32_t)MICROBIT_BLE_POWER_LEVEL[6];
    NRF_RADIO->FREQUENCY = MICROBIT_RADIO_DEFAULT_FREQUENCY;

    // Configure for 1Mbps throughput.
    // This may sound excessive, but running a high data rates reduces the chances of collisions...
    NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_1Mbit;

    // Configure the addresses we use for this protocol. We run ANONYMOUSLY at the core.
    // A 40 bit addresses is used. The first 32 bits match the ASCII character code for "uBit".
    // Statistically, this provides assurance to avoid other similar 2.4GHz protocols that may be in the vicinity.
    // We also map the assigned 8-bit GROUP id into the PREFIX field. This allows the RADIO hardware to perform
    // address matching for us, and only generate an interrupt when a packet matching our group is received.
#if TDMA_CAT_TEST_MODE == 1
    if (testRole == Collector)
        NRF_RADIO->BASE0 =  MICROBIT_RADIO_BASE_ADDRESS;
    else
        NRF_RADIO->BASE0 =  TDMA_CAT_RADIO_BASE_ADDRESS;
#else
    NRF_RADIO->BASE0 =  TDMA_CAT_RADIO_BASE_ADDRESS;
#endif

    NRF_RADIO->PREFIX0 = 0;

    // The RADIO hardware module supports the use of multiple addresses, but as we're running anonymously, we only need one.
    // Configure the RADIO module to use the default address (address 0) for both send and receive operations.
    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = 1;

    // Packet layout configuration. The nrf51822 has a highly capable and flexible RADIO module that, in addition to transmission
    // and reception of data, also contains a LENGTH field, two optional additional 1 byte fields (S0 and S1) and a CRC calculation.
    // Configure the packet format for a simple 8 bit length field and no additional fields.
    NRF_RADIO->PCNF0 = 0x00000008;
    // NRF_RADIO->PCNF1 = 0x02040000 | MICROBIT_RADIO_MAX_PACKET_SIZE;
    // NRF_RADIO->PCNF1 = 0x00040000 | MICROBIT_RADIO_MAX_PACKET_SIZE;

    NRF_RADIO->PCNF1 = 0x02040000 | TDMA_CAT_MAX_PACKET_SIZE;

    // Most communication channels contain some form of checksum - a mathematical calculation taken based on all the data
    // in a packet, that is also sent as part of the packet. When received, this calculation can be repeated, and the results
    // from the sender and receiver compared. If they are different, then some corruption of the data ahas happened in transit,
    // and we know we can't trust it. The nrf51822 RADIO uses a CRC for this - a very effective checksum calculation.
    //
    // Enable automatic 16bit CRC generation and checking, and configure how the CRC is calculated.
    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // Set the start random value of the data whitening algorithm. This can be any non zero number.
    NRF_RADIO->DATAWHITEIV = MICROBIT_RADIO_DEFAULT_FREQUENCY;

    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    // NRF_RADIO->TIFS = 300;
    NRF_RADIO->PACKETPTR = (uint32_t)&TDMACATRadio::instance->staticFrame;
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

    radioState = RADIO_STATE_RECEIVE;

    NRF_RADIO->INTENCLR = 0xffffffff;
    NRF_RADIO->INTENSET = 0x8;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 0);
    NVIC_EnableIRQ(RADIO_IRQn);

    PPI_ENABLE_CHAN(PPI_CHAN_END);

    tdmaState = TDMA_STATE_INITIALISATION;

    int disable_time = TDMA_DISCOVER_BACK_OFF + microbit_random(1000);

    // disable after 2 seconds, and perform application tasks
    TIMER_ENABLE_CC_IRQ(TIMER_CC_TDMA);
    TIMER_SET_CC(TIMER_CC_TDMA, disable_time);
    TIMER_SET_PRIORITY(1);
    TIMER_ENABLE_IRQ();
    TIMER_START();

    NRF_RADIO->TASKS_RXEN = 1;

    // Done. Record that our RADIO is configured.
    status |= MICROBIT_RADIO_STATUS_INITIALISED;

    fiber_add_idle_component(this);

    return MICROBIT_OK;
}

/**
  * Disables the radio for use as a multipoint sender/receiver.
  *
  * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int TDMACATRadio::disable()
{
    // Only attempt to enable.disable the radio if the protocol is alreayd running.
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    if (!(status & MICROBIT_RADIO_STATUS_INITIALISED))
        return MICROBIT_OK;

    TIMER_STOP();

    TDMACATSuperFrame* p = NULL;

    // drain rx / tx queues
    while((p = getBufferFromQueue(this->rxQueue, &this->rxTail, &this->rxHead)) != NULL)
        addBufferToPool(p);

    while((p = getBufferFromQueue(this->txQueue, &this->txTail, &this->txHead)) != NULL)
        addBufferToPool(p);

    // walk the pool and free memory.
    for (int i = 0; i < TDMA_CAT_BUFFER_POOL_SIZE; i++)
    {
        if (this->bufferPool[i])
        {
            delete this->bufferPool[i];
            this->bufferPool[i] = NULL;
        }
    }

    // Disable interrupts and STOP any ongoing packet reception.
    NVIC_DisableIRQ(RADIO_IRQn);

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);

    // record that the radio is now disabled
    status &= ~MICROBIT_RADIO_STATUS_INITIALISED;

    return MICROBIT_OK;
}

TDMACATSuperFrame* TDMACATRadio::peakRxQueue()
{
    if (this->rxTail == this->rxHead)
        return NULL;

    return rxQueue[this->rxTail];
}

TDMACATSuperFrame* TDMACATRadio::peakTxQueue()
{
    if (this->txTail == this->txHead)
        return NULL;

    return txQueue[this->txTail];
}

/**
  * Retrieves the next packet from the receive buffer.
  * If a data packet is available, then it will be returned immediately to
  * the caller. This call will also dequeue the buffer.
  *
  * @return The buffer containing the the packet. If no data is available, NULL is returned.
  *
  * @note Once recv() has been called, it is the callers responsibility to
  *       delete the buffer when appropriate.
  */
TDMACATSuperFrame* TDMACATRadio::recv()
{
    TDMACATSuperFrame* ret = NULL;
    TDMACATSuperFrame* p = getBufferFromQueue(this->rxQueue, &this->rxTail, &this->rxHead);

    if (p)
    {
        ret = new TDMACATSuperFrame;
        memcpy(ret, p, sizeof(TDMACATSuperFrame));
        addBufferToPool(p);
    }

    return ret;
}

void TDMACATRadio::idleTick()
{
    TDMACATSuperFrame* p = NULL;
    while((p = recv()))
        delete p;
}


/**
  * Transmits the given buffer onto the broadcast radio.
  * The call will wait until the transmission of the packet has completed before returning.
  *
  * @param data The packet contents to transmit.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int TDMACATRadio::send(TDMACATSuperFrame* buffer)
{
    return queueTxFrame(buffer);
}


/**
  * Transmits the given buffer onto the broadcast radio.
  *
  * This is a synchronous call that will wait until the transmission of the packet
  * has completed before returning.
  *
  * @param buffer The packet contents to transmit.
  *
  * @param len The number of bytes to transmit.
  *
  * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the buffer is invalid,
  *         or the number of bytes to transmit is greater than `MICROBIT_RADIO_MAX_PACKET_SIZE + MICROBIT_RADIO_HEADER_SIZE`.
  */
int TDMACATRadio::send(uint8_t *buffer, int len)
{
    if (buffer == NULL || len < 0 || len > TDMA_CAT_MAX_PACKET_SIZE + TDMA_CAT_HEADER_SIZE - 1)
        return MICROBIT_INVALID_PARAMETER;

    TDMACATSuperFrame buf;

    buf.device_id = serial_number;
    buf.slot_id = 0;
    buf.frame_id = 0;
    buf.flags = 0;
    buf.length = len + TDMA_CAT_HEADER_SIZE - 1;
    memcpy(buf.payload, buffer, len);

    return send(&buf);
}

#if TDMA_CAT_TEST_MODE == 1
int TDMACATRadio::setTestRole(TestRole t)
{
    testRole = t;
    return MICROBIT_OK;
}

int TDMACATRadio::sendTestResults(uint8_t* data, uint8_t length)
{
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;

    NRF_RADIO->BASE0 =  MICROBIT_RADIO_BASE_ADDRESS;
    NRF_RADIO->PCNF1 = 0x02040000 | 252;

    TDMACATTestFrame* buf = new TDMACATTestFrame;

    memset(buf, 0, sizeof(TDMACATTestFrame));

    buf->device_id = microbit_serial_number();
    buf->slot_id = 0;
    buf->frame_id = 0;
    buf->flags = 0;
    buf->length = length + TDMA_CAT_HEADER_SIZE - 1;
    memcpy(buf->payload, data, length);

    NRF_RADIO->PACKETPTR = (uint32_t)buf;

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_END == 0);

    NRF_RADIO->EVENTS_END = 0;

    delete buf;

    return MICROBIT_OK;
}

#endif // TEST MODE
#endif // DIRECT DEBUG