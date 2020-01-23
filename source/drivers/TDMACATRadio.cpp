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

#if MICROBIT_RADIO_VERSION == MICROBIT_RADIO_PERIDO

#include "TDMACATRadio.h"
#include "MicroBitComponent.h"
#include "EventModel.h"
#include "MicroBitDevice.h"
#include "ErrorNo.h"
#include "MicroBitFiber.h"
#include "MicroBitBLEManager.h"
#include "MicroBitHeapAllocator.h"

TDMACATRadio* TDMACATRadio::instance = NULL;

#define TIME_TO_TRANSMIT_BYTE_1MB   8
#define TX_PACKETS_SIZE             (2 * TDMA_CAT_MAXIMUM_TX_BUFFERS)

volatile uint32_t packets_received = 0;
volatile uint32_t packets_error = 0;
volatile uint32_t packets_transmitted = 0;
volatile uint32_t packets_forwarded = 0;

// volatile uint8_t last_seen_index = 0;
// volatile uint32_t last_seen[LAST_SEEN_BUFFER_SIZE] = { 0 };

volatile uint8_t  tx_packets_head = 0;
volatile uint8_t  tx_packets_tail = 0;
volatile uint32_t tx_packets[TX_PACKETS_SIZE] = { 0 };

/**
  * Driver configuration flags
  **/
#define TDMA_CAT_ASSERT 1

// 20 [txen][CC0], 21[rxen][CC0], 22[dis][CC1], 27[END][CC2]
#define TX_EN_PPI_CHAN_BIT_POS      20
#define RX_EN_PPI_CHAN_BIT_POS      21
#define DIS_PPI_CHAN_BIT_POS        22
#define END_CC_PPI_CHAN_BIT_POS     27

#define TIMER_CC_TX_RX              0
#define TIMER_CC_DISABLE            1
#define TIMER_CC_TIMESTAMP          2
#define TIMER_CC_TDMA               3

#define RADIO_STATE_RECEIVE     (1)
#define RADIO_STATE_TRANSMIT    (2)
#define RADIO_STATE_FORWARD     (3)
#define RADIO_STATE_DISCOVER    (4)

#define TDMA_STATE_DISCOVER     (1)
#define TDMA_STATE_NORMAL       (2)

volatile uint8_t radioState = RADIO_STATE_RECEIVE;
volatile uint8_t tdmaState = TDMA_STATE_DISCOVER;

TestRole testRole;

void timer_callback(uint8_t) {}


extern void set_transmission_reception_gpio(int);
extern void process_packet(TDMACATSuperFrame* p, bool, int);

#pragma GCC push_options
#pragma GCC optimize ("O0")

volatile int hw_state = 0;
#define HW_ASSERT(expected_state, panic_num) {\
                                        hw_state = NRF_RADIO->STATE;\
                                        if (hw_state != expected_state) \
                                            microbit_panic(__LINE__);\
                                        }\

extern "C" void RADIO_IRQHandler(void)
{
    NRF_RADIO->EVENTS_END = 0;
    TDMACATSuperFrame *p = TDMACATRadio::instance->rxBuf;

    if (radioState == RADIO_STATE_FORWARD)
    {
        radioState = RADIO_STATE_RECEIVE;
        memset(p, 0, sizeof(TDMACATSuperFrame));
        NRF_RADIO->PACKETPTR = (uint32_t)TDMACATRadio::instance->rxBuf;
        while(NRF_RADIO->EVENTS_DISABLED == 0);
#if TDMA_CAT_ASSERT == 1
        HW_ASSERT(0,0);
#endif
        NRF_RADIO->TASKS_RXEN = 1;
        volatile int i = 250;
        while(i-- > 0);
        NRF_RADIO->TASKS_START = 1;
        packets_forwarded++;
        return;
    }

#if TDMA_CAT_TEST_MODE == 1
    if (radioState == RADIO_STATE_RECEIVE)
    {
        if (testRole == Repeater)
        {
            packets_received++;
            if(NRF_RADIO->CRCSTATUS == 1)
            {
                if(p->ttl > 0)
                {
                    p->ttl--;
                    radioState = RADIO_STATE_FORWARD;
                    NRF_RADIO->PACKETPTR = (uint32_t)p;
#if TDMA_CAT_ASSERT == 1
                    HW_ASSERT(0,0);
#endif
                    NRF_RADIO->EVENTS_DISABLED = 0;
                    NRF_RADIO->TASKS_TXEN = 1;
                    volatile int i = 250;
                    while(i-- > 0);
                    NRF_RADIO->TASKS_START = 1;

                    // back porch here

                    return;
                }
            }
            else
            {
                packets_error++;
            }
            NRF_RADIO->PACKETPTR = (uint32_t)p;
#if TDMA_CAT_ASSERT == 1
            HW_ASSERT(0,0);
#endif
            NRF_RADIO->EVENTS_DISABLED = 0;
            NRF_RADIO->TASKS_RXEN = 1;

            volatile int i = 250;
            while(i-- > 0);
            NRF_RADIO->TASKS_START = 1;
        }
        else
        {
            if(NRF_RADIO->CRCSTATUS == 0)
                packets_error++;

            packets_received++;
            NRF_RADIO->PACKETPTR = (uint32_t)p;
#if TDMA_CAT_ASSERT == 1
            HW_ASSERT(0,0);
#endif
            NRF_RADIO->EVENTS_DISABLED = 0;
            NRF_RADIO->TASKS_RXEN = 1;

            process_packet(p, NRF_RADIO->CRCSTATUS == 1, NRF_RADIO->RSSISAMPLE);
            memset(p, 0, sizeof(TDMACATSuperFrame));
            volatile int i = 250;
            while(i-- > 0);
            NRF_RADIO->TASKS_START = 1;
        }
        return;
    }
#else
    if (radioState == RADIO_STATE_RECEIVE)
    {
        if(NRF_RADIO->CRCSTATUS == 1)
        {
            if(p->ttl > 0)
            {
                p->ttl--;
                radioState = RADIO_STATE_FORWARD;
                NRF_RADIO->PACKETPTR = (uint32_t)p;
#if TDMA_CAT_ASSERT == 1
                HW_ASSERT(0,0);
#endif
                NRF_RADIO->EVENTS_DISABLED = 0;
                NRF_RADIO->TASKS_TXEN = 1;
                packets_received++;
                return;
            }
        }
        else
        {
            packets_error++;
        }

        packets_received++;
        NRF_RADIO->PACKETPTR = (uint32_t)p;
#if TDMA_CAT_ASSERT == 1
        HW_ASSERT(0,0);
#endif
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_RXEN = 1;
        return;
    }
#endif

    if (radioState == RADIO_STATE_TRANSMIT)
    {
        radioState = RADIO_STATE_RECEIVE;
        NRF_RADIO->PACKETPTR = (uint32_t)TDMACATRadio::instance->rxBuf;
        while(NRF_RADIO->EVENTS_DISABLED == 0);
#if TDMA_CAT_TEST_MODE == 1
        HW_ASSERT(0,0);
#endif
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_RXEN = 1;
        packets_transmitted++;

        volatile int i = 250;
        while(i-- > 0);
        NRF_RADIO->TASKS_START = 1;

        // back porch here
        return;
    }
}

#endif

void manual_poke(TDMACATSuperFrame* p)
{
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;

#if TDMA_CAT_TEST_MODE == 1
        HW_ASSERT(0,0);
#endif

    radioState = RADIO_STATE_TRANSMIT;
    NRF_RADIO->PACKETPTR = (uint32_t)p;
    NRF_RADIO->TASKS_TXEN = 1;

    volatile int i = 250;
    while(i-- > 0);
    NRF_RADIO->TASKS_START = 1;
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
TDMACATRadio::TDMACATRadio(LowLevelTimer& timer, uint8_t appId, uint16_t id) : timer(timer), cloud(*this, TDMA_CAT_CLOUD_NAMESPACE), datagram(*this, TDMA_CAT_DATAGRAM_NAMESPACE), event(*this, TDMA_CAT_EVENT_NAMESPACE)
{
    this->id = id;
    this->appId = appId;
    this->status = 0;
    this->rxQueueDepth = 0;
    this->txQueueDepth = 0;

    this->rxBuf = NULL;

    memset(this->rxArray, 0, sizeof(TDMACATSuperFrame*) * TDMA_CAT_MAXIMUM_TX_BUFFERS);
    this->rxHead = 0;
    this->rxTail = 0;

    memset(this->txArray, 0, sizeof(TDMACATSuperFrame*) * TDMA_CAT_MAXIMUM_TX_BUFFERS);
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

    microbit_seed_random();

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

/**
  * Retrieve a pointer to the currently allocated receive buffer. This is the area of memory
  * actively being used by the radio hardware to store incoming data.
  *
  * @return a pointer to the current receive buffer.
  */
TDMACATSuperFrame* TDMACATRadio::getRxBuf()
{
    return rxBuf;
}

/**
  * Attempt to queue a buffer received by the radio hardware, if sufficient space is available.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NO_RESOURCES if a replacement receiver buffer
  *         could not be allocated (either by policy or memory exhaustion).
  */
int TDMACATRadio::copyRxBuf()
{
    if (rxBuf == NULL)
        return MICROBIT_INVALID_PARAMETER;

    uint8_t nextTail = (this->rxTail + 1) % MICROBIT_RADIO_MAXIMUM_RX_BUFFERS;

    if (nextTail == this->rxHead)
        return MICROBIT_NO_RESOURCES;

    // Ensure that a replacement buffer is available before queuing.
    TDMACATSuperFrame *newRxBuf = new TDMACATSuperFrame();

    if (newRxBuf == NULL)
        return MICROBIT_NO_RESOURCES;

    memcpy(newRxBuf, rxBuf, sizeof(TDMACATSuperFrame));

    // add our buffer to the array before updating the head
    // this ensures atomicity.
    this->rxArray[nextTail] = newRxBuf;
    this->rxTail = nextTail;

    // Increase our received packet count
    rxQueueDepth++;

    return MICROBIT_OK;
}

/**
  * Retrieve a pointer to the currently allocated receive buffer. This is the area of memory
  * actively being used by the radio hardware to store incoming data.
  *
  * @return a pointer to the current receive buffer.
  */
int TDMACATRadio::popTxQueue()
{
    if (this->txTail == this->txHead)
        return MICROBIT_OK;

    uint8_t nextHead = (this->txHead + 1) % TDMA_CAT_MAXIMUM_TX_BUFFERS;
    TDMACATSuperFrame *p = txArray[nextHead];
    this->txArray[nextHead] = NULL;
    this->txHead = nextHead;
    txQueueDepth--;

    tx_packets[tx_packets_tail] = (p->namespace_id << 16) | p->id;

    uint8_t next_tx_tail = (tx_packets_tail + 1) % TX_PACKETS_SIZE;
    if (next_tx_tail != tx_packets_head)
        tx_packets_tail = next_tx_tail;

    delete p;

    return MICROBIT_OK;
}

TDMACATSuperFrame* TDMACATRadio::getCurrentTxBuf()
{
    if (this->txTail == this->txHead)
        return NULL;

    uint8_t nextTx = (this->txHead + 1) % TDMA_CAT_MAXIMUM_TX_BUFFERS;
    return this->txArray[nextTx];
}

int TDMACATRadio::queueTxBuf(TDMACATSuperFrame* tx)
{
    uint8_t nextTail = (this->txTail + 1) % TDMA_CAT_MAXIMUM_TX_BUFFERS;

    if (nextTail == this->txHead)
        return MICROBIT_NO_RESOURCES;

    // Ensure that a replacement buffer is available before queuing.
    TDMACATSuperFrame *newTx = new TDMACATSuperFrame();

    if (newTx == NULL)
        return MICROBIT_NO_RESOURCES;

    memcpy(newTx, tx, sizeof(TDMACATSuperFrame));

    // add our buffer to the array before updating the head
    // this ensures atomicity.
    this->txArray[nextTail] = newTx;
    __disable_irq();
    this->txTail = nextTail;
    __enable_irq();

    txQueueDepth++;

    return MICROBIT_OK;
}

int TDMACATRadio::queueKeepAlive()
{
    // TDMACATSuperFrame buf;

    // buf.id = microbit_random(65535);
    // buf.length = 0 + TDMA_CAT_HEADER_SIZE - 1; // keep alive has no content.
    // buf.app_id = appId;
    // buf.namespace_id = 0;
    // buf.flags |= TDMA_CAT_FRAME_KEEP_ALIVE_FLAG;
    // buf.ttl = 2;
    // buf.initial_ttl = 2;
    // buf.time_since_wake = 0;
    // buf.period = 0;

    // return queueTxBuf(&buf);
    return MICROBIT_OK;
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

    // If this is the first time we've been enable, allocate our receive buffers.
    if (rxBuf == NULL)
        rxBuf = new TDMACATSuperFrame();

    if (rxBuf == NULL)
        return MICROBIT_NO_RESOURCES;

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->POWER = 0;
    NRF_RADIO->POWER = 1;

    NRF_RADIO->TXPOWER = (uint32_t)MICROBIT_BLE_POWER_LEVEL[6];
    NRF_RADIO->FREQUENCY = MICROBIT_RADIO_DEFAULT_FREQUENCY;

    // Bring up the nrf51822 RADIO module in Nordic's proprietary 1MBps packet radio mode.
    // setTransmitPower(MICROBIT_RADIO_DEFAULT_TX_POWER);

    // setFrequencyBand(MICROBIT_RADIO_DEFAULT_FREQUENCY);
    // NRF_RADIO->OVERRIDE0 = NRF_FICR->BLE_1MBIT[0];
    // NRF_RADIO->OVERRIDE1 = NRF_FICR->BLE_1MBIT[1];
    // NRF_RADIO->OVERRIDE2 = NRF_FICR->BLE_1MBIT[2];
    // NRF_RADIO->OVERRIDE3 = NRF_FICR->BLE_1MBIT[3];
    // NRF_RADIO->OVERRIDE4 = 0x80000000 | NRF_FICR->BLE_1MBIT[4];

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

    // Set up the RADIO module to read and write from our internal buffer.
    NRF_RADIO->PACKETPTR = (uint32_t)rxBuf;

    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    // NRF_RADIO->TIFS = 300;
    NRF_RADIO->PACKETPTR = (uint32_t)TDMACATRadio::instance->rxBuf;
    NRF_RADIO->SHORTS = /*RADIO_SHORTS_READY_START_Msk |*/ RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

    radioState = RADIO_STATE_RECEIVE;

    NRF_RADIO->INTENCLR = 0xffffffff;
    NRF_RADIO->INTENSET = 0x8;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 0);
    NVIC_EnableIRQ(RADIO_IRQn);

    // set cc1 to trigger radio disable, set an end event to capture timestamp in cc2
    NRF_PPI->CHEN |= (1 << DIS_PPI_CHAN_BIT_POS) | (1 << END_CC_PPI_CHAN_BIT_POS);

    // disable after 2 seconds, and perform application tasks
    NRF_TIMER0->CC[TIMER_CC_DISABLE] = 2000000;
    NRF_TIMER0->CC[TIMER_CC_TDMA] = 2000000;

    // need to express discovery states.

    NRF_RADIO->TASKS_RXEN = 1;

    volatile int i = 250;
    while(i-- > 0);
    NRF_RADIO->TASKS_START = 1;

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

    // Disable interrupts and STOP any ongoing packet reception.
    NVIC_DisableIRQ(RADIO_IRQn);

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);

    // record that the radio is now disabled
    status &= ~MICROBIT_RADIO_STATUS_INITIALISED;

    return MICROBIT_OK;
}

int TDMACATRadio::setGroup(uint8_t id)
{
    return setAppId(id);
}

int TDMACATRadio::setAppId(uint16_t id)
{
    this->appId = id;
    return MICROBIT_OK;
}

int TDMACATRadio::getAppId()
{
    return this->appId;
}

/**
  * Determines the number of packets ready to be processed.
  *
  * @return The number of packets in the receive buffer.
  */
int TDMACATRadio::dataReady()
{
    return rxQueueDepth;
}

TDMACATSuperFrame* TDMACATRadio::peakRxQueue()
{
    if (this->rxTail == this->rxHead)
        return NULL;

    uint8_t nextHead = (this->rxHead + 1) % MICROBIT_RADIO_MAXIMUM_RX_BUFFERS;
    return rxArray[nextHead];
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
    if (this->rxTail == this->rxHead)
        return NULL;

    uint8_t nextHead = (this->rxHead + 1) % MICROBIT_RADIO_MAXIMUM_RX_BUFFERS;

    TDMACATSuperFrame *p = rxArray[nextHead];
    this->rxArray[nextHead] = NULL;
    this->rxHead = nextHead;
    rxQueueDepth--;

    return p;
}

void TDMACATRadio::idleTick()
{
    // if (radio_status & RADIO_STATUS_QUEUE_KEEP_ALIVE)
    // {
    //     queueKeepAlive();
    //     PERIDO_UNSET_FLAGS(RADIO_STATUS_QUEUE_KEEP_ALIVE);
    // }

    // walk the array of tx'd packets and fire packetTransmitted for each driver...
    while (tx_packets_head != tx_packets_tail)
    {
        uint8_t next_tx_head = (tx_packets_head + 1) % TX_PACKETS_SIZE;
        uint8_t namespace_id = tx_packets[tx_packets_head] >> 16;
        uint16_t id = tx_packets[tx_packets_head] & 0xFFFF;

        if (namespace_id == cloud.getNamespaceId())
            cloud.packetTransmitted(id);

        tx_packets_head = next_tx_head;
    }

    // Walk the list of received packets and process each one.
    TDMACATSuperFrame* p = NULL;
    while ((p = peakRxQueue()) != NULL)
    {
        if (p->namespace_id == cloud.getNamespaceId())
            cloud.packetReceived();

        else if (p->namespace_id == datagram.getNamespaceId())
            datagram.packetReceived();

        else if (p->namespace_id == event.getNamespaceId())
            event.packetReceived();

        else
            delete recv();
    }
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
    return queueTxBuf(buffer);
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
int TDMACATRadio::send(uint8_t *buffer, int len, uint8_t namespaceId)
{
    if (buffer == NULL || len < 0 || len > TDMA_CAT_MAX_PACKET_SIZE + TDMA_CAT_HEADER_SIZE - 1)
        return MICROBIT_INVALID_PARAMETER;

    TDMACATSuperFrame buf;

    buf.id = microbit_random(65535);
    buf.length = len + TDMA_CAT_HEADER_SIZE - 1;
    buf.app_id = appId;
    buf.namespace_id = namespaceId;
    buf.ttl = TDMA_CAT_DEFAULT_TTL;
    buf.initial_ttl = TDMA_CAT_DEFAULT_TTL;
    buf.time_since_wake = 0;
    buf.period = 0;
    memcpy(buf.payload, buffer, len);

    return send(&buf);
}

uint16_t TDMACATRadio::generateId(uint8_t app_id, uint8_t namespace_id)
{
    uint16_t new_id;
    bool seenBefore = true;

    // while (seenBefore)
    // {
    //     seenBefore = false;
    //     new_id = microbit_random(65535);

    //     for (int i = 0; i < LAST_SEEN_BUFFER_SIZE; i++)
    //     {
    //         if (last_seen[i] > 0)
    //         {
    //             uint8_t seen_namespace_id = last_seen[i];
    //             uint8_t seen_app_id = last_seen[i] >> 8;

    //             // we can exit early here as if the namespaces don't match we're not interested.
    //             if (namespace_id != seen_namespace_id || app_id != seen_app_id)
    //                 continue;

    //             uint16_t packet_id = (last_seen[i] >> 16);
    //             if (packet_id == new_id)
    //                 seenBefore = true;
    //         }
    //     }
    // }

    return new_id;
}

int TDMACATRadio::setTestRole(TestRole t)
{
    testRole = t;
    return MICROBIT_OK;
}

int TDMACATRadio::sendTestResults(uint8_t* data, uint8_t length)
{
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;

    NRF_RADIO->BASE0 =  MICROBIT_RADIO_BASE_ADDRESS;

    TDMACATSuperFrame* buf = new TDMACATSuperFrame;

    memset(buf, 0, sizeof(TDMACATSuperFrame));

    buf->id = microbit_random(65535);
    buf->length = length + TDMA_CAT_HEADER_SIZE - 1;
    buf->app_id = 0;
    buf->namespace_id = 0;
    buf->ttl = 0;
    buf->initial_ttl = 0;
    buf->time_since_wake = 0;
    buf->period = 0;
    memcpy(buf->payload, data, length);

    NRF_RADIO->PACKETPTR = (uint32_t)buf;

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_END == 0);

    NRF_RADIO->EVENTS_END = 0;

    delete buf;

    return MICROBIT_OK;
}
