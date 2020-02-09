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

#ifndef TDMA_CAT_RADIO_H
#define TDMA_CAT_RADIO_H

class TDMACATRadio;
struct TDMACATSuperFrame;

#include "mbed.h"
#include "MicroBitConfig.h"
#include "PacketBuffer.h"
#include "MicroBitRadio.h"
#include "LowLevelTimer.h"
#include "HigherLevelTimer.h"
#include "ManagedString.h"
#include "PeridoRadioCloud.h"
#include "PeridoRadioDatagram.h"
#include "PeridoRadioEvent.h"

/**
 * Provides a simple broadcast radio abstraction, built upon the raw nrf51822 RADIO module.
 *
 * The nrf51822 RADIO module supports a number of proprietary modes of operation in addition to the typical BLE usage.
 * This class uses one of these modes to enable simple, point to multipoint communication directly between micro:bits.
 *
 * TODO: The protocols implemented here do not currently perform any significant form of energy management,
 * which means that they will consume far more energy than their BLE equivalent. Later versions of the protocol
 * should look to address this through energy efficient broadcast techniques / sleep scheduling. In particular, the GLOSSY
 * approach to efficienct rebroadcast and network synchronisation would likely provide an effective future step.
 *
 * TODO: Meshing should also be considered - again a GLOSSY approach may be effective here, and highly complementary to
 * the master/slave arachitecture of BLE.
 *
 * TODO: This implementation only operates whilst the BLE stack is disabled. The nrf51822 provides a timeslot API to allow
 * BLE to cohabit with other protocols. Future work to allow this colocation would be benefical, and would also allow for the
 * creation of wireless BLE bridges.
 *
 * NOTE: This API does not contain any form of encryption, authentication or authorization. It's purpose is solely for use as a
 * teaching aid to demonstrate how simple communications operates, and to provide a sandpit through which learning can take place.
 * For serious applications, BLE should be considered a substantially more secure alternative.
 */

#define IGNORE_FUTURE_PROBLEMS              1
#define TDMA_CAT_TEST_MODE                  1

// #ifndef TDMA_CAT_DIRECT_DEBUG
// #define TDMA_CAT_DIRECT_DEBUG               1
// #endif

// in test mode, we only transmit one packet
#if TDMA_CAT_TEST_MODE == 1
    #define TDMA_CAT_DEFAULT_TTL            1
#else
    #define TDMA_CAT_DEFAULT_TTL            2
#endif

#define MICROBIT_RADIO_STATUS_INITIALISED   0x0001
#define MICROBIT_RADIO_DEFAULT_TX_POWER     6
#define MICROBIT_RADIO_DEFAULT_FREQUENCY    7
#define MICROBIT_RADIO_BASE_ADDRESS         0x75626974
#define TDMA_CAT_RADIO_BASE_ADDRESS         0x75626975

// Default configuration values
#define TDMA_CAT_HEADER_SIZE                12

#define TDMA_CAT_MAX_PACKET_SIZE            100

#define TDMA_CAT_BUFFER_POOL_SIZE           20
// add one for the empty slot
#define TDMA_CAT_QUEUE_SIZE                 ((TDMA_CAT_BUFFER_POOL_SIZE / 2) + 1)

#ifndef MICROBIT_RADIO_MAXIMUM_RX_BUFFERS
#define MICROBIT_RADIO_MAXIMUM_RX_BUFFERS   10
#endif

#ifndef TDMA_CAT_MAXIMUM_TX_BUFFERS
#define TDMA_CAT_MAXIMUM_TX_BUFFERS         20
#endif

#define TDMA_CAT_DEFAULT_APP_ID             0

#define TDMA_CAT_CLOUD_NAMESPACE            1
#define TDMA_CAT_DATAGRAM_NAMESPACE         2
#define TDMA_CAT_EVENT_NAMESPACE            3

#define TDMA_CAT_NEW_SLOT_THRESHOLD         6

#define MICROBIT_RADIO_EVT_DATAGRAM             1

#define TMDMA_CAT_FRAME_FLAGS_ADVERT        0x01
#define TMDMA_CAT_FRAME_FLAGS_DONE          0x02
#define TMDMA_CAT_FRAME_FLAGS_ERROR         0x04

struct TDMACATSuperFrame
{
    uint8_t             length;                             // The length of the remaining bytes in the packet.
    uint8_t             slot_id;
    uint8_t             frame_id:4, flags:4;
    uint8_t             ttl:4, initial_ttl:4;
    uint64_t            device_id;
    uint8_t             payload[TDMA_CAT_MAX_PACKET_SIZE];    // User / higher layer protocol data
} __attribute__((packed));


#if TDMA_CAT_TEST_MODE == 1

struct TDMACATTestFrame
{
    uint8_t             length;                             // The length of the remaining bytes in the packet.
    uint8_t             slot_id;
    uint8_t             frame_id:4, flags:4;
    uint8_t             ttl:4, initial_ttl:4;
    uint64_t            device_id;
    uint8_t             payload[240];    // User / higher layer protocol data
} __attribute__((packed));

enum TestRole {
  Transmitter,
  Repeater,
  Observer,
  Collector
};

#endif

class TDMACATRadio : public MicroBitComponent
{
    public:

    void addBufferToPool(TDMACATSuperFrame* q);
    TDMACATSuperFrame* getBufferFromPool();
    int addBufferToQueue(TDMACATSuperFrame** q, TDMACATSuperFrame* p, uint8_t* tail, uint8_t* head);
    TDMACATSuperFrame* getBufferFromQueue(TDMACATSuperFrame** q, uint8_t* tail, uint8_t* head);

    int queueTxFrame(TDMACATSuperFrame* s);
    int queueRxFrame(TDMACATSuperFrame* rx);

    inline int queueSize(uint8_t* tail, uint8_t*head);

    TDMACATSuperFrame* peakRxQueue();
    TDMACATSuperFrame* peakTxQueue();
    TDMACATSuperFrame* popTxQueue();

    LowLevelTimer&          timer;
    // PeridoRadioCloud        cloud;          // A simple REST handling service.
    // PeridoRadioDatagram     datagram;       // A simple datagram handling service.
    // PeridoRadioEvent        event;          // A simple event handling service.

    // a fifo array of received packets
    // the array can hold a maximum of TDMA_CAT_QUEUE_SIZE - 1 packets
    TDMACATSuperFrame       *rxQueue[TDMA_CAT_QUEUE_SIZE];
    uint8_t                 rxHead; // head points to the last rx'd packet-1
    uint8_t                 rxTail; // tail points to the first rx'd packet

    // a fifo array of transmitted packets
    // the array can hold a maximum of TDMA_CAT_QUEUE_SIZE - 1 packets
    TDMACATSuperFrame       *txQueue[TDMA_CAT_QUEUE_SIZE];
    uint8_t                 txHead; // head points to the last packet to be tx'd
    uint8_t                 txTail; // head points to the first packet to be tx'd

    TDMACATSuperFrame       *bufferPool[TDMA_CAT_BUFFER_POOL_SIZE];

    TDMACATSuperFrame       staticFrame;

    static TDMACATRadio     *instance;  // A singleton reference, used purely by the interrupt service routine.

    /**
      * Constructor.
      *
      * Initialise the TDMACATRadio.
      *
      * @note This class is demand activated, as a result most resources are only
      *       committed if send/recv or event registrations calls are made.
      */
    TDMACATRadio(LowLevelTimer& timer, uint16_t id = MICROBIT_ID_RADIO);

    /**
      * Change the output power level of the transmitter to the given value.
      *
      * @param power a value in the range 0..7, where 0 is the lowest power and 7 is the highest.
      *
      * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the value is out of range.
      */
    int setTransmitPower(int power);

    /**
      * Change the transmission and reception band of the radio to the given channel
      *
      * @param band a frequency band in the range 0 - 100. Each step is 1MHz wide, based at 2400MHz.
      *
      * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the value is out of range,
      *         or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
      */
    int setFrequencyBand(int band);

    /**
      * Initialises the radio for use as a multipoint sender/receiver
      *
      * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
      */
    int enable();

    /**
      * Disables the radio for use as a multipoint sender/receiver.
      *
      * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
      */
    int disable();

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
    TDMACATSuperFrame* recv();

    void idleTick();

    /**
      * Transmits the given buffer onto the broadcast radio.
      *
      * @param data The packet contents to transmit.
      *
      * @return MICROBIT_OK on success, or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
      */
    int send(TDMACATSuperFrame* buffer);

    int send(uint8_t *buffer, int len);

#if TDMA_CAT_TEST_MODE == 1
    int setTestRole(TestRole t);

    int sendTestResults(uint8_t* data, uint8_t length);
#endif

};

#endif
