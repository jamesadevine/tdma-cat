#ifndef DEBUG_BRIDGE_H
#define DEBUG_BRIDGE_H

#include "TDMACATRadio.h"
#include "MicroBitSerial.h"
#include "MicroBitMessageBus.h"
#include "MicroBitDisplay.h"
#include "MicroBitComponent.h"

// the header is only the first two bytes, as the id is placed inside the payload

class DebugBridge : public MicroBitComponent
{
    TDMACATRadio& radio;
    MicroBitSerial& serial;
    MicroBitDisplay& display;

    uint32_t packetCount;

    void onRadioPacket(MicroBitEvent e);

    public:

    DebugBridge(TDMACATRadio& r, MicroBitSerial& s, MicroBitMessageBus& b, MicroBitDisplay& display);

    void enable();
};

#endif