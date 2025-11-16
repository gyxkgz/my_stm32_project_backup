#include "common.h"
#include"main.h"
#include"dshot.h"

FAST_CODE uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb)
{
    uint16_t packet;

    packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
    pcb->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        csum = ~csum;
    }
#endif
    csum &= 0xf;
    packet = (packet << 4) | csum;

    return packet;
}
