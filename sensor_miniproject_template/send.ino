#include "arm.h"
#include "movement.h"
#include "shared_state.h"
#include "packets.h"
#include "serial_driver.h"
#include <stdio.h>
#define SENSOR_OUT PL2
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

static void sendResponseMsg(TResponseType resp, const char* msg) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command = resp;
    strncpy(pkt.data, msg, sizeof(pkt.data) - 1);
    pkt.data[sizeof(pkt.data) - 1] = '\0';
    sendFrame(&pkt);
}
