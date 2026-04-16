#include "packets.h"

volatile TState currentState = STATE_RUNNING;

static uint8_t computeChecksum(const uint8_t *data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

static void sendFrame(const TPacket *pkt) {
    uint8_t frame[FRAME_SIZE];
    frame[0] = MAGIC_HI;
    frame[1] = MAGIC_LO;
    memcpy(&frame[2], pkt, TPACKET_SIZE);
    frame[2 + TPACKET_SIZE] = computeChecksum((const uint8_t *)pkt, TPACKET_SIZE);
    Serial.write(frame, FRAME_SIZE);
} 

static bool receiveFrame(TPacket *pkt) {
    static uint8_t state = 0;
    static uint8_t raw[TPACKET_SIZE];
    static uint8_t index = 0;

    while (Serial.available() > 0) {
        uint8_t byte = (uint8_t)Serial.read();

        switch (state) {
            case 0: // waiting for MAGIC_HI
                if (byte == MAGIC_HI) {
                    state = 1;
                }
                break;

            case 1: // waiting for MAGIC_LO
                if (byte == MAGIC_LO) {
                    state = 2;
                    index = 0;
                } else if (byte != MAGIC_HI) {
                    state = 0;
                }
                // else stay in state 1 if we got another MAGIC_HI
                break;

            case 2: // reading TPacket payload
                raw[index++] = byte;
                if (index >= TPACKET_SIZE) {
                    state = 3;
                }
                break;

            case 3: { // reading checksum
                uint8_t expected = computeChecksum(raw, TPACKET_SIZE);
                if (byte == expected) {
                    memcpy(pkt, raw, TPACKET_SIZE);
                    state = 0;
                    return true;
                }
                // checksum failed; resync
                state = (byte == MAGIC_HI) ? 1 : 0;
                break;
            }
        }
    }
}

static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

int currentSpeed = 200;

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            currentState = STATE_STOPPED;
            stop();
            {
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;

        case COMMAND_BOT_FORWARD:
            if (currentState == STATE_RUNNING)
                forward(currentSpeed);
            break;

        case COMMAND_BOT_BACKWARD:
            if (currentState == STATE_RUNNING)
                backward(currentSpeed);
            break;

        case COMMAND_BOT_LEFT:
            if (currentState == STATE_RUNNING)
                ccw(currentSpeed);
            break;

        case COMMAND_BOT_RIGHT:
            if (currentState == STATE_RUNNING)
                cw(currentSpeed);
            break;

        case COMMAND_CHANGE_MOTOR_SPEED:
            currentSpeed = (int)cmd->params[0];
            break;
    }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
