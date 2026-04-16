/*
 * packets.h
 * Studio 13: Sensor Mini-Project
 *
 * TPacket protocol: enums, struct, and framing constants.
 * This file must be kept in sync with the constants in pi_sensor.py.
 */

#pragma once

#include <stdint.h>

// =============================================================
// TPacket protocol
// =============================================================

typedef enum {
    PACKET_TYPE_COMMAND  = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
    COMMAND_ESTOP = 0,
    // TODO (Activity 2): add your own command type for the color sensor
    COMMAND_COLOR = 1, // ash: added on 13/3
    COMMAND_BOT_FORWARD = 2, //Studio 16
    COMMAND_BOT_BACKWARD = 3,
    COMMAND_BOT_LEFT = 4,
    COMMAND_BOT_RIGHT = 5,
    COMMAND_CHANGE_MOTOR_SPEED_INCREASE = 6, 
    COMMAND_CHANGE_MOTOR_SPEED_DECREASE = 7,
    COMMAND_STOP = 8 
} TCommandType;

typedef enum {
    RESP_OK     = 0,
    RESP_STATUS = 1,
    // TODO (Activity 2): add your own response type for the color sensor
    RESP_COLOR = 2, // ash: added on 13/3
} TResponseType;

typedef enum {
    STATE_RUNNING = 0,
    STATE_STOPPED = 1,
} TState;

typedef struct {
    uint8_t  packetType;
    uint8_t  command;
    uint8_t  dummy[2];
    char     data[32];
    uint32_t params[16];
} TPacket;

// =============================================================
// Framing constants
// =============================================================

#define MAGIC_HI        0xDE
#define MAGIC_LO        0xAD
#define TPACKET_SIZE    ((uint8_t)sizeof(TPacket))   // 100 bytes
#define FRAME_SIZE      (2 + TPACKET_SIZE + 1)       // 103 bytes
