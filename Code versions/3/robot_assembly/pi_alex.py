#!/usr/bin/env python3

import struct
import serial
import time
import sys
import select

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600
_ser = None

def openSerial():
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")

def closeSerial():
    global _ser
    if _ser and _ser.is_open:
        _ser.close()

PACKET_TYPE_COMMAND = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE = 2

COMMAND_ESTOP = 0
COMMAND_BOT_FORWARD = 1
COMMAND_BOT_BACKWARD = 2
COMMAND_BOT_LEFT = 3
COMMAND_BOT_RIGHT = 4
COMMAND_CHANGE_MOTOR_SPEED = 5

RESP_OK = 0
RESP_STATUS = 1

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1

def computeChecksum(data):
    result = 0
    for b in data:
        result ^= b
    return result

def packFrame(packetType, command, data=b'', params=None):
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])

def unpackTPacket(raw):
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }

def receiveFrame():
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]
    while True:
        b = _ser.read(1)
        if not b: return None
        if b[0] != MAGIC_HI: continue
        b = _ser.read(1)
        if not b: return None
        if b[0] != MAGIC_LO: continue
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk: return None
            raw += chunk
        cs_byte = _ser.read(1)
        if not cs_byte: return None
        if cs_byte[0] != computeChecksum(raw): continue
        return unpackTPacket(raw)

def sendCommand(commandType, data=b'', params=None):
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)

_estop_state = STATE_RUNNING

def isEstopActive():
    return _estop_state == STATE_STOPPED

def printPacket(pkt):
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            print("Status: RUNNING" if state == STATE_RUNNING else "Status: STOPPED")
    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")

speed = 200

def handleUserInput(line):
    global speed
    if   line == 'w': sendCommand(COMMAND_BOT_FORWARD,  params=[speed])
    elif line == 's': sendCommand(COMMAND_BOT_BACKWARD, params=[speed])
    elif line == 'a': sendCommand(COMMAND_BOT_LEFT,     params=[speed])
    elif line == 'd': sendCommand(COMMAND_BOT_RIGHT,    params=[speed])
    elif line == '+': speed = min(255, speed + 20); print(f"Speed: {speed}")
    elif line == '-': speed = max(0,   speed - 20); print(f"Speed: {speed}")
    elif line == 'e': sendCommand(COMMAND_ESTOP);   print("E-Stop sent!")
    else: print(f"Unknown: '{line}'. Use w/a/s/d/+/-/e")

def runCommandInterface():
    print("Controls: w=forward  s=backward  a=left  d=right  +/-=speed  e=stop  q=quit")
    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            if line == 'q':
                break
            handleUserInput(line)
        time.sleep(0.05)

if __name__ == '__main__':
    openSerial()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        closeSerial()