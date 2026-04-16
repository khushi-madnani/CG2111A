#!/usr/bin/env python3

import select
import struct
import sys
import time

from packets import *
from .net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame
# from .net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame


PI_HOST = 'localhost'
PI_PORT = 65432


# ---------------------------------------------------------------------------
# TPacket helpers
# ---------------------------------------------------------------------------

def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _packFrame(packetType, command, data=b'', params=None):
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded  = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

_estop_active = False


def _printPacket(pkt):
    global _estop_active

    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("[robot] OK")
        elif cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        else:
            print(f"[robot] Response: unknown command {cmd}")

        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Input handling (UPDATED HERE ONLY)
# ---------------------------------------------------------------------------

# def _handleInput(line: str, client: TCPClient):
#     line = line.strip().upper()
#     if not line:
#         return

#     # ---------------- E-STOP ----------------
#     if line == 'E':
#         frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
#         sendTPacketFrame(client.sock, frame)
#         print("[second_terminal] Sent: E-STOP")
#         return

#     # ---------------- QUIT ----------------
#     elif line == 'Q':
#         print("[second_terminal] Quitting.")
#         raise KeyboardInterrupt

#     # ---------------- ARM COMMANDS ----------------
#     # B090, S120, E130, G045, V010
#     elif len(line) == 4 and line[0] in ['B', 'S', 'E', 'G', 'V'] and line[1:].isdigit():
#         data = line.encode()
#         frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_CUSTOM, data=data)
#         sendTPacketFrame(client.sock, frame)
#         print(f"[second_terminal] Sent ARM: {line}")
#         return

#     # ---------------- HOME ----------------
#     elif line == 'H':
#         data = b'H'
#         frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_CUSTOM, data=data)
#         sendTPacketFrame(client.sock, frame)
#         print("[second_terminal] Sent: HOME")
#         return

#     # ---------------- INVALID ----------------
#     else:
#         print(f"[second_terminal] Unknown: '{line}'")
#         print("Valid:")
#         print("  E = E-Stop")
#         print("  Q = quit")
#         print("  B090 / S120 / E130 / G045")
#         print("  V010 (speed)")
#         print("  H (home)")

# ---------------------------------------------------------------------------
# Input handling (UPDATED FOR ARM INTEGRATION)
# ---------------------------------------------------------------------------

def _handleInput(line: str, client: TCPClient):
    line = line.strip().upper()
    if not line:
        return

    # ---------------- E-STOP ----------------
    if line == 'E':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP")
        return

    # ---------------- QUIT ----------------
    elif line == 'Q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    # ---------------- ARM COMMANDS (B090, S120, etc.) ----------------
    elif len(line) >= 2 and line[0] in ['B', 'S', 'E', 'G', 'V'] and line[1:].isdigit():
        joint_char = line[0]
        angle_val = int(line[1:])
        
        # We prepare the params list: params[0] is Joint, params[1] is Angle
        # In packets.h/py, COMMAND_ARM should be index 9
        arm_params = [0] * 16 # PARAMS_COUNT is 16
        arm_params[0] = ord(joint_char)
        arm_params[1] = angle_val
        
        # We use COMMAND_ARM (9) instead of COMMAND_CUSTOM
        frame = _packFrame(PACKET_TYPE_COMMAND, 9, params=arm_params)
        sendTPacketFrame(client.sock, frame)
        print(f"[second_terminal] Sent ARM: {joint_char} to {angle_val}")
        return

    # ---------------- HOME ----------------
    elif line == 'H':
        # You can either define a COMMAND_HOME or send a special custom string
        # Here we send it as a custom data packet for flexibility
        data = b'H'
        frame = _packFrame(PACKET_TYPE_COMMAND, 10, data=data) # 10 is COMMAND_CUSTOM
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: HOME")
        return

    # ---------------- INVALID ----------------
    else:
        print(f"[second_terminal] Unknown: '{line}'")
        print("Valid: E (Stop), Q (Quit), H (Home), or Joint+Angle (e.g., B090)")
# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run():
    client = TCPClient(host=PI_HOST, port=PI_PORT)
    print(f"[second_terminal] Connecting to pi_sensor.py at {PI_HOST}:{PI_PORT}...")

    if not client.connect(timeout=10.0):
        print("[second_terminal] Could not connect.")
        sys.exit(1)

    print("[second_terminal] Connected!")
    print("[second_terminal] Commands:")
    print("  E = E-Stop")
    print("  Q = quit")
    print("  B090 / S120 / E130 / G045")
    print("  V010 (speed)")
    print("  H (home)\n")

    try:
        while True:
            if client.hasData():
                frame = recvTPacketFrame(client.sock)
                if frame is None:
                    print("[second_terminal] Connection closed.")
                    break
                pkt = _unpackFrame(frame)
                if pkt:
                    _printPacket(pkt)

            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline()
                _handleInput(line, client)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[second_terminal] Exiting.")
    finally:
        client.close()


if __name__ == '__main__':
    run()