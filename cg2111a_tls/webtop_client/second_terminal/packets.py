# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP  = 0
# TODO (Activity 2): define your own command type for the color sensor here.
# It must match the value you add to TCommandType in packets.h.
COMMAND_COLOR = 1 # ash: added on 13/3

#fro Studio 16
COMMAND_BOT_FORWARD = 2 
COMMAND_BOT_BACKWARD = 3
COMMAND_BOT_LEFT = 4
COMMAND_BOT_RIGHT = 5
COMMAND_CHANGE_MOTOR_SPEED_INCREASE = 6 
COMMAND_CHANGE_MOTOR_SPEED_DECREASE = 7
COMMAND_STOP = 8
COMMAND_ARM = 9

RESP_OK     = 0
RESP_STATUS = 1
# TODO (Activity 2): define your own response type for the color sensor here.
# It must match the value you add to TResponseType in packets.h.
RESP_COLOR = 2 # ash: added on 13/3

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'


MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103
