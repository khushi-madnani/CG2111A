import serial
import time
import struct

# Match the baud rate in your usartInit (103 = 9600)
ser = serial.Serial('/dev/ttyACM0', 9600) 
time.sleep(2)

def send_arm_packet(user_input):
    try:
        # 1. Parse the 4-character input (e.g., "B180")
        joint_char = user_input[0].upper()       # 'B'
        angle = int(user_input[1:])              # 180
        
        # 2. Build the TPacket (100 bytes)
        # B=uint8, 2x=padding, 32s=char[32], 16I=uint32[16]
        # We match the struct in your packets.h
        packet_type = 0  # PACKET_TYPE_COMMAND
        command_id = 9   # COMMAND_ARM (the index we added)
        
        # Pack the header and types
        # We put joint in params[0] and angle in params[1]
        header = struct.pack("<BB2x", packet_type, command_id)
        data_field = b'\0' * 32
        params = [0] * 16
        params[0] = ord(joint_char)
        params[1] = angle
        params_field = struct.pack("<16I", *params)
        
        full_packet = header + data_field + params_field
        
        # 3. Add the Magic Bytes if your serial_driver.h requires them
        magic = bytes([0xDE, 0xAD])
        
        ser.write(magic + full_packet)
        print(f"Successfully sent {joint_char} to {angle}")
        
    except Exception as e:
        print(f"Invalid input format! Use Letter+3Numbers (e.g., B090). Error: {e}")

while True:
    cmd = input("Enter 4-char command (e.g. B180): ")
    if len(cmd) >= 4:
        send_arm_packet(cmd)
    else:
        print("Input too short!")


# import serial
# import time

# ser = serial.Serial('/dev/ttyACM0', 115200)
# time.sleep(2)

# def send(cmd):
#     ser.write((cmd + '\n').encode())
#     print(f"Sent: {cmd}")

# while True:
#     cmd = input("Enter command: ")
#     send(cmd)


#  import serial
# import time
# import sys
# import tty
# import termios

# # ─────────────────────────────────────────────
# # SERIAL SETUP
# # ─────────────────────────────────────────────
# ser = serial.Serial('/dev/ttyACM0', 115200)
# time.sleep(2)

# def send(cmd):
#     ser.write((cmd + '\n').encode())
#     print(f"[SENT] {cmd}")

# ─────────────────────────────────────────────
# PRESET POSITIONS (TUNE THESE ONCE)
# ─────────────────────────────────────────────

# def home():
#     send("H")

# def pre_pick():
#     send("S060")
#     send("E110")

# def lower():
#     send("S050")
#     send("E120")

# def grab():
#     send("G000")  # close

# def lift():
#     send("S080")
#     send("E100")

# def drop():
#     send("G000")   # open

# # FULL AUTOMATION (BEST FEATURE)
# def auto_pick():
#     print(">>> AUTO PICK")
#     pre_pick()
#     time.sleep(0.5)
#     lower()
#     time.sleep(0.5)
#     grab()
#     time.sleep(0.5)
#     send("G090")
#     time.sleep(0.5)
#     lift()

# def auto_drop():
#     print(">>> AUTO DROP")
#     drop()
#     time.sleep(0.5)
#     home()

# # ─────────────────────────────────────────────
# # MANUAL FINE CONTROL (for adjustments)
# # ─────────────────────────────────────────────

# def base_left(): send("B090")
# def base_right(): send("B200")

# def shoulder_up(): send("S180")
# def shoulder_down(): send("S020")

# def elbow_up(): send("E180")
# def elbow_down(): send("E075")

# def gripper_open(): send("G000")
# def gripper_close(): send("G090")

# # ─────────────────────────────────────────────
# # KEYBOARD INPUT (NO ENTER REQUIRED)
# # ─────────────────────────────────────────────

# def get_key():
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         key = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return key

# # ─────────────────────────────────────────────
# # CONTROLS
# # ─────────────────────────────────────────────

# print("\n=== ARM CONTROL (FINAL) ===")
# print("AUTO:")
# print("  p → auto pick")
# print("  o → auto drop")
# print("  h → home")

# print("\nMANUAL:")
# print("  q/a → base left/right")
# print("  w/s → shoulder up/down")
# print("  e/d → elbow up/down")
# print("  r/f → gripper open/close")

# print("\nPress CTRL+C to exit\n")

# # ─────────────────────────────────────────────
# # MAIN LOOP
# # ─────────────────────────────────────────────

# try:
#     while True:
#         key = get_key()
#         if key == '\x03':
#            print("\nExiting...")
#            ser.close()
#            break

#         # AUTO
#         if key == 'p':
#             auto_pick()

#         elif key == 'o':
#             auto_drop()

#         elif key == 'h':
#             home()

#         # BASE
#         elif key == 'q':
#             base_left()
#         elif key == 'a':
#             base_right()

#         # SHOULDER
#         elif key == 'w':
#             shoulder_up()
#         elif key == 's':
#             shoulder_down()

#         # ELBOW
#         elif key == 'e':
#             elbow_up()
#         elif key == 'd':
#             elbow_down()

#         # GRIPPER
#         elif key == 'r':
#             gripper_open()
#         elif key == 'f':
#             gripper_close()

# except KeyboardInterrupt:
#     print("\nExiting...")
#     ser.close()