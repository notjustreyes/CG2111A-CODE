#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""
import atexit
import struct
import serial
import time
import sys
import select
import keyboard
import tty
import termios


#import from our files
import lidar_example_cli_plot
from lidar_example_cli_plot import plot_single_scan
import alex_camera
from alex_camera import captureGreyscaleFrame, renderGreyscaleFrame
from lidar.alex_lidar import lidarDisconnect
from second_terminal import relay
from constants import *
from pi_sensor_config import *


#global vars

_driver_fd = None
_driver_old_settings = None
is_force_stop_mode = False
is_auto_stop_mode = False
auto_stop_armed = True
is_driver_mode = False
is_jerk_mode = False
driver_mode_last_command = COMMAND_MOVE_STOP
curr_move_speed = START_MOVE_SPEED
curr_turn_speed = START_TURN_SPEED
min_move_speed = START_MIN_MOVE_SPEED
min_turn_speed = START_MIN_TURN_SPEED
jerk_move_speed = START_JERK_MOVE_SPEED
jerk_turn_speed = START_JERK_TURN_SPEED
jerk_move_duration = START_JERK_MOVE_DURATION
jerk_turn_duration = START_JERK_TURN_DURATION
force_stop_speed = START_FORCE_STOP_SPEED
force_stop_duration = START_FORCE_STOP_DURATION
curr_direction = MOVING_STOP


# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    relay.start()
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()
'''
def handleWASDControl():
    if isEstopActive():
        print("E-stop active, refuse to enter driver mode.")
        return False

    print("WASD control active (press q to quit)")

    last_command = None
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)  # enter raw mode once

        while True:
            if isEstopActive():
                print("\nExiting driver mode")
                return False

            ch = sys.stdin.read(1).lower()

            if ch == 'q':
                sendCommand(COMMAND_MOVE_STOP)
                print("\nExiting driver mode")
                return False
            elif ch == 'e':
                sendCommand(COMMAND_MOVE_STOP)
                return True
            elif ch == 'w':
                command = COMMAND_FRONT
            elif ch == 's':
                command = COMMAND_BACK
            elif ch == 'a':
                command = COMMAND_CCW
            elif ch == 'd':
                command = COMMAND_CW
            else:
                command = COMMAND_MOVE_STOP

            if command != last_command:
                sendCommand(command)
                last_command = command

            time.sleep(0.05)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        '''
'''
def handleWASDControl2():
    if isEstopActive():
        print("E-stop active, refuse to enter driver mode.")
        return False
    print("WASD control active (press q to quit)")
    last_command = None
    while True:
        if isEstopActive():
            return False

        command = None

        if keyboard.is_pressed('w'):
            print('w')
            command = COMMAND_FRONT
        elif keyboard.is_pressed('s'):
            print('s')
            command = COMMAND_BACK
        elif keyboard.is_pressed('a'):
            print('a')
            command = COMMAND_CCW
        elif keyboard.is_pressed('d'):
            print('d')
            command = COMMAND_CW
        elif keyboard.is_pressed('e'):
            return True #tells program to activate estop
        else:
            command = COMMAND_MOVE_STOP

        if command != last_command:
            sendCommand(command)
            last_command = command

        if keyboard.is_pressed('q'):
            print("Exiting driver mode")
            return False

    time.sleep(0.05)  # 20Hz like a game loop
'''
# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------






# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------
MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        # Read and discard bytes until we see the first magic byte.
        b = _ser.read(1)
        if not b:
            return None          # timeout
        if b[0] != MAGIC_HI:
            continue

        # Read the second magic byte.
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            # Not the magic number; keep searching (don't skip the byte
            # we just read in case it is the first byte of another frame).
            continue

        # Magic matched; now read the TPacket body.
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        # Read and verify the checksum.
        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            # Checksum mismatch: corrupted packet, try to resync.
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form.

    The 'data' field carries an optional debug string from the Arduino.
    When non-empty, it is printed automatically so you can embed debug
    messages in any outgoing TPacket on the Arduino side (set pkt.data to
    a null-terminated string up to 31 characters before calling sendFrame).
    This works like Serial.print(), but sends output to the Pi terminal.
    """
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")
        elif cmd == RESP_COLOUR:
            pass
            # r_freq = pkt['params'][0]
            # g_freq = pkt['params'][1]
            # b_freq = pkt['params'][2]
            # colour = f"R:{r_freq} G:{g_freq} B:{b_freq}"
            # sys.stdout.write("\r" + colour + "      ")
            # sys.stdout.flush()
            #print(f"Colour Response: R: {r_freq} Hz, G: {g_freq} Hz, B: {b_freq} Hz")
        elif cmd == RESP_COLOUR_FOREVER:
            r_freq = pkt['params'][0]
            g_freq = pkt['params'][1]
            b_freq = pkt['params'][2]
            colour = f"Red: {r_freq}  Green: {g_freq}  Blue: {b_freq}"   # replace with sensor data from Pi
            sys.stdout.write("\r" + colour + "   ")
            sys.stdout.flush()
            time.sleep(0.1)

        elif cmd in (RESP_INCREASE_SPEED, RESP_DECREASE_SPEED):
            print(pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace'))
        else:
            # TODO (Activity 2): add an elif branch here to handle your color
            # response.  Display the three channel frequencies in Hz, e.g.:
            #   R: <params[0]> Hz, G: <params[1]> Hz, B: <params[2]> Hz
            print(f"Response: unknown command {cmd}")
        # Print the optional debug string from the data field.
        # On the Arduino side, fill pkt.data before calling sendFrame() to
        # send debug messages to this terminal (similar to Serial.print()).
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------


def handleColorCommand():
    """
    TODO (Activity 2): request a color reading from the Arduino and display it.

    Check the E-Stop state first; if stopped, refuse with a clear message.
    Otherwise, send your color command to the Arduino.
    """
    # TODO
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    else:
        print("Requesting Colour Reading")
        sendCommand(COMMAND_COLOUR)


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

# TODO (Activity 3): import the camera library provided (alex_camera.py).
import alex_camera
from alex_camera import captureGreyscaleFrame, renderGreyscaleFrame, cameraClose
_camera = alex_camera.cameraOpen()      # TODO (Activity 3): open the camera (cameraOpen()) before first use.
_frames_remaining = 15                  # frames remaining before further captures are refused


def handleCameraCommand():
    """
    TODO (Activity 3): capture and display a greyscale frame.

    Gate on E-Stop state and the remaining frame count.
    Use captureGreyscaleFrame() and renderGreyscaleFrame() from alex_camera.
    """
    global _frames_remaining
    if(isEstopActive()):
        print("Refused: E-Stop is active")
        return
    
    if (_frames_remaining == 0):
        print("Out of frames")
        return
    
    greyscale = captureGreyscaleFrame(_camera)
    renderGreyscaleFrame(greyscale)
    _frames_remaining -= 1
    print(f"picture frames remaining: {_frames_remaining}")


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

# TODO (Activity 4): import from lidar.alex_lidar and lidar_example_cli_plot
#   (lidar_example_cli_plot.py is in the same folder; alex_lidar.py is in lidar/).


def handleLidarCommand():
    """
    TODO (Activity 4): perform a single LIDAR scan and render it.

    Gate on E-Stop state, then use the LIDAR library to capture one scan
    and the CLI plot helpers to display it.
    """
    if(isEstopActive()):
        print("Refused: E-Stop is active")
        return
    

    lidar_example_cli_plot.plot_single_scan()

# ----------------------------------------------------------------
# ACTIVITY 5: Movement
# ----------------------------------------------------------------

def handleFrontCommand():
    global curr_direction
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    else:
        print("moving forward")
        if curr_move_speed < min_move_speed:
            handleSetSpeed(min_move_speed)
            sendCommand(COMMAND_FRONT)
            time.sleep(MOVE_STARTUP_DURATION)
            handleSetSpeed(curr_move_speed)
        else:
            sendCommand(COMMAND_FRONT)
        curr_direction = MOVING_FORWARD

def handleCCWCommand():
    global curr_direction
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    else:
        print("turning cck")
        if curr_turn_speed < min_turn_speed:
            handleSetTurnSpeed(min_turn_speed)
            sendCommand(TURNING_CCW)
            time.sleep(MOVE_STARTUP_DURATION)
            handleSetSpeed(curr_turn_speed)
        else:
            sendCommand(COMMAND_CCW)
        curr_direction = TURNING_CCW

def handleBackCommand():
    global curr_direction
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    else:
        print("moving backward")
        if curr_move_speed < min_move_speed:
            handleSetSpeed(min_move_speed)
            sendCommand(COMMAND_BACK)
            time.sleep(MOVE_STARTUP_DURATION)
            handleSetSpeed(curr_move_speed)
        else:
            sendCommand(COMMAND_BACK)
        curr_direction = MOVING_BACKWARD

def handleCWCommand():
    global curr_direction
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    else:
        print("Turning clockwise")
        if curr_turn_speed < min_turn_speed:
            handleSetTurnSpeed(min_turn_speed)
            sendCommand(COMMAND_CW)
            time.sleep(MOVE_STARTUP_DURATION)
            handleSetSpeed(curr_turn_speed)
        else:
            sendCommand(COMMAND_CW)
        curr_direction = TURNING_CW

def handleStopCommand(print_out=True):
    global curr_direction
    global is_force_stop_mode
    if is_force_stop_mode:
        print(f"bbb {curr_direction=}")
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    elif is_force_stop_mode and curr_direction in [MOVING_FORWARD, MOVING_BACKWARD]:
        print("force Stopping")
        
        print(f"aaa {curr_direction=}")
        if curr_direction == MOVING_FORWARD:
            handleBackCommand()
        else:
            handleFrontCommand()
        handleSetSpeed(force_stop_speed)
        time.sleep(force_stop_duration)
        handleSetSpeed(curr_move_speed)
        sendCommand(COMMAND_MOVE_STOP)
    else:
        print("Stopping")
        sendCommand(COMMAND_MOVE_STOP)
    curr_direction = MOVING_STOP

# ----------------------------------------------------------------
# ACTIVITY 6: Speed
# ----------------------------------------------------------------
def handleIncreaseCommand():
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    else:
        print("Increasing speed")
        sendCommand(COMMAND_INCREASE_SPEED)

def handleDecreaseCommand():
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    else:
        print("Decreasing speed")
        sendCommand(COMMAND_DECREASE_SPEED)

def handleSetSpeed(speed: int):
    if isEstopActive():
        print("Refused to change robot speed: E-Stop is active")
        return
    else:
        print(f"Changing robot speed to {speed=}")
        sendCommand(COMMAND_SET_SPEED, params=[speed] + [0] * 15)

def handleSetTurnSpeed(speed: int):
    if isEstopActive():
        print("Refused to change robot turn speed: E-Stop is active")
        return
    else:
        print(f"Changing robot turn speed to {speed=}")
        sendCommand(COMMAND_SET_TURN_SPEED, params=[speed] + [0] * 15)

        
def handleEstopCommand():
    if _estop_state == STATE_RUNNING:
            print("Sending E-Stop command...")
            sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
    else:
            print("sending release E-Stop command...")
            sendCommand(COMMAND_ESTOP_RELEASE, data=b'E-stop released, resume programme')
def enterDriverMode():
    global is_driver_mode
    global driver_mode_last_command
    global _driver_fd
    global _driver_old_settings

    if isEstopActive():
        print("E-stop active, refuse to enter driver mode.")
        return

    if is_driver_mode:
        return

    _driver_fd = sys.stdin.fileno()
    _driver_old_settings = termios.tcgetattr(_driver_fd)

    # cbreak mode lets us read single chars without waiting for Enter
    tty.setcbreak(_driver_fd)

    is_driver_mode = True
    driver_mode_last_command = COMMAND_MOVE_STOP
    print("Driver mode active (w/a/s/d to move, q to quit, e for estop)")


def exitDriverMode(print_message=True):
    global is_driver_mode
    global driver_mode_last_command
    global _driver_fd
    global _driver_old_settings

    # always stop robot when leaving driver mode
    if driver_mode_last_command != COMMAND_MOVE_STOP:
        sendCommand(COMMAND_MOVE_STOP)
        driver_mode_last_command = COMMAND_MOVE_STOP

    if _driver_fd is not None and _driver_old_settings is not None:
        termios.tcsetattr(_driver_fd, termios.TCSADRAIN, _driver_old_settings)

    _driver_fd = None
    _driver_old_settings = None
    is_driver_mode = False

    if print_message:
        print("\nExiting driver mode")


def handleDriverMode():
    global is_driver_mode
    global driver_mode_last_command

    if not is_driver_mode:
        return

    if isEstopActive():
        exitDriverMode(print_message=False)
        print("\nDriver mode cancelled because E-stop is active")
        return

    ready, _, _ = select.select([sys.stdin], [], [], DRIVER_MODE_STREAMING_THRESHOLD)

    if ready:
        ch = sys.stdin.read(1).lower()

        if ch == 'q':
            exitDriverMode()
            return

        elif ch == 'e':
            exitDriverMode(print_message=False)
            print("\nActivate e-stop")
            handleEstopCommand()
            return

        elif ch == 'w':
            command = COMMAND_FRONT
        elif ch == 's':
            command = COMMAND_BACK
        elif ch == 'a':
            command = COMMAND_CCW
        elif ch == 'd':
            command = COMMAND_CW
        else:
            command = COMMAND_MOVE_STOP
    else:
        # no repeated key within timeout -> stop
        command = COMMAND_MOVE_STOP

    if command != driver_mode_last_command:
        sendCommand(command)
        driver_mode_last_command = command  


# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

# User input -> action mapping:
#   e  send a software E-Stop command to the Arduino (pre-wired)
#   c  request color reading from the Arduino        (Activity 2 - implement yourself)
#   p  capture and display a camera frame            (Activity 3 - implement yourself)
#   l  perform a single LIDAR scan                   (Activity 4 - implement yourself)


def handleUserInput(line):
    """
    Dispatch a single line of user input.

    The 'e' case is pre-wired to send a software E-Stop command.
    TODO (Activities 2, 3 & 4): add 'c' (color), 'p' (camera) and 'l' (LIDAR).
    """
    global is_driver_mode
    global driver_mode_last_command
    global curr_move_speed
    global curr_turn_speed
    global is_jerk_mode
    global is_auto_stop_mode
    global auto_stop_armed
    global is_force_stop_mode
    global jerk_move_speed
    global jerk_move_duration
    global jerk_turn_speed
    global jerk_turn_duration
    line = line.strip().lower()
    if line == 'e':
        handleEstopCommand()
    # elif line == 'c':
    #     handleColorCommand()
    elif line == 'p':
        handleCameraCommand()
    elif line == 'l':
        handleLidarCommand()
    elif line == 'w':
        if is_jerk_mode:
            handleSetSpeed(jerk_move_speed)
            handleFrontCommand()
            time.sleep(jerk_move_duration)
            handleStopCommand()    
        else:         
            handleFrontCommand()
    elif line == 'a':       
        if is_jerk_mode:
            handleSetSpeed(jerk_turn_speed)
            handleCCWCommand()
            time.sleep(jerk_turn_duration)
            handleStopCommand()    
        else:         
            handleCCWCommand()
    elif line == 's':   
        if is_jerk_mode:
            handleSetSpeed(jerk_move_speed)
            handleBackCommand()
            time.sleep(jerk_move_duration)
            handleStopCommand()    
        else:         
            handleBackCommand()       
    elif line == 'd':               
        if is_jerk_mode:
            handleSetSpeed(jerk_turn_speed)
            handleCWCommand()
            time.sleep(jerk_turn_duration)
            handleStopCommand()    
        else:         
            handleCWCommand()

    elif line == 'j':
        is_jerk_mode = True
        print("Enabled jerk mode")
    elif line == 'qj':
        is_jerk_mode = False
        print("Disable jerk mode")
    elif line == 'x':
        auto_stop_armed = True
        is_auto_stop_mode = True
        print("started auto stop mode")
    elif line == 'qx':
        auto_stop_armed = False
        is_auto_stop_mode = False
        print("ended auto stop mode")
    
    elif line == '/' or line == '':               # Stop
       handleStopCommand()
    elif len(line) == 4 and line[0] == 'f':
        try:
            speed = int(line[1:])
            handleSetSpeed(100)
            handleFrontCommand()
            time.sleep(0.2)
            handleSetSpeed(speed)
        except ValueError:
            print("Command to set robot speed does not contain a valid int")

    # TODO Speed Controls:
    elif line == '+':
        handleIncreaseCommand()
    elif line == '-':
        handleDecreaseCommand()
    
    elif len(line) == 5 and line[0:2] == 'jm':
        try:
            new_speed = int(line[2:])
            jerk_move_speed = new_speed
            print(f"jerk_move_speed set to {jerk_move_speed}")
        except ValueError:
            print("Command to set jerk move speed does not contain a valid int")
    elif len(line) == 5 and line[0:2] == 'jt':
        try:
            new_speed = int(line[2:])
            jerk_turn_speed = new_speed
            print(f"jerk_turn_speed set to {jerk_turn_speed}")
        except ValueError:
            print("Command to set jerk turn speed does not contain a valid int")
    elif len(line) == 6 and line[0:2] == 'jm':
        try:
            new_duration = float(line[2:])
            jerk_move_duration = new_duration
            print(f"jerk_move_duration set to {jerk_move_duration}")
        except ValueError:
            print("Command to set jerk move duration does not contain a valid float")
    elif len(line) == 6 and line[0:2] == 'jt':
        try:
            new_duration = float(line[2:])
            jerk_turn_duration = new_duration
            print(f"jerk_turn_duration set to {jerk_turn_duration}")
        except ValueError:
            print("Command to set jerk turn duration does not contain a valid float")

    elif len(line) == 4 and line[0] == 'x':
        try:
            speed = int(line[1:])
            handleSetSpeed(speed)
            curr_move_speed = speed
        except ValueError:
            print("Command to set robot move speed does not contain a valid int")
    elif len(line) == 4 and line[0] == 'y':
        try:
            speed = int(line[1:])
            handleSetTurnSpeed(speed)
            curr_turn_speed = speed
        except ValueError:
            print("Command to set robot turn speed does not contain a valid int")
    elif line == 'r':
        enterDriverMode()
        print("entered driver stop mode")
    elif line == 'qr':
        exitDriverMode()
        print("exited driver stop mode")
    # elif line == 'f':
    #     is_force_stop_mode = True
    #     print("entered force stop mode")
    # elif line == 'qf':
    #     is_force_stop_mode = False
    #     print("exited force stop mode")
    elif line == 'i':
        s = f"""
Current Move Speed : {curr_move_speed}
Current Turn Speed : {curr_turn_speed}
Jerk Mode Enabled  : {is_jerk_mode}
Driver Mode Enabled : {is_driver_mode}
Auto stop mode enabled: {is_auto_stop_mode}
Auto stop armed : {auto_stop_armed}
Force stop mode enabled : {is_force_stop_mode}
"""
        print(s)
    elif line == 'i+':
        s = f"""
Move Speed Settings:
Current Move Speed : {curr_move_speed}
Min Move Speed     : {min_move_speed}
Jerk Move Speed    : {jerk_move_speed}
Jerk Move Duration : {jerk_move_duration}

Turn Speed Settings:
Current Turn Speed : {curr_turn_speed}
Min Turn Speed     : {min_turn_speed}
Jerk Turn Speed    : {jerk_turn_speed}
Jerk Turn Duration : {jerk_turn_duration}

Mode:
Jerk Mode Enabled  : {is_jerk_mode}
Driver Mode Enabled : {is_driver_mode}
Auto stop mode enabled: {is_auto_stop_mode}
Auto stop armed : {auto_stop_armed}
Force stop mode enabled : {is_force_stop_mode}
"""
        print(s)
    else:
        print(f"Unknown input: '{line}'.")
def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously receive packets from the Arduino
    and read typed user input from stdin without either blocking the other.
    """
    global is_auto_stop_mode
    global auto_stop_armed
    

    print("Sensor interface ready. Type e / c / p / l and press Enter.")
    print("Type r and press Enter to enter driver mode.")
    print("Press Ctrl+C to exit.\n")

    import os

    while True:
        # Handle incoming Arduino packets
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
                relay.onPacketReceived(packFrame(
                    pkt['packetType'],
                    pkt['command'],
                    pkt['data'],
                    pkt['params']
                ))

        # Auto-stop from SLAM collision detection
        if is_auto_stop_mode:
            if os.path.exists('/tmp/slam_collision') and auto_stop_armed:
                if not isEstopActive():
                    print("[AUTO-STOP] Obstacle too close — sending Stop")
                    handleStopCommand()
                    auto_stop_armed = False
            elif not os.path.exists('/tmp/slam_collision'):
                auto_stop_armed = True
                

        # Input handling
        if is_driver_mode:
            if isEstopActive():
                exitDriverMode(print_message=False)
                print("\nDriver mode cancelled because E-stop is active")
            else:
                handleDriverMode()
        else:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline().strip().lower()
                if line:
                    handleUserInput(line)

        # Other background checks
        relay.checkSecondTerminal(_ser)
        time.sleep(LOOP_SLEEP)

# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        # TODO (Activities 3 & 4): close the camera and disconnect the LIDAR here if you opened them.
        closeSerial()
        cameraClose(_camera)
        relay.shutdown()
        #lidarDisconnect(lidar)
        atexit.register(lambda: termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _driver_old_settings) if _driver_old_settings else None)

