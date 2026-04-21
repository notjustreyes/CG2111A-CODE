
def handleArmBaseCommand(angle: int):
    if isEstopActive():
        print("Refused to move arm base: E-Stop is active")
        return
    else:
        print(f"Moving arm base to {angle=}")
        sendCommand(COMMAND_ARM_BASE, params=[angle] + [0] * 15)

def handleArmShoulderCommand(angle: int):
    if isEstopActive():
        print("Refused to move arm shoulder: E-Stop is active")
        return
    else:
        print(f"Moving arm shoulder to {angle=}")
        sendCommand(COMMAND_ARM_SHOULDER, params=[angle] + [0] + [0] * 15)

def handleArmElbowCommand(angle: int):
    if isEstopActive():
        print("Refused to move arm elbow: E-Stop is active")
        return
    else:
        print(f"Moving arm elbow to {angle=}")
        sendCommand(COMMAND_ARM_ELBOW, params=[angle] + [1] + [0] * 14)

def handleArmGripperCommand(angle: int):
    if isEstopActive():
        print("Refused to move arm gripper: E-Stop is active")
        return
    else:
        print(f"Moving arm gripper to {angle=}")
        sendCommand(COMMAND_ARM_GRIPPER, params=[angle] + [0] * 15)

def handleArmSpeedCommand(angle: int):
    if isEstopActive():
        print("Refused to change arm speed: E-Stop is active")
        return
    else:
        print(f"Moving arm gripper to {angle=}")
        sendCommand(COMMAND_ARM_GRIPPER, params=[angle] + [0] * 15)

        
"""
def handleWASDControl2():
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
                sendCommand(COMMAND_MOVE_STOP)
                return False

            # wait up to 0.15s for input
            ready, _, _ = select.select([sys.stdin], [], [], STREAMING_THRESHOLD)

            if ready:
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
            else:
                # no input within timeout → stop movement
                command = COMMAND_MOVE_STOP

            if command != last_command:
                sendCommand(command)
                last_command = command

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
"""

    # elif len(line) == 4 and line[0] in ['b', 's', 'e', 'g', 'v']:
    #     try:
    #         angle = int(line[1:])
    #         c = line[0]
    #         if c == 'b':
    #             handleArmBaseCommand(angle)
    #         elif c == 's':
    #             handleArmShoulderCommand(angle)
    #         elif c == 'e':
    #             handleArmElbowCommand(angle)
    #         elif c == 'g':
    #             handleArmGripperCommand(angle)
    #         elif c == 'v':
    #             handleArmSpeedCommand(angle)
    #         else:
    #             assert(f"input reached invalid branch, expected first char to be in ['B', 'S', 'E', 'G', 'V'], got {c}")
    #     except ValueError:
    #         print("Command for robotic arm does not contain a valid int")

    def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously receive packets from the Arduino
    and read typed user input from stdin without either blocking the other.
    """

    print("Sensor interface ready. Type e / c / p / l and press Enter.")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
                relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'],
                                 pkt['data'], pkt['params']))

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(LOOP_SLEEP)
                continue
            handleUserInput(line)
        relay.checkSecondTerminal(_ser)
        time.sleep(LOOP_SLEEP)
        if is_driver_mode and not isEstopActive():
            handleDriverMode()

            def handleDriverMode2():
    global driver_mode_last_command
    global is_driver_mode
    try:
        tty.setraw(fd)  # enter raw mode once
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        ready, _, _ = select.select([sys.stdin], [], [], DRIVER_MODE_STREAMING_THRESHOLD)
        if ready:
            ch = sys.stdin.read(1).lower()
            
            if ch == 'q':
                is_driver_mode = False
                sendCommand(COMMAND_MOVE_STOP)
                print("\nExiting driver mode")
                return
            elif ch == 'e':
                is_driver_mode = False
                sendCommand(COMMAND_MOVE_STOP)
                print("Activate e-stop")
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
            # no input within timeout → stop movement
            command = COMMAND_MOVE_STOP

        if command != driver_mode_last_command:
            sendCommand(command)
            driver_mode_last_command = command
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        def runCommandInterface():
    print("Sensor interface ready. Type e / c / p / l and press Enter.")
    print("Type r and press Enter to enter driver mode.")
    print("Press Ctrl+C to exit.\n")

    while True:
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

        if is_driver_mode and not isEstopActive():
            handleDriverMode()
        elif is_driver_mode and isEstopActive():
            exitDriverMode(print_message=False)
            print("\nDriver mode cancelled because E-stop is active")
        else:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                line = sys.stdin.readline().strip().lower()
                if line:
                    handleUserInput(line)

        relay.checkSecondTerminal(_ser)
        time.sleep(LOOP_SLEEP)