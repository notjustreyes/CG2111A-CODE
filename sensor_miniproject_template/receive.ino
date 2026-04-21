#include "arm.h"
#include "movement.h"
#include "packets.h"
#include "serial_driver.h"
#include <stdio.h>
#define SENSOR_OUT PL2
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            move_state = MOVE_STOP;
            break;

        case COMMAND_COLOUR:
            {
                TPacket resp;
                memset(&resp, 0, sizeof(resp));
                readColourChannels(&resp.params[0], &resp.params[1], &resp.params[2]);
                resp.packetType = PACKET_TYPE_RESPONSE;
                resp.command = RESP_COLOUR;
                sendFrame(&resp);
                break;
            }
        case COMMAND_ESTOP_RELEASE:
            {
            cli();
            buttonState = STATE_RUNNING;
            stateChanged = false;
            sei();
            {
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_STATUS;
                strncpy(pkt.data, "E-stop button released", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            break;
            }
        // directions: case-by-case
        case COMMAND_FRONT: {
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            strncpy(resp.data, "Moving Forward", sizeof(resp.data) - 1);
            resp.data[sizeof(resp.data) - 1] = '\0';
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            move_state = MOVE_FRONT; 
            break;
        }
        case COMMAND_BACK: {
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            strncpy(resp.data, "Moving Backwards", sizeof(resp.data) - 1);
            resp.data[sizeof(resp.data) - 1] = '\0';
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            move_state = MOVE_BACK;
            break;
        }
        case COMMAND_CW: {
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            strncpy(resp.data, "Turning Clockwise", sizeof(resp.data) - 1);
            resp.data[sizeof(resp.data) - 1] = '\0';
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            move_state = MOVE_CW;
            break;
        }

        case COMMAND_CCW: {
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            strncpy(resp.data, "Turning Anti-Clockwise", sizeof(resp.data) - 1);
            resp.data[sizeof(resp.data) - 1] = '\0';
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            move_state = MOVE_CCW;
            break;
        }
        case COMMAND_MOVE_STOP: {
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            strncpy(resp.data, "Stopping", sizeof(resp.data) - 1);
            resp.data[sizeof(resp.data) - 1] = '\0';
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            move_state = MOVE_STOP;
            break;
        }
        case COMMAND_INCREASE_SPEED: {
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            snprintf(resp.data, sizeof(resp.data), "Increasing speed to %d", get_new_speed(change_speed_state));
            // strncpy(resp.data, "Increasing speed", sizeof(resp.data) - 1);
            // resp.data[sizeof(resp.data) - 1] = '\0';
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_INCREASE_SPEED;
            sendFrame(&resp);
            change_speed_state = INCREASE_SPEED;
            break;
        }
        case COMMAND_DECREASE_SPEED: {
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            snprintf(resp.data, sizeof(resp.data), "Decreasing speed to %d", get_new_speed(change_speed_state));;
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_DECREASE_SPEED;
            sendFrame(&resp);
            change_speed_state = DECREASE_SPEED;
            break;
        }
        case COMMAND_ARM_ANGLE: {
            ServoType servoType = cmd->params[0];
            int angle = cmd->params[1];
            changeServoAngle(angle, servoType);
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            snprintf(resp.data, sizeof(resp.data),
                    "Change arm %s angle to %d",
                    servoTypeToString(servoType),
                    angle);
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            break;
        }
        case COMMAND_ARM_HOME: {
            homeNoGripper();
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            snprintf(resp.data, sizeof(resp.data), "Home servos");
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            break;
        }
        case COMMAND_ARM_HOME_ALL: {
            homeAll();
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            snprintf(resp.data, sizeof(resp.data), "Home ALL servos");
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            break;
        }
        case COMMAND_SET_SPEED: {
            int speed = cmd->params[0];
            set_robot_speed(speed);
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            snprintf(resp.data, sizeof(resp.data),
                    "Set robot move speed to %d",
                    speed);
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            break;
        }
        case COMMAND_SET_TURN_SPEED: {
            int turn_speed = cmd->params[0];
            set_robot_turn_speed(turn_speed);
            TPacket resp;
            memset(&resp, 0, sizeof(resp));
            snprintf(resp.data, sizeof(resp.data),
                    "Set robot turn speed to %d",
                    turn_speed);
            resp.packetType = PACKET_TYPE_RESPONSE;
            resp.command = RESP_OK;
            sendFrame(&resp);
            break;
        }
        

    }
    
}
