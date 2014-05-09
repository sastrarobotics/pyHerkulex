#!/usr/bin/env python2.7

"""
This is a python library for interfacing the Herkulex range of smart 
servo motors manufactured by Dongbu Robotics.

The library was created by Achu Wilson (mailto:achu@sastrarobotics.com) 
for the internal projects of Sastra Robotics

This free software is distributed under the GNU General Public License.
See http://www.gnu.org/licenses/gpl.html for details.

For usage of this code for  commercial purposes contact Sastra Robotics 
India Pvt. Ltd. (mailto:contact@sastrarobotics.com)

""" 
import os
import sys
try:
    # PySerial Module
    import serial
except:
    print "This library requires the pySerial to be installed \
to use the Serial Port"


# Commands
EEP_WRITE_REQ = 0x01
EEP_READ_REQ  = 0x02
RAM_WRITE_REQ = 0x03
RAM_READ_REQ  = 0x04
I_JOG_REQ     = 0x05
S_JOG_REQ     = 0x06
STAT_REQ      = 0x07
ROLLBACK_REQ  = 0x08
REBOOT_REQ    = 0x09

EEP_WRITE_ACK = 0x41
EEP_READ_ACK  = 0x42
RAM_WRITE_ACK = 0x43
RAM_READ_ACK  = 0x44
I_JOG_ACK     = 0x45
S_JOG_ACK     = 0x46
STAT_ACK      = 0x47
ROLLBACK_ACK  = 0x48
REBOOT_ACK    = 0x49


#Addresses
MODEL_NO1_EEP                        = 0
MODEL_No2_EEP                        = 1
VERSION1_EEP                         = 2
VERSION2_EEP                         = 3
BAUD_RATE_EEP                        = 4
SERVO_ID_EEP                         = 6
SERVO_ID_RAM                         = 0
ACK_POLICY_EEP                       = 7
ACK_POLICY_RAM                       = 1
ALARM_LED_POLICY_EEP                 = 8
ALARM_LED_POLICY_RAM                 = 2
TORQUE_POLICY_EEP                    = 9
TORQUE_POLICY_RAM                    = 3
MAX_TEMP_EEP                         = 11
MAX_TEMP_RAM                         = 5
MIN_VOLTAGE_EEP                      = 12
MIN_VOLTAGE_RAM                      = 6
MAX_VOLTAGE_EEP                      = 13
MAX_VOLTAGE_RAM                      = 7
ACCELERATION_RATIO_EEP               = 14
ACCELERATION_RATIO_RAM               = 8
MAX_ACCELERATION_TIME_EEP            = 15
MAX_ACCELERATION_TIME_RAM            = 9
DEAD_ZONE_EEP                        = 16
DEAD_ZONE_RAM                        = 10
SATURATOR_OFFSET_EEP                 = 17 
SATURATOR_OFFSET_RAM                 = 11
SATURATOR_SLOPE_EEP                  = 18
SATURATOR_SLOPE_RAM                  = 12
PWM_OFFSET_EEP                       = 20
PWM_OFFSET_RAM                       = 14
MIN_PWM_EEP                          = 21
MIN_PWM_RAM                          = 15
MAX_PWM_EEP                          = 22
MAX_PWM_RAM                          = 16
OVERLOAD_PWM_THRESHOLD_EEP           = 24
OVERLOAD_PWM_THRESHOLD_RAM           = 18
MIN_POSITION_EEP                     = 26
MIN_POSITION_RAM                     = 20
MAX_POSITION_EEP                     = 28
MAX_POSITION_RAM                     = 22
POSITION_KP_EEP                      = 30
POSITION_KP_RAM                      = 24
POSITION_KD_EEP                      = 32
POSITION_KD_RAM                      = 26
POSITION_KI_EEP                      = 34
POSITION_KI_RAM                      = 28
POSITION_FEEDFORWARD_GAIN1_EEP       = 36
POSITION_FEEDFORWARD_GAIN1_RAM       = 30
POSITION_FEEDFORWARD_GAIN2_EEP       = 38
POSITION_FEEDFORWARD_GAIN2_RAM       = 32
VELOCITY_KP_EEP                      = 40
VELOCITY_KP_RAM                      = 34
VELOCITY_KI_EEP                      = 42
VELOCITY_KI_RAM                      = 36
LED_BLINK_PERIOD_EEP                 = 44
LED_BLINK_PERIOD_RAM                 = 38
ADC_FAULT_CHECK_PERIOD_EEP           = 45
ADC_FAULT_CHECK_PERIOD_RAM           = 39
PACKET_GARBAGE_CHECK_PERIOD_EEP      = 46
PACKET_GARBAGE_CHECK_PERIOD_RAM      = 40
STOP_DETECTION_PERIOD_EEP            = 47
STOP_DETECTION_PERIOD_RAM            = 41
OVERLOAD_DETECTION_PERIOD_EEP        = 48
OVERLOAD_DETECTION_PERIOD_RAM        = 42
STOP_THRESHOLD_EEP                   = 49
STOP_THRESHOLD_RAM                   = 43
INPOSITION_MARGIN_EEP                = 50
INPOSITION_MARGIN_RAM                = 44
CALIBRATION_DIFF_LOW_EEP             = 52
CALIBRATION_DIFF_LOW_RAM             = 46
CALIBRATION_DIFF_UP_EEP              = 53
CALIBRATION_DIFF_UP_RAM              = 47
STATUS_ERROR_RAM                     = 48
STATUS_DETAIL_RAM                    = 49
AUX1_RAM                             = 50
TORQUE_CONTROL_RAM                   = 52
LED_CONTROL_RAM                      = 53
VOLTAGE_RAM                          = 54
TEMPERATURE_RAM                      = 55
CURRENT_CONTROL_MODE_RAM             = 56
TICK_RAM                             = 57
CALIBRATED_POSITION_RAM              = 58
ABSOLUTE_POSITION_RAM                = 60
DIFFERENTIAL_POSITION_RAM            = 62
PWM_RAM                              = 64
ABSOLUTE_SECOND_POSITION_RAM         = 66
ABSOLUTE_GOAL_POSITION_RAM           = 68
ABSOLUTE_DESIRED_TRAJECTORY_POSITION = 70
DESIRED_VELOCITY_RAM                 = 72

BYTE1 = 0x01
BYTE2 = 0x02

BROADCAST_ID = 0xFE



#connect  serial port
def connect(portname,baudrate):
    global serport
    try:
        serport = serial.Serial(portname, baudrate)

    except:
        print " check serial port & permissions"

#properly close a serial port
def close():
    global serport
    try:
        serport.close()
    except:
        print " Error!.. Failed closing serial port"

#checksum 1 function
def checksum1(data,stringlength):
    buffer = 0
    for x in range(0, stringlength):
        buffer = buffer ^ data[x]
        
    return buffer&0xFE;

def checksum2(data):
    return (~data)&0xFE


#packetize the data & send to serial port
def send_data(data):
    datalength = len(data)
    csm1 = checksum1(data,datalength)
    csm2 = checksum2(csm1)
    data.insert(0,0xFF)
    data.insert(1,0xFF)
    data.insert(5,csm1)
    data.insert(6,csm2)
    stringtosend = ""
    for i in range(len(data)):
        byteformat = '%02X' % data[i]
        stringtosend = stringtosend + "\\x" + byteformat
    try:
        serport.write(stringtosend.decode('string-escape'))
    except:
        print "could not write to serial port"


#set LED color  0x00-Off, 0x01-GREEN, 0x02-BLUE, 0x03-CYAN, 0x04-RED, 0x05- green-orange ,0x06-Violet, 0x07 ALL
def  set_led(servoid,colorcode):
    data = []
    data.append(0x0A)
    data.append(servoid)
    data.append(RAM_WRITE_REQ)
    data.append(LED_CONTROL_RAM)
    data.append(0x01)
    data.append(colorcode)
    send_data(data)
	
#set the brake of the servos specified by servoid
def brake_on(servoid):
    data = []
    data.append(0x0A)
    data.append(servoid)
    data.append(RAM_WRITE_REQ)
    data.append(TORQUE_CONTROL_RAM)
    data.append(0x01)
    data.append(0x40)
    send_data(data)

#set the torque to zero of the servos specified by servoid
def torque_off(servoid):
    data = []
    data.append(0x0A)
    data.append(servoid)
    data.append(RAM_WRITE_REQ)
    data.append(TORQUE_CONTROL_RAM)
    data.append(0x01)
    data.append(0x00)
    send_data(data)


#activates the torque of the servos specified by servoid
def torque_on(servoid):
    data = []
    data.append(0x0A)
    data.append(servoid)
    data.append(RAM_WRITE_REQ)
    data.append(TORQUE_CONTROL_RAM)
    data.append(0x01)
    data.append(0x60)
    send_data(data)

# set the servo position. moves from current position to 
# goalposition in time goaltime. LED options are:
# 0x00 LED off
# 0x04 GREEN
# 0x08 BLUE
# 0x10 RED
def set_servo_position(servoid,goalposition,goaltime,led):
    goalposition_msb = int(goalposition) >> 8
    goalposition_lsb = int(goalposition) & 0xff

    data = []
    data.append(0x0C)
    data.append(servoid)
    data.append(I_JOG_REQ)
    data.append(goalposition_lsb)
    data.append(goalposition_msb)
    data.append(led)
    data.append(servoid)
    data.append(goaltime)
    send_data(data)


def get_servo_position(servoid):   
    data = []
    data.append(0x09)
    data.append(servoid)
    data.append(RAM_READ_REQ)
    data.append(CALIBRATED_POSITION_RAM)
    data.append(BYTE2)
    send_data(data)
    rxdata = []
    try:
        rxdata = serport.read(13)
    except:
        print "Could not read from the servos. Check connection"
    return ((ord(rxdata[10])&0x03)<<8) | (ord(rxdata[9])&0xFF);

def get_servo_temperature(servoid):
    data = []
    data.append(0x09)
    data.append(servoid)
    data.append(RAM_READ_REQ)
    data.append(TEMPERATURE_RAM)
    data.append(BYTE2)
    send_data(data)
    rxdata = []
    try:
        rxdata = serport.read(13)
    except:
        print "Could not read from the servos. Check connection"
    return ord(rxdata[9])

def clear_errors():
    data = []
    data.append(0x0B)
    data.append(BROADCAST_ID)
    data.append(RAM_WRITE_REQ)
    data.append(STATUS_ERROR_RAM)
    data.append(BYTE2)
    data.append(0x00)
    data.append(0x00)
    send_data(data)

def get_servo_torque(servoid):
    data = []
    data.append(0x09)
    data.append(servoid)
    data.append(RAM_READ_REQ)
    data.append(PWM_RAM)
    data.append(BYTE2)
    send_data(data)
    rxdata = []
    try:
        rxdata = serport.read(13)
    except:
        print "Could not read from the servos. Please check the connections"
    if(ord(rxdata[10])<=127):
        return ((ord(rxdata[10])&0x03)<<8) | (ord(rxdata[9])&0xFF);
    else:
        return (ord(rxdata[10])-0xFF)*0xFF + (ord(rxdata[9])&0xFF)-0xFF

def set_servo_speed(servoid,goalspeed,led):
    if(goalspeed>0):
        goalspeed_msb = (int(goalspeed)& 0xFF00) >> 8
        goalspeed_lsb = int(goalspeed) & 0xff
    elif(goalspeed<0):
        goalspeed_msb = 64+(255- ((int(goalspeed)& 0xFF00) >> 8))
        goalspeed_lsb = (abs(goalspeed) & 0xff)
        
    print goalspeed_msb,goalspeed_lsb
    data = []
    data.append(0x0C)
    data.append(servoid)
    data.append(I_JOG_REQ)
    data.append(goalspeed_lsb)
    data.append(goalspeed_msb)
    data.append(0x0A)
    data.append(servoid)
    data.append(0x00)    
    send_data(data)


