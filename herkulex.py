#!/usr/bin/env python2.7

"""
@package: pyHerkulex
@name: herkulex.py
@author: Achu Wilson (achuwilson@gmail.com), Akhil Chandran  (akhilchandran.t.r@gmail.com)
@version: 0.1

This is a python library for interfacing the Herkulex range of smart 
servo motors manufactured by Dongbu Robotics.

The library was created by Achu Wilson (mailto:achu@sastrarobotics.com) 
for the internal projects of Sastra Robotics

This free software is distributed under the GNU General Public License.
See http://www.gnu.org/licenses/gpl.html for details.

For usage of this code for  commercial purposes contact Sastra Robotics 
India Pvt. Ltd. (mailto:contact@sastrarobotics.com)


""" 
import time
try:
    # PySerial Module
    import serial
except:
    raise ImportError("couldnt find pySerial")


# Commands
EEP_WRITE_REQ = 0x01
EEP_READ_REQ = 0x02
RAM_WRITE_REQ = 0x03
RAM_READ_REQ = 0x04
I_JOG_REQ = 0x05
S_JOG_REQ = 0x06
STAT_REQ = 0x07
ROLLBACK_REQ = 0x08
REBOOT_REQ = 0x09

EEP_WRITE_ACK = 0x41
EEP_READ_ACK = 0x42
RAM_WRITE_ACK = 0x43
RAM_READ_ACK = 0x44
I_JOG_ACK = 0x45
S_JOG_ACK = 0x46
STAT_ACK = 0x47
ROLLBACK_ACK = 0x48
REBOOT_ACK = 0x49


#Addresses
MODEL_NO1_EEP = 0
MODEL_NO2_EEP = 1
VERSION1_EEP = 2
VERSION2_EEP = 3
BAUD_RATE_EEP = 4
SERVO_ID_EEP = 6
SERVO_ID_RAM = 0
ACK_POLICY_EEP = 7
ACK_POLICY_RAM = 1
ALARM_LED_POLICY_EEP = 8
ALARM_LED_POLICY_RAM = 2
TORQUE_POLICY_EEP = 9
TORQUE_POLICY_RAM = 3
MAX_TEMP_EEP = 11
MAX_TEMP_RAM = 5
MIN_VOLTAGE_EEP = 12
MIN_VOLTAGE_RAM = 6
MAX_VOLTAGE_EEP = 13
MAX_VOLTAGE_RAM = 7
ACCELERATION_RATIO_EEP = 14
ACCELERATION_RATIO_RAM = 8
MAX_ACCELERATION_TIME_EEP = 15
MAX_ACCELERATION_TIME_RAM = 9
DEAD_ZONE_EEP = 16
DEAD_ZONE_RAM = 10
SATURATOR_OFFSET_EEP = 17
SATURATOR_OFFSET_RAM = 11
SATURATOR_SLOPE_EEP = 18
SATURATOR_SLOPE_RAM = 12
PWM_OFFSET_EEP = 20
PWM_OFFSET_RAM = 14
MIN_PWM_EEP = 21
MIN_PWM_RAM = 15
MAX_PWM_EEP = 22
MAX_PWM_RAM = 16
OVERLOAD_PWM_THRESHOLD_EEP = 24
OVERLOAD_PWM_THRESHOLD_RAM = 18
MIN_POSITION_EEP = 26
MIN_POSITION_RAM = 20
MAX_POSITION_EEP = 28
MAX_POSITION_RAM = 22
POSITION_KP_EEP = 30
POSITION_KP_RAM = 24
POSITION_KD_EEP = 32
POSITION_KD_RAM = 26
POSITION_KI_EEP = 34
POSITION_KI_RAM =28
POSITION_FEEDFORWARD_GAIN1_EEP = 36
POSITION_FEEDFORWARD_GAIN1_RAM = 30
POSITION_FEEDFORWARD_GAIN2_EEP = 38
POSITION_FEEDFORWARD_GAIN2_RAM = 32
VELOCITY_KP_EEP = 40
VELOCITY_KP_RAM = 34
VELOCITY_KI_EEP = 42
VELOCITY_KI_RAM = 36
LED_BLINK_PERIOD_EEP = 44
LED_BLINK_PERIOD_RAM = 38
ADC_FAULT_CHECK_PERIOD_EEP = 45
ADC_FAULT_CHECK_PERIOD_RAM = 39
PACKET_GARBAGE_CHECK_PERIOD_EEP = 46
PACKET_GARBAGE_CHECK_PERIOD_RAM = 40
STOP_DETECTION_PERIOD_EEP = 47
STOP_DETECTION_PERIOD_RAM = 41
OVERLOAD_DETECTION_PERIOD_EEP        = 48
OVERLOAD_DETECTION_PERIOD_RAM        = 42
STOP_THRESHOLD_EEP = 49
STOP_THRESHOLD_RAM = 43
INPOSITION_MARGIN_EEP = 50
INPOSITION_MARGIN_RAM = 44
CALIBRATION_DIFF_LOW_EEP = 52
CALIBRATION_DIFF_LOW_RAM = 46
CALIBRATION_DIFF_UP_EEP = 53
CALIBRATION_DIFF_UP_RAM = 47
STATUS_ERROR_RAM = 48
STATUS_DETAIL_RAM = 49
AUX1_RAM = 50
TORQUE_CONTROL_RAM = 52
LED_CONTROL_RAM = 53
VOLTAGE_RAM = 54
TEMPERATURE_RAM = 55
CURRENT_CONTROL_MODE_RAM = 56
TICK_RAM = 57
CALIBRATED_POSITION_RAM = 58
ABSOLUTE_POSITION_RAM = 60
DIFFERENTIAL_POSITION_RAM = 62
PWM_RAM = 64
ABSOLUTE_SECOND_POSITION_RAM = 66
ABSOLUTE_GOAL_POSITION_RAM = 68
ABSOLUTE_DESIRED_TRAJECTORY_POSITION = 70
DESIRED_VELOCITY_RAM = 72

BYTE1 = 0x01
BYTE2 = 0x02

BROADCAST_ID = 0xFE

SERPORT = None

def connect(portname, baudrate):
    """ Connect to the Herkulex bus

    Connect to serial port to which Herkulex Servos are attatched

    Args:
        portname (str): The serial port name
        baudrate (int): The serial port baudrate
    Raises:
        SerialException: Error occured while opening serial port
    """
    global SERPORT
    try:
        SERPORT = serial.Serial(portname, baudrate, timeout = 0.1)

    except:
        raise HerkulexError("could not open the serial port")

def close():
    """ Close the Serial port

    Properly close the serial port before exiting the application

    Raises:
        SerialException: Error occured while closing serial port
    """
    try:
        SERPORT.close()
    except:
        raise HerkulexError("could not close the serial port")


def checksum1(data, stringlength):
    """ Calculate Checksum 1

    Calculate the ckecksum 1 required for the herkulex data packet

    Args:
        data (list): the data of which checksum is to be calculated
        stringlength (int): the length of the data

    Returns:
        int:  The calculated checksum 1
    """
    value_buffer = 0
    for count in range(0, stringlength):
        value_buffer = value_buffer ^ data[count]
    return value_buffer&0xFE

def checksum2(data):
    """ Calculate Checksum 2

    Calculate the ckecksum 2 required for the herkulex data packet

    Args:
        data (int): the data of which checksum is to be calculated

    Returns:
        int:  The calculated checksum 2
    """
    return (~data)&0xFE



def send_data(data):
    """ Send data to herkulex

    Paketize & write the packet to serial port

    Args:
        data (list): the data to be sent

    Raises:
        SerialException: Error occured while opening serial port
    """
    datalength = len(data)
    csm1 = checksum1(data, datalength)
    csm2 = checksum2(csm1)
    data.insert(0, 0xFF)
    data.insert(1, 0xFF)
    data.insert(5, csm1)
    data.insert(6, csm2)
    stringtosend = ""
    for i in range(len(data)):
        byteformat = '%02X' % data[i]
        stringtosend = stringtosend + "\\x" + byteformat

    try:

        SERPORT.write(stringtosend.decode('string-escape'))
        #print stringtosend

    except:
        raise HerkulexError("could not communicate with motors")


def clear_errors():
    """ Clears the errors register of all Herkulex servos

    Args:
        none

    """
    data = []
    data.append(0x0B)
    data.append(BROADCAST_ID)
    data.append(RAM_WRITE_REQ)
    data.append(STATUS_ERROR_RAM)
    data.append(BYTE2)
    data.append(0x00)
    data.append(0x00)
    send_data(data)

def scale(input_value, input_min, input_max, out_min, out_max):
    """ scale a value from one range to another
    """
    # Figure out how 'wide' each range is
    input_span = input_max - input_min
    output_span = out_max - out_min
    # Convert the left range into a 0-1 range (float)
    valuescaled = float(input_value - input_min) / float(input_span)
    # Convert the 0-1 range into a value in the right range.
    return out_min + (valuescaled * output_span)

def scan_servos():

    """Scan for the herkulex servos connected

	This function will scan for all the herkulex servos connected
	to the bus.

	Args:
	    none
	Returns:
	    list: a list of tuples of the form [(id, model)]
	"""
    servos = []
    for servo_id in range(0x00, 0xFE):
        model = get_model(servo_id)
        if model:
            servos += [(servo_id, model)]
    return servos

def get_model(servoid):
    """ Get the servo model

    This function gets the model of the herkules servo, provided its id

    Args:
        servoid(int): the id of the servo

    Returns:
        int:  an integer corresponding to the model number
              0x06 for DRS-602
              0x04 for DRS-402
              0x02 for DRS-202
    """
    data = []
    data.append(0x09)
    data.append(servoid)
    data.append(EEP_READ_REQ)
    data.append(MODEL_NO1_EEP)
    data.append(BYTE1)
    send_data(data)
    rxdata = []
    try:
        rxdata = SERPORT.read(12)
        return ord(rxdata[9])&0xFF
    except:
        raise HerkulexError("could not communicate with motors")

class servo:
    """ The servo class

    This class handles the interface to the herkulex smart servos

    """


    def __init__(self, servoid):
        """ servo class initialization

   	Args:
   	    servoid(int): the id of the servo
   	"""
        self.servoid = servoid


        self.servomodel = get_model(servoid):



    def get_model(self):
        """ Get the servo model

        This function gets the model of the herkules servo, provided its id

        Args:
            none

        Returns:
            int:  an integer corresponding to the model number
                  0x06 for DRS-602
                  0x04 for DRS-402
                  0x02 for DRS-202
        """





        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(EEP_READ_REQ)
        data.append(MODEL_NO1_EEP)
        data.append(BYTE1)
        send_data(data)

        rxdata = []
        try:
            rxdata = SERPORT.read(12)
            return ord(rxdata[9])&0xFF
        except:
            raise HerkulexError("could not communicate with motors")


    def get_servo_status(self):
        """ Get the error status of servo

        This function gets the  error status (if any) of the servo

        Args:
            none

        Returns:
            int:  an integer corresponding to the servo status
                   * refer datasheet

        """
        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(STATUS_ERROR_RAM)
        data.append(BYTE1)
        send_data(data)

        rxdata = []
        try:
            rxdata = SERPORT.read(12)
            return ord(rxdata[9])&0xFF
        except:
            raise HerkulexError("could not communicate with motors")

    def get_servo_status_detail(self):
        """ Get the  detailed error status of servo

        This function gets the  detailed error status (if any) of the servo

        Args:
            none

        Returns:
            int:  an integer corresponding to the servo status
                   * refer datasheet

        """

        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(STATUS_DETAIL_RAM)
        data.append(BYTE1)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(12)
            return ord(rxdata[9])&0xFF
        except HerkulexError:
            raise HerkulexError("could not communicate with motors")

    def  set_led(self, colorcode):
        """ Set the LED Color of Herkulex

        Args:
            colorcode (int): The code for colors
                            (0x00-OFF
                             0x02-BLUE
                             0x03-CYAN
                             0x04-RED
                             0x05-ORANGE
                             0x06-VIOLET
                             0x07-WHITE
        """
        data = []
        data.append(0x0A)
        data.append(self.servoid)
        data.append(RAM_WRITE_REQ)
        data.append(LED_CONTROL_RAM)
        data.append(0x01)
        data.append(colorcode)
        send_data(data)

    def brake_on(self):
        """ Set the Brakes of Herkulex

        In braked mode, position control and velocity control
        will not work, enable torque before that

        Args:
            none
        """
        data = []
        data.append(0x0A)
        data.append(self.servoid)
        data.append(RAM_WRITE_REQ)
        data.append(TORQUE_CONTROL_RAM)
        data.append(0x01)
        data.append(0x40)
        send_data(data)

    def torque_off(self):
        """ Set the torques of Herkulex to zero

        In this mode, position control and velocity control
        will not work, enable torque before that. Also the
        servo shaft is freely movable

        Args:
            none
        """
        data = []
        data.append(0x0A)
        data.append(self.servoid)
        data.append(RAM_WRITE_REQ)
        data.append(TORQUE_CONTROL_RAM)
        data.append(0x01)
        data.append(0x00)
        send_data(data)

    def torque_on(self):
        """ Enable the torques of Herkulex

        In this mode, position control and velocity control
        will work.

        Args:
            none
        """
        data = []
        data.append(0x0A)
        data.append(self.servoid)
        data.append(RAM_WRITE_REQ)
        data.append(TORQUE_CONTROL_RAM)
        data.append(0x01)
        data.append(0x60)
        send_data(data)

    def get_torque_state(self):
        """ get the torque state of motor

        Returns:
            bool: True if torque is enabled, else False
        """
        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(TORQUE_CONTROL_RAM)
        data.append(BYTE2)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(13)
            return bool(ord(rxdata[9]))
        except HerkulexError:
            raise HerkulexError("could not communicate with motors")

    def set_servo_position(self, goalposition, goaltime, led):
        """ Set the position of Herkulex

        Enable torque using torque_on function before calling this

        Args:

            goalposition (int): The desired position, min-0 & max-1023
            goaltime (int): the time taken to move from present
             position to goalposition
            led (int): the LED color
                       0x00 LED off
                       0x04 GREEN
                       0x08 BLUE
                       0x10 RED
        """
        goalposition_msb = int(goalposition) >> 8
        goalposition_lsb = int(goalposition) & 0xff

        data = []
        data.append(0x0C)
        data.append(self.servoid)
        data.append(I_JOG_REQ)
        data.append(goalposition_lsb)
        data.append(goalposition_msb)
        data.append(led)
        data.append(self.servoid)
        data.append(goaltime)
        send_data(data)

    def get_servo_position(self):
        """ Gets the current position of Herkulex

        Args:
            none

        Returns:
            int: position of the servo- 0 to 1023

        Raises:
            SerialException: Error occured while opening serial port

        """
        #global SERPORT

        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(CALIBRATED_POSITION_RAM)
        data.append(BYTE2)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(13)
            if (self.servomodel==0x06) or (self.servomodel == 0x04):
                return ((ord(rxdata[10])&0xff)<<8) | (ord(rxdata[9])&0xFF)
            else:
                #print ord(rxdata[9]),ord(rxdata[10])
                return ((ord(rxdata[10])&0x03)<<8) | (ord(rxdata[9])&0xFF)

        except HerkulexError:
            print "Could not read from the servos. Check connection"

    def get_servo_temperature(self):
        """ Gets the current temperature of Herkulex

        Args:
            none

        Returns:
            int: the current temperature register of Herkulex

        Raises:
            SerialException: Error occured while opening serial port

       """
        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(TEMPERATURE_RAM)
        data.append(BYTE2)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(13)
            return ord(rxdata[9])
        except HerkulexError:
            raise HerkulexError("Could not communicate with motors")

    def get_servo_torque(self):
        """ Gets the current torque of Herkulex

        Gives the current load on the servo shaft.
        It is actually the PWM value to the motors

        Args:
            none

        Returns:
            int: the torque on servo shaft. range from -1023 to 1023

        Raises:
            SerialException: Error occured while opening serial port

        """
        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(PWM_RAM)
        data.append(BYTE2)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(13)
            if ord(rxdata[10])<=127:
                return ((ord(rxdata[10])&0x03)<<8) | (ord(rxdata[9])&0xFF)
            else:
                return (ord(rxdata[10])-0xFF)*0xFF + (ord(rxdata[9])&0xFF)-0xFF
        except HerkulexError:
            raise HerkulexError("could not communicate with motors")

    def set_servo_speed(self, goalspeed, led):
        """ Set the Herkulex in continuous rotation mode

        Args:

            goalspeed (int): the speed , range -1023 to 1023
            led (int): the LED color
                       0x00 LED off
                       0x04 GREEN
                       0x08 BLUE
                       0x10 RED

        """
        if goalspeed>0 :
            goalspeed_msb = (int(goalspeed)& 0xFF00) >> 8
            goalspeed_lsb = int(goalspeed) & 0xff
        elif goalspeed<0 :
            goalspeed_msb = 64+(255- ((int(goalspeed)& 0xFF00) >> 8))
            goalspeed_lsb = (abs(goalspeed) & 0xff)

        #print goalspeed_msb,goalspeed_lsb
        data = []
        data.append(0x0C)
        data.append(self.servoid)
        data.append(I_JOG_REQ)
        data.append(goalspeed_lsb)
        data.append(goalspeed_msb)
        data.append(0x02|led)
        data.append(self.servoid)
        data.append(0x00)
        send_data(data)

    def set_position_p(self, pvalue):
        """ Set the P gain of the  position PID

        Args:

            pvalue (int): P value
        """
        pvalue_msb = int(pvalue) >> 8
        pvalue_lsb = int(pvalue) & 0xff
        data = []
        data.append(0x0B)
        data.append(self.servoid)
        data.append(RAM_WRITE_REQ)
        data.append(POSITION_KP_RAM)
        data.append(BYTE2)
        data.append( pvalue_lsb)
        data.append( pvalue_msb)
        send_data(data)

    def set_position_i(self, ivalue):
        """ Set the I gain of the position PID

        Args:
            ivalue (int): I value
        """
        ivalue_msb = int(ivalue) >> 8
        ivalue_lsb = int(ivalue) & 0xff

        data = []
        data.append(0x0B)
        data.append(self.servoid)
        data.append(RAM_WRITE_REQ)
        data.append(POSITION_KI_RAM)
        data.append(BYTE2)
        data.append(ivalue_lsb)
        data.append(ivalue_msb)
        send_data(data)

    def set_position_d(self, dvalue):
        """ Set the D gain of the PID

        Args:
            dvalue (int): D value
        """
        dvalue_msb = int(dvalue) >> 8
        dvalue_lsb = int(dvalue) & 0xff
        data = []
        data.append(0x0B)
        data.append(self.servoid)
        data.append(RAM_WRITE_REQ)
        data.append(POSITION_KD_RAM)
        data.append(BYTE2)
        data.append(dvalue_lsb)
        data.append(dvalue_msb)
        send_data(data)

    def get_position_p(self):
        """ Get the P value of the current PID for position

        """
        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(POSITION_KP_RAM)
        data.append(BYTE2)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(13)
            return (ord(rxdata[10])*256)+(ord(rxdata[9])&0xff)
        except HerkulexError:
            raise HerkulexError("could not communicate with motors")

    def get_position_i(self):
        """ Get the I value of the current PID for position

        """
        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(POSITION_KI_RAM)
        data.append(BYTE2)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(13)
            return (ord(rxdata[10])*256)+(ord(rxdata[9])&0xff)
        except HerkulexError:
            raise HerkulexError("Could not read from motors")

    def get_position_d(self):
        """ Get the D value of the current PID for position

        """
        data = []
        data.append(0x09)
        data.append(self.servoid)
        data.append(RAM_READ_REQ)
        data.append(POSITION_KD_RAM)
        data.append(BYTE2)
        send_data(data)
        rxdata = []
        try:
            rxdata = SERPORT.read(13)
            return (ord(rxdata[10])*256)+(ord(rxdata[9])&0xff)
        except HerkulexError:
            raise HerkulexError("could not communicate with motors")

    def save_pid_eeprom(self):
        """ saves the PID values from RAM to EEPROM

        """
        pval = self.get_position_p()
        ival = self.get_position_i()
        dval = self.get_position_d()

        #write P value
        pvalue_msb = int(pval) >> 8
        pvalue_lsb = int(pval) & 0xff

        data_p = []
        data_p.append(0x0B)
        data_p.append(self.servoid)
        data_p.append(EEP_WRITE_REQ)
        data_p.append(POSITION_KP_EEP)
        data_p.append(BYTE2)
        data_p.append( pvalue_lsb)
        data_p.append( pvalue_msb)
        send_data(data_p)

        # write I value
        ivalue_msb = int(ival) >> 8
        ivalue_lsb = int(ival) & 0xff

        data_i = []
        data_i.append(0x0B)
        data_i.append(self.servoid)
        data_i.append(EEP_WRITE_REQ)
        data_i.append(POSITION_KI_EEP)
        data_i.append(BYTE2)
        data_i.append( ivalue_lsb)
        data_i.append( ivalue_msb)
        send_data(data_i)

        # write D value
        dvalue_msb = int(dval) >> 8
        dvalue_lsb = int(dval) & 0xff

        data_d = []
        data_d.append(0x0B)
        data_d.append(self.servoid)
        data_d.append(EEP_WRITE_REQ)
        data_d.append(POSITION_KD_EEP)
        data_d.append(BYTE2)
        data_d.append( dvalue_lsb)
        data_d.append( dvalue_msb)
        send_data(data_d)

    def set_servo_angle(self, goalangle, goaltime, led):
        """ Sets the servo angle (in degrees)

        Enable torque using torque_on function before calling this

        Args:
            goalangle (int): The desired angle in degrees, range -150 to 150
            goaltime (int): the time taken to move from present
             position to goalposition
            led (int): the LED color
                       0x00 LED off
                       0x04 GREEN
                       0x08 BLUE
                       0x10 RED
        """
        if (self.servomodel==0x06) or (self.servomodel == 0x04):
            goalposition = scale(goalangle, -159.9, 159.6, 10627, 22129)
        else:
            goalposition = scale(goalangle, -150, 150, 21, 1002)

        self.set_servo_position(goalposition, goaltime, led)

    def get_servo_angle(self):
        """ Gets the current angle of the servo in degrees

        Args:
            none
        Returns:
            int : the current servo angle
        """

        servoposition =  self.get_servo_position()
        if (self.servomodel==0x06) or (self.servomodel == 0x04):
            return scale(servoposition, 10627, 22129, -159.9, 159.6)
        else:
            return scale(servoposition, 21, 1002, -150, 150)

class HerkulexError(Exception):
    """ Class to handle sservo errors
    """
    def __init__(self,  message):
        super(HerkulexError, self).__init__(message)
        self.message = message

