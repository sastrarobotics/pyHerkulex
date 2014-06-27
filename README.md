
#pyHerkulex


 A Python library for Herkulex servos

##Installation
  Either copy the pyherkulex folder to your current working directory or you can install it system wide via  below command
  >*sudo python setup.py install*
  
##Usage  
  Assuming that the Herkulex servos are connected to "/dev/ttyUSB0", at 115200 baud &  has default id of 253
  Import the library
  >from pyherkulex import herkulex
  
  Connect to the Herkulex bus
  >herkulex.connect("/dev/ttyUSB0",115200)
  
  Enable Torque
  >herkulex.torque_on(253)
  
  Clear Errors
  >herkulex.clear_errors()
  
  Set Servo to mid position(512) with a time factor of 10 & LED color of Green
  >herkulex.set_servo_position(253,512,10,2)
 
 Get The current Servo Position
 >print herkulex.get_servo_position(253)
