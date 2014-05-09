
#pyHerkulex


 A Python library for Herkulex servos

##Installation
  >*sudo python setup.py install*
  
##Usage  
  >from pyherkulex import herkulex
  
  >herkulex.connect(serial_port,baudrate)
  
  >herkulex.torque_on(servo_id)
  
  >herkulex.clear_errors()
  
  >herkulex.set_servo_position(servoid,position,speed,led)
