from lx16a import *
from math import sin, cos

# This is the port that the controller board is connected to
# This will be different for different computers
# On Windows, try the ports COM1, COM2, COM3, etc... # On Raspbian, try each port in /dev/ 
LX16A.initialize('/dev/cu.usbserial-1410')
# There should two servos connected, with IDs 1 and 2
servo11 = LX16A(11)
servo12 = LX16A(12)
servo13 = LX16A(13)
servo21 = LX16A(21)
servo22 = LX16A(22)
servo23 = LX16A(23)

t=0
while True:
                # Two sine waves out of phase
                servo11.moveTimeWrite(120+sin(t)*10)
                servo12.moveTimeWrite(105+sin(t)*50)
                servo13.moveTimeWrite(132+sin(t)*25)
                servo21.moveTimeWrite(120+sin(t)*10)
                servo22.moveTimeWrite(100+sin(t)*50)
                servo23.moveTimeWrite(60+sin(t)*25)

                # servo2.moveTimeWrite(120+cos(t)*50)
                t += 0.03

for servo in [servo11,servo12, servo13,servo21,servo22,servo23]:
    servo.moveStopAll()