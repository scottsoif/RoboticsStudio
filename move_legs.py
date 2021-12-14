from lx16a import *
from math import sin, cos

# This is the port that the controller board is connected to
# This will be different for different computers
# On Windows, try the ports COM1, COM2, COM3, etc... # On Raspbian, try each port in /dev/ 
LX16A.initialize('/dev/ttyUSB0')
# There should two servos connected, with IDs 1 and 2
servo11 = LX16A(11)
servo12 = LX16A(12)
servo13 = LX16A(13)
servo14 = LX16A(14)
servo21 = LX16A(21)
servo22 = LX16A(22)
servo23 = LX16A(23)
servo24 = LX16A(24)

offsets = {11:120, 12:105, 13:130, 14:133, 21:123, 22:101, 23:108, 24:114}
servos = {11:servo11, 12:servo12, 13:servo13, 14:servo14, 21:servo21, 22:servo22, 23:servo23, 24:servo24}
t=0
while True:
                angles = input("Enter servo,angle:\n")
                servo, angle = [int(i) for i in angles.split(',')]


                servos[servo].moveTimeWrite(offsets[servo]+angle)
