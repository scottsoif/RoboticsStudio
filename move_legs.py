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
offsets = {11:120, 12:105, 13:132, 21:120, 22:100, 23:110}
servos = {11:servo11, 12:servo12, 13:servo13, 21:servo21, 22:servo22, 23:servo23}
t=0

def home_all():

    for joint in servos:
        servos[joint].moveTimeWrite(offsets[joint])

while True:
                angles = input("Enter servo,angle:\n")
                servo, angle = [int(i) for i in angles.split(',')]


                servos[servo].moveTimeWrite(offsets[servo]+angle)
