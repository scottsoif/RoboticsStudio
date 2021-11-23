from lx16a import *
from math import sin, cos
import argparse
import time
import numpy as np

import board
import neopixel
pixels = neopixel.NeoPixel(board.D18, 12) 

# Raspberry port for DC motor driver
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

joints = [11, 12, 13, 14, 21, 22, 24, 23] # ordered list of servo names
offsets = {11:120, 12:105, 13:132, 14:133, 21:123, 22:101, 23:108, 24:116}
servos = {11:servo11, 12:servo12, 13:servo13, 14:servo14, 21:servo21, 22:servo22, 23:servo23, 24:servo24}
# servo_list =  [servos[joint] for j in joints] # ordered list of servo motors

def error_lights():
	# flash RED then remain red
	for i in range(5):
		pixels.fill((100, 0, 0))
		time.sleep(.2)
		pixels.fill((0, 0, 0))
	pixels.fill((100, 0, 0))

	# TODO: done? print("Red lights once neopixel setup")

def kill_motors():
	# disable all motors
	for servo in servos.values():
		servo.loadOrUnloadWrite(0)

def engage_motors():
	for servo in servos.values():
		servo.loadOrUnloadWrite(1)

def getKeySet(num_positions=1):
	
	pos_list = []
	home_all()
	time.sleep(2)

	pixels.fill((0, 50, 50))
	for i in range(num_positions):
		kill_motors()
		input("Enter any key to continue")
		
		servo_pos = [servo.getPhysicalPos() for servo in servos.values()]
		pos_list.append(servo_pos)

	# adjust for motor offsets
	offset_list = np.array([offsets[j] for j in joints])
	pos_list -= offset_list

	pos_list = pos_list.tolist() # convert from np array back to list

	return pos_list


class RobotError(Exception):
	def __init__(self, msg):
		error_lights()
		kill_motors()
		super().__init__(msg)

def boot_test():

	#### test joint angles
	bad_joints = []
	angles_list = np.array([servos[j].getPhysicalPos() for j in joints])
	print("Curr Servo Angles: ", angles_list)
	error_joints = np.where( (angles_list<0) | (angles_list>240) )[0] #  if angles is not in 0-240 range
	bad_joints = []
	for i in error_joints:
		bad_joints.append(joints[i])

	if len(bad_joints) > 0:
		raise RobotError(f'Servo(s) {bad_joints} out of range. Please adjust')

	for joint_name, servo in servos.items():
		pos = servo.getPhysicalPos()
		if pos < 0 or pos > 240:
			bad_joints.append(joint_name)

	#### Disable and Enable motors
	for servo in servos.values():
		servo.loadOrUnloadWrite(0)
	for servo in servos.values():
		servo.loadOrUnloadWrite(1)

	#### check servo voltages
	V_list = np.array([servos[j].vInRead() for j in joints])/1e3
	print("Curr Servo Voltages: ", V_list)
	diff = np.absolute(V_list - V_list.mean())
	error_V = np.where(diff>=1)[0] #  if voltage diff is more than 1
	bad_joints = []
	for i in error_V:
		bad_joints.append(joints[i])

	if len(bad_joints) > 0:
		raise RobotError(f'Servo(s) {bad_joints} have extreme voltages. Please inspect')


	#### show servo temps
	temp_list = np.array([servos[j].tempRead() for j in joints])
	print("Curr Servo Temperatures: ", temp_list)


boot_test()

def home_all(step_size=.3, fp=None,  start_time=None):
	
	pixels.fill((30, 0, 30))
	# for servo_num, servo in servos.items():
	# 	move_servo(servo, 0, offsets[servo_num], .06)
	x = [0, 0,0 ,-60 ,0 ,0 ,0, 60]
	move_servos(x, step_size, fp, start_time)


def walk(speed_t=0.02):
	t = 0
	pixels.fill((00, 0, 0))
	
	fp = open('joint_angles.txt', 'w')
	start_time = time.time()
	# home_all()
	# time.sleep(3)
#	while True:
#		# Two sine waves out of phase
#		#servo11.moveTimeWrite(120+sin(t)*10)
#		servo12.moveTimeWrite(105+sin(t)*15)
#		servo13.moveTimeWrite(134+sin(t-1)*10)
#		#servo21.moveTimeWrite(120+sin(t)*10)
#		servo22.moveTimeWrite(100-sin(t)*15)
#		servo23.moveTimeWrite(110-sin(t-1)*10)
#
#		# servo2.moveTimeWrite(120+cos(t)*50)
#		t += speed_t
	home_all(.1, fp, start_time)
	time.sleep(1)
	pixels.fill((00, 10, 00))
	time.sleep(.1)
	pixels.fill((00, 30, 20))



	for i in range(5):
		keysets = [
					[ -2,    0,    0,  -60,    -2,    0,    0,  60],
				   [ 20,  -3,   -3,  -60,  -15,    0,   20,  60],
				   [ 20,   10,   -6,  -60,  -15,  -20,   10,  60],
				   [ 0,    10,   -12,  -60,    5,  -20,   15,  60],
				   
				   [ -15,    0,   10,  -60,    20,  -5,   5,  60],
				 
				 # other  leg
				   [ 0,    0,    0,  -60,    0,    0,    0,  60],
				   [ -15,    0,   20,  -60,  20,  -3,   -3,  60],
				   [ -15,  -20,   10,  -60,  20,   10,   -6,  60],
				   [ 5,  -20,   10,  -60,    0,    10,   -12,  60],
				   [ 20,  -10,   5,  -60,    -15,    7,   5,  60],

		]

	
		for i in range(len(keysets)):
			print(keysets[i])
			move_servos(keysets[i], .1, fp, start_time)
			# time.sleep(1)
			time.sleep(.1)
			# if i%2==0:
			# 	pixels.fill((100, 0, 00))
			# else:
			# 	pixels.fill((00, 0, 100))


		move_servos([ 0, 0, 0,  -60, 0, 0,  0,  60], .125)

	home_all(.1, fp, start_time)

		
	# close file
	fp.close()

def flap(speed_t):
	t = 0
	home_all()
	while True:
		# Two sine waves out of phase
		servo14.moveTimeWrite(offsets[14]+sin(t)*45)
		servo24.moveTimeWrite(offsets[24]-sin(t)*45)
		t += speed_t

def walk_and_flap(speed_t=0.02):
	t = 0
	home_all()

	while True:
		# Two sine waves out of phase
		#servo11.moveTimeWrite(120+sin(t)*10)
		servo12.moveTimeWrite(105+sin(t)*25)
		servo13.moveTimeWrite(134+sin(t-1)*10)
		#servo21.moveTimeWrite(120+sin(t)*10)
		servo22.moveTimeWrite(100-sin(t)*25)
		servo23.moveTimeWrite(110-sin(t-1)*10)

		servo14.moveTimeWrite(offsets[14]+sin(t)*45)
		servo24.moveTimeWrite(offsets[24]+sin(t)*45)
		t += speed_t


def move_servos(new_pos_list, step_size=.02, fp=None, start_time=None):

	curr_pos = [servos[j].getPhysicalPos() for j in joints]

	angle_diffs = [0 for i in range(8)]
	not_done = True
	while not_done:
		not_done = False

		for i, joint in enumerate(joints):
				

			angle_diff = new_pos_list[i]+offsets[joint]-curr_pos[i]
			angle_diffs[i] = angle_diff

			if fp is not None:
				fp.write(f"{time.time()-start_time},{curr_pos}\n")

			if angle_diff < -.5:
				curr_pos[i] -= step_size
				servos[joint].moveTimeWaitWrite(curr_pos[i])
				not_done = True
			elif angle_diff > .5:
				# print(curr_pos[i])
				curr_pos[i] += step_size
				# print(curr_pos[i], type(curr_pos[i]))
				servos[joint].moveTimeWaitWrite(curr_pos[i])
				not_done = True
				# return 0

		LX16A.moveStartAll()
		# print(angle_diffs)

		# time.sleep(.2)
		# break




def move_rand(step_size=.06):

		servo_pos_list = [10, -50, -20, 40, 20, 40, -60, -50]
		move_servos(servo_pos_list, step_size)




def main():
	parser = argparse.ArgumentParser()
	required_args = parser.add_argument_group('required arguments')

	required_args.add_argument('--direct', dest="direction", help="input direction (f,l,r,b --> \
																	forward, left, right, back)")

	parser.add_argument('--home', action="store_true", help="Home everything to upright pos")
	parser.add_argument('--walk', action="store_true", help="Walk (default forward)")
	parser.add_argument('--flap', action="store_true", help="Flap Arms in sync")
	parser.add_argument('--m_flap', action="store_true", help="Walk and Flap Arms in sync")

	parser.add_argument('--move_rand', action="store_true", help="Move to preset rand pos")

	parser.add_argument('--motor_stop', action="store_true", help="Shut Down all motors")
	parser.add_argument('--motor_start', action="store_true", help="Engage all motors")
	parser.add_argument('--find_keysets', action="store_true", help="Find keysets angles for walking")


	args = parser.parse_args()
	print(args)

	if args.home:
		home_all(.3)
	elif args.walk:
		walk(.1)
	elif args.flap:
		flap(.01)
	elif args.m_flap:
		walk_and_flap(.2)
	elif args.move_rand:
		move_rand(.3)
	elif args.motor_stop:
		 kill_motors()
	elif args.motor_start:
		 engage_motors()

	elif args.find_keysets:
		print(getKeySet(7))
 

if __name__ == "__main__":
	main()

