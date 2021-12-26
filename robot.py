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
offsets = {11:120, 12:105, 13:132, 14:133, 21:123, 22:101, 23:108, 24:115}
servos = {11:servo11, 12:servo12, 13:servo13, 14:servo14, 21:servo21, 22:servo22, 23:servo23, 24:servo24}
# servo_list =  [servos[joint] for j in joints] # ordered list of servo motors

def error_lights():
	# flash RED then remain red
	for i in range(5):
		pixels.fill((50, 0, 0))
		time.sleep(.2)
		pixels.fill((0, 0, 0))
		time.sleep(.2)
	pixels.fill((50, 0, 0))

	# TODO: done? print("Red lights once neopixel setup")

def kill_motors():
	# disable all motors
	for servo in servos.values():
		servo.loadOrUnloadWrite(0)

def shut_down():
	# homes and disables all motors
	home_all(.1)

	# move_servos([-2, 99, -94, -60, -3, -84, 75, 97],.05)
	time.sleep(1.5)
	kill_motors()
	pixels.fill((10,20,20))

def engage_motors():
	for servo in servos.values():
		servo.loadOrUnloadWrite(1)

def getKeySet(num_positions=1):
	
	pos_list = []
	# home_all()
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

def home_all(step_size=.3, fp=None,  start_time=None, arms=60):
	
	pixels.fill((30, 0, 30))
	# for servo_num, servo in servos.items():
	# 	move_servo(servo, 0, offsets[servo_num], .06)

	x = [0, 0,0 ,-1*int(arms) ,0 ,0 ,0, int(arms)]
	move_servos(x, step_size, fp, start_time)


def walk(speed_t=0.02):
	t = 0
	pixels.fill((00, 0, 0))
	
	fp = open('joint_angles.txt', 'w')
	start_time = time.time()

	# home_all(.1, fp, start_time)
	move_servos([-1, 0, -1, -60, -2, 0, -1, 60], .1, fp, start_time)
	time.sleep(1)
	pixels.fill((00, 10, 00))
	time.sleep(.1)
	pixels.fill((00, 30, 20))
				# gate v1
				# 	[ -2,    0,    0,  -60,    -2,    0,    0,  60],
				#    [ 20,  -3,   -3,  -60,  -15,    0,   20,  60],
				#    [ 20,   10,   -6,  -60,  -15,  -20,   10,  60],
				#    [ 0,    10,   -12,  -60,    5,  -20,   15,  60],
				   
				#    [ -15,    0,   10,  -60,    20,  -5,   5,  60],
				 
				#  # other  leg
				#    [ 0,    0,    0,  -60,    0,    0,    0,  60],
				#    [ -15,    0,   20,  -60,  20,  -3,   -3,  60],
				#    [ -15,  -20,   10,  -60,  20,   10,   -6,  60],
				#    [ 5,  -20,   10,  -60,    0,    10,   -12,  60],
				#    [ 20,  -10,   5,  -60,    -15,    7,   5,  60],


	for i in range(6):
		keysets = [
				#    [ 20,   -3,   -4,  -55,   -15,    0,   20,  60],
				#    [ 20,   10,   -8,  -50,   -15,  -19,   10,  60],
				#    [ 0,    8,  -13,  -55,     5,  -19,   15,  60],
				   
				#  # other  leg
				#    [ -2,     0,    -1,  -60,    -2,     0,    -1,  60],
				#    [ -25,   0,   20,  -60,   17,    -3,   -3,  55],
				#    [ -15, -18,    8,  -60,   17,    13,   -6,  50],
				#    [ 5,   -18,   10,  -60,    0,    10,   -13,  55],
				#    [ -2,    0,    -2,  -60,    -2,    0,    -2,  60],


				# gate v1
					[ -2,    0,    0,  -60,    -2,    0,    0,  60],
				   [ 20,  -3,   -3,  -60,  -15,    0,   20,  60],
				   [ 20,   10,   -7,  -60,  -15,  -20,   15,  60],
				   [ 0,    10,   -12,  -60,    5,  -20,   15,  60],
				   
				   [ 0,    0,   -10,  -60,    0,  -5,   5,  60],
				 
				#  # other  leg
				   [ 0,    0,    0,  -60,    0,    0,    0,  60],
				   [ -15,    0,   20,  -60,  20,  -3,   -3,  60],
				   [ -15,  -15,   10,  -60,  20,   10,   -6,  60],
				   [ 5,  -15,   10,  -60,    0,    10,   -12,  60],
				   [ 20,  -10,   5,  -60,    -15,    7,   5,  60],


		]

	
		for i in range(len(keysets)):
			print(keysets[i])
			move_servos(keysets[i], .11, fp, start_time)
			# time.sleep(1)
			time.sleep(.2)
			# if i%2==0:
			# 	pixels.fill((100, 0, 00))
			# else:
			# 	pixels.fill((00, 0, 100))
		time.sleep(.2)


	move_servos([ 0, 0, 0,  -60, 0, 0,  0,  60], .15)

	home_all(.1, fp, start_time)

		
	# close file
	fp.close()

def side_walk(speed=None):

	pixels.fill((30, 0, 30))
	tilt_back_key =  [-5.1108220405366245, 0,1 , -60, 7.58479210453854,0 ,2 , 60]
	move_servos(tilt_back_key, .1)
	time.sleep(.1)
	pixels.fill((00, 30, 20))
	t = 3.4
	start_time = time.time()

	# og side walk shuffle
	# speed_t = .008
	# while True :
	# 	servo11.moveTimeWrite(offsets[11]+sin(t)*10)
	# 	servo21.moveTimeWrite(offsets[21]-sin(t-.04)*20)


	# 	if time.time()-start_time > 15:
	# 		break
	# 	t += speed_t


	speed_t = .006
	while True :
		servo11.moveTimeWrite(offsets[11]+3+sin(t+.65)*20)
		servo21.moveTimeWrite(offsets[21]+6-sin(t-.22)*28)

		if time.time()-start_time > 25:
			break
		t += speed_t
	
	home_all(.1)



def dance_step_1(speed_t=.009):

	# make sure to home robot before this
	pixels.fill((00, 30, 20))
	tilt_right_key_1 =  [ 20,  -2,   -3,  -0,  -15,    0,   20,  0]
	tilt_right_key_2 =  [ 30,  -2,   -3,  -0,  -15,    0,   20,  0]

	move_servos(tilt_right_key_1, .1)
	time.sleep(.1)
	move_servos(tilt_right_key_2, .1)
	time.sleep(.4)
	t = 0
	start_time = time.time()

	while True :
		servo22.moveTimeWrite(offsets[22]-sin(t)*30)
		servo14.moveTimeWrite(offsets[14]-sin(t)*25)
		servo23.moveTimeWrite(offsets[23]+sin(t)*25)

		if time.time()-start_time > 5:
			break
		t += speed_t

	home_all(.1)

def dance_step_2():

	# side shuffle
	pixels.fill((00, 30, 20))
	t = 0
	start_time = time.time()
	speed_t = .006
	while True :
		servo11.moveTimeWrite(offsets[11]+sin(t)*10)
		servo21.moveTimeWrite(offsets[21]-sin(t-.04)*20)


		if time.time()-start_time > 6:
			break
		t += speed_t
	
	home_all(.1)
	t = 0
	start_time = time.time()
	speed_t = .006
	while True :
		servo11.moveTimeWrite(offsets[11]-sin(t)*20)
		servo21.moveTimeWrite(offsets[21]+sin(t+.02)*10)
		if time.time()-start_time > 6:
			break
		t += speed_t

	home_all(.1)




def dance_step_3():

	home_all(.1)
	time.sleep(.1)
	pixels.fill((00, 30, 20))
	x = [0, 0,0 ,30 ,0 ,0 ,0, -30]
	move_servos(x, .25)

	start_time = time.time()
	speed_t = .006
	t = 0
	while True:
		servo23.moveTimeWrite(offsets[23]-30-sin(t)*25)
		if time.time()-start_time > 2:
			break
		t += speed_t

	start_time = time.time()
	speed_t = .0045
	while True:
		servo14.moveTimeWrite(offsets[14]+30-sin(t)*25)
		if time.time()-start_time > 2:
			break
		t += speed_t

	# home_all(.1)

def dance_step_4():

	home_all(.2, arms=30)
	time.sleep(.1)
	pixels.fill((00, 30, 20))

	t = 0
	speed_t = .01
	start_time = time.time()
	while True :
		servo11.moveTimeWrite(offsets[11]-sin(t)*20)
		servo21.moveTimeWrite(offsets[21]+sin(t)*20)
		servo14.moveTimeWrite(offsets[14]+20+sin(t)*40)
		servo23.moveTimeWrite(offsets[23]+20+sin(t)*40)
		if time.time()-start_time > 6:
			break
		t += speed_t

def dance_step_5(speed_t=.011):

	# make sure to home robot before this
	home_all(.2, arms=0)
	pixels.fill((00, 30, 20))
	tilt_right_key_1 =  [ 20,  -2,   -3,  -0,  -15,    0,   20,  0]
	tilt_right_key_2 =  [ 30,  -2,   -3,  -0,  -15,    0,   5,  0]

	move_servos(tilt_right_key_1, .1)
	time.sleep(.1)
	move_servos(tilt_right_key_2, .1)
	time.sleep(.4)
	t = 0
	start_time = time.time()

	while True :
		servo22.moveTimeWrite(offsets[22]-sin(t)*30)
		servo14.moveTimeWrite(offsets[14]-sin(t)*25)
		# servo23.moveTimeWrite(offsets[23]+sin(t)*25)
		servo21.moveTimeWrite(offsets[21]-cos(t+.1)*25)

		if time.time()-start_time > 5:
			break
		t += speed_t

	home_all(.1)

def dance_step_6(speed_t=.008):

	pixels.fill((00, 30, 20))
	tilt_forward_key =  [ 0, -6, 0, -60,  0,  -6, 0, 60]

	move_servos(tilt_forward_key, .1)
	time.sleep(.1)
	t = 0
	start_time = time.time()

	while True :

		servo22.moveTimeWrite(offsets[22]-6+cos(t)*20)
		servo12.moveTimeWrite(offsets[12]-6-cos(t)*20)
		servo24.moveTimeWrite(offsets[24]-cos(t)*20)
		servo13.moveTimeWrite(offsets[13]+cos(t)*20)


		if time.time()-start_time > 5:
			break
		t += speed_t

	home_all(.1, arms=-60)


def dance_step_bow(speed_t=.008):

	pixels.fill((00, 30, 20))

	# move_servos([ 0, 0, 0, 50, 0, 0, 0, -50], .2)

	t = 0
	start_time = time.time()
	
	keysets = [
				[ 0, -20, 9, 60, 0, -20, 9, -60],
				[ 0, -40, 20, 60, 0, -40, 20, -60],
				[ 0, -30, 20, 60, 0, -30, 20, -60], 
				[ 0, -25, 15, 60, 0, -25, 15, -60], 
				[ 0, -15, 15, 60, 0, -15, 15, -60], 
	]

	for i in range(len(keysets)):
		print(keysets[i])
		move_servos(keysets[i], .05)
		time.sleep(.05)

		# move_servos([ 0, 0, 0,  -60, 0, 0,  0,  60], .125)
	time.sleep(.2)
	home_all(.1)

	
def dance(rythm=1, speed_t=0.01):
	pixels.fill((00, 10, 10))
	
	home_all(.1, arms=60)
	time.sleep(1)
	pixels.fill((00, 10, 00))
	time.sleep(.1)
	pixels.fill((00, 30, 20))

	for i in range(2):
		dance_step_1()
		dance_step_2()
		dance_step_3()
		dance_step_4()
		dance_step_5()
		dance_step_6()
		
	# time.sleep(.1)
	# dance_step_bow()s


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
	parser.add_argument('--s_walk', action="store_true", help="Walk sidways")

	parser.add_argument('--dance', action="store_true", help="Dance (default move 1)")

	parser.add_argument('--flap', action="store_true", help="Flap Arms in sync")
	parser.add_argument('--m_flap', action="store_true", help="Walk and Flap Arms in sync")

	parser.add_argument('--move_rand', action="store_true", help="Move to preset rand pos")

	parser.add_argument('--motor_stop', action="store_true", help="Shut Down all motors")
	parser.add_argument('--motor_start', action="store_true", help="Engage all motors")
	parser.add_argument('--shut_down', action="store_true", help="Shut Down Robot Sequence")
	parser.add_argument('--find_keysets', action="store_true", help="find_keysets")

	args = parser.parse_args()
	print(args)

	if args.home:
		home_all(.2)
	elif args.walk:
		walk(.1)
	elif args.s_walk:
		side_walk(.1)
	elif args.dance:
		dance()
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
	elif args.shut_down:
		 shut_down()

	elif args.find_keysets:
		print(getKeySet(7))
 

if __name__ == "__main__":
	main()

