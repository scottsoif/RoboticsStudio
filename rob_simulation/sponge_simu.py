

import pybullet as p
import time
import pybullet_data
from math import sin, cos
import numpy as np

# physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
# hides side panes
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

def get_orn_diff(p, start_orn, curr_orn, specific_axes=[0,1,2]):
    """ 
    Args:
        specific_axes : list - of specific axes  you want [0, 1,2]
            (i.e. to use x and z axis then specific_axes=[0,2] )
    Returns:
            difference between two robot orientations
    """
    curr_orn = np.array(p.getEulerFromQuaternion(curr_orn))[specific_axes]
    start_orn = np.array(p.getEulerFromQuaternion(start_orn))[specific_axes]

    return np.sum((curr_orn-start_orn)**2)

def get_euclid_diff(p, start_pos, curr_pos):
    """ 
    Args:

    Returns:
            distance between two robot points
    """
    curr_pos = np.array(curr_pos)
    start_pos = np.array(start_pos)
    return np.sum((curr_pos-start_pos)**2)

def reset_robot(p, num_iters=70):
    """ resets robot position and homes joints
    Args:
        num_iters : int - determines how long robot waits before 
                        trying next walk 
    Returns:
            distance between two robot points
    """
    p.setJointMotorControlArray(botId, np.arange(0,8), controlMode=mode, targetPositions=np.zeros(8))

    p.restoreState(fileName="state.bullet")
    for i in range (num_iters):
        p.stepSimulation()
        time.sleep(1./240.)


p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")
robStartPos = [0,0,.4]
robStartOrientation = p.getQuaternionFromEuler([0,0,0])
botId = p.loadURDF("urdf_files/v1.1_urdf.urdf",robStartPos, robStartOrientation) 

jointFrictionForce = 1
for joint in range(p.getNumJoints(botId)):
    p.setJointMotorControl2(botId, joint, p.POSITION_CONTROL, force=jointFrictionForce)

robPos, robOrn = p.getBasePositionAndOrientation(botId)

# drop robot
for i in range (70):
    p.stepSimulation()
    time.sleep(1./240.)
p.saveBullet("state.bullet")

# start position
rob_start_pos, rob_start_orn = p.getBasePositionAndOrientation(botId)

mode = p.POSITION_CONTROL

#  looking for z axis rotation
# for i in range(10000):
#     rob_curr_pos, rob_curr_orn = p.getBasePositionAndOrientation(botId)

#     print(get_orn_diff(p, rob_start_orn, rob_curr_orn, [0]))
#     p.stepSimulation()
#     time.sleep(1./240.)

if False:
    for trial in range(5000):
        reset_robot(p, 40)
        angles = np.random.uniform(-1,1,(8,3))
        hips  = np.random.uniform(-.5,.5,(2,3))
        angles[(0,4),:] = hips
        omega = 0.01
        for i in range(300):
            p.stepSimulation()
            for j in [0,3,1,4,2,5]:
            # for j in [0, 3]:

                p.setJointMotorControl2(botId, j, controlMode=mode, 
                                    targetPosition=angles[j,1]*cos(i*omega+angles[j,2]))
                                    # targetPosition=angles[j,0]+angles[j,1]*sin(i*omega+angles[j,2]))


            time.sleep(1./1000.)

            # time.sleep(1./360.)

        rob_curr_pos, rob_curr_orn = p.getBasePositionAndOrientation(botId)

        # only look at diff in z direction
        diff_orn_z = get_orn_diff(p, rob_start_orn, rob_curr_orn, [2])
        diff_orn_x = get_orn_diff(p, rob_start_orn, rob_curr_orn, [0])

        diff_pos = get_euclid_diff(p, rob_start_pos, rob_curr_pos)
        if diff_orn_z < .5 and diff_orn_x < .5:
            print(f"\n{i}. Rob orn diff_x {diff_orn_x:.4f}, diff_z {diff_orn_z:.4f}")
            print(repr(angles))
            # print(f"Rob pos diff {diff_pos:.4f}\n")

elif False:
    angles_list = np.array([
        
                [[ 0.05163892,  0.04821397, -0.44009867], [-0.71307706, -0.2672934 ,  0.79934517], [ 0.41700986, -0.79970848,  0.18455155], [-0.31784218, -0.28467679,  0.99167244], [ 0.10688461,  0.41387358, -0.02378711], [-0.19077266,  0.32953808,  0.64262975], [ 0.59196898,  0.6878502 ,  0.03317822], [ 0.11408896, -0.45810959, -0.97060831]]
      
       ])

    omega = 0.01
    for angles in angles_list:
        for i in range (1000):
            p.stepSimulation()
            for j in [1,4,2,5]:
                p.setJointMotorControl2(botId, j, controlMode=mode, 
                                    targetPosition=angles[j,0]+angles[j,1]*sin(i*omega+angles[j,2]))

            time.sleep(1./360.)
        reset_robot(p, 40)

elif True:

    omega = 0.01
    t, step = 0, .06
    for i in range (10000):
        p.stepSimulation()

        p.setJointMotorControl2(botId, 3, controlMode=mode, 
                                targetPosition=cos(t)*0.3490658504)
        p.setJointMotorControl2(botId, 0, controlMode=mode, 
                                targetPosition=sin(t)*0.3490658504)
        p.setJointMotorControl2(botId, 6, controlMode=mode, 
                                targetPosition=sin(t)*0.6981317008)
        p.setJointMotorControl2(botId, 7, controlMode=mode, 
                                targetPosition=sin(t)*0.6981317008)

        # p.setJointMotorControl2(botId, 1, controlMode=mode, 
        #                         targetPosition=(cos(t)*0.5-.1))
        # p.setJointMotorControl2(botId, 4, controlMode=mode, 
        #                         targetPosition=(-cos(t)*0.5-.1))
        # p.setJointMotorControl2(botId, 2, controlMode=mode, 
        #                         targetPosition=-cos(t)*0.5)
        # p.setJointMotorControl2(botId, 5, controlMode=mode, 
        #                         targetPosition=cos(t)*0.5)
        t += step


        time.sleep(1./360.)
    reset_robot(p, 40)

p.disconnect()
# p.getJointInfo(1,2)
# 0, b'left_hipj'
# 1, b'left_kneej'
# 2, b'left_anklej'
# 3, b'right_hipj'
# 4, b'right_kneej'
# 5, b'right_anklej'
# 6, b'left_armj'
# 7, b'right_armj'
