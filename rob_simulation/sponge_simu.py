

import pybullet as p
import time
import pybullet_data
from math import sin, cos
import numpy as np

rob_start_pos, rob_start_orn, botId = None, None, None
mode = p.POSITION_CONTROL

def init_pb(gui=0):

    if gui==1:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)
        
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    # hides side panes
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

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

    return rob_start_pos, rob_start_orn, botId

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


def hill_climb():

    #  looking for z axis rotation
    # for i in range(10000):
    #     rob_curr_pos, rob_curr_orn = p.getBasePositionAndOrientation(botId)
    #     # print(rob_curr_pos[:2])
    #      print(get_orn_diff(p, rob_start_orn, rob_curr_orn, [0]))
    #     p.stepSimulation()
    #     time.sleep(1./240.)

    # hill climb
    # angles = np.random.uniform(-1,1,(8,3))
    angles = np.array(
        [[ 0.27199891,  0.08189551,  0.29555326],
       [ 0.04795808, -0.35      ,  0.47848877],
       [-0.16327489,  0.30921064, -0.02064897],
       [ 0.77580598, -0.36585413, -0.74581517],
       [-0.06374235,  0.3549037 , -0.4610206 ],
       [ 0.29852617, -0.31421001,  0.43341183],
       [ 0.85266139,  0.04888316, -0.37132398],
       [ 0.82464599,  0.52800857,  0.02889313]]
    )

    # hips  = np.random.uniform(-.5,.5,(2,3))
    # angles[(0,4),:] = hips
    if True:
        prev_orn_x = float("inf")
        prev_orn_z = float("inf")
        prev_frwd_pos = float("inf")

        for trial in range(5000):

            for k in [1,4,2,5]: 
                for l in range(3):
                    reset_robot(p, 40)
                    new_val = np.random.uniform(-.1,.1,)
                    angles[k, l] +=  new_val
                    omega = 0.01
                    for i in range(400):
                        p.stepSimulation()
                        for j in [1,4,2,5]:
                        # for j in [0, 3]:

                            p.setJointMotorControl2(botId, j, controlMode=mode, 
                                                targetPosition=angles[j,0]+angles[j,1]*sin(i*omega+angles[j,2]))
                                                # targetPosition=angles[j,0]+angles[j,1]*sin(i*omega+angles[j,2]))
                        time.sleep(1./1000.)


                    rob_curr_pos, rob_curr_orn = p.getBasePositionAndOrientation(botId)

                    # only look at diff in z direction
                    diff_orn_z = get_orn_diff(p, rob_start_orn, rob_curr_orn, [2])
                    diff_orn_x = get_orn_diff(p, rob_start_orn, rob_curr_orn, [0])

                    diff_pos = get_euclid_diff(p, rob_start_pos, rob_curr_pos)
                    forward_pos = rob_curr_pos[1]
                    if diff_orn_z < prev_orn_z and diff_orn_x < prev_orn_x and forward_pos < prev_frwd_pos:
                        prev_orn_x = diff_orn_x
                        prev_orn_z = diff_orn_z
                        prev_frwd_pos = forward_pos
                        print(f"\n{trial}-{k}-{l}. Rob orn diff_x {diff_orn_x:.4f}, diff_z {diff_orn_z:.4f}")
                        print(repr(angles))

                    else: # undo angle change
                        angles[k, l] -= new_val

# random search
if False:
    for trial in range(3000):
        reset_robot(p, 40)
        angles = np.random.uniform(-1,1,(8,3))
        hips  = np.random.uniform(-.5,.5,(2,3))
        angles[(0,4),:] = hips
        omega = 0.01
        for i in range(500):
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
        if diff_orn_z < .3 and diff_orn_x < .3:
            print(f"\n{trial}. Rob orn diff_x {diff_orn_x:.4f}, diff_z {diff_orn_z:.4f}")
            print(repr(angles))
            # print(f"Rob pos diff {diff_pos:.4f}\n")

elif False:
    angles_list = np.array([
        # p.getJointInfo(1,2)

                # [[ 0.05163892,  0.04821397, -0.44009867],  # 0, b'left_hipj'
                #  [-.5, -0.2672934 ,  .5],  # 1, b'left_kneej'
                #  [ 0.18, -0.19970848,  -0.18455155],  # 2, b'left_anklej'
                #  [-0.31784218, -0.28467679,  0.99167244],  # 3, b'right_hipj'
                #  [ 0.10688461,  0.41387358, -0.02378711],  # 4, b'right_kneej'
                #  [-0.19077266,  0.32953808,  0.4262975],  # 5, b'right_anklej'
                #  [ 0.59196898,  0.6878502 ,  0.03317822],  # 6, b'left_armj'
                #  [ 0.11408896, -0.45810959, -0.97060831]]  # 7, b'right_armj'


            [[  0.27199891,  0.08189551,  0.29555326], 
            [.03909574, -0.44309764,  0.52543079],  # l knee
            [-.4,  0.35972496,  0.48252196],  # l ankle
            [ 0.77580598, -0.46585413, -0.54581517], 
            [-0.08832095, 0.34, -0.40555079],  # r knee
            [-0.03890384, -0.37331425,  0.39773976],  # r ankle
            [ 0.85266139,  0.04888316, -0.37132398], 
            [ 0.82464599,  0.52800857,  0.02889313]],


            
       ])

    omega = 0.01
    for angles in angles_list:
        print("       new trials")
        for i in range (2500):
            p.stepSimulation()
            for j in [1,4,2,5]:
                p.setJointMotorControl2(botId, j, controlMode=mode, 
                                    targetPosition=angles[j,0]+angles[j,1]*sin(i*omega+angles[j,2]))
        


            time.sleep(1./1000.)
        rob_curr_pos, rob_curr_orn = p.getBasePositionAndOrientation(botId)

        diff_orn_z = get_orn_diff(p, rob_start_orn, rob_curr_orn, [2])
        diff_orn_x = get_orn_diff(p, rob_start_orn, rob_curr_orn, [0])

        diff_pos = get_euclid_diff(p, rob_start_pos, rob_curr_pos)
        if diff_orn_z > .3 and diff_orn_x > .3:
            print("       FAIL")

        reset_robot(p, 40)


elif False:

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

# p.getJointInfo(1,2)
# 0, b'left_hipj'
# 1, b'left_kneej'
# 2, b'left_anklej'
# 3, b'right_hipj'
# 4, b'right_kneej'
# 5, b'right_anklej'
# 6, b'left_armj'
# 7, b'right_armj'

if __name__=="__main__":

    rob_start_pos, rob_start_orn, botId = init_pb(0)

    hill_climb()

    p.disconnect()


