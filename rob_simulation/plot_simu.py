import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos


times = list(range(1000))



angles_list = np.array([


      [[ 0.27199891,  0.08189551,  0.29555326],
       [ 0.02333844, -0.35       ,  0.47848877], # l knee
       [-.16327489,  0.3091608,  -.02064897], # l ankle
       [ 0.77580598, -0.36585413, -0.74581517], 
       [-0.06374235,  0.3549037, -0.40555079], # r knee
       [ 0.29852617, -0.30618517,  0.43341183],  # r ankle
       [ 0.85266139,  0.04888316, -0.37132398],
       [ 0.82464599,  0.52800857,  0.02889313]]

        ]) 

fig, ax = plt.subplots(4, figsize=(15,7))

# 1, 'left_kneej'
# 2, 'left_anklej'
# 4, 'right_kneej'
# 5, 'right_anklej'


for ax_idx, j in enumerate([2, 1, 5, 4]): # l_ankle, l_knee, r_ankle, r_knee
    angles = angles_list[0]
    omega = 0.01

    y = [angles[j,0]+angles[j,1]*sin(i*omega+angles[j,2]) for i in times]
    ax[ax_idx].plot(times, y )
    ax[ax_idx].set_xlabel('time')
    ax[ax_idx].set_ylabel(f"Degrees (motor_id = {j})")
    ax[ax_idx].set_ylim([-1.5,1.5])
    ax[ax_idx].margins(x=0)
    ax[ax_idx].margins(y=0)

print("Note: varying y scales because of different types and units of measurements for each feature")
plt.tight_layout()
plt.show(block=False)

plt.pause(4)
plt.close(fig)


