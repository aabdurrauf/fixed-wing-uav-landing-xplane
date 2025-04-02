# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 20:57:54 2024

@author: ammar
"""

from src.utils import switch_tab, set_camera_behind
import xpc
import time

xplane = xpc.XPlaneConnect()

data_posi = xplane.getPOSI(ac=0)
print(f'lat: {data_posi[0]}')
print(f'long: {data_posi[1]}')
print(f'alt: {data_posi[2]}')
print(f'pitch: {data_posi[3]}')
print(f'roll: {data_posi[4]}')
print(f'head: {data_posi[5]}')
print(f'gear: {data_posi[6]}')

# In[0]
X = -998
values = [X, X, X, 0.0, 0.0, 180.0, X]
xplane.sendPOSI(values, ac=2)

values2 = [X, X, X, X, X, 180.0, X]
xplane.sendPOSI(values2, ac=2)

xplane.sendDREF('sim/multiplayer/position/plane2_psi', 180)

# In[0]
xplane.sendDREFs(['sim/multiplayer/position/plane2_v_x',
                  'sim/multiplayer/position/plane2_v_y',
                  'sim/multiplayer/position/plane2_v_z'], 
                 [0, 0, 0])


data_ovr = xplane.getDREF('sim/operation/override/override_plane_ai_autopilot')

# In[1]

ovr_thr = [0] * 20
ovr_thr[1] = 1
ovr_thr[2] = 1
xplane.sendDREF('sim/operation/override/override_plane_ai_autopilot', ovr_thr)
xplane.sendDREF('sim/multiplayer/position/plane2_throttle', [1, 0, 0, 0, 0, 0, 0, 0])


# In[2]

# xplane.sendDREF('sim/multiplayer/position/plane2_x', 18148.98674522735)

X = -998
ctrl = [X, X, X, X, X, 0.0]
xplane.sendCTRL(ctrl, ac=0)

# xplane.sendDREF('sim/multiplayer/position/plane2_yolk_pitch', 1)

# In[send elevator to multiplayer]
control_val = [0] * 20
control_val[2] = 1
xplane.sendDREF('sim/multiplayer/controls/yoke_roll_ratio', control_val)

# In[]

v=xplane.getDREF("sim/multiplayer/controls/yoke_roll_ratio")
xplane.sendDREF("sim/multiplayer/controls/yoke_roll_ratio", [v[0], v[1], 0])

# In[]

plane = 'plane3'



for i in range(150):

    data = xplane.getDREFs([f'sim/multiplayer/position/{plane}_x', 
                            f'sim/multiplayer/position/{plane}_y',
                            f'sim/multiplayer/position/{plane}_z'])
    
    
    print(f'data: {data}')

    time.sleep(0.25)
