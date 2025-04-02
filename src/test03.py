# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 00:22:19 2024

@author: ammar
"""

from src.utils import switch_tab, set_camera_behind
from src.uavs import UAVs
import xpc
import time

# set the xpc client
xplane = xpc.XPlaneConnect()

# get the user UAV initial position
coordinates = xplane.getDREFs(['sim/flightmodel/position/local_x',
                               'sim/flightmodel/position/local_y',
                               'sim/flightmodel/position/local_z'])

# override the AI UAVs
ovr = [0] * 20
ovr[1] = 1
ovr[2] = 1
xplane.sendDREF('sim/operation/override/override_plane_ai_autopilot', ovr)
# xplane.sendDREF('sim/operation/override/override_planepath', ovr)

uav2 = UAVs(xplane, uav_num=2)
uav2.set_position(true_head=180)
uav2.set_coordinate(x=coordinates[0][0]+5, y=coordinates[1][0], z=coordinates[2][0])
uav2.shift_coordinate(z=-1)

uav2.send_throttle(1)
uav2.send_throttle(0)

uav1 = UAVs(xplane, uav_num=1)
uav1.set_position(true_head=180)
uav1.set_coordinate(x=coordinates[0][0]-5, y=coordinates[1][0], z=coordinates[2][0])
uav1.shift_coordinate(z=-2)

uav1.send_throttle(1)
uav1.send_throttle(0)
