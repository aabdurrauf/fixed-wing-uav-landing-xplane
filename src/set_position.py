# -*- coding: utf-8 -*-
"""
Created on Sat Nov 30 11:17:13 2024

@author: ammar
"""

# In[]
from src.utils import switch_tab, set_camera_behind
from src.uavs import UAVs
import xpc
import time

# In[]
# set the xpc client
xplane = xpc.XPlaneConnect()

# In[]
coordinates = xplane.getDREFs(['sim/flightmodel/position/local_x',
                               'sim/flightmodel/position/local_y',
                               'sim/flightmodel/position/local_z'])

# In[] get altitude
alti = xplane.getDREF('sim/flightmodel2/position/y_agl')
# In[]

local_x = 18229.258703120653
local_z = -52361.92335423521
xplane.sendDREFs(['sim/flightmodel/position/local_x',
                  'sim/flightmodel/position/local_y',
                  'sim/flightmodel/position/local_z'], 
                 [18291.6, -56.0, -52060.0])


# In[] rotate heading clocwwise (right) 5 degrees
heading = xplane.getPOSI()[5]
xplane.sendPOSI([-998, -998, -998, -998, -998, heading+5, -998])

# In[] rotate heading counterclocwwise (left) 5 degrees
heading = xplane.getPOSI()[5]
xplane.sendPOSI([-998, -998, -998, -998, -998, heading-5, -998])
# heading = xplane.getPOSI()[5]

# In[] set heading properly 185 degree
xplane.sendPOSI([-998, -998, -998, -998, -998, 185, -998])

# In[] shift up 10
current_pos_y = xplane.getDREF('sim/flightmodel/position/local_y')[0]
xplane.sendDREF('sim/flightmodel/position/local_y', current_pos_y+10)

# In[] shift up 5
current_pos_y = xplane.getDREF('sim/flightmodel/position/local_y')[0]
xplane.sendDREF('sim/flightmodel/position/local_y', current_pos_y+5)

# In[] shift up 1
current_pos_y = xplane.getDREF('sim/flightmodel/position/local_y')[0]
xplane.sendDREF('sim/flightmodel/position/local_y', current_pos_y+1)

# In[] shift down 10
current_pos_y = xplane.getDREF('sim/flightmodel/position/local_y')[0]
xplane.sendDREF('sim/flightmodel/position/local_y', current_pos_y-10)

# In[] shift down 5
current_pos_y = xplane.getDREF('sim/flightmodel/position/local_y')[0]
xplane.sendDREF('sim/flightmodel/position/local_y', current_pos_y-5)

# In[] shift down 1
current_pos_y = xplane.getDREF('sim/flightmodel/position/local_y')[0]
xplane.sendDREF('sim/flightmodel/position/local_y', current_pos_y-1)

# In[] shift right 10
current_pos_x = xplane.getDREF('sim/flightmodel/position/local_x')[0]
xplane.sendDREF('sim/flightmodel/position/local_x', current_pos_x-10)

# In[] shift right 5
current_pos_x = xplane.getDREF('sim/flightmodel/position/local_x')[0]
xplane.sendDREF('sim/flightmodel/position/local_x', current_pos_x-5)

# In[] shift right 1
current_pos_x = xplane.getDREF('sim/flightmodel/position/local_x')[0]
xplane.sendDREF('sim/flightmodel/position/local_x', current_pos_x-1)

# In[] shift left 10
current_pos_x = xplane.getDREF('sim/flightmodel/position/local_x')[0]
xplane.sendDREF('sim/flightmodel/position/local_x', current_pos_x+10)

# In[] shift left 5
current_pos_x = xplane.getDREF('sim/flightmodel/position/local_x')[0]
xplane.sendDREF('sim/flightmodel/position/local_x', current_pos_x+5)

# In[] shift left 1
current_pos_x = xplane.getDREF('sim/flightmodel/position/local_x')[0]
xplane.sendDREF('sim/flightmodel/position/local_x', current_pos_x+1)

# In[] shift back 10
current_pos_z = xplane.getDREF('sim/flightmodel/position/local_z')[0]
xplane.sendDREF('sim/flightmodel/position/local_z', current_pos_z-10)

# In[] shift back 5
current_pos_z = xplane.getDREF('sim/flightmodel/position/local_z')[0]
xplane.sendDREF('sim/flightmodel/position/local_z', current_pos_z-5)

# In[] shift back 1
current_pos_z = xplane.getDREF('sim/flightmodel/position/local_z')[0]
xplane.sendDREF('sim/flightmodel/position/local_z', current_pos_z-1)

# In[] shift forward 10
current_pos_z = xplane.getDREF('sim/flightmodel/position/local_z')[0]
xplane.sendDREF('sim/flightmodel/position/local_z', current_pos_z+10)

# In[] shift forward 5
current_pos_z = xplane.getDREF('sim/flightmodel/position/local_z')[0]
xplane.sendDREF('sim/flightmodel/position/local_z', current_pos_z+5)

# In[] shift forward 1
current_pos_z = xplane.getDREF('sim/flightmodel/position/local_z')[0]
xplane.sendDREF('sim/flightmodel/position/local_z', current_pos_z+1)






