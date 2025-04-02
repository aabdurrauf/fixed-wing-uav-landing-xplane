# -*- coding: utf-8 -*-
"""
Created on Sun Nov 10 17:34:18 2024

@author: ammar
"""

# In[playground]

from utils import switch_tab
from pyKey.pyKey_windows import pressKey, releaseKey
import xpc
import time

def send_control(client, control_values = [], 
                 elevator=-998, aileron=-998, rudder=-998, 
                 throttle=-998, gear=-998):
    
    if len(control_values) == 0:
        controls = [elevator, aileron, rudder, throttle, gear, -998]
    else:
        controls = control_values            
        
    try:
        client.sendCTRL(controls)
    except:
        pass

xplane = xpc.XPlaneConnect()
send_control(xplane, aileron=0)

# In[test view]
# from src.utils import switch_tab, set_camera_behind

# switch_tab()
# time.sleep(1)
# set_camera_behind()

# In[main]
from utils import switch_tab, set_camera_behind
import xpc
import time

def send_control(client, control_values = [], 
                 elevator=-998, aileron=-998, rudder=-998, 
                 throttle=-998, gear=-998):
    
    if len(control_values) == 0:
        controls = [elevator, aileron, rudder, throttle, gear, -998]
    else:
        controls = control_values            
        
    try:
        client.sendCTRL(controls)
    except:
        pass

def stabilize_flight(client, 
                     yaw, yaw_rate, 
                     roll, roll_rate,
                     yaw_target=180, 
                     roll_target=0,
                     Pr=0.05, Dr=0.01,
                     Pa=0.01, Da=0.001):
    
    # yaw stabilizer
    yaw_err = yaw_target - yaw
    rudder_value = yaw_err * Pr - yaw_rate * Dr
    rudder_value = max(min(rudder_value, 1.0), -1.0)
    send_control(client, rudder=rudder_value)
    
    # roll stabilizer
    roll_err = roll_target - roll
    aileron_value = roll_err * Pa - roll_rate * Da
    aileron_value = max(min(aileron_value, 1.0), -1.0)
    send_control(client, aileron=aileron_value)
    
    
    
    return rudder_value, aileron_value


switch_tab()
time.sleep(1)
set_camera_behind()
time.sleep(0.5)

xplane = xpc.XPlaneConnect()
send_control(xplane, throttle=1.0)

drefs = ['sim/flightmodel2/position/y_agl', # altitude
         'sim/flightmodel/position/local_x', # position x
         'sim/flightmodel/position/local_z', # position z (position y is the altitude)
         'sim/cockpit/radios/gps_dme_speed_kts', # speed on knots
         'sim/flightmodel/position/vh_ind', # vertical velocity
         'sim/flightmodel/position/theta', # pitch angle
         'sim/flightmodel/position/phi', # roll angle
         
         'sim/flightmodel/position/psi', # yaw angle
         # 'sim/flightmodel/forces/vx_acf_axis', # velocity across x axis
         # 'sim/flightmodel/forces/vz_acf_axis', # velocity acros z axis
         'sim/flightmodel/position/local_vx', # velocity across x axis
         'sim/flightmodel/position/local_vz', # velocity acros z axis
         'sim/flightmodel/position/Q', # pitch rate
         'sim/flightmodel/position/P', # roll rate
         'sim/flightmodel/position/R', # yaw rate
         'sim/flightmodel2/misc/has_crashed'] # crash or not

has_crashed = 0.0

while not has_crashed:
    flight_data = xplane.getDREFs(drefs)
    time.sleep(0.1)
    
    altitude = flight_data[0][0]
    speed_kts = flight_data[3][0]
    
    pitch = flight_data[5][0]
    pitch_rate = flight_data[10][0]
    roll = flight_data[6][0]
    roll_rate = flight_data[11][0]
    yaw = flight_data[7][0]
    yaw_rate = flight_data[12][0]
    
    has_crashed = flight_data[-1][0]
    
    rudder, aileron = stabilize_flight(xplane, 
                                       yaw, yaw_rate,
                                       roll, roll_rate)

    text_data = f"""
--- flight_data ---
altitude: {altitude}
speed_kts: {speed_kts}
pitch: {pitch}
pitch_rate: {pitch_rate}
roll: {roll}
roll_rate: {roll_rate}
yaw: {yaw}
yaw_rate: {yaw_rate}
rudder: {rudder}
aileron: {aileron}
"""
    
    
    print(text_data)
    