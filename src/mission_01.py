# -*- coding: utf-8 -*-
"""
Created on Sat Nov 30 11:46:46 2024

@author: ammar
"""

# In[0]
from utils import switch_tab, set_camera_behind, set_camera_outside
from uavs import UAVs
import xpc
import time, math

## In[1]
# set the xpc client
xplane = xpc.XPlaneConnect()

## In[2]
# set the coordinate

x = 18743.50
y = -88.01
z = -52647.00
xplane.sendDREFs(['sim/flightmodel/position/local_x',
                  'sim/flightmodel/position/local_y',
                  'sim/flightmodel/position/local_z'], 
                 [x, y, z])
xplane.sendPOSI([-998, -998, -998, -998, -998, 290, -998])

## In[3] sim/flightmodel2/position/y_agl
# sim/flightmodel/position/l 
# define the waypoints [latitude, longitude, altitude]
waypoint_01 = [40.97299857842195, 29.218362868652815, 60.858067] # [18353.6, -38.0, -52677.0]
waypoint_02 = [40.97061434835756, 29.21740418756978, 46.314426] # [18273.6, -64.0, -52412.0]
waypoint_03 = [40.96926758758702, 29.21745961615572, 1.514426] # [18278.6, -74.0, -52262.0]
waypoint_04 = [40.968979707377784, 29.21761328207183, 0.5475694] # [18291.6, -72.0, -52230.0] # under the bridge
waypoint_05 = [40.96871361972544, 29.217790866990253, 0.76425024] # [18306.6, -71.0, -52200.0]
waypoint_06 = [40.96745149699333, 29.217607745883345, 7.7969315] # [18291.6, -56.0, -52060.0]
waypoints = [waypoint_01, waypoint_02, waypoint_03,
             waypoint_04, waypoint_05, waypoint_06]
wp_index = 0

## In[4]
# define helper functions
def send_control(client, control_values = [],  ac=0,
                 elevator=-998, aileron=-998, rudder=-998, 
                 throttle=-998, gear=-998, flaps=-998):
    
    if len(control_values) == 0:
        controls = [elevator, aileron, rudder, throttle, gear, flaps]
    else:
        controls = control_values            
        
    try:
        client.sendCTRL(controls, ac)
    except:
        pass

def stabilize_flight(client, 
                     yaw, yaw_rate, 
                     roll, roll_rate,
                     pitch=None, pitch_rate=None, accumulated_pitch_error=0,
                     yaw_target=290, 
                     roll_target=0,
                     pitch_target=0,
                     Pr=0.1, Dr=0.01,
                     Pa=0.01, Da=0.001,
                     Pe=0.1, Ie=0, De=0.01,
                     current_alti=None, target_alti=None, ver_vel=None, accumulated_alti_error=0):
    
    # yaw stabilizer
    yaw_err = yaw_target - yaw
    rudder_value = yaw_err * Pr - yaw_rate * Dr
    rudder_value = max(min(rudder_value, 1.0), -1.0)
    # send_control(client, rudder=rudder_value)
    
    # roll stabilizer
    roll_err = roll_target - roll
    aileron_value = roll_err * Pa - roll_rate * Da
    aileron_value = max(min(aileron_value, 1.0), -1.0)
    # send_control(client, aileron=aileron_value)
    
    elevator_value = -998
    if pitch != None and pitch_rate != None:
        # pitch stabilizer
        pitch_err = pitch_target - pitch
        accumulated_pitch_error += pitch_err
        elevator_value = pitch_err * Pe + accumulated_pitch_error * Ie - pitch_rate * De
        elevator_value = max(min(elevator_value, 1.0), -1.0)
        # send_control(client, elevator=elevator_value)
        
    if current_alti!= None and target_alti != None and ver_vel != None:
        # altitude keeper
        alti_err = target_alti - current_alti
        accumulated_alti_error += alti_err
        elevator_value = alti_err * 0.012 - ver_vel * 0.028 + accumulated_alti_error * 0.0001
        elevator_value = max(min(elevator_value, 1.0), -1.0)
        # send_control(client, elevator=elevator_value)
        
    
    send_control(client, 
                 elevator=elevator_value,
                 aileron=aileron_value, 
                 rudder=rudder_value)
    
    
    return rudder_value, aileron_value, elevator_value, accumulated_pitch_error, accumulated_alti_error

def distance_to_target(current_pos, target_pos, radius=6378):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [current_pos[0], current_pos[1], target_pos[0], target_pos[1]])

    # Differences in latitude and longitude
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    # Haversine formula
    a = math.sin(delta_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in the specified radius unit
    distance = radius * c
    return distance

def calculate_target_heading(current_pos, waypoint):
    
    assert len(current_pos) == 3
    assert len(waypoint) == 3
    
    current_lat, current_lon, target_lat, target_lon = map(math.radians, [current_pos[0], current_pos[1], waypoint[0], waypoint[1]])
    delta_lon = target_lon - current_lon
    
    x = math.sin(delta_lon) * math.cos(target_lat)
    y = math.cos(current_lat) * math.sin(target_lat) - math.sin(current_lat) * math.cos(target_lat) * math.cos(delta_lon)
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360  # Normalize to [0, 360)
    
    
#     text_data = f"""
# ----- flight_data -----
# waypoint:              {waypoint}
# current_pos:           [{"{:.3f}".format(current_pos[0])}, {"{:.3f}".format(current_pos[1])}, {"{:.3f}".format(current_pos[2])}]
# bearing:               {"{:.3f}".format(bearing)}
# -----------------------
# dist_to_target:        {"{:.3f}".format(distance_to_target(current_pos, waypoint)*1000)}"""
        
#     print(text_data)
    return bearing

def calculate_heading_error(current_heading, target_heading):
    heading_error = (target_heading - current_heading + 360) % 360
    if heading_error > 180:
        heading_error -= 360  # Normalize to [-180, 180]
        
    return heading_error

# def distance_to_target(current_pos, target_pos):
#         assert len(current_pos) == 3
#         assert len(target_pos) == 3
        
#         x = pow((current_pos[0] - target_pos[0]), 2)  # x-axis difference
#         y = pow((current_pos[1] - target_pos[1]), 2)  # y-axis difference
#         z = pow((current_pos[2] - target_pos[2]), 2)  # z-axis difference
        
#         distance = math.sqrt(x + y + z)
        
#         # Debugging print
#         # print(f'square root of ({x} + {y} + {z}) is {distance}')
        
#         return distance
    
# def calculate_target_heading(current_pos, waypoint):
    
#     current_x, _, current_z = current_pos
#     waypoint_x, _, waypoint_z = waypoint
    
#     dx = waypoint_x - current_x
#     dz = waypoint_z - current_z
    
#     heading_radians = math.atan2(dz, dx)
#     heading_degrees = math.degrees(heading_radians)
#     heading_degrees_norm = (heading_degrees + 360) % 360 
    
#     text_data = f"""
# ----- flight_data -----
# waypoint:              {waypoint}
# current_pos:           [{"{:.3f}".format(current_pos[0])}, {"{:.3f}".format(current_pos[1])}, {"{:.3f}".format(current_pos[2])}]
# heading_radians:       {"{:.3f}".format(heading_radians)}
# heading_degrees:       {"{:.3f}".format(heading_degrees)}
# heading_degrees_norm:  {"{:.3f}".format(heading_degrees_norm)}
# dx:                    {"{:.3f}".format(dx)}
# dz:                    {"{:.3f}".format(dz)}
# -----------------------
# dist_to_target:        {"{:.3f}".format(distance_to_target(current_pos, waypoint))}
# """
        
#     print(text_data)

#     return heading_degrees_norm

def go_to_waypoint(client, 
                   yaw, yaw_rate,
                   roll, roll_rate,
                   pitch, pitch_rate,
                   current_pos, waypoint, ver_vel,
                   Dp=0.004, Di=0, Dd=0.0012):
    
    target_heading = calculate_target_heading(current_pos, waypoint)
    
    # error_heading = target_heading - yaw
    error_heading = calculate_heading_error(yaw, target_heading)
    if error_heading > 180:
        error_heading -= 360  # Normalize to [-180, 180]
    roll_target = max(min(error_heading * 0.36, 30.0), -30.0)
    roll_error = roll_target - roll
    aileron_value = roll_error * Dp - roll_rate * Dd
    aileron_value = max(min(aileron_value, 1.0), -1.0)
    
    target_alti = waypoint[2]
    current_alti = current_pos[2]
    alti_err = target_alti - current_alti
    # accumulated_alti_error += alti_err
    pitch_target = max(min(1.4*alti_err, 40.0), -40.0)
    pitch_error = pitch_target - pitch
    elevator_value = pitch_error * 0.045 - pitch_rate * 0.01 # + accumulated_alti_error * 0.0001
    elevator_value = max(min(elevator_value, 1.0), -1.0)
    
    rudder = 0.0
    send_control(client, 
                 elevator=elevator_value,
                 aileron=aileron_value,
                 rudder=rudder)
    
    text_data = f"""----- flight_data -----
waypoint:        [{"{:.3f}".format(waypoint[0])}, {"{:.3f}".format(waypoint[1])}, {"{:.3f}".format(waypoint[2])}]
current_pos:     [{"{:.3f}".format(current_pos[0])}, {"{:.3f}".format(current_pos[1])}, {"{:.3f}".format(current_pos[2])}]
target_heading:  {"{:.3f}".format(target_heading)}
current_heading: {"{:.3f}".format(yaw)}
error_heading:   {"{:.3f}".format(error_heading)}
roll_target:     {"{:.3f}".format(roll_target)}
pitch_target:    {"{:.3f}".format(pitch_target)}
pitch_error:     {"{:.3f}".format(pitch_error)}
-----------------------
dist_to_target:  {"{:.3f}".format(distance_to_target(current_pos, waypoint)*1000)}
rudder:          {"{:.3f}".format(rudder)}
aileron:         {"{:.3f}".format(aileron_value)}
elevator:        {"{:.3f}".format(elevator_value)}
"""
        
    print(text_data)
    # print(f'current_heading:       {"{:.3f}".format(yaw)}\nerror_heading:         {"{:.3f}".format(error_heading)}')
    return rudder, aileron, elevator
    
    

## In[5]
# set x-plane
switch_tab()
time.sleep(1)
set_camera_outside()
time.sleep(0.5)
# set_camera_behind()
# time.sleep(0.5)

## In[7]
# define variables

drefs = ['sim/flightmodel2/position/y_agl', # altitude ~ 0
         'sim/flightmodel/position/local_x', # position x ~ 1
         'sim/flightmodel/position/local_z', # position z (position y is the altitude) ~ 2
         'sim/flightmodel/position/indicated_airspeed', # Knots Indicated Airspeed (KIAS) ~ 3
         'sim/flightmodel/position/vh_ind', # vertical velocity ~ 4
         'sim/flightmodel/position/theta', # pitch angle ~ 5
         'sim/flightmodel/position/phi', # roll angle ~ 6
         'sim/flightmodel/position/psi', # yaw angle ~ 7
         'sim/flightmodel/position/Q', # pitch rate ~ 8
         'sim/flightmodel/position/P', # roll rate ~ 9
         'sim/flightmodel/position/R', # yaw rate ~ 10
         'sim/flightmodel2/misc/has_crashed', # crash or not ~ 11
         'sim/flightmodel/position/local_y', # position y ~ 12
         'sim/flightmodel/position/latitude', # latitude ~ 13
         'sim/flightmodel/position/longitude' # longitude ~ 14
         ]

has_crashed = 0.0
X = -998
target_altitude = 30
accumulated_pitch_error = 0
accumulated_alti_error = 0

phase = 'takeoff'
phase_info = phase
set_engine_and_flap_takeoff = True

## In[8]
# take off
while not has_crashed:
    dist_to_target = 99999.000
    
    # retrieve flight data
    flight_data = xplane.getDREFs(drefs)
    altitude = flight_data[0][0]
    pos_x = flight_data[1][0]
    pos_z = flight_data[2][0]
    pos_y = flight_data[12][0] # altitude in coordinate
    speed_kias = flight_data[3][0]
    vertical_vel = flight_data[4][0]
    pitch = flight_data[5][0]
    pitch_rate = flight_data[8][0]
    roll = flight_data[6][0]
    roll_rate = flight_data[9][0]
    yaw = flight_data[7][0]
    yaw_rate = flight_data[10][0]
    has_crashed = flight_data[11][0]
    latitude = flight_data[13][0]
    longitude = flight_data[14][0]
    
    current_pos = [latitude, longitude, altitude]
    
    # current_pos = [pos_x, pos_y, pos_z]
    
    if phase == 'takeoff':

        rudder, aileron, elevator, _, _ = stabilize_flight(xplane, 
                                                        yaw, yaw_rate,
                                                        roll, roll_rate)

        elevator = 0.0
        if set_engine_and_flap_takeoff:
            send_control(xplane, throttle=1.0, flaps=0.2)
            set_engine_and_flap_takeoff = False

        if speed_kias >= 18.0 and speed_kias < 20.0:
            send_control(xplane, elevator=0.4)
            elevator = 0.4
            
        if speed_kias > 20.0:
            phase = 'climb'
            elevator = 1.0

        
    elif phase == 'climb':
        
        if altitude < target_altitude:
            rudder, aileron, elevator, _, _ = stabilize_flight(xplane, 
                                                            yaw, yaw_rate,
                                                            roll, roll_rate,
                                                            pitch, pitch_rate,
                                                            pitch_target=10)
            
        else:
            phase = 'cruise'
            send_control(xplane, flaps=0.0)
            # send_control(xplane, elevator=0.0, throttle=0.4, flaps=0.0)
            # elevator = 0.0
    
    elif phase == 'cruise':
        # rudder, aileron, elevator, accumulated_pitch_error, accumulated_alti_error = stabilize_flight(xplane, 
        #                                    yaw, yaw_rate,
        #                                    roll, roll_rate,
        #                                    pitch, pitch_rate,
        #                                    accumulated_pitch_error=accumulated_pitch_error,
        #                                    current_alti=altitude, target_alti=target_altitude+5,
        #                                    ver_vel=vertical_vel, accumulated_alti_error=accumulated_alti_error)
        
        dist_to_target = distance_to_target(current_pos, waypoints[wp_index])
        if dist_to_target*1000 <= 50:
            wp_index += 1
            if wp_index >= len(waypoints):
                print('MISSION COMPLETE...')
                break
        
        rudder, aileron, elevator = go_to_waypoint(xplane, 
                                                   yaw, yaw_rate,
                                                   roll, roll_rate,
                                                   pitch, pitch_rate,
                                                   current_pos, waypoints[wp_index], vertical_vel)
        
        phase_info = f'going to waypoint-0{wp_index}'
        print(f'WAYPOINT-{wp_index}')
        
#     text_data = f"""
# ----- flight_data -----
# PHASE:          {phase_info}
# altitude:       {"{:.3f}".format(altitude)}
# ver_vel:        {"{:.3f}".format(vertical_vel)}
# speed_kias:     {"{:.3f}".format(speed_kias)}
# pitch:          {"{:.3f}".format(pitch)}
# roll:           {"{:.3f}".format(roll)}
# yaw:            {"{:.3f}".format(yaw)}
# -----------------------
# dist_to_target: {"{:.3f}".format(dist_to_target)}
# rudder:         {"{:.3f}".format(rudder)}
# aileron:        {"{:.3f}".format(aileron)}
# elevator:       {"{:.3f}".format(elevator)}
# """
        
#     print(text_data)
    
    time.sleep(0.1)



