# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 21:59:03 2024

@author: ammar
"""
# =============================================================================
#     This UAVs class is for controlling the AI aircrafts.
#     Each UAV must have its own instance of this class.
# =============================================================================

import xpc
import time
import numpy as np
    
class UAVs:
    
    def __init__(self, xplane_client=None, uav_num=1):
        assert xplane_client is not None, "please define the XPC client"
        if uav_num == 1:
            print('Do not forget to override the AI aircrafts:')
            print('sim/operation/override/override_plane_ai_autopilot\n')
        
        self.xplane = xplane_client
        self.uav_num = uav_num
        self.uav_name = f'plane{self.uav_num}'
    
    def set_altitude(self, alt):
        X = -998  # set value to -998 to keep unchanged
        values = [X, X, alt, X, X, X, X]
        try:
            self.xplane.sendPOSI(values, ac=self.uav_num)
        except:
            print(f"Error setting the altitude of {self.uav_name}.")
    
    
    def shift_coordinate(self, x=None, y=None, z=None):
        if type(x) == int or type(x) == float:
            self.set_coordinate(x=self.xplane.getDREF(
                f'sim/multiplayer/position/{self.uav_name}_x')[0] + x)
        
        if type(y) == int or type(y) == float:
            self.set_coordinate(y=self.xplane.getDREF(
                f'sim/multiplayer/position/{self.uav_name}_y')[0] + y)
                
        if type(z) == int or type(z) == float:
            self.set_coordinate(z=self.xplane.getDREF(
                f'sim/multiplayer/position/{self.uav_name}_z')[0] + z)
    
    
    def set_coordinate(self, x=None, y=None, z=None):
        if type(x) == int or type(x) == float:
            try:
                self.xplane.sendDREF(f'sim/multiplayer/position/{self.uav_name}_x', x)
            except:
                print(f"Error setting the x coordinate of {self.uav_name}.")
        
        if type(y) == int or type(y) == float:
            try:
                self.xplane.sendDREF(f'sim/multiplayer/position/{self.uav_name}_y', y)
            except:
                print(f"Error setting the y coordinate of {self.uav_name}.") 
                
        if type(z) == int or type(z) == float:
            try:
                self.xplane.sendDREF(f'sim/multiplayer/position/{self.uav_name}_z', z)
            except:
                print(f"Error setting the z coordinate of {self.uav_name}.")      
                
            
    def set_velocity(self, vx=None, vy=None, vz=None):
        if type(vx) == int or type(vx) == float:
            try:
                self.xplane.sendDREF(f'sim/multiplayer/position/{self.uav_name}_v_x', vx)
            except:
                print(f"Error setting the velocity x of {self.uav_name}.")
        if type(vy) == int or type(vy) == float:
            try:
                self.xplane.sendDREF(f'sim/multiplayer/position/{self.uav_name}_v_y', vy)
            except:
                print(f"Error setting the velocity y of {self.uav_name}.")
        if type(vz) == int or type(vz) == float:
            try:
                self.xplane.sendDREF(f'sim/multiplayer/position/{self.uav_name}_v_z', vz)
            except:
                print(f"Error setting the velocity z of {self.uav_name}.")            
            
            
    def set_position(self, latitude=-998, longitude=-998, altitude=-998, 
                     pitch=-998, roll=-998, true_head=-998, gear=-998):
        
        if true_head != -998:
            ovr = [0] * 20
            ovr[self.uav_num] = 1
            self.xplane.sendDREF('sim/operation/override/override_planepath', ovr)
            
        values = [latitude, longitude, altitude, pitch, roll, true_head, gear]
        try:
            self.xplane.sendPOSI(values, ac=self.uav_num)
        except:
            print(f"Error setting the position of {self.uav_name}.")
        
        time.sleep(0.5)
        if true_head != -998:
            ovr = [0] * 20
            self.xplane.sendDREF('sim/operation/override/override_planepath', ovr)
    
    
    def send_throttle(self, throttle=1):
        self.xplane.sendDREF(f'sim/multiplayer/position/{self.uav_name}_throttle', 
                             [throttle, 0, 0, 0, 0, 0, 0, 0])


    def send_control(self, elevator=None, aileron=None, rudder=None):
        if type(elevator) == int or type(elevator) == float:
            try:
                elev_values = self.xplane.getDREF('sim/multiplayer/controls/yoke_pitch_ratio')
                elev_values[self.uav_num] = elevator
                self.xplane.sendDREF('sim/multiplayer/controls/yoke_pitch_ratio', elev_values)
            except:
                print(f"Error setting the {self.uav_name} elevator.")
                
        if type(aileron) == int or type(aileron) == float:
            try:
                ail_values = self.xplane.getDREF('sim/multiplayer/controls/yoke_roll_ratio')
                ail_values[self.uav_num] = aileron
                self.xplane.sendDREF('sim/multiplayer/controls/yoke_roll_ratio', ail_values)
            except:
                print(f"Error setting the {self.uav_name} aileron.")
                
        if type(rudder) == int or type(rudder) == float:
            try:
                rud_values = self.xplane.getDREF('sim/multiplayer/controls/yoke_heading_ratio')
                rud_values[self.uav_num] = rudder
                self.xplane.sendDREF('sim/multiplayer/controls/yoke_heading_ratio', rud_values)
            except:
                print(f"Error setting the {self.uav_name} rudder.")
                
    def get_states(self):
        
        drefs = [f'sim/multiplayer/position/{self.uav_name}_el', # altitude
                 f'sim/multiplayer/position/{self.uav_name}_lat', # latitude
                 f'sim/multiplayer/position/{self.uav_name}_lon', # longitude
                 f'sim/multiplayer/position/{self.uav_name}_the', # pitch angle
                 f'sim/multiplayer/position/{self.uav_name}_phi', # roll angle
                 f'sim/multiplayer/position/{self.uav_name}_psi', # yaw angle
                 f'sim/multiplayer/position/{self.uav_name}_x', # position x (position y is the altitude)
                 f'sim/multiplayer/position/{self.uav_name}_y', # position y (position y is the altitude)
                 f'sim/multiplayer/position/{self.uav_name}_z', # position z (position y is the altitude)
                 f'sim/multiplayer/position/{self.uav_name}_v_x', # velocity across x axis
                 f'sim/multiplayer/position/{self.uav_name}_v_y', # velocity across y axis
                 f'sim/multiplayer/position/{self.uav_name}_v_z' # velocity across z axis
                 ]

        try:
            self.xplane.clearBuffer()
            values = self.xplane.getDREFs(drefs)
            values = np.array(values).flatten()            
            
            if len(values) < 12:
                raise Exception()

        except:
            self.client.clearBuffer()
            # set values to 0 if error occurred in communication with x-plane
            values = np.zeros(12)
            
        return values


    def set_view_spot(self):
        # self.client.sendVIEW(xpc.ViewType.Spot)
        self.xplane.sendVIEW(xpc.ViewType.Follow)
    
    def pause_sim(self):
        self.xplane.pauseSim(True)
    
    def resume_sim(self):
        self.xplane.pauseSim(False)
        