import time
from utils import switch_tab, set_camera_behind, unpause_game
import xpc

# Set up the X-Plane Connect client
xplane = xpc.XPlaneConnect()

drefs = [
    'sim/flightmodel2/position/y_agl',  # altitude ~ 0
    'sim/flightmodel/position/indicated_airspeed',  # Knots Indicated Airspeed (KIAS) ~ 1
    'sim/flightmodel/position/theta',  # pitch angle ~ 2
    'sim/flightmodel/position/phi',  # roll angle ~ 3
    'sim/flightmodel/position/psi',  # yaw angle ~ 4
    'sim/flightmodel/position/Q',  # pitch rate ~ 5
    'sim/flightmodel/position/P',  # roll rate ~ 6
    'sim/flightmodel/position/R',  # yaw rate ~ 7
    'sim/flightmodel2/misc/has_crashed',  # crash or not ~ 8
]

def send_control(client, elevator=-998, aileron=-998, rudder=-998, throttle=-998, gear=-998, flaps=-998):
    controls = [elevator, aileron, rudder, throttle, gear, flaps]
    try:
        client.sendCTRL(controls)
    except:
        pass

def stabilize_flight(client, yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                     yaw_target, roll_target, pitch_target,
                     Pr=0.1, Dr=0.01, Pa=0.01, Da=0.001, Pe=0.1, De=0.01):
    # Yaw stabilization
    yaw_err = yaw_target - yaw
    rudder_value = max(min(yaw_err * Pr - yaw_rate * Dr, 1.0), -1.0)
    
    # Roll stabilization
    roll_err = roll_target - roll
    aileron_value = max(min(roll_err * Pa - roll_rate * Da, 1.0), -1.0)
    
    # Pitch stabilization
    pitch_err = pitch_target - pitch
    elevator_value = max(min(pitch_err * Pe - pitch_rate * De, 1.0), -1.0)
    
    send_control(client, elevator=elevator_value, aileron=aileron_value, rudder=rudder_value)
    return rudder_value, aileron_value, elevator_value

switch_tab()
time.sleep(1)
set_camera_behind()
unpause_game()
time.sleep(0.5)

phase = 'approach'
target_altitude = 10.0  # Descend to 10 meters before final landing
target_speed = 20.0  # Slow down before landing
flare_altitude = 3.0  # Flare at 3 meters
landing_complete = False

while not landing_complete:
    flight_data = xplane.getDREFs(drefs)
    altitude, speed_kias, pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate, has_crashed = [fd[0] for fd in flight_data]
    
    if has_crashed:
        print("Crashed! Ending simulation.")
        break
    
    if phase == 'approach':
        send_control(xplane, throttle=0.25, flaps=0.5)  # Reduce throttle, extend flaps
        stabilize_flight(xplane, yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                         yaw_target=225, roll_target=0, pitch_target=-3)
        if altitude <= target_altitude:
            phase = 'final_descent'
    
    elif phase == 'final_descent':
        send_control(xplane, throttle=0.1)  # Further reduce throttle
        stabilize_flight(xplane, yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                         yaw_target=225, roll_target=0, pitch_target=-5)
        if altitude <= flare_altitude:
            phase = 'flare'
    
    elif phase == 'flare':
        send_control(xplane, throttle=0.0)  # Cut throttle
        stabilize_flight(xplane, yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                         yaw_target=225, roll_target=0, pitch_target=2)  # Slightly nose up
        if altitude < 0.5:  # Touchdown
            phase = 'landing'
    
    elif phase == 'landing':
        send_control(xplane, throttle=0.0, flaps=0.0, gear=1)  # Deploy landing gear
        print("Landing complete.")
        landing_complete = True
    
    print(f"PHASE: {phase}, ALT: {altitude:.2f}, SPD: {speed_kias:.2f}, PITCH: {pitch:.2f}")
    time.sleep(0.1)
