import xpc
import time
import numpy as np
import pandas as pd
from datetime import datetime

# set the xpc client
xplane = xpc.XPlaneConnect()

import xpc
import numpy as np

# Set the xpc client
xplane = xpc.XPlaneConnect()

# Function to get UAV states from X-Plane
def get_states():
    drefs = [
        'sim/flightmodel2/position/y_agl',  # Altitude above ground
        'sim/flightmodel/position/vh_ind',  # Vertical velocity
        'sim/flightmodel/position/groundspeed',  # Horizontal speed
        'sim/flightmodel/position/theta',  # Pitch angle
        'sim/flightmodel/position/Q',  # Pitch rate
        'sim/cockpit2/tcas/targets/position/throttle',  # Throttle
        'sim/cockpit2/tcas/targets/position/yolk_pitch',  # Elevator Control
    ]

    try:
        xplane.clearBuffer()
        values = xplane.getDREFs(drefs)
        
        # Flatten and extract first element from each tuple if necessary
        values = np.array([v[0] if isinstance(v, tuple) else v for v in values])

        # Ensure we have the correct number of values
        if len(values) < len(drefs):
            raise Exception("Incomplete data received")

    except Exception as e:
        xplane.clearBuffer()
        print("Error:", e)
        values = np.zeros(len(drefs))  # Set to zero if communication fails

    return values


# Placeholder to hold the states (NumPy array)
num_samples = 2500  # Number of data points to collect
state_history = np.zeros((num_samples, 7))  # 7 state variables

# Collect data
loop_time = time.time()
for i in range(num_samples):
    print('interval: {}'.format(time.time() - loop_time))
    loop_time = time.time()
    state_history[i] = get_states()
    print(f"Data {i+1}: {state_history[i]}")
    # time.sleep(0.05) # 20 Hz frequency

# Convert data to Pandas DataFrame
df = pd.DataFrame(state_history, columns=["Altitude", "Vertical Speed", "Horizontal Speed", 
                                          "Pitch Angle", "Pitch Rate", "Throttle", "Elevator Control"])

# Save to Excel
timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
output_filename = f"uav_state_data_{timestamp}.xlsx"
with pd.ExcelWriter(output_filename) as writer:
    df.to_excel(writer, index=False)

print(f"Data saved to {output_filename}")