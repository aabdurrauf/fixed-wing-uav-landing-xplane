# import pandas as pd
# import numpy as np
# from scipy.linalg import lstsq

# # Step 1: Load the collected X-Plane data
# df = pd.read_excel("D:\\Projects\\FixedWingUAV_vsc\\uav_state_data_9Hz.xlsx")

# # Extract state variables (X) and control inputs (U)
# X = df[['Altitude', 'Vertical Speed', 'Horizontal Speed', 'Pitch Angle', 'Pitch Rate']].values
# U = df[['Elevator Control', 'Throttle']].values  # Control inputs

# # Compute time step (assuming constant sampling interval)
# dt = 0.1132  # Example: 50ms sampling (adjust as per your data collection)

# # Step 2: Compute X_dot (State Derivatives)
# X_dot = np.diff(X, axis=0) / dt  # Approximate time derivative using finite differences
# U = U[:-1, :]  # Remove last row to match X_dot size

# # Step 3: Solve for A and B using Least Squares
# AB = np.hstack((X[:-1, :], U))  # Combine X and U into a single regression matrix
# X_dot = X_dot  # Target values for regression

# # Solve the least squares problem X_dot = AB * Theta
# Theta, _, _, _ = lstsq(AB, X_dot)

# # Extract A and B matrices
# n_states = X.shape[1]
# A = Theta[:n_states, :].T  # State Transition Matrix
# B = Theta[n_states:, :].T  # Control Input Matrix

# print("Estimated A Matrix:\n", A)
# print("Estimated B Matrix:\n", B)


### version 2

import pandas as pd
import numpy as np
from scipy.linalg import lstsq
from scipy.signal import savgol_filter
from sklearn.linear_model import Ridge

# Step 1: Load the collected X-Plane data
df = pd.read_excel("D:\\Projects\\FixedWingUAV_vsc\\uav_state_data_9Hz.xlsx")

# Extract state variables (X) and control inputs (U)
X = df[['Altitude', 'Vertical Speed', 'Horizontal Speed', 'Pitch Angle', 'Pitch Rate']].values
U = df[['Elevator Control', 'Throttle']].values  # Control inputs

# Compute time step (assuming constant sampling interval)
dt = 0.1132  # Adjust as per actual sampling rate

# Step 2: Compute X_dot (State Derivatives)
X_smoothed = savgol_filter(X, window_length=5, polyorder=2, axis=0)  # Smooth X
X_dot = np.gradient(X_smoothed, dt, axis=0)  # Better derivative

# Align U with X_dot (midpoint averaging)
U = (U[1:] + U[:-1]) / 2  

X_dot = X_dot[:-1, :]  # Remove last row to match X[:-1, :] and U

# Step 3: Solve for A and B using Ridge Regression (more stable than lstsq)
AB = np.hstack((X[:-1, :], U))  # Combine X and U into a single regression matrix
ridge = Ridge(alpha=0.1)  # Small regularization factor
ridge.fit(AB, X_dot)
Theta = ridge.coef_

# Extract A and B matrices
n_states = X.shape[1]  # Number of state variables
n_inputs = U.shape[1]   # Number of control inputs

A = Theta[:, :n_states].T  # State Transition Matrix (Corrected)
B = Theta[:, n_states:].T  # Control Input Matrix (Corrected)


print("Estimated A Matrix:\n", A)
print("Estimated B Matrix:\n", B)
