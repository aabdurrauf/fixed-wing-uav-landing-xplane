import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

# UAV System Matrices (from estimation)
# A = np.array([
#     [-3.61318326e-02,  1.27775384e-01, -2.01126135e-03,  2.71957130e-01, 4.89518608e-02],
#     [-4.55353014e-02, -6.74244202e-01,  2.20080180e-02,  2.55149537e-01, 1.60716268e-02],
#     [ 1.23580882e-02, -6.68101144e-02, -5.31437649e-02, -1.39415642e-01, -4.57938886e-02],
#     [-4.29432923e-02,  1.94339387e-01,  2.05697734e-02, -8.08521480e-02, 9.94257156e-01],
#     [-3.59430999e-01,  2.49732643e+00,  8.34532783e-01, -1.41344887e+00, -8.88225175e-01]
# ])

# B = np.array([
#     [-5.73305175, -0.25401044],
#     [-4.99742858, -0.43290374],
#     [ 3.48269989,  2.31105271],
#     [ 1.52470841,  0.47419379],
#     [84.93820867,  5.21836285]
# ])

### A B Matrices version 2
A = np.array([
    [-0.04337676, -0.0348592,  0.01674898,  0.01281664, -0.118132],
    [ 0.37843768, -0.88917593, -0.11030596, -2.50129042,  3.77629519],
    [ 0.10725369,  0.11273493, -0.09997649, -0.11942229,  0.17452466],
    [ 0.24130543,  0.30812949, -0.14042204,  0.53054868, -1.30677151],
    [ 0.03383849,  0.05214188, -0.03963042,  1.5847158,   0.19518028]
])

B = np.array([
    [-1.7568457,  -0.24982281],
    [-4.06601517, -0.22660005],
    [ 2.19740751,  2.38126848],
    [-12.34338271,  0.50864361],
    [ 45.32597124,  2.8961817]
])


# Cost matrices (tuned for smooth landing)
# Q = np.diag([0.2, 0.1, 0.01, 0.01, 0.005])  # Reduce weights on altitude & vertical speed
# R = np.diag([10, 10])  # Increase control effort penalty to avoid large inputs
Q = np.diag([100, 50, 1, 1, 1])  # Larger values for altitude and vertical speed
R = np.diag([10, 10])  # Decrease control penalty

# Solve Riccati equation to compute LQR gain
P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P  # LQR gain matrix
K *= 0.0001
print("K: ", K)

# Simulation parameters
dt = 0.1132  # Time step (matches data collection rate)
num_steps = 5  # Simulate for fewer steps, adjust as needed

# Initial UAV state (Altitude, Vertical Speed, Horizontal Speed, Pitch Angle, Pitch Rate)
x = np.array([50, -0.3, 0, 0, 0])  # More neutral initial conditions
# x = np.array([0.1, 0, 0, 0, 0])

# Storage for plotting
altitudes = []
vertical_speeds = []
elevator_inputs = []
throttle_inputs = []

# Run simulation
for _ in range(num_steps):
    u = -K @ x  # Compute optimal control (elevator, throttle)
    print("u", u)
    x = A @ x + B @ u  # Update state using system dynamics
    print("x", x)
    # Store values
    altitudes.append(x[0])
    vertical_speeds.append(x[1])
    elevator_inputs.append(u[0])
    throttle_inputs.append(u[1])

# Plot results
plt.figure(figsize=(12, 5))

plt.subplot(2, 2, 1)
plt.plot(altitudes, label="Altitude (m)")
plt.xlabel("Time Step")
plt.ylabel("Altitude")
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(vertical_speeds, label="Vertical Speed (m/s)", color='r')
plt.xlabel("Time Step")
plt.ylabel("Vertical Speed")
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(elevator_inputs, label="Elevator Control", color='g')
plt.xlabel("Time Step")
plt.ylabel("Elevator Input")
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(throttle_inputs, label="Throttle Control", color='m')
plt.xlabel("Time Step")
plt.ylabel("Throttle Input")
plt.legend()

plt.tight_layout()
plt.show()

# eig_vals, _ = np.linalg.eig(A)
# print("Eigenvalues of A:", eig_vals)
