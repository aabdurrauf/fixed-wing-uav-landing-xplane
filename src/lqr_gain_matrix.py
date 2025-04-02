import numpy as np
import scipy.linalg

# Define A and B matrices (replace with your estimated ones)
A = np.array([
    [-3.61318326e-02,  1.27775384e-01, -2.01126135e-03,  2.71957130e-01, 4.89518608e-02],
    [-4.55353014e-02, -6.74244202e-01,  2.20080180e-02,  2.55149537e-01, 1.60716268e-02],
    [ 1.23580882e-02, -6.68101144e-02, -5.31437649e-02, -1.39415642e-01, -4.57938886e-02],
    [-4.29432923e-02,  1.94339387e-01,  2.05697734e-02, -8.08521480e-02, 9.94257156e-01],
    [-3.59430999e-01,  2.49732643e+00,  8.34532783e-01, -1.41344887e+00, -8.88225175e-01]
])

B = np.array([
    [-5.73305175, -0.25401044],
    [-4.99742858, -0.43290374],
    [ 3.48269989,  2.31105271],
    [ 1.52470841,  0.47419379],
    [84.93820867,  5.21836285]
])

# Define cost matrices
Q = np.diag([10, 10, 5, 20, 10])  # Penalizing altitude, vertical speed, and pitch heavily
R = np.diag([1, 1])  # Penalizing large control efforts equally

# Solve Riccati equation for P
P = scipy.linalg.solve_continuous_are(A, B, Q, R)

# Compute LQR gain matrix K
K = np.linalg.inv(R) @ B.T @ P

print("LQR Gain Matrix K:\n", K)
