import numpy as np

# Define your matrices and vectors here. These are placeholders.
A = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, -.1, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, -.1, 0, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, -.15]
])

B = np.array([
    [0.0001, 0, 0, 0],
    [-.1, 0.0001, .1, 0],
    [0, 0, 0.0001, 0],
    [0, -.1, 0, .00011],
    [.0001, 0, 0.0001, 0],
    [0.475, 0.475, 0.475, 0.475]
])

qNear = np.array([5.8, 3.47, 3.211, -1.79, 6.93, -14.65]) # Current state
point = np.array([5.57, 4.8273, 1.8, 3.9, 6.1, -15]) # Target state
dt = 200 # Time step
m = 2.0 # Mass
g = 9.80 # Gravitational acceleration

# Calculate xDot
xDot = (point - qNear) / dt

# Calculate control input using pseudo-inverse of B
# Assuming B might not be square, hence the use of pseudo-inverse
B_pinv = np.linalg.pinv(B)
potentialControl = np.dot(B_pinv, (xDot - np.dot(A, qNear)))

print("Calculated Control:", potentialControl)

# Calculate xDotActual for verification
xDotActual = np.dot(A, qNear) + np.dot(B, potentialControl)
print("xDot Desired vs Actual:")
print("xDot:", xDot)
print("xDotActual:", xDotActual)