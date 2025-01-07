import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation
import pandas as pd
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# Load data
path = pd.read_csv("pathPreSmooth.csv")
obstacleData = pd.read_csv("obstacles.csv") 

# Separate out position and velocity
positions = path[['x', 'y', 'z']].to_numpy()
velocities = path[['v_x', 'v_y', 'v_z']].to_numpy()

# Calculate time steps based on velocity (assuming constant acceleration between points)
dt = 0.1  # Time step for simulation, adjust as needed
times = [0]
for i in range(1, len(positions)):
    dist = np.linalg.norm(positions[i] - positions[i-1])
    v_avg = (velocities[i] + velocities[i-1]) / 2  # Average velocity between two points
    v_magnitude = np.linalg.norm(v_avg)
    times.append(times[-1] + dist / v_magnitude if v_magnitude != 0 else 0)

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the path
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', label='Path')
ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='g', s=100, label='Start')
ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='r', s=100, label='End')

# Initialize the drone position
drone, = ax.plot([positions[0, 0]], [positions[0, 1]], [positions[0, 2]], 'ko', markersize=10, label='Drone')

# Group obstacle vertices by ID and plot each as a filled shape
for (obstacleID, face), group in obstacleData.groupby(['id', 'face']):
    faceVertices = group[['x', 'y', 'z']].values

    for i in range(len(faceVertices)):
        v0 = faceVertices[i]
        v1 = faceVertices[(i+1)%len(faceVertices)]
        v2 = faceVertices[(i+2)%len(faceVertices)]
        
        poly3d = Poly3DCollection([[v0, v1, v2]], alpha=0.5, edgecolor="k")
        poly3d.set_facecolor("red")  # Change color if needed
        ax.add_collection3d(poly3d)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Drone Movement Visualization')
ax.legend()

# Animation function which updates the drone's position
def update(frame):
    t = frame * dt
    for i in range(1, len(positions)):
        if t <= times[i]:
            # Interpolate between the last point and the current point
            fraction = (t - times[i-1]) / (times[i] - times[i-1])
            pos = positions[i-1] + fraction * (positions[i] - positions[i-1])
            drone.set_data([pos[0]], [pos[1]])
            drone.set_3d_properties([pos[2]], 'z')
            return drone,

# Create animation
anim = FuncAnimation(fig, update, frames=int(max(times)/dt), interval=50, blit=True)

plt.show()