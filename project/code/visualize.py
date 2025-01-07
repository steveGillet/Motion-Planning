import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pandas as pd
import numpy as np

# Load data
path = pd.read_csv("pathPreSmooth.csv")
pathSmooth = pd.read_csv("pathPostSmooth.csv")
obstacleData = pd.read_csv("obstacles.csv")  # With columns: id, x, y, z

# Start and goal points
start_point = path.iloc[0]  # First waypoint
goal_point = path.iloc[-1]  # Last waypoint

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the path
ax.plot(path['x'], path['y'], path['z'], label="Path", color="yellow", linewidth=2)
ax.plot(pathSmooth['x'], pathSmooth['y'], pathSmooth['z'], label="Path Smoothed", color="orange", linewidth=2)

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

# Plot start and goal points as spheres
ax.scatter(
    [start_point['x']], [start_point['y']], [start_point['z']],
    color="green", s=100, label="Start Point", edgecolors="black", zorder=10
)
ax.scatter(
    [goal_point['x']], [goal_point['y']], [goal_point['z']],
    color="red", s=100, label="Goal Point", edgecolors="black", zorder=10
)

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Path Planning Visualization with Obstacles')

# Add legend
ax.legend()

# Show plot
plt.show()
