import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Example 1: Line within the face
def plot_example1(ax):
    # Define the face (triangle)
    A1, B1, C1 = np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0])
    
    # Define the line
    line_start1, line_end1 = np.array([0.3, 0.3, 0]), np.array([0.7, 0.7, 0])
    
    # Plot the triangle
    ax.plot([A1[0], B1[0], C1[0], A1[0]], [A1[1], B1[1], C1[1], A1[1]], [A1[2], B1[2], C1[2], A1[2]], color='b', alpha=0.5)
    
    # Plot the line
    ax.plot([line_start1[0], line_end1[0]], [line_start1[1], line_end1[1]], [line_start1[2], line_end1[2]], color='r')

# Example 2: Line not in the face
def plot_example2(ax):
    # Define the face (same triangle)
    A2, B2, C2 = np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0])
    
    # Define the line
    line_start2, line_end2 = np.array([0.5, 0.5, 1]), np.array([0.5, 0.5, -1])
    
    # Plot the triangle
    ax.plot([A2[0], B2[0], C2[0], A2[0]], [A2[1], B2[1], C2[1], A2[1]], [A2[2], B2[2], C2[2], A2[2]], color='b', alpha=0.5)
    
    # Plot the line
    ax.plot([line_start2[0], line_end2[0]], [line_start2[1], line_end2[1]], [line_start2[2], line_end2[2]], color='r')

# Create a figure with subplots
fig = plt.figure(figsize=(12, 6))

# Example 1
ax1 = fig.add_subplot(1, 2, 1, projection='3d')
plot_example1(ax1)
ax1.set_title('Line Intersecting Face')

# Example 2
ax2 = fig.add_subplot(1, 2, 2, projection='3d')
plot_example2(ax2)
ax2.set_title('Line Not Intersecting Face')

# Adjusting the view angle for better visualization
ax1.view_init(elev=20., azim=-35)
ax2.view_init(elev=20., azim=-35)

# Labels and limits for better clarity
for ax in (ax1, ax2):
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([0, 1])
    ax.set_ylim([0, 1])
    ax.set_zlim([-1, 1])

plt.tight_layout()
plt.show()