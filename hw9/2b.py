import pandas as pd
import matplotlib.pyplot as plt

# Load the control data
data = pd.read_csv('2b.csv')

# Plot velocity vs. time
plt.figure(figsize=(10, 5))
plt.plot(data['Time'], data['Velocity'], label='Velocity', color='b')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity vs Time')
plt.legend()
plt.grid()
plt.show()

# Plot steering angle vs. time
plt.figure(figsize=(10, 5))
plt.plot(data['Time'], data['SteeringAngle'], label='Steering Angle', color='r')
plt.xlabel('Time (s)')
plt.ylabel('Steering Angle (rad)')
plt.title('Steering Angle vs Time')
plt.legend()
plt.grid()
plt.show()
