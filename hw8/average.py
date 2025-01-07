import pandas as pd
import matplotlib.pyplot as plt

# Initialize lists to store the results
agent_counts = list(range(2, 7))  # m = 2 to m = 6
average_times = []
average_tree_sizes = []

# Loop over each agent count and process the corresponding CSV file
for m in agent_counts:
    # Load the CSV file for each agent count
    file_path = f"/home/steve0gillet/Desktop/algMotionPlanning/hw8/benchmarkCentralized{m}.csv"
    data = pd.read_csv(file_path)
    
    # Compute the average computation time and tree size
    average_time = data["ComputationTime"].mean()
    average_tree_size = data["TreeSize"].mean()
    
    # Append to lists
    average_times.append(average_time)
    average_tree_sizes.append(average_tree_size)

# Plot 1: Average computation time vs. number of agents
plt.figure()
plt.plot(agent_counts, average_times, marker='o')
plt.xlabel("Number of Agents (m)")
plt.ylabel("Average Computation Time (ms)")
plt.title("Average Computation Time vs. Number of Agents")
plt.grid(True)
plt.show()

# Plot 2: Average size of tree vs. number of agents
plt.figure()
plt.plot(agent_counts, average_tree_sizes, marker='o')
plt.xlabel("Number of Agents (m)")
plt.ylabel("Average Size of Tree")
plt.title("Average Tree Size vs. Number of Agents")
plt.grid(True)
plt.show()
