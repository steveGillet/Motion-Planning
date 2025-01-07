import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
data = pd.read_csv("benchmarkCentralized6.csv")

# Create a boxplot for Computation Time
plt.figure(figsize=(10, 5))
plt.boxplot(data['ComputationTime'], vert=False, patch_artist=True, 
            boxprops=dict(facecolor='skyblue', color='blue'),
            medianprops=dict(color='red'))
plt.title("Computation Time (ms) across Simulations")
plt.xlabel("Time (ms)")
plt.grid(True, linestyle='--', alpha=0.6)
plt.savefig("computation_time_boxplot.png")
plt.show()

# Create a boxplot for Tree Size
plt.figure(figsize=(10, 5))
plt.boxplot(data['TreeSize'], vert=False, patch_artist=True, 
            boxprops=dict(facecolor='lightgreen', color='green'),
            medianprops=dict(color='red'))
plt.title("RRT Tree Size across Simulations")
plt.xlabel("Tree Size")
plt.grid(True, linestyle='--', alpha=0.6)
plt.savefig("tree_size_boxplot.png")
plt.show()
