import pandas as pd
import matplotlib.pyplot as plt
import glob
import re

# Initialize lists to store the data for each configuration
computation_times = []
path_costs = []
success_rates = []
labels = []

# Loop over all relevant CSV files
for filename in sorted(glob.glob("1dii*.csv")):
    # Extract information from filename for labeling
    match = re.match(r'1dii(\d)(\d+)\.csv', filename)
    if match:
        agent = int(int(match.group(1)) / 2 + 1)    # 4th index character (agent)
        num_samples = int(match.group(2))  # 5th index on (number of samples)
        label = f"Agent {agent}, {num_samples} State Sample(s)"
        
        # Load the data
        df = pd.read_csv(filename)
        
        # Extract necessary columns
        computation_times.append(df['ComputationTime'])
        path_costs.append(df['PathCost'])
        success_rates.append(df['ValidPath'].sum() / len(df))  # Calculate success rate as ratio
        labels.append(label)

# Sort data by agent and number of samples
sorted_data = sorted(zip(labels, computation_times, path_costs, success_rates), key=lambda x: (int(re.search(r'Agent (\d+)', x[0]).group(1)), int(re.search(r'(\d+) State Sample', x[0]).group(1))))
labels, computation_times, path_costs, success_rates = zip(*sorted_data)

# Plot Computation Times Boxplot
plt.figure(figsize=(12, 6))
plt.boxplot(computation_times, labels=labels, vert=False)
plt.title('Computation Times for Different Configurations')
plt.xlabel('Computation Time (s)')
plt.ylabel('Configuration')
plt.show()

# Plot Path Costs Boxplot
plt.figure(figsize=(12, 6))
plt.boxplot(path_costs, labels=labels, vert=False)
plt.title('Path Costs for Different Configurations')
plt.xlabel('Path Cost')
plt.ylabel('Configuration')
plt.show()

# Plot Success Rates
plt.figure(figsize=(12, 6))
plt.barh(labels, success_rates)
plt.title('Success Rates for Different Configurations')
plt.xlabel('Success Rate')
plt.ylabel('Configuration')
plt.xlim(0, 1)
plt.show()
