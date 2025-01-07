import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files for each workspace into separate DataFrames
df_ws1 = pd.read_csv('e2bw1.csv')
df_ws2 = pd.read_csv('e2bw2.csv')
df_ws3 = pd.read_csv('e2bw3.csv')

# Label the data by workspace
df_ws1['workspace'] = 'Workspace 1'
df_ws2['workspace'] = 'Workspace 2'
df_ws3['workspace'] = 'Workspace 3'

# Combine the data from the three workspaces into a single DataFrame
df_all = pd.concat([df_ws1, df_ws2, df_ws3])

# Filter for valid paths only (where valid == 1)
df_valid = df_all[df_all['valid'] == 1]

# Generate boxplot for path length across workspaces
plt.figure(figsize=(10, 6))
plt.boxplot([df_valid[df_valid['workspace'] == ws]['path_length'] for ws in df_valid['workspace'].unique()])
plt.xticks(range(1, len(df_valid['workspace'].unique()) + 1), df_valid['workspace'].unique(), rotation=45)

plt.title('Path Lengths Across Workspaces')
plt.ylabel('Path Length')
plt.xlabel('Workspace')
plt.tight_layout()
plt.show()

# Generate boxplot for computation time across workspaces
plt.figure(figsize=(10, 6))
plt.boxplot([df_all[df_all['workspace'] == ws]['computation_time'] for ws in df_all['workspace'].unique()])
plt.xticks(range(1, len(df_all['workspace'].unique()) + 1), df_all['workspace'].unique(), rotation=45)

plt.title('Computation Times Across Workspaces')
plt.ylabel('Computation Time (seconds)')
plt.xlabel('Workspace')
plt.tight_layout()
plt.show()

# Bar plot for the number of valid solutions (successes) across workspaces
success_counts = df_all.groupby('workspace')['valid'].sum()

plt.figure(figsize=(10, 6))
plt.bar(success_counts.index, success_counts)
plt.xticks(rotation=45)

plt.title('Number of Valid Solutions Across Workspaces')
plt.ylabel('Number of Valid Solutions')
plt.xlabel('Workspace')
plt.tight_layout()
plt.show()
