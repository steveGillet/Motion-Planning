import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
df = pd.read_csv('e2bw1.csv')

# Combine n and r into a single column for labeling
df['n_r'] = df['n'].astype(str) + ", " + df['r'].astype(str)

# Generate a boxplot for path length by (n, r) pairs
plt.figure(figsize=(10, 6))
df_valid = df[df['valid'] == 1]  # Only include valid paths
plt.boxplot([df_valid[df_valid['n_r'] == label]['path_length'] for label in sorted(df_valid['n_r'].unique())])
plt.xticks(range(1, len(df_valid['n_r'].unique()) + 1), sorted(df_valid['n_r'].unique()), rotation=45)

plt.title('Path Lengths for Different (n, r) Pairs')
plt.ylabel('Path Length')
plt.xlabel('(n, r)')
plt.tight_layout()
plt.show()

# Boxplot for computation time
plt.figure(figsize=(10, 6))
plt.boxplot([df[df['n_r'] == label]['computation_time'] for label in sorted(df['n_r'].unique())])
plt.xticks(range(1, len(df['n_r'].unique()) + 1), sorted(df['n_r'].unique()), rotation=45)

plt.title('Computation Times for Different (n, r) Pairs')
plt.ylabel('Computation Time (seconds)')
plt.xlabel('(n, r)')
plt.tight_layout()
plt.show()

# Generate a boxplot for the number of successes by (n, r) pairs
# Count the number of valid solutions (success == 1) for each (n, r) pair
success_counts = df.groupby('n_r')['valid'].sum()

# Since we're plotting counts (not distributions), we use bar plots instead of boxplots
plt.figure(figsize=(10, 6))
plt.bar(success_counts.index, success_counts)
plt.xticks(rotation=45)

plt.title('Number of Successes for Different (n, r) Pairs')
plt.ylabel('Number of Successes')
plt.xlabel('(n, r)')
plt.tight_layout()
plt.show()
