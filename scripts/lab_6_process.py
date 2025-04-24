import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

### Map build time ###
# file_name = "map.csv"
# file_path = f"../data/lab_6/{file_name}"

# df = pd.read_csv(file_path)

# # Get the data from a specific column
# column_name = "time"  # Replace with your actual column name
# column_data = df[column_name]

# print(np.mean(column_data.to_list()[1:]))

### A* time ###
# file_name = "a_star_short.csv"
# file_name = "a_star_medium.csv"
# file_name = "a_star_long.csv"
# file_path = f"../data/lab_6/{file_name}"

# df = pd.read_csv(file_path)

# # Get the data from a specific column
# column_name = "time"  # Replace with your actual column name
# column_data = df[column_name]

# print(np.mean(column_data.to_list()[1:]))

### Cross track error ###
file_name = "cte_real.csv"
file_path = f"../data/lab_6/{file_name}"

df = pd.read_csv(file_path)

time = df["timestamp"]
cte = df["cross_track error"]

# Convert to numeric just in case there are strings
cte = pd.to_numeric(cte, errors='coerce')
time = pd.to_numeric(time, errors='coerce')

# Normalize time to start at 0
time = time - time.iloc[0]

# Drop initial zeroes from CTE
nonzero_start_index = cte.ne(0).idxmax()

time = time[nonzero_start_index:]
cte = cte[nonzero_start_index:]

average_cte = cte.mean()
std_cte = cte.std()

print(f"Average CTE: {average_cte:.4f}")
print(f"Standard Deviation of CTE: {std_cte:.4f}")

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(time, cte, label='Cross Track Error')
plt.xlabel("Time (seconds)")
plt.ylabel("CTE")
plt.title("Cross Track Error over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig("cte_real.png", dpi=300)
plt.show()
