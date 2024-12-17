import csv
import matplotlib.pyplot as plt
from datetime import datetime

# File name (assuming it's in the same folder as the script)
file_name = 'continuous_joint_read.csv'

# Initialize containers for data
time = []
positions = {}

# Read the CSV file
with open(file_name, 'r') as file:
    reader = csv.reader(file)
    headers = next(reader)  # Get the column headers

    # Identify position columns
    position_columns = [i for i, h in enumerate(headers) if 'leg3' in h]
    for col in position_columns:
        positions[headers[col]] = []

    # Parse data
    for row in reader:
        try:
            # Combine seconds and nanoseconds for accurate time parsing
            secs = int(row[1])  # header_stamp_secs
            nsecs = int(row[2])  # header_stamp_nsecs
            time_value = secs + nsecs * 1e-9  # Time in seconds
            time.append(time_value)

            # Append positions
            for col in position_columns:
                positions[headers[col]].append(float(row[col]))
        except (ValueError, IndexError):
            # Skip invalid rows
            continue

# Convert time to duration (elapsed time from 0)
time_start = time[0]
time = [t - time_start for t in time]  # Duration in seconds

# Plot joint positions
plt.figure(figsize=(12, 6))
for name, values in positions.items():
    plt.plot(time, values, label=name)

# Format the x-axis to display elapsed time
plt.xlabel('Time (seconds)')
plt.ylabel('Joint Angular Positions')
plt.title('Joint Positions Over Elapsed Time')
plt.legend()
plt.grid(True)

# Save the plot
plot_file_name = 'joint_positions_duration.png'
plt.savefig(plot_file_name, bbox_inches='tight')  # Save with proper spacing
plt.show()

print(f"Plot saved to {plot_file_name}")
