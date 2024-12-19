import csv
import matplotlib.pyplot as plt
from datetime import datetime
import matplotlib.font_manager as fm

# Font setup
try:
    font_path = "/usr/share/fonts/truetype/msttcorefonts/times.ttf"
    times_new_roman_font = fm.FontProperties(fname=font_path)
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = "Times New Roman"
except:
    plt.rcParams["font.family"] = "serif"

# General appearance
plt.rcParams["lines.linewidth"] = 1.0
plt.rcParams["grid.linestyle"] = "--"  # Dotted grid lines
plt.rcParams["grid.alpha"] = 0.3       # Subtle grid lines
plt.rcParams["legend.fontsize"] = 10
plt.rcParams["axes.titlesize"] = 12
plt.rcParams["axes.labelsize"] = 11
plt.rcParams["xtick.labelsize"] = 10
plt.rcParams["ytick.labelsize"] = 10

# Custom colors
ieee_black = "#000000"
ieee_blue = "#0071C5"

# Function to remove upper and right spines
def clean_axes(ax):
    ax.spines["right"].set_visible(False)
    ax.spines["top"].set_visible(False)
    ax.spines['bottom'].set_position(('outward', -2))  # Move bottom spine slightly outward

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
fig, ax = plt.subplots(figsize=(12, 6))

for name, values in positions.items():
    ax.plot(time, values, label=name, linewidth=1.2)

# Format axes and grid
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Joint Angular Positions')
ax.set_title('Joint Positions Over Elapsed Time')
ax.grid(True)
clean_axes(ax)

# Add legend
ax.legend(loc='upper right')

# Save the plot as PDF
plot_file_name_pdf = 'joint_positions_duration.pdf'
plt.tight_layout()
plt.savefig(plot_file_name_pdf, bbox_inches='tight', format='pdf')
plt.show()

print(f"Plot saved to {plot_file_name_pdf}")

