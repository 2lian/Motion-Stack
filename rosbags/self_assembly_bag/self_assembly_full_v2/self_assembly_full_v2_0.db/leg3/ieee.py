import csv
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import matplotlib.font_manager as fm

# Path to Times New Roman font
font_path = "/usr/share/fonts/truetype/msttcorefonts/times.ttf"
times_new_roman_font = fm.FontProperties(fname=font_path)

# Set font and general style parameters
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = "Times New Roman"
plt.rcParams["lines.linewidth"] = 0.8        # Finer line width
plt.rcParams["lines.markersize"] = 3         # Smaller markers
plt.rcParams["grid.linestyle"] = ":"         # Dotted grid lines
plt.rcParams["grid.alpha"] = 0.2             # Even lighter grid lines
plt.rcParams["legend.fontsize"] = 9
plt.rcParams["axes.titlesize"] = 11
plt.rcParams["axes.labelsize"] = 10
plt.rcParams["xtick.labelsize"] = 9
plt.rcParams["ytick.labelsize"] = 9

# Custom colors (black and IEEE blue)
ieee_black = "#000000"
ieee_blue = "#0071C5"

# Function to remove upper and right spines
def clean_axes(ax):
    ax.spines["right"].set_visible(False)
    ax.spines["top"].set_visible(False)

# File paths
set_ik_target_file = 'set_ik_target.csv'
tip_pos_file = 'tip_pos.csv'

# Initialize data dictionaries
set_ik_data = {'time': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}
tip_pos_data = {'time': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}

# Function to convert time to seconds since the first timestamp
def convert_time_to_seconds(time_str, reference_time=None):
    time_format = "%Y/%m/%d %H:%M:%S.%f"
    # Truncate microseconds if longer than 6 digits
    if '.' in time_str:
        parts = time_str.split('.')
        if len(parts) > 1 and len(parts[1]) > 6:
            time_str = parts[0] + '.' + parts[1][:6]
    time_obj = datetime.strptime(time_str, time_format)
    if reference_time is None:
        return time_obj, 0.0
    return time_obj, (time_obj - reference_time).total_seconds()

# Function to read CSV into a dictionary and handle time gaps
def read_csv_to_dict(filepath, data_dict, translation_cols, rotation_cols, reference_time=None, time_gap_threshold=10.0):
    with open(filepath, 'r') as file:
        reader = csv.DictReader(file)
        previous_time = None
        for row in reader:
            time_obj, time_in_seconds = convert_time_to_seconds(row['time'], reference_time)
            if reference_time is None:
                reference_time = time_obj
            # Detect large time gaps and insert NaN
            if previous_time is not None and (time_in_seconds - previous_time > time_gap_threshold):
                for key in data_dict:
                    data_dict[key].append(np.nan)
            data_dict['time'].append(time_in_seconds)
            data_dict['x'].append(float(row[translation_cols[0]].strip()))
            data_dict['y'].append(float(row[translation_cols[1]].strip()))
            data_dict['z'].append(float(row[translation_cols[2]].strip()))
            data_dict['qx'].append(float(row[rotation_cols[0]].strip()))
            data_dict['qy'].append(float(row[rotation_cols[1]].strip()))
            data_dict['qz'].append(float(row[rotation_cols[2]].strip()))
            data_dict['qw'].append(float(row[rotation_cols[3]].strip()))
            previous_time = time_in_seconds
    return reference_time

# Read data files
tip_pos_start_time = read_csv_to_dict(tip_pos_file, tip_pos_data, 
                                      ['translation_x', 'translation_y', 'translation_z'], 
                                      ['rotation_x', 'rotation_y', 'rotation_z', 'rotation_w'])

read_csv_to_dict(set_ik_target_file, set_ik_data, 
                 ['translation_x', 'translation_y', 'translation_z'], 
                 ['rotation_x', 'rotation_y', 'rotation_z', 'rotation_w'], 
                 reference_time=tip_pos_start_time)

# Translational Data Comparison
fig, ax = plt.subplots(3, 1, figsize=(10, 8))

# Plot order: first the blue line (IK target), then the black line (Actual Tip) on top
# so both are visible. You can reverse if you prefer.
# X Position
ax[0].plot(set_ik_data['time'], set_ik_data['x'], label='Set IK Target X', color=ieee_blue, marker='s', linewidth=0.8)
ax[0].plot(tip_pos_data['time'], tip_pos_data['x'], label='Actual Tip X', color=ieee_black, marker='o', linewidth=0.8)
ax[0].set_title('X Position Comparison', fontproperties=times_new_roman_font)
ax[0].set_ylabel('X (mm)', fontproperties=times_new_roman_font)
ax[0].grid(True)
clean_axes(ax[0])

# Y Position
ax[1].plot(set_ik_data['time'], set_ik_data['y'], label='Set IK Target Y', color=ieee_blue, marker='s', linewidth=0.8)
ax[1].plot(tip_pos_data['time'], tip_pos_data['y'], label='Actual Tip Y', color=ieee_black, marker='o', linewidth=0.8)
ax[1].set_title('Y Position Comparison', fontproperties=times_new_roman_font)
ax[1].set_ylabel('Y (mm)', fontproperties=times_new_roman_font)
ax[1].grid(True)
clean_axes(ax[1])

# Z Position
ax[2].plot(set_ik_data['time'], set_ik_data['z'], label='Set IK Target Z', color=ieee_blue, marker='s', linewidth=0.8)
ax[2].plot(tip_pos_data['time'], tip_pos_data['z'], label='Actual Tip Z', color=ieee_black, marker='o', linewidth=0.8)
ax[2].set_title('Z Position Comparison', fontproperties=times_new_roman_font)
ax[2].set_xlabel('Time (s)', fontproperties=times_new_roman_font)
ax[2].set_ylabel('Z (mm)', fontproperties=times_new_roman_font)
ax[2].grid(True)
clean_axes(ax[2])

# Apply Times New Roman to tick labels
for axis in ax:
    for label in axis.get_xticklabels() + axis.get_yticklabels():
        label.set_fontproperties(times_new_roman_font)

# Create a single legend for the top axes, without a bounding box
leg = ax[0].legend(prop=times_new_roman_font, frameon=False)

plt.tight_layout()
plt.savefig('translational_comparison_ieee.pdf', format='pdf')
plt.close()

print("Plot saved as 'translational_comparison_ieee.pdf'")

