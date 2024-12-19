import csv
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import matplotlib.font_manager as fm

# Font setup
font_path = "/usr/share/fonts/truetype/msttcorefonts/times.ttf"
times_new_roman_font = fm.FontProperties(fname=font_path)
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = "Times New Roman"

scaling_factor = 1.2
for key in plt.rcParams:
    if "size" in key:
        if isinstance(plt.rcParams[key], float):
            plt.rcParams[key] *= scaling_factor

# General appearance
plt.rcParams["lines.linewidth"] = 1.0
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.3
plt.rcParams["legend.fontsize"] = 10
plt.rcParams["axes.titlesize"] = 12
plt.rcParams["axes.labelsize"] = 11
plt.rcParams["xtick.labelsize"] = 10
plt.rcParams["ytick.labelsize"] = 10

# Custom colors
ieee_black = "#000000"
ieee_blue = "#0071C5"

# Function to remove upper and right spines
def clean_axes(ax, time):
    ax.spines["right"].set_visible(False)
    ax.spines["top"].set_visible(False)
    # ax.spines['left'].set_position(('outward', -10))  # Move left spine outward by 10 points
    ax.spines['bottom'].set_position(('outward', -2))  # Move bottom spine outward by 10 points
    # ax.tick_params(axis='both', which='both', direction='out', length=6, width=1.5)  # Ticks outward
    ax.set_xlim(0, max(time))  # Set x-axis limits from 0 to the maximum value of 'time'



# File paths
set_ik_target_file = 'set_ik_target.csv'
tip_pos_file = 'tip_pos.csv'

# Initialize data dictionaries
set_ik_data = {'time': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}
tip_pos_data = {'time': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}

# Function to convert time to seconds since the first timestamp
def convert_time_to_seconds(time_str, reference_time=None):
    time_format = "%Y/%m/%d %H:%M:%S.%f"
    if '.' in time_str:
        time_str = time_str.split('.')[0] + '.' + time_str.split('.')[1][:6]
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
fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

# X Position
ax[0].plot(tip_pos_data['time'], tip_pos_data['x'], label='Actual Tip X', color=ieee_black)
ax[0].plot(set_ik_data['time'], set_ik_data['x'], label='Set IK Target X', color=ieee_blue, linewidth=1.2)
ax[0].set_title('X Position Comparison')
ax[0].set_ylabel('X (mm)')
ax[0].grid(True)
ax[0].spines["bottom"].set_visible(False)
clean_axes(ax[0], tip_pos_data['time'])

# Y Position
ax[1].plot(tip_pos_data['time'], tip_pos_data['y'], label='Actual Tip Y', color=ieee_black)
ax[1].plot(set_ik_data['time'], set_ik_data['y'], label='Set IK Target Y', color=ieee_blue, linewidth=1.2)
ax[1].set_title('Y Position Comparison')
ax[1].set_ylabel('Y (mm)')
ax[1].grid(True)
ax[1].spines["bottom"].set_visible(False)
clean_axes(ax[1], tip_pos_data['time'])

# Z Position
ax[2].plot(tip_pos_data['time'], tip_pos_data['z'], label='Actual Tip Z', color=ieee_black)
ax[2].plot(set_ik_data['time'], set_ik_data['z'], label='Set IK Target Z', color=ieee_blue, linewidth=1.2)
ax[2].set_title('Z Position Comparison')
ax[2].set_xlabel('Time (s)')
ax[2].set_ylabel('Z (mm)')
ax[2].grid(True)
clean_axes(ax[2], tip_pos_data['time'])
for a in ax[:-1]:  # Loops through all but the last subplot
    a.tick_params(axis="x", labelbottom=False)

# Legend
ax[0].legend()
ax[1].legend()
ax[2].legend()

plt.tight_layout()
plt.savefig('translational_comparison_ieee.pdf', format='pdf')
plt.close()

# Rotational Data Comparison
fig, ax = plt.subplots(4, 1, sharex=True, figsize=(10, 10))

# QX
ax[0].plot(tip_pos_data['time'], tip_pos_data['qx'], label='Actual Tip QX', color=ieee_black)
ax[0].plot(set_ik_data['time'], set_ik_data['qx'], label='Set IK Target QX', color=ieee_blue, linewidth=1.2)
ax[0].set_title('QX Rotation Comparison')
ax[0].legend(loc='upper right')  # Specify position
ax[0].grid(True)
ax[0].spines["bottom"].set_visible(False)

clean_axes(ax[0], tip_pos_data['time'])

# QY
ax[1].plot(tip_pos_data['time'], tip_pos_data['qy'], label='Actual Tip QY', color=ieee_black)
ax[1].plot(set_ik_data['time'], set_ik_data['qy'], label='Set IK Target QY', color=ieee_blue, linewidth=1.2)
ax[1].set_title('QY Rotation Comparison')
ax[1].legend(loc='upper right')  # Specify position
ax[1].grid(True)
ax[1].spines["bottom"].set_visible(False)
clean_axes(ax[1], tip_pos_data['time'])

# QZ
ax[2].plot(tip_pos_data['time'], tip_pos_data['qz'], label='Actual Tip QZ', color=ieee_black)
ax[2].plot(set_ik_data['time'], set_ik_data['qz'], label='Set IK Target QZ', color=ieee_blue, linewidth=1.2)
ax[2].set_title('QZ Rotation Comparison')
ax[2].legend(loc='upper right')  # Specify position
ax[2].grid(True)
ax[2].spines["bottom"].set_visible(False)
clean_axes(ax[2], tip_pos_data['time'])

# QW
ax[3].plot(tip_pos_data['time'], tip_pos_data['qw'], label='Actual Tip QW', color=ieee_black)
ax[3].plot(set_ik_data['time'], set_ik_data['qw'], label='Set IK Target QW', color=ieee_blue, linewidth=1.2)
ax[3].set_title('QW Rotation Comparison')
ax[3].legend(loc='upper right')  # Specify position
ax[3].set_xlabel('Time (s)')
ax[3].grid(True)
clean_axes(ax[3], tip_pos_data['time'])
for a in ax[:-1]:  # Loops through all but the last subplot
    a.tick_params(axis="x", labelbottom=False)

plt.tight_layout()
plt.savefig('rotational_comparison_ieee.pdf', format='pdf')
plt.close()
