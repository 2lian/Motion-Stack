import csv
from datetime import datetime

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np

font_path = "/usr/share/fonts/truetype/msttcorefonts/times.ttf"
times_new_roman_font = fm.FontProperties(fname=font_path)
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = "Times New Roman"
plt.rcParams["lines.linewidth"] = 1.0
plt.rcParams["legend.fontsize"] = 10
plt.rcParams["axes.titlesize"] = 12
plt.rcParams["axes.labelsize"] = 11
plt.rcParams["xtick.labelsize"] = 10
plt.rcParams["ytick.labelsize"] = 10

ieee_black = "#000000"
ieee_blue = "#0071C5"
START_TIME = 240

tip_pos_file = "tip_pos.csv"
mocap_pose_file = "mocap_pose.csv"


def initialize_data():
    return {
        "time": [],
        "x": [],
        "y": [],
        "z": [],
        "qx": [],
        "qy": [],
        "qz": [],
        "qw": [],
    }


tip_pos_data = initialize_data()
mocap_pose_data = initialize_data()


def convert_time_to_seconds(time_str, reference_time=None):
    time_format = "%Y/%m/%d %H:%M:%S.%f"
    try:
        if "." in time_str:
            time_str = time_str.split(".")[0] + "." + time_str.split(".")[1][:6]
        time_obj = datetime.strptime(time_str, time_format)
    except ValueError as e:
        raise ValueError(f"Error parsing time string: {time_str}") from e

    if reference_time is None:
        return time_obj, 0.0
    return time_obj, (time_obj - reference_time).total_seconds()


def read_csv(filepath, data_dict, translation_cols, rotation_cols, reference_time=None):
    with open(filepath, "r") as file:
        reader = csv.DictReader(file)
        for row in reader:
            time_obj, time_in_seconds = convert_time_to_seconds(
                row["time"], reference_time
            )
            if reference_time is None:
                reference_time = time_obj
            data_dict["time"].append(time_in_seconds)
            data_dict["x"].append(float(row[translation_cols[0]]))
            data_dict["y"].append(float(row[translation_cols[1]]))
            data_dict["z"].append(float(row[translation_cols[2]]))
            data_dict["qx"].append(float(row[rotation_cols[0]]))
            data_dict["qy"].append(float(row[rotation_cols[1]]))
            data_dict["qz"].append(float(row[rotation_cols[2]]))
            data_dict["qw"].append(float(row[rotation_cols[3]]))
    return reference_time


def adjust_mocap_to_tip_pos(mocap_data, tip_data):
    adjusted_mocap = {"time": [], "x": [], "y": [], "z": []}
    for tip_time, tip_x, tip_y, tip_z in zip(
        tip_data["time"], tip_data["x"], tip_data["y"], tip_data["z"]
    ):
        idx = np.argmin(np.abs(np.array(mocap_data["time"]) - tip_time))
        mocap_x, mocap_y, mocap_z = (
            mocap_data["x"][idx],
            mocap_data["y"][idx],
            mocap_data["z"][idx],
        )
        adjusted_mocap["time"].append(tip_time)
        adjusted_mocap["x"].append(mocap_x + tip_x)
        adjusted_mocap["y"].append(mocap_y + tip_y)
        adjusted_mocap["z"].append(mocap_z + tip_z)
    return adjusted_mocap


def truncate_data(data, time_key, start_time=START_TIME):
    start_idx = next(i for i, t in enumerate(data[time_key]) if t >= start_time)
    for key in data:
        data[key] = data[key][start_idx:]


reference_time = read_csv(
    tip_pos_file,
    tip_pos_data,
    ["translation_x", "translation_y", "translation_z"],
    ["rotation_x", "rotation_y", "rotation_z", "rotation_w"],
)
read_csv(
    mocap_pose_file,
    mocap_pose_data,
    ["translation_x", "translation_y", "translation_z"],
    ["rotation_x", "rotation_y", "rotation_z", "rotation_w"],
    reference_time,
)

truncate_data(mocap_pose_data, time_key="time", start_time=START_TIME)
truncate_data(tip_pos_data, time_key="time", start_time=START_TIME)

adjusted_mocap_pose = adjust_mocap_to_tip_pos(mocap_pose_data, tip_pos_data)

ratios = [1, 1, 1, 1, 1, 1, 1]
fig, axes = plt.subplots(
    7, 1, figsize=(6, 2 * len(ratios)), gridspec_kw={"height_ratios": ratios}
)

for i, (label, key) in enumerate(zip(["X (mm)", "Y (mm)", "Z (mm)"], ["x", "y", "z"])):
    axes[i].plot(
        tip_pos_data["time"],
        tip_pos_data[key],
        label="Tip Pos",
        color=ieee_black,
        linestyle="dashed",
    )
    axes[i].plot(
        adjusted_mocap_pose["time"],
        adjusted_mocap_pose[key],
        label="Mocap Pose (Adjusted)",
        color=ieee_blue,
    )
    axes[i].set_ylabel(label)
    axes[i].spines["right"].set_visible(False)
    axes[i].spines["top"].set_visible(False)

for i, (label, key) in enumerate(
    zip(["QW", "QX", "QY", "QZ"], ["qw", "qx", "qy", "qz"]), start=3
):
    axes[i].plot(
        tip_pos_data["time"],
        tip_pos_data[key],
        label="Tip Pos",
        color=ieee_black,
        linestyle="dashed",
    )
    axes[i].plot(
        mocap_pose_data["time"],
        mocap_pose_data[key],
        label="Mocap Pose",
        color=ieee_blue,
    )
    axes[i].set_ylabel(label)
    axes[i].spines["right"].set_visible(False)
    axes[i].spines["top"].set_visible(False)

axes[-1].set_xlabel("Time (s)")

plt.tight_layout()
plt.savefig("mocap_tip_comparison_fixed.pdf", format="pdf")
plt.close()

