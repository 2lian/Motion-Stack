from datetime import datetime

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np

default_colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

plt.rcParams["lines.linewidth"] = 1.5

# Path to the Times New Roman font
font_path = "/usr/share/fonts/truetype/msttcorefonts/times.ttf"
times_new_roman_font = fm.FontProperties(fname=font_path)
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = "Times New Roman"

# Scaling factor for fonts because Times is smaller than default
scaling_factor = 1.2
for key in plt.rcParams:
    if "size" in key:
        if isinstance(plt.rcParams[key], float):
            plt.rcParams[key] *= scaling_factor

float_formatter = "{:.2f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})


def parse_date(date: str) -> int:
    truncated_date_str = date[:26]
    nano = date[26:]
    date_format = "%Y/%m/%d %H:%M:%S.%f"

    date_obj = datetime.strptime(truncated_date_str, date_format)
    epoch = datetime(1970, 1, 1)
    nanosec_since_epoch = int((date_obj - epoch).total_seconds() * 1e9) + int(nano)
    return nanosec_since_epoch
