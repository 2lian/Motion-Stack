import matplotlib.pyplot as plt
import numpy as np

# Example data
time = np.linspace(0, 2, 100)
y1 = np.sin(2 * np.pi * time)
y2 = np.sin(4 * np.pi * time)
y3 = np.sin(6 * np.pi * time)

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))  # 3 subplots stacked vertically

# Plot data on all 3 axes
axes[0].plot(time, y1, label="Plot 1")
axes[1].plot(time, y2, label="Plot 2")
axes[2].plot(time, y3, label="Plot 3")

# Remove x-axis tick labels for the first two subplots
for ax in axes[:-1]:  # Loops through all but the last subplot
    ax.tick_params(axis="x", labelbottom=False)  # Remove x-axis tick labels

# Add spines/gridlines for beauty (just like the example)
for ax in axes:
    ax.grid(True)  # Add grid
    ax.spines['bottom'].set_linewidth(1)  # Spine thickness

# Add labels
axes[2].set_xlabel("Time (s)")
axes[0].set_ylabel("Y1")
axes[1].set_ylabel("Y2")
axes[2].set_ylabel("Y3")

# Add legends (optional)
axes[0].legend()
axes[1].legend()
axes[2].legend()

plt.tight_layout()  # Adjust layout
plt.show()

