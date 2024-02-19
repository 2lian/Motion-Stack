# -*- coding: utf-8 -*-
"""
This creates several maps of foothold to be used by the Astar

@author: Elian NEPPEL
@laboratory: Space Robotic Lab, Tohoku University
"""
import numpy as np

density_in_pt_per_m2 = 17 #10
x_min, x_max = -700, 4000
y_min, y_max = -700, 4000
point_density = density_in_pt_per_m2 * 1e-6

area = (x_max - x_min) * (y_max - y_min)
num_points = int(point_density * area)

np.random.seed(42)

x_coords = np.random.uniform(x_min, x_max, size=num_points)
y_coords = np.random.uniform(y_min, y_max, size=num_points)
z_coords = np.zeros(num_points)

random_map = np.column_stack((x_coords, y_coords, z_coords))

# plt.scatter(random_map[:, 0], random_map[:, 1])
# plt.axis("equal")
# plt.show()

x_map2 = np.arange(-700, 7000, 200)
y_map2 = np.arange(-500, 501, 200)
z_map2 = 0
X_map2, Y_map2, Z_map2 = np.meshgrid(x_map2, y_map2, z_map2)

flat_map = np.concatenate([X_map2.flatten().reshape((len(X_map2.flatten()), 1)),
                           Y_map2.flatten().reshape((len(Y_map2.flatten()), 1)),
                           Z_map2.flatten().reshape((len(Z_map2.flatten()), 1))], axis=1).astype('float32')
x = flat_map[:, 0]
y = flat_map[:, 1]
obs_map = flat_map[~((x > 700) & (x < 4800) & (y > -500) & (y < 500))]
obs_map = obs_map * np.array([1, 1.15, 1])

x_map2 = np.arange(-700, 6800, 200)
y_map2 = np.arange(-500, 2001, 200)
z_map2 = 0
X_map2, Y_map2, Z_map2 = np.meshgrid(x_map2, y_map2, z_map2)

flat_map = np.concatenate([X_map2.flatten().reshape((len(X_map2.flatten()), 1)),
                           Y_map2.flatten().reshape((len(Y_map2.flatten()), 1)),
                           Z_map2.flatten().reshape((len(Z_map2.flatten()), 1))], axis=1).astype('float32')
x = flat_map[:, 0]
y = flat_map[:, 1]
obs_map_large = flat_map[~((x > 700) & (x < 4800) & (y > -500) & (y < 500))]
obs_map_large[:, 1] = obs_map_large[:, 1] * 1.0

x_map2 = np.arange(0, 8000, 200)
y_map2 = np.arange(-1000, 6000, 200)
z_map2 = 0
X_map2, Y_map2, Z_map2 = np.meshgrid(x_map2, y_map2, z_map2)

flat_map = np.concatenate([X_map2.flatten().reshape((len(X_map2.flatten()), 1)),
                           Y_map2.flatten().reshape((len(Y_map2.flatten()), 1)),
                           Z_map2.flatten().reshape((len(Z_map2.flatten()), 1))], axis=1).astype('float32')
minimap = flat_map[~((flat_map[:, 1] < 3500) & (flat_map[:, 1] > 1000)
                     & (flat_map[:, 0] <= 6000) & (2000 <= flat_map[:, 0]))]
minimap = minimap[~((minimap[:, 1] <= 1000)
                    & (minimap[:, 0] <= 6000) & (4000 <= minimap[:, 0]))]
# plt.scatter(minimap[:, 0], minimap[:, 1])
# plt.axis("equal")
# plt.xlabel('x (mm)')
# plt.ylabel('y (mm)')
# plt.show()

x_map2 = np.arange(-1000, 8500, 200)
y_map2 = np.arange(-1500, 1500, 200)
z_map2 = 0
X_map2, Y_map2, Z_map2 = np.meshgrid(x_map2, y_map2, z_map2)

flat_map = np.concatenate([X_map2.flatten().reshape((len(X_map2.flatten()), 1)),
                           Y_map2.flatten().reshape((len(Y_map2.flatten()), 1)),
                           Z_map2.flatten().reshape((len(Z_map2.flatten()), 1))], axis=1).astype('float32')

