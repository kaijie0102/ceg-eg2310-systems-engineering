import numpy as np
import matplotlib.pyplot as plt
laser_range = np.loadtxt('lidar.txt')

laser_range[laser_range==0] = np.nan
# lr2i = np.nanargmax(laser_range)
# print(lr2i)
# laser_range[lr2i]

# quat = np.loadtxt('odom.txt', skiprows=13, delimiter=':', usecols=1, max_rows=4)
# print(quat)


plt.figure()
plt.polar(np.arange(0,360)/180*np.pi,laser_range)
plt.show()

# omap = np.loadtxt('map.txt')
# plt.imshow(omap, origin='lower')
# plt.show()