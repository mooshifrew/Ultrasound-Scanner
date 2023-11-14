import numpy as np
import matplotlib.pyplot as plt
import pyvista as pv
from scipy.signal import convolve2d
from mpl_toolkits.mplot3d import Axes3D 


delimiter = 9999.0

maxDistance = 20
minDistance = 0
sensor2CenterDistance = 10.3

dtheta = 1.8
dz = 0.1

raw_data = []

with open('farmer.txt') as f:
    [raw_data.append(float(line.strip())) for line in f.readlines()]

raw_data = np.array(raw_data)
print(raw_data)

raw_data = np.array(raw_data)
raw_data[raw_data<0] = 0
indices = np.where(raw_data==delimiter)
raw_data[raw_data>maxDistance]=maxDistance
indices = indices[0]

r = np.zeros((len(indices)-1, indices[0]))

r[0,:] = raw_data[0:indices[0]]
for i in range(1,len(indices)-1):
    r[i,:] = raw_data[indices[i-1]+1:indices[i]]

# distances converted to radius measurements
r = sensor2CenterDistance - r

# matrix of theta values same shape as r
theta = np.arange(0, r.shape[1]*dtheta, dtheta)
theta = np.tile(theta, (r.shape[0], 1))

# matrix of z values same shape as r
z = np.arange(start=0, stop=r.shape[0]*dz, step=dz)
z = np.transpose(np.tile(z, (r.shape[1], 1)))

# cylindrical to cartesian 
x = r*np.cos(np.deg2rad(theta))
y = r*np.sin(np.deg2rad(theta))

for i in range(0, x.shape[0]):
    if np.sum(np.isnan(x[i, :]))==x.shape[1]:
        x[i:, :]=[]
        y[i:, :]= []
        z[i:, :]= []
        break

for i in range(0, x.shape[0]):
    lastIdx = np.argmax(~np.isnan(x[i,:]))
    lastX = x[i, lastIdx]
    lastY = y[i, lastIdx]
    for j in range(0, x.shape[1]):
        if ~np.isnan(x[i,j]):
            lastX = x[i,j]
            lastY = y[i,j]
        else:
            x[i,j] = lastX
            y[i,j] = lastY

    
interpIdx = np.arange(0, x.shape[0], 1)
xInterp = x[interpIdx, :]
yInterp = y[interpIdx, :]
zInterp = z[interpIdx, :]

window_size = 2

h = np.ones((window_size, window_size))/ (window_size**2)

# symmetric padding along rows
xInterp = np.pad(xInterp, ((0,0), (window_size, window_size)), mode='symmetric')
yInterp = np.pad(yInterp, ((0,0), (window_size, window_size)), mode='symmetric')

xInterp = convolve2d(xInterp, h)
yInterp = convolve2d(yInterp, h)

xInterp = xInterp[:, window_size:-window_size]
yInterp = yInterp[:, window_size:-window_size]


xInterp[:, -1]=xInterp[:, 0]
yInterp[:, -1]=yInterp[:, 0]
zInterp[:, -1]=zInterp[:, 0]

xTop = np.mean(xInterp[-1, :])
yTop = np.mean(yInterp[-1, :])
zTop = zInterp[-1, 0]+dz

pc_list = list(zip(xInterp.flatten(), yInterp.flatten(), zInterp.flatten()))
pc_list.append((xTop, yTop, zTop))

pc_data = np.array(pc_list)

cloud = pv.PolyData(pc_data)
cloud.plot()

volume = cloud.delaunay_3d()
shell = volume.extract_geometry()
shell.plot()

# triangulation = plt.mplot3d.art3d.Poly3DCollections.triangulate(plt.scatter, x, y, z)

# fig = plt.figure()
# ax = fig.add_subplot(111,projection="3d")
# ax.plot_trisurf(triangulation, x, y, z)


# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# ax.scatter(x,y,z)
# plt.show()
