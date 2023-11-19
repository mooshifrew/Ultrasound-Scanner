import serial
import numpy as np
import pyvista as pv
from scipy.signal import convolve2d


port = input("What port is in use? (eg COM3): ")
ser = serial.Serial(port, 9600)  # Adjust the port and baud rate to match Arduino

raw_data = []

# scan conversion variables
maxDistance = 18.0 # [cm]
minDistance =  2.0 # [cm]
sensor2CenterDistance = 9.0 #[cm]
loop=True 

while loop:
    data = ser.readline().decode('utf-8').strip()
    print(data)
    if data=="Start scan":
        Z_MAX = float(ser.readline().decode('utf-8').strip())
        dz = float(ser.readline().decode('utf-8').strip())
        dtheta = float(ser.readline().decode('utf-8').strip())
        delimiter = float(ser.readline().decode('utf-8').strip())

        scan_complete = False

        while not scan_complete:
            data = ser.readline().decode('utf-8').strip()
            print(data)
            if data == 'Scan Complete':
                scan_complete = True
            if not scan_complete:
                try:
                    raw_data.append(float(data))
                except ValueError:
                    print(f"unable to convert data to float, {data} skipped")


        # Process Data
        
        raw_data = np.array(raw_data)
        raw_data[raw_data<minDistance] = None
        indices = np.where(raw_data==delimiter)
        raw_data[raw_data>maxDistance]= None #maxDistance
        indices = indices[0]

        # raw_data[raw_data>maxDistance] = np.NaN
   
        r = np.zeros((len(indices)-1, indices[0]))

        # save data in txt
        np.savetxt("ultrasound_data_paper.txt",raw_data)

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

        pc_data_unfiltered=np.array( list(zip(x.flatten(), y.flatten(), z.flatten())))

        # filtering
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
        zTop = zInterp[-1, 0]

        pc_list = list(zip(xInterp.flatten(), yInterp.flatten(), zInterp.flatten()))
        pc_list.append((xTop, yTop, zTop))

        pc_data = np.array(pc_list)

        # cloud = pv.PolyData(pc_data)
        cloud = pv.PolyData(pc_data_unfiltered)
        cloud.plot()

        if input("create a mesh object? (yes/no): ")=='yes':
            volume = cloud.delaunay_3d()
            shell = volume.extract_geometry()
            shell.plot()

        if input("Plot the mesh of the unfiltered? (yes/no): ")=='yes':
            uf_cloud = pv.PolyData(pc_data_unfiltered)
            volume =uf_cloud.delaunay_3d()
            shell = volume.extract_geometry()
            shell.plot()

        loop = False
            



