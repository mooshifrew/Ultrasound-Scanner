import serial
import numpy as np
import pyvista as pv
from scipy.signal import convolve2d
import matplotlib.pyplot as plt



port = input("What port is in use? (eg COM3): ")
ser = serial.Serial(port, 9600)  # Adjust the port and baud rate to match Arduino

raw_data = []

# scan conversion variables
maxDistance = 18.0 # [cm]
minDistance =  2.0 # [cm]
sensor2CenterDistance = 8.6 #[cm]
loop=True 

while loop:
    data = ser.readline().decode('utf-8').strip()
    print(data)
    if data=="Start scan":
        scan_mode = int(ser.readline().decode('utf-8').strip())
        Z_MAX = float(ser.readline().decode('utf-8').strip())
        dz = float(ser.readline().decode('utf-8').strip())
        dtheta = float(ser.readline().decode('utf-8').strip())
        delimiter = float(ser.readline().decode('utf-8').strip())
        txPerScan = int(ser.readline().decode('utf-8').strip())

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

        # 2D scanner
        if scan_mode == 2:
            np.savetxt("2D_Scan_Data.txt", raw_data)
            
            raw_data = np.array(raw_data)
            raw_data = np.array(raw_data)
            raw_data[raw_data<minDistance] = None
            raw_data[raw_data>maxDistance] = None
            

            depths = np.reshape(raw_data, [5, len(raw_data)//5])
            depths = np.transpose(depths)


            sensor_width = 1 # set this based on physical system

            x_min = -2 * sensor_width
            x_max = 2 * sensor_width
            z_min = 0
            z_max = Z_MAX

            plt.imshow(depths, interpolation='nearest', extent=(x_min, x_max, z_min, z_max))
            plt.colorbar()
            plt.xlabel('X (cm)')
            plt.ylabel('Z (cm)')

            plt.show()
            


            




        # Process Data

        # 3D scanner
        if scan_mode == 3:
            # save data in txt
            np.savetxt("3D_Scan_Data.txt",raw_data)
            
            raw_data = np.array(raw_data)
            raw_data[raw_data<minDistance] = None
            indices = np.where(raw_data==delimiter)
            raw_data[raw_data>maxDistance]= None #maxDistance
            indices = indices[0]

            # raw_data[raw_data>maxDistance] = np.NaN
    
            r_unprocessed = np.zeros((len(indices)-1, indices[0]//txPerScan, txPerScan))
            r = np.zeros((len(indices)-1, indices[0]//txPerScan))

            for z in range(r_unprocessed.shape[0]):
                for theta in range(r_unprocessed.shape[1]):
                    for scan in range(r_unprocessed.shape[2]):
                        r_unprocessed[z, theta, scan] = raw_data[z*r_unprocessed.shape[1] + theta*r_unprocessed.shape[2] + scan]

            for z in range(0, r_unprocessed.shape[0]):
                for theta in range(0, r_unprocessed.shape[1]):
                    r[z, theta] = np.mean(r_unprocessed[z, theta, (r_unprocessed[z,theta,:]!=None)])
            
            
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
            cloud = pv.PolyData(pc_data)
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
            



