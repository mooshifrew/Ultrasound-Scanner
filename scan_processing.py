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
            
            maxDistance = 10

            raw_data = np.array(raw_data)
            raw_data = np.array(raw_data)
            raw_data[raw_data<minDistance] = None
            raw_data[raw_data>maxDistance] = None

            plotMax = 10
            
            depths = np.reshape(raw_data, (len(raw_data)//5, 5))
            depths = depths[::-1, :]

            sensor_width = 3 # set this based on physical system

            x_min = -2 * sensor_width
            x_max = 2 * sensor_width
            z_min = 0
            z_max = Z_MAX

            cmap = plt.cm.colors.ListedColormap(['orange', 'white'])

            plt.imshow(depths, cmap=cmap, interpolation='nearest', extent=(x_min, x_max, z_min, z_max))
            plt.colorbar()
            plt.xlabel('X (cm)')
            plt.ylabel('Z (cm)')

            plt.show()
            

        # Process Data

        # 3D scanner
        if scan_mode == 3:
            # save data in txt
            np.savetxt("3D_Scan_Data_Cone.txt",raw_data)
            
            raw_data = np.array(raw_data)
            raw_data[raw_data<minDistance] = None
            indices = np.where(raw_data==delimiter)
            indices = indices[0]
            raw_data = np.delete(raw_data, indices)
            raw_data[raw_data>maxDistance]= None
    
            height = len(indices)
            num_angle_steps = indices[0]//txPerScan

            r_unprocessed = np.zeros((height, num_angle_steps, txPerScan))
            r = np.zeros((height, num_angle_steps))

            for z in range(height): 
                for theta in range(num_angle_steps):
                    for scan in range(txPerScan):
                        r_unprocessed[z, theta, scan] = raw_data[z*num_angle_steps*txPerScan + theta*txPerScan + scan]
            

            for z in range(height):
                for theta in range(num_angle_steps):
                    r[z, theta] = np.mean(r_unprocessed[z, theta, (r_unprocessed[z,theta,:]!= None)])
            
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

            cloud = pv.PolyData(pc_data_unfiltered)

            # plot using matplotlib
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(x,y,z, s = 100)
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            plt.show()

            # plot mesh
            if input("create a mesh object? (yes/no): ")=='yes':
                volume = cloud.delaunay_3d()
                shell = volume.extract_geometry()
                shell.plot()

        loop = False
            



