import serial
import numpy as np
import pyvista as pv

port = input("What port is in use? (eg COM3): ")
ser = serial.Serial(port, 9600)  # Adjust the port and baud rate to match Arduino

raw_data = []

# scan conversion variables
maxDistance = 25.0 # [cm]
minDistance =  3 # [cm]
sensor2CenterDistance = 9.5 #12.5 # [cm]

while True:
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
            scan_complete = data=="Scan Complete"
            if not scan_complete:
                raw_data.append(float(data))

        # Process Data
        
        raw_data = np.array(raw_data)
        raw_data[raw_data<minDistance] = None
        indices = np.where(raw_data==delimiter)
        raw_data[raw_data>maxDistance]= None#maxDistance
        indices = indices[0]

        # raw_data[raw_data>maxDistance] = np.NaN
   
        r = np.zeros((len(indices)-1, indices[0]))

        #  raw_data.to_csv('raw_data.csv')
        np.savetxt("ultrasound_data.txt",raw_data)

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

        pc_data=np.array( list(zip(x.flatten(), y.flatten(), z.flatten())))

        cloud = pv.PolyData(pc_data)
        cloud.plot()

        volume = cloud.delaunay_3d()
        shell = volume.extract_geometry()
        shell.plot()

            



