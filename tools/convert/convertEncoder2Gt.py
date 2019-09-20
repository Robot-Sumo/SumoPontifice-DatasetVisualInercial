import csv
import numpy as np
import matplotlib.pyplot as plt
import math



# Convierte la data del encoder a gt respecto al sistema de la imu (body)

def main():

    plot_est = [1, 0, 0, 0] # estimacion: pos


    time = np.array([], dtype = np.uint64)
    gtPosition = np.zeros([0, 3])
    encoderLeft = np.array([])
    encoderRight = np.array([])
    positionX = np.array([])
    positionY = np.array([])
    positionTheta = np.array([])

   
    R = 3.74 # Radio de la rueda (cm)
    D = 18.8 # Distancia entre los bordes exteriores de las ruedas 
    G = 1.85 # Grosor de las ruedas
    L = D-G  # Distancia entre punto medio de contacto de las ruedas (cm)
    encodersPorVuelta = 80 # existen 40 ranuras por encoder, pero tambien se cuentan las no ranuras

    rotation_w2imu = RPY2rotationMatrix(0.0, np.pi/2.0, 0.0) # transformacion rotacional entre mundo e imu


    maxTime = 142.0
    index = 0
    with open('../../../outputRobotEncoder.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            time = np.append( time, [np.uint64(row[0])], 0)
            encoderLeft = np.append( encoderLeft , [float(row[1])], 0)
            encoderRight = np.append( encoderRight, [float(row[2])], 0)
            index = index+1
            #estAngVelocity = np.append( estAngVelocity, [[float(row[23]), float(row[24]), float(row[25])]], 0)

    
    for i in np.arange(index):
        deltaTheta = R/L*(encoderRight[i]-encoderLeft[i])*2*np.pi/encodersPorVuelta
        if i == 0:
            
           
            positionX = np.append( positionX, [0.0], 0)
            positionY = np.append( positionY, [0.0], 0)
            positionTheta =  np.append( positionTheta, [deltaTheta], 0)
        else:
            deltaX = R/2*np.cos(positionTheta[i-1])*(encoderRight[i]+encoderLeft[i])*2*np.pi/encodersPorVuelta
            deltaY = R/2*np.sin(positionTheta[i-1])*(encoderRight[i]+encoderLeft[i])*2*np.pi/encodersPorVuelta
            positionX = np.append( positionX, [positionX[i-1]+deltaX], 0)
            positionY = np.append( positionY, [positionY[i-1]+deltaY], 0)
            positionTheta =  np.append( positionTheta, [positionTheta[i-1]+deltaTheta], 0)



    # convert positionX and positionY to gtX, gtY, gtZ
    # convert position Theta to quaternion
    outFile = open('data.csv', 'r+')
    outFile.truncate(0)
        
    for i in np.arange(index):
               
            #roll, pitch, yaw = quaternion2RPY(gtOrientationQ[index])

            rotation_imu = RPY2rotationMatrix(-positionTheta[i], 0.0, 0.0) # rotaticion de la imu, respecto a sus ejes
            imu_in_w = rotation_w2imu.dot(rotation_imu)  # current orientation of IMU in world
            angles = rotationMatrix2RPY(imu_in_w )
            print(imu_in_w)
            print(angles[0][0])
            print(angles[0][1])
            print(angles[0][2])
            print("jad")
            qw , qx, qy, qz = toQuaternion(angles[0][0], angles[0][1], angles[0][2]) # positionTheta is yaw angle
            outString = str(time[i]) + "," + str(positionY[i]/100.0)+","+str(-positionX[i]/100.0)+","+str(0.0)+"," # position (m), se cambia las posiciones para ponerla respecto a los ejes de la imu
            outString += str(qw) + "," + str(qx)+","+str(qy)+","+str(qz)+","
            outString += str(0.0) + "," + str(0.0)+","+str(0.0)+"," # velocity
            outString += str(0.0) + "," + str(0.0)+","+str(0.0)+"\n" # bias


            #outString += str(T_cam[0, 0])+","+str(T_cam[1, 0])+","+str(T_cam[2, 0])+","
            #outString += str(T_cam[0, 1])+","+str(T_cam[1, 1])+","+str(T_cam[2, 1])+","
            #outString += str(T_cam[0, 2])+","+str(T_cam[1, 2])+","+str(T_cam[2, 2])+"\n"

            outFile.write(outString)
            

    outFile.close()

           
        
    if plot_est[0] == 1:
        # Plot position
        plt.figure()

        plt.subplot(4, 1, 1)
        plt.plot(time, positionX, 'b-', linewidth=2, label='Posicion x estimada')
        plt.ylabel("x(cm)")
        plt.xlabel("t(s)")
        plt.legend()


        plt.subplot(4, 1, 2)
        plt.plot(time, positionY, 'b-', linewidth=2, label='Posicion y estimada')
        plt.ylabel("y(cm)")
        plt.xlabel("t(s)")
        plt.legend()


        plt.subplot(4, 1, 3)
        plt.plot(time, positionTheta*180/np.pi, 'b-', linewidth=2, label='Posicion angular estimada')
        plt.ylabel("theta(grados)")
        plt.xlabel("t(s)")
        plt.legend()

        plt.subplot(4, 1, 4)
        plt.plot(positionX, positionY, 'b-', linewidth=2, label='Trayectoria del robot')
        plt.ylabel("y(cm)")
        plt.xlabel("x(cm)")
        plt.legend()


    print(np.sum(encoderLeft))
    print(np.sum(encoderRight))

    plt.show()
       



def toQuaternion( roll, pitch, yaw):
	cy = np.cos(yaw * 0.5);
	sy = np.sin(yaw * 0.5);
	cr = np.cos(roll * 0.5);
	sr = np.sin(roll * 0.5);
	cp = np.cos(pitch * 0.5);
	sp = np.sin(pitch * 0.5);

	qw = cy * cr * cp + sy * sr * sp;
	qx = cy * sr * cp - sy * cr * sp;
	qy = cy * cr * sp + sy * sr * cp;
	qz = sy * cr * cp - cy * sr * sp;
	return qw, qx, qy, qz

def RPY2rotationMatrix(roll, pitch, yaw ):


    c1 = np.cos(roll)
    s1 = np.sin(roll)
    c2 = np.cos(pitch)
    s2 = np.sin(pitch)
    c3 = np.cos(yaw)
    s3 = np.sin(yaw)
    
    rotationMatrix = np.zeros([3, 3])


    rotationMatrix[0, 0] = c3*c2
    rotationMatrix[0, 1] = c3*s2*s1-s3*c1
    rotationMatrix[0, 2] = c3*s2*c1+s3*s1

    rotationMatrix[1, 0] = s3*c2
    rotationMatrix[1, 1] = s3*s2*s1+c3*c1
    rotationMatrix[1, 2] = s3*s2*c1-c3*s1

    rotationMatrix[2, 0] = -s2
    rotationMatrix[2, 1] = c2*s1
    rotationMatrix[2, 2] = c2*c1



    return rotationMatrix




def rotationMatrix2RPY(rotationMatrix):

    
    
    r11 = rotationMatrix[0, 0] 
    r12 = rotationMatrix[0, 1] 
    r13 = rotationMatrix[0, 2] 

    r21 = rotationMatrix[1, 0] 
    r22 = rotationMatrix[1, 1] 
    r23 = rotationMatrix[1, 2] 

    r31 = rotationMatrix[2, 0] 
    r32 = rotationMatrix[2, 1] 
    r33 = rotationMatrix[2, 2] 

    yaw = np.arctan2(r21, r11)
    pitch = np.arctan2(-r31, np.sqrt(r32*r32+r33*r33))
    roll = np.arctan2(r32, r33)

    angles = np.zeros([1, 3])

    angles[0][0] = roll
    angles[0][1] = pitch
    angles[0][2] = yaw

    return angles

   
if __name__ == "__main__": main()

