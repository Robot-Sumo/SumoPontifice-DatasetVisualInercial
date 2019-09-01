import csv
import numpy as np
import matplotlib.pyplot as plt
import math


def main():

    plot_est = [1, 0, 0, 0] # estimacion: pos


    time = np.array([])
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

    maxTime = 142.0
    index = 0
    with open('../../../outputRobotEncoder.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            time = np.append( time, [float(row[0])], 0)
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
       
   
if __name__ == "__main__": main()

