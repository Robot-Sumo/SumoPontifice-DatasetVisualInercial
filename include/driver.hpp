#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "joystick.hpp"
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>

using namespace std;
using namespace std::chrono;

// PS3 Joystick Buttons
#define Cross 0
#define Circle 1
#define Triangle 2
#define Rectangle 3
#define L1 4
#define R1 5
#define L2 6
#define R2 7
#define Select 8
#define START 9
#define PS 10
#define L3_button 11
#define R3_button 12
#define Up 13
#define Down 14
#define Left 15
#define Right 16
#define L3_x_axis 0
#define L3_y_axis 1
#define R3_x_axis 3
#define R3_y_axis 4
#define R2_x_axis 5
#define L2_x_axis 2



//Comandos del dispositivo
#define setLED_RGB 1 // Poner LED_RGB en un color determinado
#define setBearingVector 2 // Poner punto de referencia de direccion y velocidad
#define getBatteryState 3  // Leer voltaje de las baterias y enviar al master
#define startEncoderSampling 4 // comando que solicita empezar a medir el dezplazamiento en las ruedas
#define setSampleFrecuency 6 // Configurar frecuencia de muestreo del dispositivo
#define getBufferData 7 // Leer la data muestreada por el dispositivo y almacenada en el buffer
#define resetRobot  8 // Reniciar el robot

#define masterDeviceAddress 2
#define pantiltDeviceAddress 255
#define robotDeviceAddress 1 



struct DataPacketEncoder
{
    int rightWheel;
    int leftWheel;
    int64_t timestamp; //tiempo en el que se tomo la medida
};

enum robotStates
{
    idle,
    forward,
    reverse,
    stop,
    pantiltRoll,
    pantiltYaw,
    pantilt,
    readDataFromRobot,
    reset,
    powerOff,
    startDeviceSampling

};

struct Buttons // from ps3 joystick
{

    // movimiento del robot 
    bool stop;
    bool enableForward;
    bool enableReverse;

    
    int acceleration; // -32k to 32k
    int direction ;   //  -32k to 32k

    // pantilt
    bool enableYaw;
    bool enableRoll;
    bool freeze; // guardar valores definidos
    int pantiltYaw;   // -32k-32k
    int pantiltRoll;  // -32k-32k
    

    bool reset;
    bool powerOff;
    bool startToggle; // boton de start y finish
     

    int64_t timestampReset;
    int counterReset;
    
    int64_t timestampStart;
    int counterStart;

};


class TimestampDriver{


public:
    TimestampDriver(){}

    int64_t getNanoSecs(){
    
    nanoseconds ns = duration_cast< nanoseconds >(
        system_clock::now().time_since_epoch()
    );
    return ns.count();
    }



};

struct __CaptureData_Encoder_t
{
	int encoderLeft[100];		// x y z
	int encoderRight[100];		// x y z
	int packets_readed;      // numero de paquetes leidos
	int64_t timestamp[100]; // tiempo en que fue tomada el ultimo dato
};

class Driver
{
    public:

        Driver();
        Driver(bool externalSource);
        Driver(bool externalSource, bool * start, bool * finish, bool *dataReady );
        void openOutputFile(string outputDirectory);
        void openJoystickDev(string device );
        void openSerialDev(string device, int baudrate);
        void closeSerialDev();
        void closeJoystickDev();
        void run();
        bool joystickFound;
        bool serialFound;
        
        void setSamplingRate(int samplingRate ); // falta por implementar
        void setSourceSampling(bool externalSource); // 
        // activa la bandera de startsampling en la maquina de estados
        // Cabe destacar que el robot debe estar en idle para que se tome en cuenta esta bandera
        void startDeviceSampling();
        // start y finish son banderas que el driver levanta cuando se activan por el joystick
        // dataReady es una bandera externa que el driver levanta cuando se ha finalizado de Leer
        // el buffer del arduino
        void setExternalFlags(bool * start, bool * finish, bool *dataReady);
        // estructura en la que se guardara los datos del buffer del arduino
        // en caso de haber seleccionado fuente externa de muestreo
        void setExternalData(__CaptureData_Encoder_t * eData);

        // funcion que activa la bandera de goToGetBufferData en la maquina de estados
        // Diseñada para
        void getEncoderData();

        void finishDriver();
        
        void sendCommandGetBufferData(); // Funcion de muestreo (lectura de buffer de medidas)
        int sizeDataPackage;
        char bufferData [1000]; // Buffer de entrada serial



    private:
        std::ofstream outputFilecsv;
        Joystick joystick;
        int serialPort;
        

        void decodeJoystickButton(JoystickEvent event);
        void stateMachine();
        void setYaw();
        void setRoll();
        void setCommandPWM();
        void setCommandEncoderSampling();
        
        void setCommandSampleFrecuency();
        
        void setCommandResetRobot();
        void activateSampling();
        void reset();
        void dataPackage2File();

        static void alarmWakeup(int sig_num);
        

        int remap(int lowest, int highest, int newLowest, int newHighest, int analogValue);
        int remap2(int acceleration, int analogValue);


        // Variables actuales
        int samplingRate;

        int driverDir; // dirección, forward, backward, stop (0, 1, 2)
        int driverBearing; // orientacion, de 0 a 255
        int driverVel; // = Acc, donde Acc viene del boton del mando, de 0 a 255

        int lastDriverDir;
        int lastDriverBearing;
        int lastDriverVel;
        
        int lastYaw;
        int lastRoll;

        int yaw;
        int roll;


        // Calibracion pantilt
        const uint8_t yaw_180 =228;
        const uint8_t yaw_0 = 36;
        const uint8_t yaw_90 = 96;

        const uint8_t roll_90 = 245;
        const uint8_t roll_0 = 131;
        const uint8_t roll_min = 100;  // mini

        Buttons robotButton;
        int robotState;

        static TimestampDriver timestamp;
        static int64_t time_stamp1, time_stamp2;
        static bool externalSourceSampling; // Utilizar fuente externa de muestreo o fuente interna

        bool * startExternal;
        bool * finishExternal;
        bool startArduinoSampling; //flag que activa el muestreo
        

        static bool goToGetBufferData;
        static bool startSampling;
        static bool finishSampling;

        int dataEncoderLeft;
        int dataEncoderRight;
        double currentTimeSeconds;



            
};

#endif

