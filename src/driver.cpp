#include "driver.hpp"
#include <unistd.h>
#include <wiringSerial.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>


using namespace std;



// static
TimestampDriver Driver::timestamp;
int64_t Driver::time_stamp1;
int64_t Driver::time_stamp2;
bool Driver::externalSourceSampling;
bool Driver::goToGetBufferData;
bool Driver::startSampling;
bool Driver::finishSampling;

// public




Driver::Driver(bool externalSource)
{
    setSourceSampling(externalSource); // fuente de muestreo interno por default

    robotState = robotStates::reset;
    joystickFound = false;
    serialFound = false;
    goToGetBufferData = false;
    startSampling = false;
    finishSampling = false;
    currentTimeSeconds = 0.0;
    startArduinoSampling = false;
    signal(SIGALRM, alarmWakeup);
    alarm(0); // desactivar alarma hasta que se reciba Start;   
    

    reset();
}


Driver::Driver(bool externalSource,  bool * start, bool * finish, bool *dataReady )
{
    setExternalFlags(start, finish, dataReady);  // Colocar banderas externas
    setSourceSampling(externalSource); // fuente de muestreo interno por default
    robotState = robotStates::reset;
    joystickFound = false;
    serialFound = false;
    goToGetBufferData = false;
    startSampling = false;
    finishSampling = false;
    currentTimeSeconds = 0.0;
    startArduinoSampling = false;
    signal(SIGALRM, alarmWakeup);
    alarm(0); // desactivar alarma hasta que se reciba Start;   
    

    reset();
}

Driver::Driver()
{

    setSourceSampling(false); // fuente de muestreo interno por default
    robotState = robotStates::reset;
    joystickFound = false;
    serialFound = false;
    goToGetBufferData = false;
    currentTimeSeconds = 0.0;
    startSampling = false;
    finishSampling = false;
    startArduinoSampling = false;
    signal(SIGALRM, alarmWakeup);
    alarm(0); // desactivar alarma hasta que se reciba Start;
    

    reset();
}


void Driver::setSourceSampling(bool externalSource)
{
    externalSourceSampling = externalSource;
}


void Driver::startDeviceSampling()
{
    startArduinoSampling = true;
}


void Driver::getData()
{
   goToGetBufferData = true; 
}

void Driver::setExternalFlags(bool * start, bool * finish, bool* dataReady)
{

    startExternal = start;
    finishExternal = finish;
    // inicializacion
    *startExternal = false;
    *finishExternal = false;

}

void Driver::reset()
{
    robotButton.stop = false;
    robotButton.enableForward = false;
    robotButton.enableReverse = false;
    robotButton.acceleration = -32767;
    robotButton.direction = 0;
    robotButton.enableRoll = false;
    robotButton.enableYaw = false;
    robotButton.pantiltYaw = 0;
    robotButton.pantiltRoll = 0;
    robotButton.freeze = false;
    robotButton.reset = false;
    robotButton.powerOff = false;

    robotButton.counterReset = 0;
    robotButton.counterStart =0;
    robotButton.startToggle = false;
    robotButton.timestampReset = timestamp.getNanoSecs();
    robotButton.timestampStart = timestamp.getNanoSecs();
    


    driverDir = 0;
    driverBearing = 127;
    driverVel = 0;
    yaw = yaw_90;
    roll = roll_0;

    setCommandPWM();
    /*
    usleep(10000);
    setYaw();
    usleep(20000);
    setRoll();
    */
}


void Driver::openJoystickDev(string device)
{
    joystick.openPath(device);
    joystickFound = joystick.isFound();
  
}

void Driver::openSerialDev(string device, int baudrate )
{
    serialPort = serialOpen (device.c_str(), baudrate);
    serialFound = (serialPort != -1) ;

}


void Driver::openOutputFile(string outputDirectory)
{
    outputFilecsv.open(outputDirectory+"outputRobotEncoder.csv", std::ofstream::out | std::ofstream::trunc);
}


// private
void Driver::decodeJoystickButton(JoystickEvent event)
{
   if (event.isButton())
    {
       
        switch (event.number)
        {
            case Cross:
                robotButton.enableForward = (event.value == 1);   
                break;
            case Circle:
                break;
            case Triangle:
                robotButton.enableReverse = (event.value == 1);
                break;
            case Rectangle:
                robotButton.stop = (event.value == 1);
                break;
            case L1:
                robotButton.freeze = (event.value == 1);
                break;
            case R1:
                break;
            case L2:
                break;
            case R2:
                break;
            case Select:
                if(event.value==1)
                {
                    robotButton.counterReset++;
                    
                    if(robotButton.counterReset>1)
                    {
                        if((timestamp.getNanoSecs()-robotButton.timestampReset)>2000000) // bounce
                            robotButton.reset= true;

                    }
                    robotButton.timestampReset = timestamp.getNanoSecs();
    
                }
                else
                {
                    robotButton.reset= false;
                }
                
                
            
                robotButton.reset = (event.value==1);
                break;
            case START:

                if(event.value==1)
                {
                    robotButton.counterStart++;
                    
                    if(robotButton.counterStart>0)
                    {
                        // Evaluar validez de pulsación
                        if((timestamp.getNanoSecs()-robotButton.timestampStart)>500000000) // bounce 500 ms
                        {
                            robotButton.startToggle = !robotButton.startToggle;
                            startSampling = robotButton.startToggle;
                            finishSampling = !robotButton.startToggle;
                           
                           
                            time_stamp2 = timestamp.getNanoSecs(); 
                        }
                        robotButton.timestampStart = timestamp.getNanoSecs();

                        if(startSampling)
                        {
                             if(!externalSourceSampling) // si la fuente de muestreo es interna
                            {
                                startArduinoSampling = true;
                                setCommandEncoderSampling(); // empezar muestreo en arduino
                                time_stamp2 = timestamp.getNanoSecs();                 
                                ualarm(500000, 500000); // colocar alarma periodica (10 medidas de arduino)
                            }
                            else // fuente externa
                            {
                                *startExternal = startSampling;
                            }
                            
                        }

                         if(finishSampling)
                        {
                             if(!externalSourceSampling) // si la fuente de muestreo es interna
                            {
                                setCommandResetRobot();
                                alarm(0); // cancelar alarma
                                cout << "Sampling finished" << endl;
                                outputFilecsv.close();
                            }
                            else // fuente externa
                            {
                                *finishExternal =  finishSampling;
                                
                            }
                            
                        }


                    }
  
    
                }
        
                
                break;
            case PS:
                robotButton.powerOff = (event.value==1);
                break;
            case L3_button:
                break;
            case R3_button:
                break;
            case Up:
                robotButton.enableRoll = (event.value==1);
                break;
            case Down:
                //robotButton.enableRoll = (event.value==1);
                break;
            case Left:
                robotButton.enableYaw = (event.value==1);
                break;
            case Right:
                break;
            default:
                break;
         }

    }
    else if (event.isAxis())
    {   
        switch (event.number)
        {
            case R3_x_axis:
                robotButton.pantiltYaw = event.value;
                break;
            case R3_y_axis:
                robotButton.pantiltRoll = event.value;
            case L3_x_axis:
                robotButton.direction = event.value;
                break;
            case L3_y_axis:
                break;
            case R2_x_axis:
                robotButton.acceleration = event.value;
                break;
            case L2_x_axis:
                break;
            default:
                break;

        }

    }
}



void Driver::run()
{
    // Attempt to sample an event from the joystick
    TimestampDriver timestamp;
    int64_t time_stamp1, time_stamp2;

    stateMachine();

    JoystickEvent event;
    if (joystick.sample(&event))
    {
        decodeJoystickButton(event);
        //time_stamp2 = timestamp.getNanoSecs();
        
        stateMachine();
        
        
    }
    


    if(lastDriverDir != driverDir ||  lastDriverBearing != driverBearing|| 
    lastDriverVel != driverVel ||
    lastYaw != yaw ||
    lastRoll != roll )
    {
        cout << "direction = " <<(driverDir == 1? "forward" : "reverse")
        << " vel = " << driverVel
        << " bearing = " << driverBearing
        << " yaw = " << yaw
        << " roll = " << roll
        <<endl;
    }
  

    lastDriverBearing = driverBearing;
    lastDriverVel = driverVel;
    lastDriverDir = driverDir;
    lastYaw = yaw;
    lastRoll = roll;
  
    usleep(5000);

}


void Driver::stateMachine()
{  

    switch (robotState)
    {
        case robotStates::idle:

            // Desplazamiento
            if( startArduinoSampling)
            {
                startArduinoSampling = false;
                robotState = robotStates::startDeviceSampling;

            }
            else if(goToGetBufferData) 
            {
                goToGetBufferData = false;
                robotState = robotStates::readDataFromRobot;
            }
            else if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else if(robotButton.reset) {robotState = robotStates::reset;
            robotButton.reset = false;
            }
            else if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw; // No está pulsado ningun boton de Desplazamiento ni el reset
            else robotState = robotStates::idle;
            //cout << "idle" <<endl;
    
            
            break;
        
        case robotStates::startDeviceSampling:

            // comenzar muestreo del encoder
            setCommandEncoderSampling();
            robotState = robotStates::idle;
            break;

        case robotStates::forward:

            driverDir = 1;

            driverVel = remap(-32767, 32767, 0, 255, robotButton.acceleration);
            driverBearing = remap(-32767, 32767, 0, 255, robotButton.direction);
            
            setCommandPWM();

            // Pantilt
            if(goToGetBufferData) 
            {
                goToGetBufferData = false;
                robotState = robotStates::readDataFromRobot;
            }
            else if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw;
            else robotState = robotStates::idle;
            //cout << "forward" << endl;

            break;
        case robotStates::reverse:
            driverDir = 1;

            driverVel = remap(-32767, 32767, 0, 255, robotButton.acceleration);
            driverBearing = remap(-32767, 32767, 0, 255, robotButton.direction);
            
            setCommandPWM();

            // Pantilt
            if(goToGetBufferData) 
            {
                goToGetBufferData = false;
                robotState = robotStates::readDataFromRobot;
            }
            else if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw;
            else robotState = robotStates::idle;
            

            //cout << "reverse" << endl;
            break;
        case robotStates::stop:
            driverDir = 1;

            driverVel = 0;
            driverBearing = 0;
            
            setCommandPWM();
            if(goToGetBufferData) 
            {
                goToGetBufferData = false;
                robotState = robotStates::readDataFromRobot;
            }
            else if(robotButton.enableRoll && robotButton.enableYaw) robotState = robotStates::pantilt;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableRoll) robotState = robotStates::pantiltRoll;
            else if(robotButton.enableYaw) robotState = robotStates::pantiltYaw;
            else robotState = robotStates::idle;
            //cout << "stop" << endl;
            break;

        case robotStates::pantilt:
            yaw =  remap(-32767, 32767, 0, 255, robotButton.pantiltYaw);
            roll = remap(-32767, 32767, 0, 255, robotButton.pantiltRoll);

            setYaw();
            usleep(10000);
            setRoll();

            if(goToGetBufferData) 
            {
                goToGetBufferData = false;
                robotState = robotStates::readDataFromRobot;
            }
            else if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else robotState = robotStates::stop;
            //cout << "pantilt" << endl;
            break;

        case robotStates::pantiltYaw:
            yaw =  remap(-32767, 32767, 0, 255, robotButton.pantiltYaw);

            setYaw();

            if(goToGetBufferData) 
            {
                goToGetBufferData = false;
                robotState = robotStates::readDataFromRobot;
            }
            else if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else robotState = robotStates::stop;
            //cout << "pantiltYaw" << endl;
            break;

        case robotStates::pantiltRoll:
            roll =  remap(-32767, 32767, 0, 255, robotButton.pantiltRoll);

            setRoll();
            
            if(goToGetBufferData) 
            {
                goToGetBufferData = false;
                robotState = robotStates::readDataFromRobot;
            }
            else if(robotButton.stop) robotState = robotStates::stop;
            else if(robotButton.powerOff) robotState = robotStates::powerOff;
            else if(robotButton.enableForward) robotState = robotStates::forward;
            else if(robotButton.enableReverse) robotState = robotStates::reverse;
            else robotState = robotStates::stop;
            //cout << "pantiltRoll" << endl;
            break;
        
        case robotStates::reset:
            yaw = yaw_90;
            roll = roll_0;

     
            setCommandResetRobot();
            usleep(20000);
            setYaw();
            usleep(20000);
            setRoll();
            usleep(20000);
            

            

            robotState = robotStates::idle;
            //cout << "reset" << endl;

            break;

        case robotStates::powerOff:
            serialClose(serialPort);
            joystick.closeDevice();
            //string powerDown = "sudo shutdown -h now";
            // Delete old directories
            system("sudo shutdown -h now");
            exit(1);
            break;

        case robotStates::readDataFromRobot:

            sendCommandGetBufferData();
            robotState = robotStates::idle;
            break;

        default:
            break;

    }
}

void Driver::setYaw()
{

    char write_buffer[] = {255, 1, yaw_0 }; 
    if (yaw>yaw_180)
    {
        write_buffer[2] = yaw_180;
    }
    else if (yaw<yaw_0)
    {
        write_buffer[2] = yaw_0;
    }
    else
    {
        write_buffer[2] = yaw;
        
    }

    if(lastYaw != yaw )
    {
        write(serialPort , write_buffer, sizeof(write_buffer));
    }
        
    

}



void Driver::setRoll()
{
    char write_buffer[] = {255, 2, roll_min }; 
    if (roll>roll_90)
    {
        write_buffer[2] = roll_90;
    }
    else if (roll<roll_min)
    {
        write_buffer[2] = roll_min;
    }
    else
    {
        write_buffer[2] = roll;
    }
    if(lastRoll != roll)
    {
        write(serialPort, write_buffer,sizeof(write_buffer));
    }
    
 
}


void Driver::setCommandPWM()
{
    char address = robotDeviceAddress;
    char command = setBearingVector; // set pwm
    char lastchar = '\n';
    char write_buffer[] = {address, command, char(driverBearing), char(driverVel), char(driverDir), lastchar };  
    if(lastDriverDir != driverDir ||  lastDriverBearing != driverBearing|| 
    lastDriverVel != driverVel )
    {
        if(driverVel>=0 && driverVel <=255 && driverBearing>=0 && driverBearing<=255)
        {

            write(serialPort ,write_buffer,sizeof(write_buffer));

        }
    
    }

}


int Driver::remap(int lowest, int highest, int newLowest, int newHighest, int analogValue)
{
    float newValue = 1.0*(analogValue-lowest)/(highest-lowest); // 0 to 1.0

    float map = newValue*(newHighest-newLowest)+newLowest;

    return int(map);
}

int Driver::remap2(int acceleration, int analogValue)
{
    float newValue = acceleration*(1.0-abs(analogValue)/(32767.0)); // 1.0 to 0.0


    return int(newValue);
}

void Driver::setCommandEncoderSampling()
{
    char address = robotDeviceAddress;
    char command = startEncoderSampling; // set pwm
    char lastchar = '\n';
    char write_buffer[] = {address, command, lastchar };  
    write(serialPort ,write_buffer,sizeof(write_buffer));

}


void Driver::setCommandSampleFrecuency()
{
    char address = robotDeviceAddress;
    char command = setSampleFrecuency; // set pwm
    char lastchar = '\n';
    char write_buffer[] = {address, command, lastchar };  
    write(serialPort ,write_buffer,sizeof(write_buffer));

}


void Driver::setCommandResetRobot()
{
    char address = robotDeviceAddress;
    char command = resetRobot; // set pwm
    char lastchar = '\n';
    char write_buffer[] = {address, command, lastchar };  
    write(serialPort ,write_buffer,sizeof(write_buffer));

}

void Driver::sendCommandGetBufferData()
{
    char address = robotDeviceAddress;
    char command = getBufferData; // set pwm
    char lastchar = '\n';
    char write_buffer[] = {address, command, lastchar };  
    write(serialPort ,write_buffer,sizeof(write_buffer));

    usleep(30000);

    sizeDataPackage = read (serialPort, bufferData, sizeof bufferData);  // read up to 100 characters if ready to read
    
    cout << "measurements recieved = " << sizeDataPackage/4 <<endl;

    if(!externalSourceSampling)
    {
        dataPackage2File();
    }

    
    /*
    for (int i = 0; i < n ; i++)
    {
        cout<< buf[i] << " " ;
    }
    cout << endl;
    */

}


void Driver::dataPackage2File()
{

    int j;
    j= 0;
    for(int i = 0; i< sizeDataPackage/4; i++) 
    {

        currentTimeSeconds = currentTimeSeconds+0.05;
        dataEncoderLeft = bufferData[j+1]+(bufferData[j]<<8);
        dataEncoderRight = bufferData[j+3]+(bufferData[j+2]<<8);
        outputFilecsv <<  currentTimeSeconds<<  ","  // indice de tiempo en segundos
        << dataEncoderLeft <<"," // Data del encoder de la rueda izquierda
        << dataEncoderRight     // Data del encoder de la rueda derecha
        <<endl;
        j = j+4;


    }
   
}



void Driver::alarmWakeup(int sig_num)
{
    unsigned int i;
    
    if(sig_num == SIGALRM)
    {
        time_stamp1 = timestamp.getNanoSecs();
        
        if(startSampling)
        {
        
            std::cout << " time  = " << (time_stamp1-time_stamp2)/1000000.0<< "ms"<<std::endl;
            goToGetBufferData = true;

        }
        //time_stamp2 = time_stamp1;
        
        
    }
    
}

void Driver::closeSerialDev()
{
     serialClose(serialPort);
            
}
void Driver::closeJoystickDev()
{
    joystick.closeDevice();
}