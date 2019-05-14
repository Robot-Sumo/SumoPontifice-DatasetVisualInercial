#include "../include/driver.hpp"
#include <unistd.h>
#include <iostream>
#include <wiringSerial.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;


int main(int argc, char** argv)
{
    // Create an instance of driver
    Driver driver;
    usleep(1000); // esperar un segundo
    driver.openJoystickDev("/dev/input/js0");
    

    // Ensure that it was found and that we can use it
   
 

     while (!driver.joystickFound)
    {
        printf("open joystick failed.\n");
        usleep(10000000); // esperar un segundo
        driver.openJoystickDev("/dev/input/js0");
    }
    usleep(1500); // esperar un segundo
    driver.openSerialDev("/dev/ttyUSB0", 57600);
    while (!driver.serialFound)
    {
        printf("open serial port failed.\n");
        usleep(10000000); // esperar un segundo
        driver.openSerialDev("/dev/ttyUSB0", 57600);
    }

    driver.openOutputFile("/home/pi/");

    while (true)
    {
        driver.run();
    }
    
  
  
  
  
}