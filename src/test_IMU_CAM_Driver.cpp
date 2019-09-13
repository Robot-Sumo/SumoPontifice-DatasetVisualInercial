/**********************************************************
 Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es)
 Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 This software is released under BSD license as expressed below
-------------------------------------------------------------------
Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:

   This product includes software developed by the Ava group of the University of Cordoba.

4. Neither the name of the University nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************/

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <sys/timeb.h>
#include "raspicam/raspicam.h"
#include "mpumanager.h"
#include "driver.hpp"
#include "math.h"
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>


#define MAX_NUM_FRAMES 3000

using namespace std;


bool startFlag, finishFlag; // banderas de inicio y finalizacion del dataset
bool dataReadyDriverFlag; // dataReadyDriverFlag

bool doTestSpeedOnly=false;
size_t nFramesCaptured=100;

int divisor = 1;
const int divisorEncoder = 10;
const float GravityModule = 9.68f;
const int sizeOfImageBuffer = 700; // 400 Imagenes
const int sizeOfBlockImu = 10; // 10 medidas de imu por bloque
const int sizeOfImuBuffer = 400*sizeOfBlockImu; // 400 bloques de medidas de IMU máximo
const int sizeOfBlockEncoder = 2*10; // 2 medidas de encoder (rueda izquierda y derecha) por  bloque
const int sizeOfEncoderBuffer = 400*sizeOfBlockImu; // 400 bloques de medidas de encoders máximo

DataPacketImu dataImu[sizeOfImuBuffer];		// x y z

DataPacketEncoder dataEncoder[sizeOfBlockEncoder]; // Encoder right and left

int indexOfImu = 0;
int indexOfEncoder = 0;

static pthread_mutex_t threadMutex_get_imu_data = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_get_Encoder_data = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t threadMutex_image_grabbed = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_imu_grabbed = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_Encoder_grabbed = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t threadMutex_image_write = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_imu_write = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_Encoder_write = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t threadMutex_cout= PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t threadMutex_indexImu = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_indexImage = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_indexEncoder = PTHREAD_MUTEX_INITIALIZER;

bool _bWriteData = false;
bool _bStop = false; // Finaliza los threads de la imu, y write disk
bool _bStopDriver = false; // Finaliza el thread del driver

bool _bGetImuData = false;
bool _bGetEncoderData = false;

bool _bEncoderStarted = false;
bool _bImuStarted = false;

bool _bImageGrabbed= false;
bool _bImuGrabbed= false;
bool _bEncoderGrabbed= false;

bool _bImageWrite= true;
bool _bImuWrite= true;
bool _bEncoderWrite= true;

bool _bBottleneck= false;
bool _bHoldCamera= false;

unsigned short iSampleRate = 200;

// objeto camara
raspicam::RaspiCam Camera;
// objeto Driver
 Driver driver;


unsigned char *dataImage[sizeOfImageBuffer]; //  apuntador a la imagen actual
int indexOfImage= 0;



Timestamp timestamp; // timestamp en nanoseconds;
int64_t timestamp_image[sizeOfImageBuffer];
int64_t timestamp_imu;
int64_t timestamp_Encoder;

int64_t timestamp_EncoderTemp;

int width, height, format, size;

//<<< Camera functions >>>

//parse command line
//returns the index of a command line param in argv. If not found, return -1

int findParam ( string param,int argc,char **argv ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    return idx;

}
//parse command line
//returns the value of a command line param. If not found, defvalue is returned
float getParamVal ( string param,int argc,char **argv,float defvalue=-1 ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    if ( idx==-1 ) return defvalue;
    else return atof ( argv[  idx+1] );
}

raspicam::RASPICAM_EXPOSURE getExposureFromString ( string str ) {
    if ( str=="OFF" ) return raspicam::RASPICAM_EXPOSURE_OFF;
    if ( str=="AUTO" ) return raspicam::RASPICAM_EXPOSURE_AUTO;
    if ( str=="NIGHT" ) return raspicam::RASPICAM_EXPOSURE_NIGHT;
    if ( str=="NIGHTPREVIEW" ) return raspicam::RASPICAM_EXPOSURE_NIGHTPREVIEW;
    if ( str=="BACKLIGHT" ) return raspicam::RASPICAM_EXPOSURE_BACKLIGHT;
    if ( str=="SPOTLIGHT" ) return raspicam::RASPICAM_EXPOSURE_SPOTLIGHT;
    if ( str=="SPORTS" ) return raspicam::RASPICAM_EXPOSURE_SPORTS;
    if ( str=="SNOW" ) return raspicam::RASPICAM_EXPOSURE_SNOW;
    if ( str=="BEACH" ) return raspicam::RASPICAM_EXPOSURE_BEACH;
    if ( str=="VERYLONG" ) return raspicam::RASPICAM_EXPOSURE_VERYLONG;
    if ( str=="FIXEDFPS" ) return raspicam::RASPICAM_EXPOSURE_FIXEDFPS;
    if ( str=="ANTISHAKE" ) return raspicam::RASPICAM_EXPOSURE_ANTISHAKE;
    if ( str=="FIREWORKS" ) return raspicam::RASPICAM_EXPOSURE_FIREWORKS;
    return raspicam::RASPICAM_EXPOSURE_AUTO;
}

raspicam::RASPICAM_AWB getAwbFromString ( string str ) {
if ( str=="OFF" ) return raspicam::RASPICAM_AWB_OFF;
if ( str=="AUTO" ) return raspicam::RASPICAM_AWB_AUTO;
if ( str=="SUNLIGHT" ) return raspicam::RASPICAM_AWB_SUNLIGHT;
if ( str=="CLOUDY" ) return raspicam::RASPICAM_AWB_CLOUDY;
if ( str=="SHADE" ) return raspicam::RASPICAM_AWB_SHADE;
if ( str=="TUNGSTEN" ) return raspicam::RASPICAM_AWB_TUNGSTEN;
if ( str=="FLUORESCENT" ) return raspicam::RASPICAM_AWB_FLUORESCENT;
if ( str=="INCANDESCENT" ) return raspicam::RASPICAM_AWB_INCANDESCENT;
if ( str=="FLASH" ) return raspicam::RASPICAM_AWB_FLASH;
if ( str=="HORIZON" ) return raspicam::RASPICAM_AWB_HORIZON;
return raspicam::RASPICAM_AWB_AUTO;
}
void processCommandLine ( int argc,char **argv,raspicam::RaspiCam &Camera ) {
    Camera.setWidth ( getParamVal ( "-w",argc,argv,1280 ) );
    Camera.setHeight ( getParamVal ( "-h",argc,argv,960 ) );
    Camera.setBrightness ( getParamVal ( "-br",argc,argv,50 ) );
    Camera.setFrameRate ( getParamVal ( "-fr",argc,argv,30 ) );

    Camera.setSharpness ( getParamVal ( "-sh",argc,argv,0 ) );
    Camera.setContrast ( getParamVal ( "-co",argc,argv,0 ) );
    Camera.setSaturation ( getParamVal ( "-sa",argc,argv,0 ) );
    Camera.setShutterSpeed( getParamVal ( "-ss",argc,argv,0 ) );
    Camera.setISO ( getParamVal ( "-iso",argc,argv ,400 ) );
    if ( findParam ( "-vs",argc,argv ) !=-1 )
        Camera.setVideoStabilization ( true );
    Camera.setExposureCompensation ( getParamVal ( "-ec",argc,argv ,0 ) );

    if ( findParam ( "-gr",argc,argv ) !=-1 )
      Camera.setFormat(raspicam::RASPICAM_FORMAT_GRAY);
    if ( findParam ( "-yuv",argc,argv ) !=-1 ) 
      Camera.setFormat(raspicam::RASPICAM_FORMAT_YUV420);
    if ( findParam ( "-test_speed",argc,argv ) !=-1 )
        doTestSpeedOnly=true;
    int idx;
    if ( ( idx=findParam ( "-ex",argc,argv ) ) !=-1 )
        Camera.setExposure ( getExposureFromString ( argv[idx+1] ) );
    if ( ( idx=findParam ( "-awb",argc,argv ) ) !=-1 )
        Camera.setAWB( getAwbFromString ( argv[idx+1] ) );
    nFramesCaptured=getParamVal("-nframes",argc,argv,20);
    Camera.setAWB_RB(getParamVal("-awb_b",argc,argv ,1), getParamVal("-awb_g",argc,argv ,1));

}
void showUsage() {
    cout<<"Usage: "<<endl;
    cout<<"[-help shows this help]\n"<<endl;
    cout<<"[-div sets division rate (1 default). Integer]\n"<<endl;
    cout<<"[-fr sets frame rate]\n"<<endl;
    cout<<"[-gr sets gray color mode]\n"<<endl;
    cout<<"[-test_speed use for test speed and no images will be saved]\n";
    cout<<"[-yuv sets yuv420 color mode]\n"<<endl;
    cout<<"[-w width] [-h height] \n[-br brightness_val(0,100)]\n[-sh  sharpness_val (-100 to 100)]\n";
    cout<<"[-co contrast_val (-100 to 100)]\n[-sa saturation_val (-100 to 100)]\n";
    cout<<"[-iso ISO_val  (100 to 800)]\n[-vs turns on video stabilisation]\n[-ec exposure_compensation_value(-10,10)]\n";
    cout<<"[-ss shutter_speed (value in microsecs (max 330000)]\n[-ec exposure_compensation_value(-10,10)]\n";
    cout<<"[-exp mode (OFF,AUTO,NIGHT,NIGHTPREVIEW,BACKLIGHT,SPOTLIGHT,SPORTS,SNOW,BEACH,VERYLONG,FIXEDFPS,ANTISHAKE,FIREWORKS)]"<<endl;
    cout<<"[-awb (OFF,AUTO,SUNLIGHT,CLOUDY,TUNGSTEN,FLUORESCENT,INCANDESCENT,FLASH,HORIZON)]"<<endl;
    cout<<"[-nframes val: number of frames captured (100 default). 0 == Infinite lopp]\n";
    cout<<"[-awb_r val:(0,8):set the value for the red component of white balance]"<<endl;
    cout<<"[-awb_g val:(0,8):set the value for the green component of white balance]"<<endl;

    cout<<endl;
}

//timer functions
#include <sys/time.h>
#include <unistd.h>
class Timer{
    private:
    struct timeval _start, _end;

public:
    Timer(){}
    void start(){
        gettimeofday(&_start, NULL);
    }
    void end(){
        gettimeofday(&_end, NULL);
    }
    double getSecs(){
    return double(((_end.tv_sec  - _start.tv_sec) * 1000 + (_end.tv_usec - _start.tv_usec)/1000.0) + 0.5)/1000.;
    }

};

void saveImage ( string filepath,unsigned char *data, int format, int w, int h, int size ) {
    std::ofstream outFile ( filepath.c_str(),std::ios::binary );

    if ( format==raspicam::RASPICAM_FORMAT_BGR ||  format==raspicam::RASPICAM_FORMAT_RGB ) {
        outFile<<"P6\n";
    } else if ( format==raspicam::RASPICAM_FORMAT_GRAY ) {
        outFile<<"P5\n";
    } else if ( format==raspicam::RASPICAM_FORMAT_YUV420 ) { //made up format
        outFile<<"P7\n";
    }
    outFile<<w<<" "<<h<<" 255\n";
    outFile.write ( ( char* ) data,size );
}






void __sigroutine( int p_iSigNum )
{
	switch( p_iSigNum )
	{
	case SIGHUP:
	case SIGINT:
	case SIGQUIT:
	case SIGTERM:
		finishFlag = true;
		printf("recv signal %d\n", p_iSigNum);
		break;
    case SIGALRM:
        printf("recv signal %d\n", p_iSigNum);

        
        break;

	default:
		break;
	}
}

bool InitCapture()
{
	if( SIG_ERR == signal( SIGHUP, __sigroutine ) )
	{
		printf("signal SIGHUP failed\n");
		return false;
	}

	if( SIG_ERR == signal( SIGINT, __sigroutine ) )
	{
		printf("signal SIGINT failed\n");
		return false;
	}

	if( SIG_ERR == signal( SIGQUIT, __sigroutine ) )
	{
		printf("signal SIGQUIT failed\n");
		return false;
	}

	if( SIG_ERR == signal( SIGTERM, __sigroutine ) )
	{
		printf("signal SIGTERM failed\n");
		return false;
    }

    if( SIG_ERR == signal( SIGALRM , __sigroutine ) )
	{
		printf("signal SIGALARM failed\n");
		return false;
	}

	return true;
}


static void * threadImu(void *arg)
{
	MPUManager *pManager = MPUManager::GetInstance();
   

	if( !pManager->SetSampleRate(iSampleRate) )
	{
		printf("SetSampleRate failed\n");
        return NULL;
		
	}

	if( !pManager->RemoveDataQuaternion() )
	{
		printf("remove data quaternion failed\n");
		return NULL;
	}

	if( !pManager->SetUpdateDataType( MPUManager::__UDT_PullRealTime, 0 ) )
	{
		printf("SetUpdateDataType failed\n");
        return NULL;
	
	}



	if( !pManager->Start(false) )
	{
		printf("Start failed\n");
        _bImuStarted = false;
        return NULL; // retornar algun codigo de error
	
	}

	const __CaptureData_Buffer_t *pData = 0;
    _bImuStarted = true;
    _bImuGrabbed = false;
    _bImuWrite = true;

    indexOfImu = 0;
    int numImuMeasurements = 0;
    int64_t samplePeriodNs = 1000000000/iSampleRate;
    int numOverflows = 0;
    
	for(;!_bStop;)
	{
		while(!_bGetImuData && !_bStop);
        if(!_bStop)
        {
            pthread_mutex_lock(&threadMutex_get_imu_data);
            _bGetImuData = false; // bajar bandera de grab imu data 
            pthread_mutex_unlock(&threadMutex_get_imu_data);
            if( 0 != (pData=pManager->GetAllBufferData()) )
            {
                if(_bWriteData) // Si se ha activado la escritura de datos
                {
                    if(!_bImuWrite) // Si no se han escrito los datos de la imu anteriores
                    {
                        // Existe un cuello de botella en la escritura de la sd de las medidas
                        numOverflows++;
                        pthread_mutex_lock(&threadMutex_indexImu);
                        indexOfImu = indexOfImu+1;
                        pthread_mutex_unlock(&threadMutex_indexImu);
                        
                        if(indexOfImu>=sizeOfImuBuffer) // buffer lleno
                        {
                            pthread_mutex_lock(&threadMutex_cout);
                            cout << "Bottleneck error Imu" <<endl;
                            pthread_mutex_unlock(&threadMutex_cout);
                    
                            break;
                        }
                        else
                        {
                        pthread_mutex_lock(&threadMutex_cout);
                        cout << "index buffer Imu =" << indexOfImu<< ", imu measure = " << numImuMeasurements <<endl; 
                        pthread_mutex_unlock(&threadMutex_cout);
                        }
                        
                        
                    }
                    else
                    {
                        pthread_mutex_lock(&threadMutex_imu_write);
                        _bImuWrite = false; // bajar banderas
                        pthread_mutex_unlock(&threadMutex_imu_write);
                        pthread_mutex_lock(&threadMutex_indexImu);
                        indexOfImu = 0; // se escribieron las medidas en disco. reiniciar buffer
                        pthread_mutex_unlock(&threadMutex_indexImu);

                    }
        

                    // guardar datos de IMU en Buffer
                    // tiempo en que se tomó la primera medida de la imu respecto a la imagen
                    timestamp_imu = timestamp_image[indexOfImage]-(pData->packets_readed -1)*samplePeriodNs;
                    int indexAcc, indexGyro;
                    indexAcc = 0;
                    indexGyro = 0;
                    for(int i = 0; i < pData->packets_readed; i++)
                    {
                        numImuMeasurements++;
                        dataImu[indexOfImu+i].dataGyro_x = pData->dataGyro[indexGyro]*M_PI/180.0; // rad/s
                        dataImu[indexOfImu+i].dataGyro_y = pData->dataGyro[indexGyro+1]*M_PI/180.0;
                        dataImu[indexOfImu+i].dataGyro_z = pData->dataGyro[indexGyro+2]*M_PI/180.0;
                        dataImu[indexOfImu+i].dataAccel_x = pData->dataAccel[indexAcc]*GravityModule; // m/s2
                        dataImu[indexOfImu+i].dataAccel_y = pData->dataAccel[indexAcc+1]*GravityModule;
                        dataImu[indexOfImu+i].dataAccel_z = pData->dataAccel[indexAcc+2]*GravityModule;  
                        dataImu[indexOfImu+i].timestamp = timestamp_imu+i*samplePeriodNs;
                        
                        indexAcc = indexAcc+3;
                        indexGyro = indexGyro+3;
                        
            
                    }
                    pthread_mutex_lock(&threadMutex_indexImu);
                    indexOfImu = indexOfImu +pData->packets_readed-1; // Nuevo indice del buffer
                    pthread_mutex_unlock(&threadMutex_indexImu);
                    
                    pthread_mutex_lock(&threadMutex_imu_grabbed);
                    _bImuGrabbed = true;             
                    pthread_mutex_unlock(&threadMutex_imu_grabbed);
                
                    //__OutputFile( pData );
                }
                }

        }
  
	}
    pthread_mutex_lock(&threadMutex_cout);
    cout << "Numero de medidas de la imu = " << numImuMeasurements <<endl;
    cout << "Numero de overflows de la imu = " << numOverflows <<endl;
    pthread_mutex_unlock(&threadMutex_cout);

    if(_bImuStarted) MPUManager::GetInstance()->Stop();
 
    return NULL;
}

static void * threadCam(void *arg)
{
    dataImage[0] =new unsigned char[  Camera.getImageBufferSize( )];

    Timer timer;
    //cout<< Camera.getExposure() <<endl;
    size_t i=0;
    // Obtener parametros basicos de la configuracion de la camara
  
    size = Camera.getImageBufferSize( );
    format = Camera.getFormat();
    width = Camera.getWidth();
    height = Camera.getHeight();
    if(divisor<1) divisor = 1; // divisor de frecuencia
    cout << "frame rate divisor =  " << divisor <<endl;
    cout << "frame rate = " << Camera.getFrameRate()<<endl;
   

    indexOfImage = 0;
    _bImageGrabbed = false; 
    int numImages = 0;
    int counter = 0;
    int counterEncoder = 0; // contador que debe contar 10 medidas de imagenes para pedir medidas de encoder
    bool initEncoder = true;


    
    usleep(3000000); // Esperar tres segundos por el balance de blancos de la camara y el inicio de la IMU, y el arduino

    if(!_bImuStarted) finishFlag = true; // errpr de apertura de IMU. Finalizar dataset
    if(!_bEncoderStarted) finishFlag = true; // errpr de apertura de IMU. Finalizar dataset
   


    int getEncoderCounter = 0;


    for(;(numImages<MAX_NUM_FRAMES);)
    {
        
        Camera.grab();
        counter++;




            
        if(counter == divisor)
        {
            counterEncoder++; // Divisor del encoder, ya que se muestrea 10 veces mas lento que la imu y cam
            if(counterEncoder == divisorEncoder)
            {
                if (finishFlag ) break;
            }
            
            // Evaluar que hacer con la imagen tomada

            if(initEncoder) // Comenzar muestreo sincrono de encoder, activado por disparo de camara
            {
                initEncoder = false;
                driver.startDeviceSampling(); // comienza el muestreo de los encoders
                counterEncoder = 0; // reinicio de contador
                cout << "start Sampling encoder " <<endl;
                timestamp_EncoderTemp = timestamp.getNanoSecs();
                
            }

            if(startFlag) // si se comienza el dataset
            {
                
                startFlag = false; // bajar bandera
                _bWriteData = true; // comenzar a guardar datos de camera, imu
                timer.start();
                cout << "start Flag " <<endl;

            }

        
            if(_bWriteData) // si se a activado la escritura de datos
            {
                numImages++;
                
                if(!_bImageWrite) // Si no se ha escrito la imagen
                {
                    // Existe un cuello de botella en la escritura de la sd de la imagen
                    pthread_mutex_lock(&threadMutex_indexImage);
                    indexOfImage = indexOfImage+1;
                    pthread_mutex_unlock(&threadMutex_indexImage);
                    if(indexOfImage>=sizeOfImageBuffer) // buffer lleno
                    {
                        pthread_mutex_lock(&threadMutex_cout);
                        cout << "Bottleneck error" <<endl;
                        pthread_mutex_unlock(&threadMutex_cout);
                        _bBottleneck = true;
                        break;
                    }
                    else
                    {   
                        pthread_mutex_lock(&threadMutex_cout);
                        cout << " Image Buffer End =" << indexOfImage<< ", image = " << numImages <<endl;
                        pthread_mutex_unlock(&threadMutex_cout);
                        dataImage[indexOfImage] =new unsigned char[  Camera.getImageBufferSize( )];
                    
                    }
                    
                }
                else
                {
                    pthread_mutex_lock(&threadMutex_image_write);
                    _bImageWrite = false; // bajar banderas
                    pthread_mutex_unlock(&threadMutex_image_write); 

                    pthread_mutex_lock(&threadMutex_indexImage);
                    indexOfImage= 0; // se escribieron las imagenes en disco. reiniciar buffer
                    pthread_mutex_unlock(&threadMutex_indexImage);
                }

                
                pthread_mutex_lock(&threadMutex_indexImage);
                timestamp_image[indexOfImage] = timestamp.getNanoSecs();
                Camera.retrieve ( dataImage[indexOfImage] );
                pthread_mutex_unlock(&threadMutex_indexImage);

                pthread_mutex_lock(&threadMutex_get_imu_data);
                _bGetImuData = true; // get data del buffer de la imu en otro thread
                pthread_mutex_unlock(&threadMutex_get_imu_data);

                if(counterEncoder == divisorEncoder)
                {
                    counterEncoder = 0;
                    pthread_mutex_unlock(&threadMutex_get_Encoder_data);
                    //usleep (1000); // esperar un milisengudo para que esten disponibles las 10 medidas del encoder
                    // ya que si el muestreo es totalmente sincrono se puede dejar de leer una medida
                    _bGetEncoderData = true; // get data del driver en otro thread
                    getEncoderCounter++;
                    pthread_mutex_unlock(&threadMutex_get_Encoder_data);     
            
                }

                pthread_mutex_lock(&threadMutex_image_grabbed);
                _bImageGrabbed = true;   
                pthread_mutex_unlock(&threadMutex_image_grabbed);
            }
            else
            {
                // Si no se ha activado de la escritura de datos
                // mantenerse leyendo el buffer de la imu y del encoder

                pthread_mutex_lock(&threadMutex_get_imu_data);
                _bGetImuData = true; // get data del buffer de la imu en otro thread
                pthread_mutex_unlock(&threadMutex_get_imu_data);

                if(counterEncoder == divisorEncoder)
                {
                    counterEncoder = 0;
                    pthread_mutex_unlock(&threadMutex_get_Encoder_data);
                    //usleep (5000);
                    _bGetEncoderData = true; // get data del driver en otro thread
                    pthread_mutex_unlock(&threadMutex_get_Encoder_data);
/*                     pthread_mutex_lock(&threadMutex_cout);
                    cout << " Get encoder" <<endl;
                    cout << "first time =" << timestamp.getNanoSecs()-timestamp_EncoderTemp <<endl;
                    pthread_mutex_unlock(&threadMutex_cout); */
            
                } 
            }
            
            
            counter = 0;          
        
        }
        
        

       
        
        
    };//stops when nFrames captured or at infinity lpif nFramesCaptured<0
    timer.end();
    
    cerr<< timer.getSecs()<< " seconds for "<< numImages<< "  frames : FPS " << ( ( float ) ( numImages ) / timer.getSecs() ) <<endl;
    cout << "Num images = " << numImages << endl;
    cout << "getencodercounter " << getEncoderCounter <<endl;
    Camera.release();
    _bStop = true; // detener hilo de IMU
    _bStopDriver = true; // detener hilo del driver
    return NULL;
}



static void * threadWriteDisk(void *arg)
{
     std::ofstream outputFileImucsv; // archivo de salida IMU
     std::ofstream outputFileEncodercsv; // archivo de salida encoder del robot
     outputFileImucsv.open("../../outputImu.csv", std::ofstream::out | std::ofstream::trunc);
     outputFileEncodercsv.open("../../outputRobotEncoder.csv", std::ofstream::out | std::ofstream::trunc);

    double maxTime , timePos;
    int numImuWritten= 0;
    int numEncoderWritten= 0;
    int numImagesWritten= 0;

    bool _bStopWriteThread = false;
    for(;!_bStopWriteThread;)
	{
		while(!_bStop && !_bImuGrabbed && !_bImageGrabbed && !_bEncoderGrabbed )
        {
            usleep(10000); 
        }; // Si se obtiene una imagen o datos de la imu, o del encoder, atender las solicitudes de escritura
        int64_t timestampHere = timestamp.getNanoSecs();
    
        if (_bImuGrabbed) // bajar banderas 
        {
                pthread_mutex_lock(&threadMutex_imu_write);
                _bImuWrite = false; // aun no se han escrito las imagenes
                pthread_mutex_unlock(&threadMutex_imu_write);
        }
        
        if (_bEncoderGrabbed) // bajar banderas 
        {
                pthread_mutex_lock(&threadMutex_Encoder_write);
                _bEncoderWrite = false; // aun no se han escrito las imagenes
                pthread_mutex_unlock(&threadMutex_Encoder_write);
        }

        if (_bImageGrabbed) // escribir imagenes en disco
        {
            
                pthread_mutex_lock(&threadMutex_image_write);
                _bImageWrite = false; // aun no se han escrito las imagenes
                pthread_mutex_unlock(&threadMutex_image_write);
            
        }
    

        if (_bImageGrabbed) // escribir imagenes en disco
        {
        
            pthread_mutex_lock(&threadMutex_indexImage);
            int index = indexOfImage;
            pthread_mutex_unlock(&threadMutex_indexImage);
            
            for(int i = 0; i<= index; i++) {
                numImagesWritten++;
                std::stringstream fn;
                fn<<"/home/pi/outputImages/"<< timestamp_image[i]<<".ppm"; 
                if(!_bBottleneck) saveImage ( fn.str(), dataImage [i], format, width, height ,size );
                else break;
                index = indexOfImage;


            }

            pthread_mutex_lock(&threadMutex_image_grabbed);
                _bImageGrabbed = false;
                pthread_mutex_unlock(&threadMutex_image_grabbed);

                pthread_mutex_lock(&threadMutex_image_write);
                _bImageWrite = true; // se escribieron las imagenes en disco
                pthread_mutex_unlock(&threadMutex_image_write);
            

        }

        if (_bImuGrabbed) // escribir medidas de imu en disco
        {
            

            pthread_mutex_lock(&threadMutex_indexImu);
            int index = indexOfImu;
            pthread_mutex_unlock(&threadMutex_indexImu);
            for(int i = 0; i<= index; i++) {
                
                if(!_bBottleneck)
                {
                    numImuWritten++;
                    outputFileImucsv <<  dataImu[i].timestamp << "," // indice de tiempo
                    << dataImu[i].dataGyro_x <<  "," 
                    << dataImu[i].dataGyro_y <<"," // rad/s
                    << dataImu[i].dataGyro_z <<","
                    << dataImu[i].dataAccel_x<<","
                    << dataImu[i].dataAccel_y<<"," // m/s2
                    << dataImu[i].dataAccel_z
                    <<std::endl;
                }
                else break;
                index = indexOfImu;
            }
            pthread_mutex_lock(&threadMutex_imu_grabbed);
                _bImuGrabbed = false;
                pthread_mutex_unlock(&threadMutex_imu_grabbed);

                        
            pthread_mutex_lock(&threadMutex_imu_write);
            _bImuWrite = true; // se escribieron las medidas de la imu en disco
            pthread_mutex_unlock(&threadMutex_imu_write);


        }


        if (_bEncoderGrabbed) // escribir medidas de encoder en disco
        {
            

            pthread_mutex_lock(&threadMutex_indexEncoder);
            int index = indexOfEncoder;
            pthread_mutex_unlock(&threadMutex_indexEncoder);
            for(int i = 0; i<= index; i++) {
                
                if(!_bBottleneck)
                {
                    numEncoderWritten++;
                    outputFileEncodercsv <<  dataEncoder[i].timestamp << "," // indice de tiempo
                    << dataEncoder[i].leftWheel <<  "," 
                    << dataEncoder[i].rightWheel 
                    <<std::endl;
                }
                else break;
                index = indexOfEncoder;

            }

            pthread_mutex_lock(&threadMutex_Encoder_grabbed);
                _bEncoderGrabbed = false; // bajar bandera
                pthread_mutex_unlock(&threadMutex_Encoder_grabbed);

                        
            pthread_mutex_lock(&threadMutex_Encoder_write);
            _bEncoderWrite = true; // se escribieron las medidas de la imu en disco
            pthread_mutex_unlock(&threadMutex_Encoder_write);


        }

            timePos = (timestamp.getNanoSecs()-timestampHere)/1000000.0 ;
        if (timePos>maxTime)
        {
            maxTime = timePos;
            pthread_mutex_lock(&threadMutex_cout);
            cout << "max Time write on disk " << maxTime<< " ms"<<endl;
            pthread_mutex_unlock(&threadMutex_cout);
        }


        // Este thread se detendra cuando se reciba la señal de stop desde el imu de cam o ctrl+c
        // y no haya ninguna solictud de escritura
        if( _bStop && !_bImuGrabbed && !_bImageGrabbed && !_bEncoderGrabbed )
        {
            _bStopWriteThread = true;

        }
        
        
      
        
  
	}
    
    

    // revisar si faltó algo por escribir
    pthread_mutex_lock(&threadMutex_cout);
    cout << "Imu written = " << numImuWritten<<endl;
    cout << "Encoder written = " << numEncoderWritten<<endl;
    cout << "Images written = " << numImagesWritten<<endl;
    pthread_mutex_unlock(&threadMutex_cout);
    outputFileImucsv.close();
    outputFileEncodercsv.close();

    return NULL;
}

static void * threadDriver(void *arg)
{
    // Create an instance of driver
   
    int numOverflows = 0; // Numero de posibles overflows del driver
    int numEncoderMeasurements = 0; 
    int64_t samplePeriodEncoderNs = 1000000000/iSampleRate*10;
    usleep(1000); // esperar un segundo
    driver.openJoystickDev("/dev/input/js0");
    driver.setSourceSampling(true); // La funcion que habilita el muestreo estará fuera del hilo
    driver.setExternalFlags(&startFlag, &finishFlag, &dataReadyDriverFlag);
    

    // Ensure that it was found and that we can use it

     while (!driver.joystickFound & !_bStopDriver)
    {
        printf("open joystick failed.\n");
        usleep(10000000); // esperar un segundo
        driver.openJoystickDev("/dev/input/js0");
    }
    usleep(1500); // esperar un segundo
    driver.openSerialDev("/dev/ttyUSB0", 38400);
    while (!driver.serialFound & !_bStopDriver)
    {
        printf("open serial port failed.\n");
        usleep(10000000); // esperar un segundo
        driver.openSerialDev("/dev/ttyUSB0", 38400);
    }

    _bEncoderStarted = true;
    pthread_mutex_lock(&threadMutex_cout);
    cout << " Encoder started" <<endl;
    pthread_mutex_unlock(&threadMutex_cout);
    for(;!_bStopDriver;)
	{

        driver.run();
    
        // Si se solicta la lectura de datos del encoder
        if(_bGetEncoderData)
        {
            //driver.getEncoderData(); 

             pthread_mutex_lock(&threadMutex_get_Encoder_data);
             _bGetEncoderData = false; // bajar bandera de grab encoder data 
             pthread_mutex_unlock(&threadMutex_get_Encoder_data);

            driver.sendCommandGetBufferData();
/*             pthread_mutex_lock(&threadMutex_cout);
            cout << "size data recieved of Encoder buffer =" << driver.sizeDataPackage/4 <<endl; 
            pthread_mutex_unlock(&threadMutex_cout); */


            if(_bWriteData) // Si se ha activado la escritura de datos
            {
                    if(!_bEncoderWrite) // Si no se han escrito los datos de la imu anteriores
                    {
                        // Existe un cuello de botella en la escritura de la sd de las medidas
                        numOverflows++;
                        pthread_mutex_lock(&threadMutex_indexEncoder);
                        indexOfEncoder = indexOfEncoder+1;
                        pthread_mutex_unlock(&threadMutex_indexEncoder);
                        
                        if(indexOfEncoder>=sizeOfEncoderBuffer) // buffer lleno
                        {
                            pthread_mutex_lock(&threadMutex_cout);
                            cout << "Bottleneck error Encoder" <<endl;
                            pthread_mutex_unlock(&threadMutex_cout);
                    
                            break;
                        }
                        else
                        {
/*                         pthread_mutex_lock(&threadMutex_cout);
                        cout << "index buffer Encoder =" << indexOfEncoder<< ", Encoder measure = " << numEncoderMeasurements <<endl; 
                        pthread_mutex_unlock(&threadMutex_cout); */
                        }
                        
                        
                    }
                    else
                    {
                        pthread_mutex_lock(&threadMutex_Encoder_write);
                        _bEncoderWrite = false; // bajar banderas
                        pthread_mutex_unlock(&threadMutex_Encoder_write);
                        pthread_mutex_lock(&threadMutex_indexEncoder);
                        indexOfEncoder = 0; // se escribieron las medidas en disco. reiniciar buffer
                        pthread_mutex_unlock(&threadMutex_indexEncoder);

                    }
        

                    // guardar datos de encoder en Buffer
                    // tiempo en que se tomó la primera medida del encoder respecto a la imagen
                    timestamp_Encoder = timestamp_image[indexOfImage]-(driver.sizeDataPackage/4  -1)*samplePeriodEncoderNs;
                    int j;
                    j= 0;
                    int complementMeasure = 0;


                    for(int i = 0; i < 9; i++)
                    {
                        numEncoderMeasurements++;
                        dataEncoder[indexOfEncoder+i].rightWheel = driver.bufferData[j+1]+(driver.bufferData[j]<<8);; // rad/s
                        dataEncoder[indexOfEncoder+i].leftWheel = driver.bufferData[j+3]+(driver.bufferData[j+2]<<8) ; // rad/s
                        dataEncoder[indexOfEncoder+i].timestamp = timestamp_Encoder+i*samplePeriodEncoderNs;
                        j = j+4;
                       
            
                    }

                    // complementar datos en caso de desincronizacion
                    numEncoderMeasurements++;
                    if(driver.sizeDataPackage/4 == 10) 
                    {
                        dataEncoder[indexOfEncoder+9].rightWheel = driver.bufferData[j+1]+(driver.bufferData[j]<<8);; // rad/s
                        dataEncoder[indexOfEncoder+9].leftWheel = driver.bufferData[j+3]+(driver.bufferData[j+2]<<8) ; // rad/s
                        dataEncoder[indexOfEncoder+9].timestamp = timestamp_Encoder+9*samplePeriodEncoderNs;
                    } 
                    else if(driver.sizeDataPackage/4 < 10)
                    {
                        dataEncoder[indexOfEncoder+9].rightWheel =0;; // rad/s
                        dataEncoder[indexOfEncoder+9].leftWheel = 0 ; // rad/s
                        dataEncoder[indexOfEncoder+9].timestamp = timestamp_Encoder+9*samplePeriodEncoderNs;
                    }
                    else if(driver.sizeDataPackage/4 > 10)
                    {
                        dataEncoder[indexOfEncoder+9].rightWheel = driver.bufferData[j+1]+(driver.bufferData[j]<<8);; // rad/s
                        dataEncoder[indexOfEncoder+9].leftWheel = driver.bufferData[j+3]+(driver.bufferData[j+2]<<8) ; // rad/s
                        dataEncoder[indexOfEncoder+9].timestamp = timestamp_Encoder+9*samplePeriodEncoderNs;
                    }

                 


                    pthread_mutex_lock(&threadMutex_indexEncoder);
                    indexOfEncoder= indexOfEncoder + 9; // Nuevo indice del buffer
                    pthread_mutex_unlock(&threadMutex_indexEncoder);
                    
                    pthread_mutex_lock(&threadMutex_Encoder_grabbed);
                    _bEncoderGrabbed = true;             
                    pthread_mutex_unlock(&threadMutex_Encoder_grabbed);
                
                    //__OutputFile( pData );
            }

        }
        
    }
    pthread_mutex_lock(&threadMutex_cout);
    cout << "Numero de medidas del encoder = " << numEncoderMeasurements <<endl;
    cout << "Numero de overflows del encoder = " << numOverflows <<endl;
    pthread_mutex_unlock(&threadMutex_cout);

    driver.finishDriver();
    driver.closeSerialDev();
    driver.closeJoystickDev();
    return NULL;
    
}
int main ( int argc,char **argv ) {
    if ( argc==1 ) {
        cerr<<"Usage (-help for help)"<<endl;
    }

    if ( findParam ( "-help",argc,argv ) !=-1) {
        showUsage();
        return -1;
    }

  
    processCommandLine ( argc,argv,Camera );
    cout<<"Connecting to camera"<<endl;
    
    if ( !Camera.open() ) {
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<" bufs="<<Camera.getImageBufferSize( )<<endl;
    divisor = getParamVal ( "-div",argc,argv, 1 ) ;
   



    if( !InitCapture() ) // signal handler
	{
		printf("InitCapture failed\n");
		return -1;
	
    }
	//const __CaptureData_Buffer_t *pData = 0;
    pthread_t h1; // Hilo de la camara
    pthread_t h2; // Hilo de la imu
    pthread_t h3; // Hilo de escritura en disco de las imagenes y medidas de imu
    pthread_t h4; // Hilo de escritura en disco de las imagenes y medidas de imu


    pthread_create(&h1, NULL, threadCam, NULL);
    pthread_create(&h2, NULL, threadImu, NULL);
    pthread_create(&h3, NULL, threadWriteDisk, NULL);
    pthread_create(&h4, NULL, threadDriver, NULL);
    pthread_join(h1, NULL);
    pthread_join(h2, NULL);
    pthread_join(h3, NULL);
    pthread_join(h4, NULL);



    

	return 0;


}

//./test_IMU_CAM_Driver -fr 60 -w 640 -h 480 -gr -div 3
