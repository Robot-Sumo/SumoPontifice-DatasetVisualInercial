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
#include "math.h"
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>


using namespace std;



bool doTestSpeedOnly=false;
size_t nFramesCaptured=100;
int divisor = 1;
const float GravityModule = 9.68f;
const int sizeOfImageBuffer = 700; // 400 Imagenes
const int sizeOfBlockImu = 10; // 10 medidas de imu por bloque
const int sizeOfImuBuffer = 400*sizeOfBlockImu; // 400 bloques de medidas de IMU para cada

DataPacketImu dataImu[sizeOfImuBuffer];		// x y z

int indexOfImu = 0;


static pthread_mutex_t threadMutex_get_imu_data = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_image_grabbed = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_imu_grabbed = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_image_write = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_imu_write = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_cout= PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t threadMutex_indexImu = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t threadMutex_indexImage = PTHREAD_MUTEX_INITIALIZER;


bool _bStop = false;
bool _bGetImuData = false;
bool _bImuStarted = false;
bool _bImageGrabbed= false;
bool _bImuGrabbed= false;
bool _bImageWrite= true;
bool _bImuWrite= true;
bool _bBottleneck= false;
unsigned short iSampleRate = 200;
raspicam::RaspiCam Camera;

std::ofstream outputFilecsv; // archivo de salida IMU
unsigned char *dataImage[sizeOfImageBuffer]; //  apuntador a la imagen actual
int indexOfImage= 0;



Timestamp timestamp; // timestamp en nanoseconds;
int64_t timestamp_image[sizeOfImageBuffer];
int64_t timestamp_imu;
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



//<<< IMU functions >>>
void __OutputFile( const __CaptureData_Buffer_t *p_pData ) // IMU
{

	int indexGyro = 0;
	int indexAcc = 0;
	for(int i= 0; i < p_pData->packets_readed ; i++ )       
	{
		outputFilecsv <<  p_pData->timestamp[i] <<  ","  // indice de tiempo
		<< p_pData->dataGyro[indexGyro]*M_PI/180.0<<"," // rad/s
		<< p_pData->dataGyro[indexGyro+1]*M_PI/180.0<<","
		<< p_pData->dataGyro[indexGyro+2]*M_PI/180.0<<","
		<< p_pData->dataAccel[indexAcc]*9.68<<"," // m/s2
		<< p_pData->dataAccel[indexAcc+1]*9.68<<","
		<< p_pData->dataAccel[indexAcc+2]*9.68
		<<std::endl;

		indexAcc = indexAcc+3;
		indexGyro = indexGyro+3;
		
	}


}


void __sigroutine( int p_iSigNum )
{
	switch( p_iSigNum )
	{
	case SIGHUP:
	case SIGINT:
	case SIGQUIT:
	case SIGTERM:
		_bStop = true;
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

    if( SIG_ERR == signal( SIGALRM, __sigroutine ) )
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
    pthread_mutex_lock(&threadMutex_cout);
    cout << "Numero de medidas de la imu = " << numImuMeasurements <<endl;
    cout << "Numero de overflows de la imu = " << numOverflows <<endl;
    pthread_mutex_unlock(&threadMutex_cout);

    MPUManager::GetInstance()->Stop();
 
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
     while(!_bImuStarted);
     timer.start();
do{
        
        Camera.grab();
        counter++;

        
        if(counter == divisor)
        {
            // Evaluar que hacer con la imagen tomada
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
           
            pthread_mutex_lock(&threadMutex_image_grabbed);
            _bImageGrabbed = true;   
            pthread_mutex_unlock(&threadMutex_image_grabbed);
            counter = 0;          
         
        }

       
        
        
    }while((++i/divisor<nFramesCaptured || nFramesCaptured==0) && !_bStop);//stops when nFrames captured or at infinity lpif nFramesCaptured<0
    timer.end();
    cerr<< timer.getSecs()<< " seconds for "<< nFramesCaptured<< "  frames : FPS " << ( ( float ) ( nFramesCaptured ) / timer.getSecs() ) <<endl;
    cout << "Num images = " << numImages << endl;
    Camera.release();
    _bStop = true; // detener hilo de IMU
    return NULL;
}



static void * threadWriteDisk(void *arg)
{
     outputFilecsv.open("../../outputImu.csv", std::ofstream::out | std::ofstream::trunc);

    double maxTime , timePos;
    int numImuWritten= 0;
    int numImagesWritten= 0;
    for(;!_bStop;)
	{
		while(!_bStop && !_bImuGrabbed && !_bImageGrabbed); // Si se obtiene una imagen o datos de la imu sale
        int64_t timestampHere = timestamp.getNanoSecs();
        
        if(!_bStop)
        {
            if (_bImuGrabbed) // bajar banderas 
            {
                  pthread_mutex_lock(&threadMutex_imu_write);
                 _bImuWrite = false; // aun no se han escrito las imagenes
                 pthread_mutex_unlock(&threadMutex_imu_write);
            }

            if (_bImageGrabbed) // escribir imagenes en disco
            {
                
                 pthread_mutex_lock(&threadMutex_image_write);
                 _bImageWrite = false; // aun no se han escrito las imagenes
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
                        outputFilecsv <<  dataImu[i].timestamp << "," // indice de tiempo
                        << dataImu[i].dataGyro_x <<  "," 
                        << dataImu[i].dataGyro_y <<"," // rad/s
                        << dataImu[i].dataGyro_z <<","
                        << dataImu[i].dataAccel_x<<","
                        << dataImu[i].dataAccel_y<<"," // m/s2
                        << dataImu[i].dataAccel_z
                        <<std::endl;
                    }
                    else break;
                    pthread_mutex_lock(&threadMutex_indexImu);
                    index = indexOfImu;
                    pthread_mutex_unlock(&threadMutex_indexImu);
                }
                pthread_mutex_lock(&threadMutex_imu_grabbed);
                 _bImuGrabbed = false;
                 pthread_mutex_unlock(&threadMutex_imu_grabbed);

                          
                pthread_mutex_lock(&threadMutex_imu_write);
                _bImuWrite = true; // se escribieron las medidas de la imu en disco
                pthread_mutex_unlock(&threadMutex_imu_write);


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
                    pthread_mutex_lock(&threadMutex_indexImage);
                    index = indexOfImage;
                    pthread_mutex_unlock(&threadMutex_indexImage);

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
                        outputFilecsv <<  dataImu[i].timestamp << "," // indice de tiempo
                        << dataImu[i].dataGyro_x <<  "," 
                        << dataImu[i].dataGyro_y <<"," // rad/s
                        << dataImu[i].dataGyro_z <<","
                        << dataImu[i].dataAccel_x<<","
                        << dataImu[i].dataAccel_y<<"," // m/s2
                        << dataImu[i].dataAccel_z
                        <<std::endl;
                    }
                    else break;
                    pthread_mutex_lock(&threadMutex_indexImu);
                    index = indexOfImu;
                    pthread_mutex_unlock(&threadMutex_indexImu);
                }
                pthread_mutex_lock(&threadMutex_imu_grabbed);
                 _bImuGrabbed = false;
                 pthread_mutex_unlock(&threadMutex_imu_grabbed);

                          
                pthread_mutex_lock(&threadMutex_imu_write);
                _bImuWrite = true; // se escribieron las medidas de la imu en disco
                pthread_mutex_unlock(&threadMutex_imu_write);


            }

    
        

             timePos = (timestamp.getNanoSecs()-timestampHere)/1000000.0 ;
            if (timePos>maxTime)
            {
                maxTime = timePos;
                pthread_mutex_lock(&threadMutex_cout);
                cout << "max Time write on disk " << maxTime<< " ms"<<endl;
                pthread_mutex_unlock(&threadMutex_cout);
            }
           
        }
  
	}
    
    pthread_mutex_lock(&threadMutex_cout);
    cout << "Imu written = " << numImuWritten<<endl;
    cout << "Images written = " << numImagesWritten<<endl;
    pthread_mutex_unlock(&threadMutex_cout);
    outputFilecsv.close();

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
    usleep(2000000); // Esperar dos segundos por el balance de blancos de la camara

    pthread_create(&h1, NULL, threadCam, NULL);
    pthread_create(&h2, NULL, threadImu, NULL);
    pthread_create(&h3, NULL, threadWriteDisk, NULL);
    pthread_join(h1, NULL);
    pthread_join(h2, NULL);
    pthread_join(h3, NULL);



    

	return 0;


}


//./raspicam_test -fr 60 -w 640 -h 480 -gr -nframes 50 -div 3
