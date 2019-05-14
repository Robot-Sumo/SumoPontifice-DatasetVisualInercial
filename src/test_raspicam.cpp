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
using namespace std;
bool doTestSpeedOnly=false;
size_t nFramesCaptured=100;

bool _bStop = false;
unsigned short iSampleRate = 200;


std::ofstream outputFilecsv;
MPUManager *pManager = MPUManager::GetInstance();

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

void saveImage ( string filepath,unsigned char *data,raspicam::RaspiCam &Camera, int format, int w, int h, int size ) {
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




void __OutputFile( const __CaptureData_Buffer_t *p_pData )
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

	return true;
}

void __thread_output( const __CaptureData_Buffer_t *p_pData )
{
	__OutputFile( p_pData );
}


void __testpush2()
{
	
	if( !pManager->SetSampleRate(iSampleRate) )
	{
		printf("SetSampleRate failed\n");
		return;
	}


	if( !pManager->RemoveDataQuaternion() )
	{
		printf("remove data quaternion failed\n");
		return;
	}


	if( !pManager->SetUpdateDataType( MPUManager::__UDT_Push, __thread_output ) )
	{
		printf("SetUpdateDataType failed\n");
		return;
	}

	if( !pManager->Start(false) ) // comenezar sin self test (false)
	{
		printf("Start failed\n");
		return;
	}

}

int main ( int argc,char **argv ) {
    if ( argc==1 ) {
        cerr<<"Usage (-help for help)"<<endl;
    }

    if ( findParam ( "-help",argc,argv ) !=-1) {
        showUsage();
        return -1;
    }

	outputFilecsv.open("../../outputImu.csv", std::ofstream::out | std::ofstream::trunc);
  
    raspicam::RaspiCam Camera;
    processCommandLine ( argc,argv,Camera );
    cout<<"Connecting to camera"<<endl;
    
    if ( !Camera.open() ) {
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<" bufs="<<Camera.getImageBufferSize( )<<endl;
    unsigned char *data=new unsigned char[  Camera.getImageBufferSize( )];
    unsigned char *data2=new unsigned char[  Camera.getImageBufferSize( )];
    Timer timer;
    //cout<< Camera.getExposure() <<endl;
 
    size_t i=0;

    int width, height, format, size;
    size = Camera.getImageBufferSize( );
    format = Camera.getFormat();
    width = Camera.getWidth();
    height = Camera.getHeight();
    

    int64_t time1, time2;
    

    double maxTime , timePos;
    Timestamp timestamp; // timestamp en nanoseconds;
    int64_t timestamp_image, timestamp_image2, last_timestamp_image;
    int divisor = 1;
    int counter = 0;

    divisor = getParamVal ( "-div",argc,argv, 1 ) ;
    if(divisor<1) divisor = 1;
    cout << "frame rate divisor =  " << divisor <<endl;
    cout << "frame rate = " << Camera.getFrameRate()<<endl;
    cout<<"Capturing...."<<endl;
    maxTime = 0.0;


    // Imu


	//const __CaptureData_Buffer_t *pData = 0;

    
    usleep(2000000); // Esperar dos segundos por el balance de blancos

 
   

    int64_t aqui = 0;

    if( !InitCapture() ) // imu
	{
		printf("InitCapture failed\n");
		return 1;
	}

    __testpush2();
    
    timer.start();
 do{
        
        Camera.grab();
        timestamp_image = timestamp.getNanoSecs();
        counter++;

        
        if(counter == divisor)
        {
            pManager->SetGetBufferData(); // get data del buffer de la imu en otro thread
            Camera.retrieve ( data2 );
           
            timestamp_image2 = timestamp_image; // guardar primera
           std::stringstream fn, fn2;
          
            fn2<<"/mnt/home/pi/outputImages/"<< timestamp_image2<<".ppm";              
            counter = 0;
            saveImage ( fn2.str(),data2,Camera, format, width, height ,size );
            
            aqui = timestamp.getNanoSecs();
            
            //cout << "difTime = " << (timestamp_image - last_timestamp_image)/1000000.0 << " ms" <<endl;
            //last_timestamp_image = timestamp_image;
            /*
            if( 0 != (pData=pManager->GetAllBufferData()) )
            {
                
                __OutputFile( pData );

                //cout << "Time file = " << (aqui-timestamp.getNanoSecs())/1000000.0  << " ms"
                //<<" for "<< pData->packets_readed << "measurements"<< endl;
            }
            */

            
        }

        /*if(counter == 2*divisor)
            {
                 pManager->SetGetBufferData(); // get data del buffer de la imu en otro thread
                Camera.retrieve ( data );

                 std::stringstream fn, fn2;
                // Save 2 two images  
                fn<<timestamp_image<<".ppm";              
                fn2<<timestamp_image2<<".ppm";              
                counter = 0;
                saveImage ( fn2.str(),data2,Camera, format, width, height ,size );
                saveImage ( fn.str(),data,Camera, format, width, height, size );
            }
        */
        //if (i<10) fn<<"0";
		//fn<<i<<".ppm";
        //
        time2 = timestamp.getNanoSecs();
        timePos = (time2-timestamp_image)/1000000.0 ;
        if (timePos>maxTime)
        {
            maxTime = timePos;
            cout << "max Time " << maxTime<< " ms"<<endl;
        }
        //cout<<"\r capturing ..."<<i<<"/ Timestamp ="<<(time1-time2)/1000000.0 <<" ms"<<std::endl;

        /*
        if ( !doTestSpeedOnly ) {
            if ( i%5==0 ) 	  cout<<"\r capturing ..."<<i<<"/"<<nFramesCaptured<<std::flush;
            if ( i%30==0 && i!=0  && nFramesCaptured>0 ) { //save image if not in inifite loop
                std::stringstream fn;
                fn<<"image";
		if (i<10) fn<<"0";
		fn<<i<<".ppm";
                
                saveImage ( fn.str(),data,Camera );
                time2 = timestamp.getNanoSecs();
                cout<<"\r capturing ..."<<i<<"/ Timestamp ="<<(time1-time2)/1000000.0 <<" ms"<<std::endl;

		cerr<<"Saving "<<fn.str()<<endl;
            }
        }
        */
        
        
    }while(++i/divisor<nFramesCaptured || nFramesCaptured==0);//stops when nFrames captured or at infinity lpif nFramesCaptured<0
    MPUManager::GetInstance()->Stop();
    timer.end();
    if ( !doTestSpeedOnly )    cout<<endl<<"Images saved in imagexx.ppm"<<endl;



    cerr<< timer.getSecs()<< " seconds for "<< nFramesCaptured<< "  frames : FPS " << ( ( float ) ( nFramesCaptured ) / timer.getSecs() ) <<endl;

    Camera.release();
    outputFilecsv.close();
	
    

	return 0;


}


//./raspicam_test -fr 60 -w 640 -h 480 -gr -nframes 50 -div 3
