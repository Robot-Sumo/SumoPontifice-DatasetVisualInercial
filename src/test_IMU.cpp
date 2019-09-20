// test.cpp

#include <stdio.h>
#include "mpumanager.h"
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include <math.h>

bool _bStop = false;
unsigned short iSampleRate = 200;
std::ofstream outputFilecsv;

void __OutputData( const __CaptureData_t *p_pData )
{
	static unsigned long _ts = 0;

	if( 0 == _ts )
	{
		_ts = p_pData->m_ulTimestamp;
	}

	printf("%2ld:  %.2fº | a: x=%.5f, y=%.5f, z=%.5f | g: x=%.5f, y=%.5f, z=%.5f | q: w=%.5f, x=%.5f, y=%.5f, z=%.5f\n",
		p_pData->m_ulTimestamp - _ts,
		p_pData->m_fTemperature,
		p_pData->m_sDataAccel[0],
		p_pData->m_sDataAccel[1],
		p_pData->m_sDataAccel[2],
		p_pData->m_sDataGyro[0],
		p_pData->m_sDataGyro[1],
		p_pData->m_sDataGyro[2],
		p_pData->m_fDataQuat[0],
		p_pData->m_fDataQuat[1],
		p_pData->m_fDataQuat[2],
		p_pData->m_fDataQuat[3]
		);

	_ts = p_pData->m_ulTimestamp;
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

void __cb( const __CaptureData_t *p_pData )
{
	__OutputData( p_pData );
}


void __thread_output( const __CaptureData_Buffer_t *p_pData )
{
	__OutputFile( p_pData );
}

// push mode
/*
void __testpush()
{
	MPUManager *pManager = MPUManager::GetInstance();
	

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


	if( !pManager->SetUpdateDataType( MPUManager::__UDT_Push, __cb ) )
	{
		printf("SetUpdateDataType failed\n");
		return;
	}

	if( !pManager->Start(false) ) // comenezar sin self test (false)
	{
		printf("Start failed\n");
		return;
	}

	for(;!_bStop;)
	{
		usleep(50*1000);
	}
}

*/

// push mode 2
void __testpush2()
{
	MPUManager *pManager = MPUManager::GetInstance();
	

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

	for(;!_bStop;)
	{
		usleep(50*1000);
	}
}

// realtime mode
void __testrt()
{
	MPUManager *pManager = MPUManager::GetInstance();

	if( !pManager->SetSampleRate(iSampleRate) )
	{
		printf("SetSampleRate failed\n");
		return;
	}

	if( !pManager->SetUpdateDataType( MPUManager::__UDT_PullRealTime, 0 ) )
	{
		printf("SetUpdateDataType failed\n");
		return;
	}

	if( !pManager->Start(true) )
	{
		printf("Start failed\n");
		return;
	}

	const __CaptureData_t *pData = 0;

	for(;!_bStop;)
	{
		usleep(5*1000);
		if( 0 != (pData=pManager->GetData()) )
		{
			printf("rt ");
			__OutputData( pData );
		}
	}
}

// pull mode
void __test()
{
	MPUManager *pManager = MPUManager::GetInstance();

// 	if( !pManager->RemoveDataAccel() )
// 	{
// 		printf("RemoveDataAccel failed\n");
// 		return;
// 	}
// 
// 	if( !pManager->RemoveDataGyro() )
// 	{
// 		printf("RemoveDataGyro failed\n");
// 		return;
// 	}
// 
// 	if( !pManager->RemoveDataQuaternion() )
// 	{
// 		printf("RemoveDataQuaternion failed\n");
// 		return;
// 	}

	if( !pManager->SetSampleRate(iSampleRate) )
	{
		printf("SetSampleRate failed\n");
		return;
	}

	if( !pManager->SetUpdateDataType( MPUManager::__UDT_Pull, 0 ) )
	{
		printf("SetUpdateDataType failed\n");
		return;
	}

	if( !pManager->Start(true) )
	{
		printf("Start failed\n");
		return;
	}

	const __CaptureData_t *pData = 0;

	for(;!_bStop;)
	{
		usleep(50*1000);
		if( 0 != (pData=pManager->GetData()) )
		{
			printf("Pull ");
			__OutputData( pData );
		}
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

int main( int argc, char * argv[] )
{
	if( !InitCapture() )
	{
		printf("InitCapture failed\n");
		return 1;
	}

	outputFilecsv.open("../../outputFileImu.csv", std::ofstream::out | std::ofstream::trunc);

	// Pull mode 
	//No se utiliza el buffer. Se lee la medida mas reciente
	//__test();

	// Realtime mode
	// Se utiliza el buffer.
	// Se lee la rafaga del buffer y se imprime sin guardar
	// Si la frecunecia con la que se lee el buffer es baja puede
	// resultar en overflow
//	__testrt();

	// Push mode
	// Utiliza el buffer
	// Existe un callback periodico que lee el buffer y
	// guarda los datos. Luego imprime los datos guardados
	__test();

	outputFilecsv.close();

	MPUManager::GetInstance()->Stop();

	return 0;
}
