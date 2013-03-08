#include "SerialPort.h"
#include <stdio.h>
#include <tchar.h>

using namespace std;

CSerialPort ardPort;
LPCTSTR portname = _T("COM4"); //set the proper COM port here
int baudRate = 4800;	//set matching BaudRate here

HRESULT connectArduino();

void send2Ard(int motorIndex, int angle);

HRESULT connectArduino(){
	ardPort.Open(portname, baudRate, 8, NOPARITY, ONESTOPBIT, GENERIC_READ | GENERIC_WRITE);

	if(ardPort.IsOpen()) {
		return S_OK;
	} else {
		return S_FALSE;
	}
}

void shutdownArduino(){
	ardPort.Close();
}

void send2Ard(int motorIndex, int angle){
	char command[6];
	  switch (motorIndex){
		case 1:
			//pino 02
			 //printf("Motor 1 (base) escolhido\nposicao atual: %d\n", angle);
			 sprintf(command, "A %d", angle);
			 ardPort.Write(command, 5);
			 break;
		case 2:
			//pino 10
			 //printf("Motor 2 (ombro) escolhido\nposicao atual: %d\n", angle);
			 sprintf(command, "B %d", angle);
			 ardPort.Write(command, 5);
			 break;
		case 3:
			//pino 11
			 //printf("Motor 3 (cotovelo) escolhido\nposicao atual: %d\n", angle);
			 sprintf(command, "C %d", angle);
			 ardPort.Write(command, 5);
			 break;
		case 4:
			//pino 12
			 //printf("Motor 4 (pulso) escolhido\nposicao atual: %d\n", angle);
			 sprintf(command, "D %d", angle);
			 ardPort.Write(command, 5);
			 break;
		case 5:
			//pino 13
			// printf("Motor 5 (garra) escolhido\nposicao atual: %d\n", angle);
			 sprintf(command, "E %d", angle);
			 ardPort.Write(command, 5);
			 break;
		default:
			 printf("Erro na seleção do motor\n");
			 break;  
		}
}