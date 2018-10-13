#include "JY901.h"
#include "string.h"

//***************************iic part********************************

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;

void SerialEnventListener(unsigned char data) {
	static unsigned char rxBuffer[250];
	static unsigned char rxCnt = 0;
	
	rxBuffer[rxCnt++] = data;
	if (rxBuffer[0] != 0x55) {
		rxCnt = 0;
		return;
	}
	if (rxCnt < 11) {
		return;
	} else {
		switch (rxBuffer[1]) {
			case 0x50:	memcpy(&stcTime, &rxBuffer[2],8); break;
			case 0x51:	memcpy(&stcAcc, &rxBuffer[2],8); break;
			case 0x52:	memcpy(&stcGyro, &rxBuffer[2],8); break;
			case 0x53:	memcpy(&stcAngle, &rxBuffer[2],8); break;
			case 0x54:	memcpy(&stcMag, &rxBuffer[2],8); break;
			case 0x55:	memcpy(&stcDStatus, &rxBuffer[2],8); break;
			case 0x56:	memcpy(&stcPress, &rxBuffer[2],8); break;
			case 0x57:	memcpy(&stcLonLat, &rxBuffer[2],8); break;
			case 0x58:	memcpy(&stcGPSV, &rxBuffer[2],8); break;
		} 
		rxCnt = 0;
	}
		
}