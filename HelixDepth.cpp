#include <stdio.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include <unistd.h>
#include <string> 
#include <string.h>
#include "HelixDepth.h"

HelixDepth::HelixDepth(char* szSerialPort) {
	m_szSerialPort = new char[strlen(szSerialPort) + 1];
	strcpy(m_szSerialPort, szSerialPort);
	
	m_dTemperature = 0.0;//the current temperature reading in deg C
	m_dDepth = 0.0;//the current depth reading in m
	m_nNumDepthReadings = 0;//number of depth readings recorded
	m_nNumTempReadings = 0;//number of temperature readings recorded
	m_serportThreadId = 0;
	m_bSerportThreadRunning = false;
	m_bExitDataCollectThread = false;
	memset(m_inBuffer,0,SER_BUFSIZE);
	m_nBufWriteIndex = 0;//index in input buffer where data will be written next
	m_nNumInBuffer = 0;//number of bytes in the input buffer that have not been processed yet
	StartCollectionThread();
}

HelixDepth::~HelixDepth() {
	StopCollectionThread();
	if (m_szSerialPort != nullptr) {
		delete[] m_szSerialPort;
		m_szSerialPort = nullptr;
	}
}

double HelixDepth::GetDepth() {//gets the current depth measurement in m
	return m_dDepth;
}

double HelixDepth::GetTemperature() {//gets the current temperature measurement in deg C
	return m_dTemperature;
}

void HelixDepth::StartCollectionThread() {//start thread for collecting fish finder depth and temperature data
	int nError = pthread_create(&m_serportThreadId, NULL, &serportFunction, (void *)this);
	if (nError != 0) {
		printf("Can't create thread: %s", strerror(nError));
	}
}

void HelixDepth::StopCollectionThread() {//stop the data collection thread
	m_bExitDataCollectThread = true;
	unsigned int uiTimeoutTime = millis() + 2000;//timeout after 2 seconds
	while (m_bSerportThreadRunning && millis() < uiTimeoutTime) {
		delay(100);
	}
}

void* HelixDepth::serportFunction(void* pParam) {//function receives depth and temperature data (in NMEA0183 format) from fish finder over serial link
	HelixDepth* pDepthObj = (HelixDepth*)pParam;
	pDepthObj->m_bSerportThreadRunning = true;
	int fd = serialOpen(pDepthObj->m_szSerialPort, 4800);
	if (fd < 0) {
		printf("Unable to open serial device: %s\n", strerror(errno));
		pDepthObj->m_bSerportThreadRunning = false;
		return nullptr;
	}
	printf("Listening for depth data on port %s\n", pDepthObj->m_szSerialPort);
	while (!pDepthObj->m_bExitDataCollectThread) {
		int nNumBytesAvailable =  serialDataAvail(fd);
		if (nNumBytesAvailable > 0) {
			if (nNumBytesAvailable > SER_BUFSIZE) {
				nNumBytesAvailable = SER_BUFSIZE;
			}
			pDepthObj->ReadInBytes(fd, nNumBytesAvailable);
			pDepthObj->ProcessBytes();
		}
		delay(300);//pause for 300 ms
	}
	pDepthObj->m_bSerportThreadRunning = false;
	//test
	printf("depth reading thread finished\n");
	//end test
	return nullptr;
}

void HelixDepth::ReadInBytes(int fd, int nNumToRead) {
	for (int i = 0; i < nNumToRead; i++) {
		int nCharVal = serialGetchar(fd);//	Returns the next character available on the serial device.This call will block for up to 10 seconds if no data is available(when it will return -1)
		if (nCharVal < 0) {
			return;
		}
		m_inBuffer[m_nBufWriteIndex] = (char)nCharVal;
		m_nNumInBuffer++;
		m_nBufWriteIndex++;
		if (m_nBufWriteIndex >= SER_BUFSIZE) {
			m_nBufWriteIndex = 0;
		}
	}
	if (m_nNumInBuffer > SER_BUFSIZE) {
		m_nNumInBuffer = SER_BUFSIZE;
	}
}

void HelixDepth::ProcessBytes() {
	//process whatever bytes are currently in the buffer
	if (m_nNumInBuffer <= 0) return;
	int nNumProcessed = 0;
	int j = m_nBufWriteIndex - m_nNumInBuffer;
	if (j < 0) j += SER_BUFSIZE;
	nNumProcessed = ParseBufferAt(j,m_nNumInBuffer);
	if (nNumProcessed > 0) {
		m_nNumInBuffer -= nNumProcessed;
	}
}

int HelixDepth::ParseBufferAt(int nBufIndex, int nNumInBuffer) {//try to parse buffer at nBufIndex to see if it contains any valid data
	char readingText[128];
	//test
	/*char* testBuf = new char[nNumInBuffer + 1];
	for (int i = 0; i < nNumInBuffer; i++) {
		int j = (i + nBufIndex) % SER_BUFSIZE;
		testBuf[i] = m_inBuffer[j];
	}
	testBuf[nNumInBuffer] = 0;
	printf("%s\n", testBuf);
	delete[]testBuf;*/
	//end test

	if (nNumInBuffer < 6) {
		return 0;//not enough to parse anything yet
	}
	int nParsedBytes = 0;
	for (int i = 0; i < nNumInBuffer; i++) {
		int j = (nBufIndex + i)%SER_BUFSIZE;
		if ((i + 6) < nNumInBuffer) {
			if (m_inBuffer[j] == '$') {
				
				if (strncmp(&m_inBuffer[j], (char*)"$INDPT",6) == 0) {//start of depth reading
					//check to see if we can read in depth (i.e. read up to CR/LF)
					i += 6;
					j = (nBufIndex + i)%SER_BUFSIZE;
					int m = 0;
					while (i < nNumInBuffer && m_inBuffer[j] != '\r' && m_inBuffer[j] != '\n') {
						readingText[m] = m_inBuffer[j];
						i++;
						j = (j + 1) % SER_BUFSIZE;
						m++;
					}
					if (i < nNumInBuffer && (m_inBuffer[j] == '\r' || m_inBuffer[j] == '\n')) {
						//reached end of depth reading
						GetDepthReading(readingText, m);
						while (i < nNumInBuffer && (m_inBuffer[j] == '\r' || m_inBuffer[j] == '\n')) {
							i++;
							j = (j + 1) % SER_BUFSIZE;
						}
						nParsedBytes = i;
					}
				}
				else if (strncmp(&m_inBuffer[j], (char *)"$INMTW",6)==0) {//start of temperature reading
					//check to see if we can read in temperature (i.e. read up to CR/LF)
					i += 6;
					j = (nBufIndex + i) % SER_BUFSIZE;
					int m = 0;
					while (i < nNumInBuffer && m_inBuffer[j] != '\r' && m_inBuffer[j] != '\n') {
						readingText[m] = m_inBuffer[j];
						i++;
						j = (j + 1) % SER_BUFSIZE;
						m++;
					}
					if (i < nNumInBuffer && (m_inBuffer[j] == '\r' || m_inBuffer[j] == '\n')) {
						//reached end of depth reading
						GetTemperatureReading(readingText, m);
						while (i < nNumInBuffer && (m_inBuffer[j] == '\r' || m_inBuffer[j] == '\n')) {
							i++;
							j = (j + 1) % SER_BUFSIZE;
						}
						nParsedBytes = i;
					}
				}
			}
		}
		if (m_inBuffer[j] == '\r' || m_inBuffer[j] == '\n') {//reached a terminator character
			//end of a sentence
			nParsedBytes = i;
		}
	}
	return nParsedBytes;
}

void HelixDepth::GetDepthReading(char* depthText, int nSize) {//get depth reading from NMEA0183 string
	//$INDPT part is not included in depthText
	double dDepthM = 0.0;//depth to bottom measured from transducer in m
	double dDistToSurfaceM = 0.0;//distance of transducer to surface (or keel if negative) in m
	//test
	//printf("About to scan depth reading.\n");
	//end test
	int nNumscanned = sscanf(depthText, ",%lf,%lf", &dDepthM, &dDistToSurfaceM);
	//test
	//printf("depthText = %s, nNumscanned = %d\n", depthText, nNumscanned);
	//end test
	if (nNumscanned==2) {
		if (dDistToSurfaceM >= 0.0) {
			dDepthM += dDistToSurfaceM;
			m_dDepth = dDepthM;
			//printf("Depth = %.1f m\n", dDepthM);
			m_nNumDepthReadings++;
		}
	}
}

void HelixDepth::GetTemperatureReading(char* tempText, int nSize) {//get temperature reading from NMEA0183 string
	//#INMTW part is not included
	double dTempDegC = 0.0;
	bool bCelsius = true;
	if (strstr(tempText, (char*)",C*") == nullptr) {
		bCelsius = false;
	}
	//test
	//printf("About to scan temperature.\n");
	//end test
	if (sscanf(tempText, ",%lf", &dTempDegC) == 1) {
		if (!bCelsius) {
			dTempDegC = (dTempDegC - 32) / 1.8;
		}
		m_dTemperature = dTempDegC;
		//printf("Temperature = %.1f deg C\n", dTempDegC);
		m_nNumTempReadings++;
	}
}

