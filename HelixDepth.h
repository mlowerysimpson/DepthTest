#pragma once
#include <wiringPi.h>
#include <iostream>
#include <pthread.h>

#define SER_BUFSIZE 1024  //size of input serial buffer
/**
 * @brief class for getting depth readings from the Helix series of fish finders from Humminbird (
 *
 */
class HelixDepth {
public:
    HelixDepth(char *szSerialPort);
    ~HelixDepth();
    //data:
    bool m_bExitDataCollectThread;//true if it is time to exit the data collection thread
    bool m_bSerportThreadRunning;//true if the serial data collection thread is running
    char* m_szSerialPort;//the serial port used to receive the data from the fish finder (ex: "/dev/ttyUSBPort3")
    //functions
    double GetDepth();//gets the current depth measurement in m
    double GetTemperature();//gets the current temperature measurement in deg C
    void ProcessBytes();
    void ReadInBytes(int fd, int nNumToRead);
private:
    //data
    int m_nNumDepthReadings;//number of depth readings recorded
    int m_nNumTempReadings;//number of temperature readings recorded
    char m_inBuffer[SER_BUFSIZE];
    int m_nBufWriteIndex;//index in input buffer where data will be written next
    int m_nNumInBuffer;//number of bytes in the input buffer that have not been processed yet
    
    pthread_t m_serportThreadId;//thread id for getting data over a serial link
    double m_dTemperature;//the current temperature reading in deg C
    double m_dDepth;//the current depth reading in m
    
    //functions
    static void* serportFunction(void* pParam);//function receives depth and temperature data (in NMEA0183 format) from fish finder over serial link
    void GetDepthReading(char *depthText, int nSize);//get depth reading from NMEA0183 string
    void GetTemperatureReading(char* tempText, int nSize);//get temperature reading from NMEA0183 string
    int ParseBufferAt(int nBufIndex, int nNumInBuffer);//try to parse buffer at nBufIndex to see if it contains any valid data
    
    void StartCollectionThread();//start thread for collecting fish finder depth and temperature data
    void StopCollectionThread();//stop the data collection thread
};