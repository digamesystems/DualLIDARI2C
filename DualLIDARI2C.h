#ifndef __DUAL_LIDAR_H__
#define __DUAL_LIDAR_H__


#include "digameDebug.h"  // debug defines
#include <TFMPI2C.h>      // Include Bud Ryerson's TFMini Plus LIDAR Library for I2C
                          // https://github.com/budryerson/TFMini-Plus

#include <CircularBuffer.h> // And Roberto Lo Giacco's library for working with circular buffers. 
                            // https://github.com/rlogiacco/CircularBuffer

#define NEITHER 0  // Visibility of a target by each sensor
#define SENSOR1 1
#define SENSOR2 2 
#define BOTH    3

#define NOEVENT 0
#define INEVENT 1
#define OUTEVENT 2

class DualLIDARI2C{

  public: 
    DualLIDARI2C();
    ~DualLIDARI2C();

    uint8_t version[3]; // FW version
    uint8_t status;     // Error status

    CircularBuffer<int, 150> lidarHistoryBuffer; // A buffer for visualization of the signal history
                                                 // before the algorithm makes a decision.

    CircularBuffer<int,5> visibilityHistory; 


    bool begin(uint8_t sda, uint8_t scl, bool triggered); // Specify Pins
    bool begin(bool triggered);                 // Use Default Pins 
    
    bool getRanges(int16_t &dist1, int16_t &dist2);
    
    void setZone(int16_t zoneMin,  int16_t zoneMax);
    void getZone(int16_t &zoneMin, int16_t &zoneMax);

    void  setSmoothingFactor(float newSmoothingFactor);
    float getSmoothingFactor();

    int16_t  getVisibility();
    int16_t  getEvent();
    
    uint8_t status1 = 0;  // Status code of last command to each sensor
    uint8_t status2 = 0;  // See TFMPI2C.h for list

  private: 
    TFMPI2C tfmP;

    bool  triggeredOperation = false; 

    float smoothingFactor = 0.95;
    float smoothedDist1 = 0;
    float smoothedDist2 = 0;

    uint8_t SDA=21, SCL=22; 
    uint8_t lidar_1_addr = 10;  // Change these to match your sensors' addresses
    uint8_t lidar_2_addr = 11;  // TODO: make part of initialization. 

    int16_t zoneMin = 0; 
    int16_t zoneMax = 100;
    int16_t visibility = NEITHER;
  
    bool initLIDAR();
    int16_t deGlitch1(int16_t currentPoint);
    int16_t deGlitch2(int16_t currentPoint); 
    int16_t getRange(uint8_t sensorAddr);


};

#endif //__DUAL_LIDAR_H__
