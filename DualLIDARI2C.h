#ifndef __DUAL_LIDAR_H__
#define __DUAL_LIDAR_H__
/*
  DualLidarI2C - A class for working with two TFMINI-S/Plus sensors to determine direction of 
  travel for a target along with presence detection. 

  Makes Use of Bud Ryerson's TFMini Plus LIDAR library for I2C. 

  To use this library, you'll need two TFMinis configured for I2C operation, set to two
  different I2C addresses. (Bud's library contains example code for doing this.) The default 
  addresses are asumed to be decimal 10 and 11 but you can use what you like, if you specify 
  them at initialization. 

  The gist of the direction of travel determination is to track changes in 'visibility' on each 
  sensor. A target is considered 'visible' if is detected on a sensor at a distance within a 
  user configurable 'read zone' (range_min, range_max). 

  If the LIDARS are positioned a bit apart so that their read zones overlap a bit, you can watch 
  visibility change as a target passes. For example: 
    
    VisibleOnNeither -> VisibleOn1 -> VisibleOnBoth -> VisibleOn2 -> VisibleOnNeither

  Might be a target passing from "left to right". A different pattern would obtain for moving 
  "right to left". 

  The class provides some functions for 'de-Glitching' and 'smoothing' the signals to improve
  performance. 

  Author: J. Price, Digame Systems, 2022. 
  See: https://github.com/digamesystems/DualLIDARI2C



*/

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

    CircularBuffer<int,5> visibilityHistory;     // The last 5 visibility states for a target on the
                                                 // two sensors


    bool begin(uint8_t sda, uint8_t scl, bool triggered); // Specify Pins
    bool begin(bool triggered);                           // Use Default Pins 
    
    bool getRanges(int16_t &dist1, int16_t &dist2);       // Distance measured on each sensor
    
    void setZone(int16_t zoneMin,  int16_t zoneMax);      // The read zone where a target is eligible to be "visible"
    void getZone(int16_t &zoneMin, int16_t &zoneMax);

    void  setSmoothingFactor(float newSmoothingFactor);   // Exponentially weighted moving average
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
  
    bool resetSensor(uint8_t sensorAddr);
    bool initLIDAR();
    int16_t deGlitch1(int16_t currentPoint);
    int16_t deGlitch2(int16_t currentPoint); 
    int16_t getRange(uint8_t sensorAddr);


};

#endif //__DUAL_LIDAR_H__
