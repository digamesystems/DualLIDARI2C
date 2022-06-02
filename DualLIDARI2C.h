#ifndef __DUAL_LIDAR_H__
#define __DUAL_LIDAR_H__


#include <digameDebug.h>  // debug defines
#include <TFMPI2C.h>      // Include TFMini Plus LIDAR Library v1.5.0
                          // https://github.com/budryerson/TFMini-Plus

#define NEITHER 0  // Visibility of a target by each sensor
#define SENSOR1 1
#define SENSOR2 2 
#define BOTH    3

class DualLIDARI2C{

  public: 
    DualLIDARI2C();
    ~DualLIDARI2C();

    uint8_t version[3]; // FW version
    uint8_t status;     // Error status

    bool begin(uint8_t sda, uint8_t scl); // Specify Pins
    bool begin();                 // Use Default Pins 
    
    bool getRanges(int16_t &dist1, int16_t &dist2);
    
    void setZone(int zoneMin, int zoneMax);
    void getZone(int &zoneMin, int &zoneMax);

    void  setSmoothingFactor(float newSmoothingFactor);
    float getSmoothingFactor();

    int  getVisibility();

  private: 
    TFMPI2C tfmP;

    float smoothingFactor = 0.95;
    float smoothedDist1 = 0;
    float smoothedDist2 = 0;

    uint8_t SDA=21, SCL=22; 
    uint8_t lidar_1_addr = 10;
    uint8_t lidar_2_addr = 11; 

    int zoneMin = 0; 
    int zoneMax = 100;
    int visibility = NEITHER;
  
    void initLIDAR();


};

#endif //__DUAL_LIDAR_H__
