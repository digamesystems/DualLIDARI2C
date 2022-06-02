#include <digameDebug.h>  // debug defines

#include <Wire.h>         // Arduino standard I2C/Two-Wire Library
#include <TFMPI2C.h>      // Include TFMini Plus LIDAR Library v1.5.0
                          // https://github.com/budryerson/TFMini-Plus-I2C

#include <DualLIDARI2C.h> 

DualLIDARI2C::DualLIDARI2C(){}
DualLIDARI2C::~DualLIDARI2C(){}


//****************************************************************************************
bool DualLIDARI2C::begin(uint8_t sda, uint8_t scl)
//****************************************************************************************
{
  SDA = sda;
  SCL = scl; 
  initLIDAR();
  return true; //fix.
}


//****************************************************************************************
bool DualLIDARI2C::begin()
//****************************************************************************************
{
  return begin(SDA, SCL);
}
    

int16_t deGlitch1(int16_t currentPoint){
  static int16_t p1, p2, p3,retVal;

  p1 = currentPoint;

    // Glitch detector
  if (abs(p3-p1) < 10) {   // The latest point and the two points back are pretty close 
    if (abs(p2-p3) > 10) { // The point in the middle is too different from the adjacent points -- ignore
      p2 = p3;
    }
  }

  retVal = p3;

  // Shift the history
  p3 = p2;
  p2 = p1;

  return retVal;
  
}

int16_t deGlitch2(int16_t currentPoint){
  static int16_t p1, p2, p3,retVal;

  p1 = currentPoint;

    // Glitch detector
  if (abs(p3-p1) < 10) {   // The latest point and the two points back are pretty close 
    if (abs(p2-p3) > 10) { // The point in the middle is too different from the adjacent points -- ignore
      p2 = p3;
    }
  }

  retVal = p3;

  // Shift the history
  p3 = p2;
  p2 = p1;

  return retVal;
  
}

//****************************************************************************************
bool DualLIDARI2C::getRanges( int16_t &dist1, int16_t &dist2)
//****************************************************************************************
{
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  
  // Read the LIDAR Sensor
  if( tfmP.getData( tfDist, tfFlux, tfTemp, lidar_1_addr)) { 
    dist1 = deGlitch1(tfDist);
  } else { return false; }
 
  if( tfmP.getData( tfDist, tfFlux, tfTemp, lidar_2_addr)) { 
    dist2 = deGlitch2(tfDist);
  } else { return false; }

  smoothedDist1 = smoothedDist1 * smoothingFactor + (float)dist1 * (1-smoothingFactor);
  dist1 = smoothedDist1;

  smoothedDist2 = smoothedDist2 * smoothingFactor + (float)dist2 * (1-smoothingFactor);
  dist2 = smoothedDist2;

  visibility = 0; 

  if ((dist1>=zoneMin)&&(dist1<=zoneMax)){
    visibility +=1;
  }

  if ((dist2>=zoneMin)&&(dist2<=zoneMax)){
    visibility +=2;
  }

  return true;
}



void DualLIDARI2C::setZone(int zMin, int zMax)
{
  zoneMin = zMin;
  zoneMax = zMax;
}

void DualLIDARI2C::getZone(int &zMin, int &zMax)
{
  zMin = zoneMin;
  zMax = zoneMax;
}

void DualLIDARI2C::setSmoothingFactor(float newSmoothingFactor)
{
  smoothingFactor = newSmoothingFactor;
}

float DualLIDARI2C::getSmoothingFactor()
{
  return smoothingFactor;
}

int DualLIDARI2C::getVisibility()
{
  return visibility;
}


//**************************************************************************************** 
void DualLIDARI2C::initLIDAR() // Initialize a LIDAR sensor on a 
                                                   // serial port.
//****************************************************************************************
{
  // - - - - -   RECOVER I2C BUS  - - - - - - - - - - - - - - - 
  // An I2C device that quits unexpectedly can leave the I2C bus hung,
  // waiting for a transfer to finish.  This function bypasses the Wire
  // library and sends 8 pseudo clock cycles, a NAK, and a STOP signal
  // to the SDA and SCL pin numbers. It flushes any I2C data transfer
  // that may have been in progress, and ends by calling `Wire.begin()`.
  //tfmP.recoverI2CBus();

  Wire.begin();
  //Wire.begin(SCL,SDA);           
  Wire.setClock(400000);  // Set I2C bus speed to 'Fast' if
                          // your Arduino supports 400KHz.

  // Send some example commands to the TFMini-Plus
  // - - Perform a system reset - - - - - - - - - - -
  tfmP.sendCommand( SOFT_RESET, 0, lidar_1_addr);
  tfmP.sendCommand( SOFT_RESET, 0, lidar_2_addr);

  tfmP.sendCommand( SET_FRAME_RATE, FRAME_200, lidar_1_addr);
  tfmP.sendCommand( SET_FRAME_RATE, FRAME_200, lidar_2_addr);

  delay(500);

  int16_t d1, d2;

  getRanges(d1,d2);
  smoothedDist1 = d1;
  smoothedDist2 = d2;

}


