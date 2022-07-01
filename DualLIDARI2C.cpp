#include <digameDebug.h>  // debug defines

#include <Wire.h>         // Arduino standard I2C/Two-Wire Library
#include <TFMPI2C.h>      // Include TFMini Plus LIDAR Library v1.5.0
                          // https://github.com/budryerson/TFMini-Plus-I2C

#include <DualLIDARI2C.h> 

DualLIDARI2C::DualLIDARI2C(){}
DualLIDARI2C::~DualLIDARI2C(){}


//****************************************************************************************
bool DualLIDARI2C::begin(uint8_t sda, uint8_t scl, bool triggered)
//****************************************************************************************
{
  triggeredOperation = triggered; 
  SDA = sda;
  SCL = scl; 
  return initLIDAR(); 
}


//****************************************************************************************
bool DualLIDARI2C::begin(bool triggered)
//****************************************************************************************
{
  return begin(SDA, SCL, triggered);
}
    

//****************************************************************************************
int16_t DualLIDARI2C::getRange(uint8_t sensorAddr)
//****************************************************************************************
{
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  const int16_t fluxMin = 100; // Lower bound for valid signal strength from sensor.
                               // Signals below this will be reported at a distance 
                               // of 1300 cm. (outide our range of interest)
  int16_t dist;

  if( tfmP.getData( tfDist, tfFlux, tfTemp, sensorAddr)) { 
    if (sensorAddr == lidar_1_addr){
      dist = deGlitch1(tfDist);
    } else{
      dist = deGlitch2(tfDist);
    }
  } else {   
    if (tfmP.status >= TFMP_WEAK){ // higher errors are sensor read issues
      dist = 1300; // The getData command will return false if there is no 
                    // target detected. Here we call 'infinity' 1300.
    } else { // Communication error -- serial or i2c.
      dist = 1301;
      DEBUG_PRINT("ERROR: ");
      DEBUG_PRINTLN(tfmP.status);
      bool stat = initLIDAR();
      if( stat) { DEBUG_PRINTLN("Reinitialized."); }
      else { DEBUG_PRINTLN("Reinitialization failed.")}  
    }
  }

  if (sensorAddr==lidar_1_addr){
    status1 = tfmP.status;
  } else {
    status2 = tfmP.status;
  }

  return dist;

}

//****************************************************************************************
bool DualLIDARI2C::getRanges( int16_t &dist1, int16_t &dist2)
//****************************************************************************************
{
  visibility = 0; 

  smoothedDist1 = smoothedDist1 * smoothingFactor + \
                  (float) getRange(lidar_1_addr) * (1-smoothingFactor);
  
  smoothedDist2 = smoothedDist2 * smoothingFactor + \
                  (float) getRange(lidar_2_addr) * (1-smoothingFactor);

  dist1 = smoothedDist1;
  dist2 = smoothedDist2;
  
  lidarHistoryBuffer.push((dist1-dist2)); // A history of the signals for display.

  if ((dist1>=zoneMin)&&(dist1<=zoneMax)){
    visibility +=1;
  }

  if ((dist2>=zoneMin)&&(dist2<=zoneMax)){
    visibility +=2;
  }

  if (triggeredOperation){
    // Trigger sensor for the next time around
    if( tfmP.sendCommand( TRIGGER_DETECTION, 0, lidar_1_addr) != true) {
      DEBUG_PRINTLN("Trouble Triggering LIDAR 1");
    }

    if( tfmP.sendCommand( TRIGGER_DETECTION, 0, lidar_2_addr) != true) {
      DEBUG_PRINTLN("Trouble Triggering LIDAR 2");
    }
  }

  return true;
}


void DualLIDARI2C::setZone(int16_t zMin, int16_t zMax)
{
  zoneMin = zMin;
  zoneMax = zMax;
}

void DualLIDARI2C::getZone(int16_t &zMin, int16_t &zMax)
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

int16_t DualLIDARI2C::getVisibility()
{
  return visibility;
}

//**************************************************************************************** 
int16_t DualLIDARI2C::getEvent()
//**************************************************************************************** 
{

  /*
  The event detection relies on a change in visibility 
  on the two LIDARs.

  Visibility States:
   * SENSOR1 - visible on LIDAR 1
   * SENSOR2 - visible on LIDAR 2
   * BOTH - visible on both sensors
   * NEITHER - no target seen by either sensor

  Original algoritm: 
    BOTH-> SENSOR1 = IN. 
    BOTH-> SENSOR2 = OUT. 

  There are conditions where this isn't strict enough. 
  
  New Algorithms: 
    NEITHER-> SENSOR1 -> BOTH -> SENSOR2 -> NEITHER = OUT.
    NEITHER-> SENSOR2 -> BOTH -> SENSOR1 -> NEITHER = IN.
    Other options are explored below
  
  For this, we need a little state history buffer to generate events.
  Using a 5 point circular buffer. 
  */

  int16_t retVal; 
  
  static int16_t previousVisibility;
  int16_t        currentVisibility;

  retVal = NOEVENT;

  currentVisibility = getVisibility();

  if (currentVisibility != previousVisibility){
    // We've had a state change. 
    visibilityHistory.push(currentVisibility);
    if (
       //(visibilityHistory[0]==NEITHER) &&
        (visibilityHistory[1]==SENSOR1) &&
        (visibilityHistory[2]==BOTH)    &&
        (visibilityHistory[3]==SENSOR2) &&
        (visibilityHistory[4]==NEITHER)
    ) { retVal = OUTEVENT;} 
    else if (
        //(visibilityHistory[0]==NEITHER) &&
        (visibilityHistory[1]==SENSOR2) &&
        (visibilityHistory[2]==BOTH)    &&
        (visibilityHistory[3]==SENSOR1) &&
        (visibilityHistory[4]==NEITHER)
    ) { retVal = INEVENT;}
    else if ( // Sometimes we don't get a reading on the other sensor as we leave
        //(visibilityHistory[0]==NEITHER) &&
        (visibilityHistory[2]==SENSOR2) &&
        (visibilityHistory[3]==BOTH)    &&
        (visibilityHistory[4]==NEITHER)
    ) { retVal = INEVENT;}
    else if ( // Sometimes we don't get a reading on the other sensor as we leave
        //(visibilityHistory[0]==NEITHER) &&
        (visibilityHistory[2]==SENSOR1) &&
        (visibilityHistory[3]==BOTH)    &&
        (visibilityHistory[4]==NEITHER)
    ) { retVal = OUTEVENT;}; 

  }

  previousVisibility = currentVisibility;

  return retVal;
}


//**************************************************************************************** 
bool DualLIDARI2C::initLIDAR() // Initialize a LIDAR sensor on a 
                                                   // serial port.
//****************************************************************************************
{
  bool retVal = false;
 

  Wire.begin();          
  Wire.setClock(400000);  // Set I2C bus speed to 'Fast' if
                          // your Arduino supports 400KHz.

  if( tfmP.sendCommand( SOFT_RESET, 0, lidar_1_addr)) { retVal = true; }
  else { return false;}  
  
  if( tfmP.sendCommand( SOFT_RESET, 0, lidar_2_addr)) { retVal = true; }
  else { return false;}  
  
  if (triggeredOperation){
      tfmP.sendCommand( SET_FRAME_RATE, FRAME_0, lidar_1_addr);
      tfmP.sendCommand( SET_FRAME_RATE, FRAME_0, lidar_2_addr);

      // using this with FRAME_0 and triggered operation
      if( tfmP.sendCommand( TRIGGER_DETECTION, 0, lidar_1_addr) != true) {
        DEBUG_PRINTLN("Trouble Triggering LIDAR 1");
      }

      if( tfmP.sendCommand( TRIGGER_DETECTION, 0, lidar_2_addr) != true) {
        DEBUG_PRINTLN("Trouble Triggering LIDAR 2");
      }
  
  } else {
    tfmP.sendCommand( SET_FRAME_RATE, FRAME_1000, lidar_1_addr);
    tfmP.sendCommand( SET_FRAME_RATE, FRAME_1000, lidar_2_addr);
  }

  int16_t d1, d2;

  getRanges(d1,d2);
  smoothedDist1 = d1;
  smoothedDist2 = d2;
  
  return retVal;
}

//****************************************************************************************
int16_t DualLIDARI2C::deGlitch1(int16_t currentPoint){
//****************************************************************************************
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
int16_t DualLIDARI2C::deGlitch2(int16_t currentPoint){
//****************************************************************************************
  static int16_t p1, p2, p3,retVal;

  p1 = currentPoint;

    // Hard-Coded Glitch detector
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



