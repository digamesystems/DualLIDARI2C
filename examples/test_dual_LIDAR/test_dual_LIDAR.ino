/* test_dual_LIDAR.ino
 * 
 * A test program to demonstrate how to work with two TFMini-S LIDAR distance sensors using I2C communication. 
 * 
 * Makes use of Bud Ryerson's TFMini Plus LIDAR Library for I2C
 *  https://github.com/budryerson/TFMini-Plus
 * 
 * And Roberto Lo Giacco's library for working with circular buffers under the hood.
 *  https://github.com/rlogiacco/CircularBuffer
 * 
 * See: DualLIDARI2c.h for includes. Both of the above are available via the Arduino Library Manager.
 * 
 * BIG NOTE: System assumes the two LIDAR sensors have I2C addresses of DECIMAL, not hex, values 10 and 11. 
 * (The TFMini Plus library examples show how to configure your sensor's address.)
 * 
 * Program output is designed to display nicely in the Arduino IDE's Seraial Plotter utility.
 * 
 * Author: J. Price, Digame Systems, 2022
 * GitHub Repo: https://github.com/digamesystems/DualLIDARI2C
 * 
 */

#include <DualLIDARI2C.h>

DualLIDARI2C dualLIDAR;

bool    lidarOK = false; 
int16_t zoneMin = 0;            // Zone of interest (cm)
int16_t zoneMax = 150;
float   smoothingFactor = 0.5; // Exponentially weighted moving average -- range = 0.. < 1.0 
int16_t dist1, dist2;          // Smoothed distances

//**********************************************************************************
void setup() {
//**********************************************************************************
  Serial.begin(115200);
  delay(100);
  
  //lidarOK = dualLIDAR.begin(false); // 'false' is 'self-triggered' running at 1000 Hz
  
  lidarOK = dualLIDAR.begin(true); // 'true' means the sensor will trigger itself after a 
                                   // reading and wait for the next read to trigger again.
                                   // This consumes less power. - About 25mA / sensor at 100Hz.
    
  if (lidarOK){   
    dualLIDAR.setZone(zoneMin, zoneMax);

    int16_t zMin, zMax;
    dualLIDAR.getZone(zMin, zMax);
    Serial.printf("Zone set to: Min = %d, Max = %d\n",zMin, zMax);
    
    dualLIDAR.setSmoothingFactor(smoothingFactor); 
    Serial.printf("Smoothing factor set to: %f\n", dualLIDAR.getSmoothingFactor());
     
  } else {
    Serial.printf("ERROR: Trouble initializing dualLIDAR.\n");    
  }

  delay(2000);

  Serial.printf("LIDAR1, STAT1, LIDAR2, STAT2, VISIBILITY, IN/OUT\n");
  
}


//**********************************************************************************
void loop() {
//**********************************************************************************
  // Read the sensors and plot distances, return status codes, 
  // visibility on one, none or both sensors, and direction of travel. 
  // For a quick demo, point your sensors at the ceiling and wave your hand across
  // them. 
  
  if (lidarOK){
    dualLIDAR.getRanges(dist1,dist2);
    Serial.printf("Dist1:%d,Stat1:%d,Dist2:%d,Stat2:%d,Visibility:%d,IN-OUT:%d\n",\
                  dist1, dualLIDAR.status1,\
                  dist2, dualLIDAR.status2,\
                  dualLIDAR.getVisibility()*100+300,\
                  dualLIDAR.getEvent()*100 + 650);    // in&out events have different heights in the serial plotter. 
  }
  delay(10); // Wait so polling rate is c.a. 100Hz.
}
