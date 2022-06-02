/*
 * Building classes to support the Digame Directional LIDAR counter.
 * 
 */
#include <digameDebug.h>  // debug defines
#include <DualLIDARI2C.h>

DualLIDARI2C dL; 
int16_t dist1;
int16_t dist2;

void setup() {
  
  Serial.begin(115200);
  
  int sda=21, scl=22; 
  int z1, z2;

  dL.begin(sda,scl);
  dL.setZone(0,122);
  dL.getZone(z1,z2);
  dL.setSmoothingFactor(0.9);
  
  
  DEBUG_PRINT(z1);
  DEBUG_PRINT(" ");
  DEBUG_PRINTLN(z2);
  DEBUG_PRINTLN(dL.getVisibility()); 
}


int currentVisibility;
int previousVisibility;

void loop() {

  dL.getRanges(dist1, dist2);
  
  DEBUG_PRINT(dist1);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(dist2);

  currentVisibility = dL.getVisibility();

  if (previousVisibility == BOTH){
    if (currentVisibility == SENSOR1){
      DEBUG_PRINT(" IN EVENT");  
    }  
    if (currentVisibility == SENSOR2){
      DEBUG_PRINT(" OUT EVENT");  
    }   
  }

  previousVisibility = currentVisibility;

  DEBUG_PRINTLN();
  //DEBUG_PRINTLN(dL.getVisibility()*100);
  
  delay(10);
}
