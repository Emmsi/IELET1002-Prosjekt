// FORELØPIG 
//gammel kode, finner ikke den nye ヽ(ಠ_ಠ)ノ

#include <Zumo32U4.h>
#include <speed_controls.h>
#include <limits>

int batteryUsage = 0; 
float batteryLife = 8800; // Reele batteristørrelsen i mAh, bruker mAs for enklere testing.
float batteryHealth = 100;

float constrainedBatteryLife = constrain(batteryLife, 0, 2000);
float constrainedBatteryHealth = constrain(batteryHealth, 0, 100);

void loop()
  
  // Utlading i forhold til fart 
if (speedInCmPerSec >= 0){  
  int batteryUsage(float 0.1 * speedInCmPerSec){
  constrainedBatteryLife -= (0.1 * speedInCmPerSec);

   // Lading i forhold til ryggefart 
else{
  Int batteryUsage(float 0.09 * speedInCmPerSec){
  constrainedBatterylife += (0.09 * speedInCmPerSec);
  

  if (constrainedBatteryLife >= 0){
    return round(constrainedBatteryLife / 20);
    }
  else{
    return 0;
    }
  }
