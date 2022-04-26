// FORELØPIG 
//gammel kode, finner ikke den nye ヽ(ಠ_ಠ)ノ

#include <Wire.h>
#include <Zumo32U4.h>
#include <speed_controls.h>

int batteryUsage = 0; 
float batteryLife = 8800; // Reele batteristørrelsen i mAh, bruker mAs for enklere testing.
float batteryHealth = 100;

float constrainedBatteryLife = constrain(batteryLife, 0, 2000);
float constrainedBatteryHealth = constrain(batteryHealth, 0, 100);

void loop()
  // Batteri degradering ved høy ytelse
if (timeOverSeventyPercent >= 0){
  float constrainedBatteryHealth -= (0.00005 * timeOverSeventyPercent);
}

  // Utlading i forhold til fart 
if (speedInCmPerSec >= 0){  
  int batteryUsage(0.1 * speedInCmPerSec){
  float constrainedBatteryLife -= (0.1 * speedInCmPerSec);
  float constrainedBatteryHealth -= (0.000001 * speedInCmPerSec); // batteri degradering

   // Lading i forhold til ryggefart 
else{
  int batteryUsage(0.09 * -speedInCmPerSec){
  float constrainedBatterylife -= (0.09 * speedInCmPerSec);
  float constrainedBatteryHealth += (0.0000009 * speedInCmPerSec); // batteri degradering
  

  if (constrainedBatteryLife >= 0){
    return round(constrainedBatteryLife / 20);
    }
  else{
    return 0;
    }
  }
