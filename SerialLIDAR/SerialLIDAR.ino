#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>

LIDARLite_v3HP myLidarLite;

#define FAST_I2C

void gameover(){
  int melody[] = {
    262, 247, 233, 220
  };
  int noteDurations[] = {
    2, 2, 2, 1
  };
  for (int thisNote = 0; thisNote < 4; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(8);
  }
}

void set_siren(int runtime){
  runtime /= 200;
  for(int i =0;i<runtime;++i){
    for(int hz = 2000; hz < 2500; hz+=25){
      tone(8, hz, 50);
      delay(5);
    }
    for(int hz = 2500; hz > 2000; hz-=25){
      tone(8, hz, 50);
      delay(5);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  #ifdef FAST_I2C
    #if ARDUINO >= 157
      Wire.setClock(400000UL);
    #else
      TWBR = ((F_CPU / 400000UL) - 16) / 2;
    #endif
  #endif
  myLidarLite.configure(0);
}


void loop()
{
  uint8_t distance;
  uint8_t newDistance = 0;
  uint8_t oldDistance;
  myLidarLite.waitForBusy();
  myLidarLite.takeRange();
  myLidarLite.waitForBusy();
  oldDistance = myLidarLite.readDistance();
  Serial.println(oldDistance);
  Serial.println("Start:");
  uint8_t acceptableError = 10;
  while (1)
  {
    newDistance = distanceContinuous(&distance);
    if (newDistance && distance<1000)
    {
      if (abs(distance - oldDistance) > acceptableError) {
        Serial.println("SIREN");
        set_siren(1000);
      }
    }
    delay(10);
  }
}
uint8_t distanceContinuous(uint8_t * distance)
{
  uint8_t newDistance = 0;
  if (myLidarLite.getBusyFlag() == 0)
  {
    myLidarLite.takeRange();
    *distance = myLidarLite.readDistance();
    newDistance = 1;
  }
  return newDistance;
}
