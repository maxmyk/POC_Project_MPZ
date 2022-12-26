#include<Servo.h>

Servo serX;
Servo serY;

String serialData;

void setup() {
  pinMode(13, OUTPUT);
  serX.attach(9);
  serY.attach(10);
  digitalWrite(13, 1);   // turn the LED on (HIGH is the voltage level)
  serX.write(60);
  serY.write(60);
  delay(1000);                       // wait for a second
  digitalWrite(13, 0);    // turn the LED off by making the voltage LOW
  Serial.begin(9600);
  Serial.setTimeout(10);
}

void loop() {
  //lol
}

void serialEvent() {
  serialData = Serial.readString();
  digitalWrite(13, parseDataL(serialData));
  serX.write(parseDataX(serialData));
  serY.write(parseDataY(serialData));
}

int parseDataL(String data) {
  data.remove(data.indexOf("X"));
  data.remove(data.indexOf("L"), 1);
  Serial.print("L ");
  Serial.println(data);
  return data.toInt();
}

int parseDataX(String data) {
  data.remove(data.indexOf("Y"));
  data.remove(data.indexOf("X"), 1);
  data.remove(data.indexOf("L"), 2);
  
  Serial.print("X ");
  Serial.println(data);
  return data.toInt();
}

int parseDataY(String data) {
  data.remove(0, data.indexOf("Y") + 1);
  Serial.print("Y ");
  Serial.println(data);
  return data.toInt();
}
