#include<Servo.h>
#include <EEPROM.h>

Servo serX;
Servo serY;

String serialData;

void setup() {
  pinMode(11, OUTPUT);
  serX.attach(9);
  serY.attach(10);
  digitalWrite(11, 1);   // turn the LED on (HIGH is the voltage level)
  serX.write(60);
  serY.write(60);
  delay(500);                       // wait for a second
  digitalWrite(11, 0);    // turn the LED off by making the voltage LOW
  Serial.begin(9600);
  Serial.setTimeout(10);
}

void loop() {
  //lol
}

void serialEvent() {
  serialData = Serial.readString();
  int values[2];
  int* values_a = parseDataM(serialData);
  for(int i=0; i<2; i++)
    values[i] = values_a[i];
  if (values[0] != -1){
    for(int i = 0; i < 2; ++i){
      if (EEPROM.read(i) != values[i]){
        EEPROM.write(i, values[i]);
      }
    }
  }
  if(parseDataD(serialData)){
    digitalWrite(11, parseDataL(serialData));
    serX.write(parseDataX(serialData));
    serY.write(parseDataY(serialData));
  }
}

int parseDataL(String data) {
  data.remove(data.indexOf("X"));
  data.remove(data.indexOf("L"), 1);
  return data.toInt();
}

int parseDataX(String data) {
  data.remove(data.indexOf("Y"));
  data.remove(data.indexOf("X"), 1);
  data.remove(data.indexOf("L"), 2);
  return data.toInt();
}

int parseDataY(String data) {
  data.remove(0, data.indexOf("Y") + 1);
  return data.toInt();
}

int* parseDataM(String data) {
  static int values[2];
  values[0] = -1;
  if (data[0] == 'M') {
    int index = 0;
    String currentValue = "";
    for (int i = 2; i < data.length(); i++) {
      if (data[i] == ',') {
        values[index++] = currentValue.toInt();
        currentValue = "";
      } else {
        currentValue += data[i];
      }
    }
    values[index] = currentValue.toInt();
  }
  return values;
}

int parseDataD(String data) {
  int ans = 1;
  if (data[0] == 'D') {
    ans = 0;
    Serial.print('<');
    for(int i = 0; i < 2; ++i){
      Serial.print(EEPROM.read(i));
      if(i<1)
        Serial.print(',');
    }
    Serial.println('>');
  }
  return ans;
}
