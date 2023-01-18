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
  int values[4];
  if (parseDataM(serialData, values)){
    writeValuesToEEPROM(values);
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

uint8_t parseDataM(String data, int * values){
  uint8_t flag = 0;
  if (data[0] == 'M') {
    int index = 0;
    String currentValue = "";
    bool negative = false;
    flag = 1;
    for (int i = 2; i < data.length(); i++) {
      if(data[i] == '-'){
        negative = true;
        continue;
      }
      if (data[i] == ',') {
        values[index++] = (negative) ? -currentValue.toInt(): currentValue.toInt();
        currentValue = "";
        negative = false;
      } else {
        currentValue += data[i];
      }
    }
    values[index] = (negative) ? -currentValue.toInt(): currentValue.toInt();
  }
  return flag;
}

int parseDataD(String data) {
  int ans = 1;
  if (data[0] == 'D') {
    ans = 0;
    Serial.print('<');
    for (int i = 0; i < 4; ++i) {
      byte sign = EEPROM.read(i * 2);
      byte value = EEPROM.read(i * 2 + 1);
      Serial.print((sign == 1) ? -value : value);
      if(i<3)
        Serial.print(',');
    }
    Serial.println('>');
  }
  return ans;
}

void writeValuesToEEPROM(int* values) {
  for (int i = 0; i < 4; i++) {
    int value = values[i];
    byte sign = (value < 0) ? 1 : 0;
    value = abs(value);
    EEPROM.write(i * 2, sign);
    EEPROM.write(i * 2 + 1, (byte)value);
  }
}
