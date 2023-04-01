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
    serX.write(90);
    serY.write(90);
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
    if (parseDataM(serialData, values)) {
        for (int i = 0; i < 4; ++i) {
            Serial.print(values[i]);
            Serial.print(',');
        }
        writeValuesToEEPROM(values);
    }
    if (!parseDataD(serialData)) {
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

uint8_t parseDataM(String data, int *values) {
    uint8_t flag = 0;
    if (data[0] == 'M') {
        flag = 1;
        data.remove(0, 1);
        for (int i = 0; i < 4; ++i) {
            values[i] = data.substring(0, data.indexOf(',')).toInt();
            data.remove(0, data.indexOf(',') + 1);
        }
    }
    return flag;
}

uint8_t parseDataD(String data) {
    uint8_t flag = 0;
    int my_values[4];
    if (data[0] == 'D') {
        flag = 1;
        EEPROM_read(EEPROM_read(EEPROM_read(EEPROM_read(0, my_values[0]), my_values[1]), my_values[2]), my_values[3]);
        Serial.print('<');
        for (int i = 0; i < 4; ++i) {
            Serial.print(my_values[i]);
            if (i < 3) Serial.print(',');
        }
        Serial.println('>');
    }
    return flag;
}

void writeValuesToEEPROM(int *values) {
    EEPROM_write(EEPROM_write(EEPROM_write(EEPROM_write(0, values[0]), values[1]), values[2]), values[3]);
}

template<class T>
int EEPROM_write(int ee, const T &value) {
    const byte *p = (const byte *) (const void *) &value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
    return ee;
}

template<class T>
int EEPROM_read(int ee, T &value) {
    byte *p = (byte * )(void * ) & value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return ee;
}
