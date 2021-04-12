#include <Wire.h>

#define ADDRESS           0x04

byte dataFromPi[32];
byte dataToPi[4];
int piState = 0;
int state = 0;
float angleRead = 360;
float distanceRead = 0;

void setup() {
  Wire.begin(ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

}

void loop() {
  // put your main code here, to run repeatedly:

}

//Callback for receiving data from Pi
void receiveData(int byteCount) {
  int i = 0;
  while (Wire.available()) {
    dataFromPi[i] = Wire.read();
    i++;
  }
  piState = dataFromPi[0];
  angleRead = dataFromPi[1];
  distanceRead = dataFromPi[2];
  state = dataFromPi[3];
}

//Callback for sending data to Pi
void sendData() {
  Wire.write(dataToPi, 32);
}
