#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

int pot[4];

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  pot[0] = analogRead(A0);
  pot[1] = analogRead(A1);
  pot[2] = analogRead(A2);
  pot[3] = analogRead(A3);
  radio.write(pot, sizeof(pot));
  //radio.write(green, sizeof(green));
 // radio.write(blue, sizeof(blue));
}
