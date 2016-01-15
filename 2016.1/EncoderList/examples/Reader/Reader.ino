#include "encoderlist.h"
#include "simpleencoder.h"

SimpleEncoder encoder(10);
SimpleEncoder encoder2(10);
SimpleEncoder encoder3(10);
SimpleEncoder encoder4(10);
EncoderList encoder_list(&Serial);

void setup() {

	Serial.begin(9600);
	encoder_list.addEncoder(&encoder);
	encoder_list.addEncoder(&encoder2);
	encoder_list.addEncoder(&encoder3);
	encoder_list.addEncoder(&encoder4);
	encoder_list.start();
	
}

void loop() {
	encoder_list.read();
	Serial.print(encoder.getPulses());
	Serial.print("\t");
	Serial.print(encoder2.getPulses());
	Serial.print("\t");
	Serial.print(encoder3.getPulses());
	Serial.print("\t");
	Serial.println(encoder4.getPulses());
	delay(1000);
}
