#include "encoderlist.h"
#include "simpleencoder.h"

SimpleEncoder encoder(10);
SimpleEncoder encoder2(10);
SimpleEncoder encoder3(10);
SimpleEncoder encoder4(10);
EncoderList encoder_list(&Serial3);

long int time;
long int delta_t = 0;
float velocity = 0;
float convertion_const = (1000 * 2 * 3.1415)/1024.0;
float reference = 2;

long int max_time = 5000;
long int time_counter = 0;
bool stop_loop = false;

void printData() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(reference);
  Serial.print("\t");
  Serial.print(velocity);
  Serial.println("\t;");
  
}
void setup() {

  Serial.begin(57600);
  Serial3.begin(57600);
  encoder_list.addEncoder(&encoder);
  encoder_list.addEncoder(&encoder2);
  encoder_list.addEncoder(&encoder3);
  encoder_list.addEncoder(&encoder4);
  encoder_list.start();
  time = millis();
  // Serial.print("data=[");
  time_counter = millis();
}

void loop() {
  
  // if(stop_loop) {
  //   return ;
  // }
  
  // if(millis() - time_counter > max_time) {
  //   Serial.println("];");
  //   stop_loop = true;
  //   return ;
  // }
  
  if(Serial.available()) {
    if(Serial.read() == RESET) {
      encoder_list.reset();
      Serial3.println(RESET);
    }
  }
  if(encoder_list.read()) {
   Serial.print(encoder.getPulses());
   Serial.print("\t");
   Serial.print(encoder2.getPulses());
   Serial.print("\t");
   Serial.print(encoder3.getPulses());
   Serial.print("\t");
   Serial.println(encoder4.getPulses());
   Serial.println("");
    
    delta_t = millis() - time;
    if(delta_t) {
      velocity = (encoder.getDeltaPulses() * convertion_const) / delta_t;
//      Serial.println("");
//      Serial.print("Velocity = ");
//      Serial.println(velocity);
//      Serial.println("");
    }
    time = millis();
    // printData();

  }
  delay(50);
  
//  delay(1000);
}
