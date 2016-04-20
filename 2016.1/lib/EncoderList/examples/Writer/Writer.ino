/*
  Read the encoders and send it to the other arduino
*/
#include "encoderlist.h"
#include "simpleencoder.h"
#include "SoftwareSerial.h"

#define SIRENE_PIN  9
#define R_ENC_1 6
#define R_ENC_2 7
#define L_ENC_1 10
#define L_ENC_2 11

struct Encoder {
  public:
  byte pin_1, pin_2;
  int out_1, out_2;
  int last_out_1, last_out_2;
  int delta_pulses;
  int pulses;

  Encoder(byte pin_1, byte pin_2){
    this->pin_1 = pin_1;
    this->pin_2 = pin_2;
    this->out_1 = 0;
    this->out_2 = 0;
    this->last_out_1 = 0;
    this->last_out_2 = 0;
    delta_pulses = 0;
    pulses = 0;
  }

  int read() {
    delta_pulses = 0;

    //Keep the last state of the encoder
    last_out_1 = out_1;
    last_out_2 = out_2;
    //Get the new state
    out_1 = digitalRead(pin_1);
    out_2 = digitalRead(pin_2);

    //Sums 1 for each change, in order to keep
    //the precision
    if(last_out_1 != out_1) {
      delta_pulses ++;
    }

    if(last_out_2 != out_2) {
      delta_pulses ++;
    }

    pulses += delta_pulses;

    return delta_pulses;
  }

  int reset() {
    delta_pulses = 0;
    pulses = 0;
  }
};

Encoder front_left_encoder(L_ENC_1,L_ENC_2);
//Encoder back_left_encoder(4,5);
Encoder front_right_encoder(R_ENC_1,R_ENC_2);
//Encoder back_right_encoder(2,3);

SoftwareSerial my_serial(10,11);
Stream *current_stream = &Serial;

void read() {
  front_left_encoder.read();
  front_right_encoder.read();
//  back_right_encoder.read();
//  back_left_encoder.read();
}

void reset() {
  front_left_encoder.reset();
  front_right_encoder.reset();
//  back_right_encoder.reset();
//  back_left_encoder.reset();
}

void send() {
  String message = "";
  message += front_left_encoder.pulses;
  message += TOKEN;
//  message += back_left_encoder.pulses;
//  message += TOKEN;
  message += front_right_encoder.pulses;
//  message += TOKEN;
//  message += back_right_encoder.pulses;

  current_stream->println(message);
  // Serial.println(message);
}

void setup() {
  Serial.begin(57600);
  my_serial.begin(57600);
}

void loop() {

  //Read the encoders
  read();

  //Read the serial port
  if (current_stream->available()) {
    char incoming_message = (char) current_stream->read();
    if (incoming_message == ACKNOWLEDGEMENT) {
      send();
      reset();
    } else if(incoming_message == RESET) {
      reset();
    } else if(incoming_message == LIGHT_ON) {
      digitalWrite(SIRENE_PIN,HIGH);
    } else if(incoming_message == LIGHT_OFF) {
      digitalWrite(SIRENE_PIN,LOW);
    }
  }
//  Serial.print(back_right_encoder.pulses);
//  Serial.print("\t");
//  Serial.print(back_right_encoder.delta_pulses);
//  Serial.print("\t");
//  Serial.print(back_right_encoder.out_1);
//  Serial.print("\t");
//  Serial.println(back_right_encoder.out_2);
}
