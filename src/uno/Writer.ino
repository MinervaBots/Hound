/*
  Read the encoders and send it to the other arduino
*/
#include "encoderlist.h"
#include "simpleencoder.h"
#include "SoftwareSerial.h"
#include "LinkedList.h"

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
    int last_state_index;
    int state_array[4][2] = {
      {0,0},{0,1},{1,0},{1,1}};

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

    int findIndex(int *state){
      int index = -1;
      for (int i = 0; i<4; i++){
        if(state_array[i][0] == state[0] &&
          state_array[i][1] == state[1]){
          index = i;
        }
      }
      return index;
    }

    int read() {
      //Keep the last state of the encoder
      // last_out_1 = out_1;
      // last_out_2 = out_2;
      int current_state[] = {digitalRead(pin_1), digitalRead(pin_2)};

      //Get the new state
      int current_state_index = findIndex(current_state);

      // Serial.print(current_state[0]);Serial.print(current_state[1]);
      // Serial.print('\t');
      // Serial.print("O índice eh:");
      Serial.print(current_state_index);

      if(last_state_index == 3 && current_state_index == 0){
        delta_pulses++;
      } else if(last_state_index == 0 && current_state_index == 3){
        delta_pulses--;
      } else if(last_state_index < current_state_index){
        delta_pulses++;
      } else if(last_state_index > current_state_index){
        delta_pulses--;
      }

      last_state_index = current_state_index;

//      out_1 = digitalRead(pin_1);
//      out_2 = digitalRead(pin_2);
//
//      //Sums 1 for each change, in order to keep the precision
//      if(last_out_1 != out_1) {delta_pulses ++;}
//      if(last_out_2 != out_2) {delta_pulses ++;}

      pulses += delta_pulses;

      return delta_pulses;
    }

    void reset() {
      delta_pulses = 0;
      pulses = 0;
    }

    void start(){
      reset();
      int current_state[] = {digitalRead(pin_1), digitalRead(pin_2)};
      this->last_state_index = findIndex(current_state);
    }
};


SoftwareSerial my_serial(10,11);
Stream *current_stream = &Serial;

namespace Slave {
  Encoder front_left_encoder(L_ENC_1,L_ENC_2);
  Encoder front_right_encoder(R_ENC_1,R_ENC_2);

  void start(){
    front_left_encoder.start();
    front_right_encoder.start();
  }

  void send() {
    String message = "";
    message += front_left_encoder.delta_pulses;
    message += TOKEN;
    message += front_right_encoder.delta_pulses;
    current_stream->println(message);
    front_left_encoder.delta_pulses = 0;
    front_right_encoder.delta_pulses = 0;
  }

  void readEncoders(){
    front_left_encoder.read();
    front_right_encoder.read();
  }

  void resetEncoders(){
    front_left_encoder.reset();
    front_right_encoder.reset();
  }
};


void setup() {
  Slave::start();
  Serial.begin(57600);
  my_serial.begin(57600);
}


void loop() {
  Slave::readEncoders();//Read the encoders
   Serial.print('\t');

  String message = "";
  message += Slave::front_left_encoder.delta_pulses;
  message += TOKEN;
  message += Slave::front_right_encoder.delta_pulses;
  Serial.println(message);

  // Sample Period = 3ms --> Sample Rate = 333Hz
  // 333Hz > 2x max encoder signal freq. (142Hz)
  delay(2);

 //Read the serial port
 if (current_stream->available()) {
   char incoming_message = (char) current_stream->read();
   if (incoming_message == ACKNOWLEDGEMENT) {
     Slave::send();
     // Slave::resetEncoders();

   } else if(incoming_message == RESET) {
     Slave::resetEncoders();
   } else if(incoming_message == LIGHT_ON) {
     digitalWrite(SIRENE_PIN,HIGH);
   } else if(incoming_message == LIGHT_OFF) {
     digitalWrite(SIRENE_PIN,LOW);
   }
 }
}
