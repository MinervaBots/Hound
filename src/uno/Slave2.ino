//Set mode to 7(packet serial address 0x80) and option 4(38400)
//Includes required to use Roboclaw library
#include "BMSerial.h"
#include "RoboClaw.h"
//Roboclaw Address
#define address 0x80
//Definte terminal for display. Use hardware serial pins 0 and 1
BMSerial terminal(0,1);
//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw(10,11,10000);


void setup() {
  //Open terminal and roboclaw at 38400bps
  terminal.begin(57600);
  roboclaw.begin(38400);
}



void loop() {
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  //Read all the data from Roboclaw before displaying on terminal window
  //This prevents the hardware serial interrupt from interfering with
  //reading data using software serial.
  int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  terminal.print("Encoder1:");
  if(valid1){
    terminal.print(enc1,HEX);
    terminal.print(" ");
    terminal.print(status1,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Encoder2:");
  if(valid2){
    terminal.print(enc2,HEX);
    terminal.print(" ");
    terminal.print(status2,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }

  terminal.print("Speed1:");

  if(valid3){
    terminal.print(speed1,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Speed2:");
  if(valid4){
    terminal.print(speed2,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.println();
  delay(100);
}
