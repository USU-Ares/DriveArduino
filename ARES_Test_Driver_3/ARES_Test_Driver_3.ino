/* This program drives the test-bench robot
 * for ARES team navigation testing 
 * 
 * Author: Derek S Workman
 * Email: derek.workman@aggiemail.usu.edu
 */

#include <Servo.h>

//Definitions

#define DIR 0 //index for motor direction byte in serial data
#define ML  1 //index for left motor PWM value in serial data
#define MR  2 //index for right motor PWM value in serial data

#define DIR_LEFT 0x01  //bit selection for left motor direction
#define DIR_RIGHT 0x02 //bit selection for right motor direction

#define TIMEOUT 1000 //data timeout at x milliseconds

//Global Variables
Servo sMOTOR_L_CONT[3];
Servo sMOTOR_R_CONT[3];

const uint8_t MOTOR_L_CONT[3] = {3, 5, 6};  //PWM pin assignments for left three motors
const uint8_t MOTOR_R_CONT[3] = {9, 10, 11}; //PWM pin assignments for right three motors

char pid_stat = 0;  //contains status for serial Packet ID
const size_t N = 3; //The data index size [direction_bits, PWM_L, PWM_R]
byte data[N] = {0}; //the data for the motors (PWM values)
byte crc = 0; //Cyclic redundancy check, (will be a sum of all bits in the data portion of the packet)
unsigned long lastTime = 0; //keeps track of data timeout


bool ChecksumMatch(byte* buf, byte crc, size_t n) {

  uint8_t bitCount = 0;
  
  for(uint8_t i = 0; i < n; i++) {
    bitCount += buf[i]&0x01;  //count the first bit if set
    for(uint8_t j = 1; j < 8; j++) {
      bitCount += (buf[i] >> j)&0x01; //count the rest of the set bits
    }
  }
  //Serial.print("bitCount: "); //print statements for debug
  //Serial.println(bitCount);
  //Serial.print("crc: ");
  //Serial.println(crc);
  if(bitCount == crc) return true;
  else return false;
}

void ReadSerial() {

  byte buf[N] = {0};
  
  if(Serial.available()) {
    switch(pid_stat) {
      case 0:
        if(Serial.read() == 'G') {
          pid_stat = 'G';
        }
        break;
      case 'G':
        if(Serial.read() == 'O') {
          pid_stat = 'O';
        }
        break;
      case 'O':
        if(Serial.available() == (N+1)) {
          pid_stat = 0;
          lastTime = millis();  //get the current time for data timeout check
          for(size_t i = 0; i < N; i++) {
            buf[i] = Serial.read();     //read in all N data bytes
          }
          crc = Serial.read();  //get the checksum at the end
          if(ChecksumMatch(buf,crc,N)) {
            for(size_t i = 0; i < N; i++) {
              data[i] = buf[i]; //If the checksum matched, then transfer
                                //  data from the buffer to the registers
            }
          }
        }
        break;
      default: pid_stat = 0;  //In case of default, reset pid_stat
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Serial port has baudrate of 9600
  
  for(byte i = 0; i < 3; i++) { 
    pinMode(MOTOR_L_CONT[i], OUTPUT); //Set motor pins as outputs
    //pinMode(MOTOR_L_STAT, OUTPUT);
    pinMode(MOTOR_R_CONT[i], OUTPUT);
    //pinMode(MOTOR_R_STAT, OUTPUT);
    sMOTOR_L_CONT[i].attach(MOTOR_L_CONT[i]);
    sMOTOR_R_CONT[i].attach(MOTOR_R_CONT[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  ReadSerial();

  if((millis() - lastTime) > TIMEOUT) {  //if data has not been recieved for <TIMEOUT> milliseconds
    for(byte i = 2; i < N; i++) {        //  then stop the motors
      data[i] = 0;
    }
  }

  //PErform motor opperations (version 2)
  if((data[DIR]&DIR_LEFT) == DIR_LEFT) {  //Update the direction for each motor
    for(byte i = 0; i < 3; i++) {
      //digitalWrite(MOTOR_L_STAT, HIGH);  //if in reverse
      //analogWrite(MOTOR_L_CONT[i], (0xbc - (data[ML]/4)));  //PWM polarity must also be reversed
      sMOTOR_L_CONT[i].write(90);
    }
  }
  else {
    for(byte i = 0; i < 3; i++) {
      //digitalWrite(MOTOR_L_STAT, LOW);
      analogWrite(MOTOR_L_CONT[i], (0xbc + (data[ML]/4)));
      sMOTOR_L_CONT[i].write(180);
    } 
  }
  if((data[DIR]&DIR_RIGHT) == DIR_RIGHT) {
    for(byte i = 0; i < 3; i++) {
      //digitalWrite(MOTOR_R_STAT, HIGH);
      analogWrite(MOTOR_R_CONT[i], (0xbc - (data[MR]/4)));
    }
  }
  else {
    for(byte i = 0; i < 3; i++) {
      //digitalWrite(MOTOR_R_STAT, LOW);
      analogWrite(MOTOR_R_CONT[i], (0xbc + (data[MR]/4)));
    }
  }
}
