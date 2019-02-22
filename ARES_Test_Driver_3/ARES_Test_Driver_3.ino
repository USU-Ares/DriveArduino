/* This program drives the test-bench robot
 * for ARES team navigation testing 
 * 
 * Author: Derek S Workman
 * Email: derek.workman@aggiemail.usu.edu
 */

#include <Servo.h>

//Definitions
//#define MOTOR_L_CONT 10   //Left motor control at pin 6 (For PWM)
//#define MOTOR_L_STAT 9   //Left motor status at pin 9 (LOW = forward)
//#define MOTOR_R_CONT 11  //Right motor control at pin 11 (For PWM)
//#define MOTOR_R_STAT 10  //Right motor status at pin 10 (LOW = forward)

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

/****************PWM Frequency Subroutine**********************/

 /**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

/***************************************************************/


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
  
  //setPwmFrequency(3,16);// pins 3, 9, 10, and 11 have base frequency of 31250 Hz. Choosing 30 Hz as our desired frequency, 16=490Hz/30Hz
  //setPwmFrequency(9,16);
  //setPwmFrequency(10,16);
  //setPwmFrequency(11,16);
  
  //setPwmFrequency(3,32);//pins 5 and 6 have a base frequency of 62500 Hz. Choosing 30 Hz as our desired frequency, 32=980Hz/30Hz 
  //setPwmFrequency(3,32);
  
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

/*
  //Perform motor opperations
  if((data[DIR]&DIR_LEFT) == DIR_LEFT) {  //Update the direction for each motor
    digitalWrite(MOTOR_L_STAT, HIGH);  //if in reverse
    analogWrite(MOTOR_L_CONT, (0xff - data[ML]));  //PWM polarity must also be reversed
  }
  else {
    digitalWrite(MOTOR_L_STAT, LOW);
    analogWrite(MOTOR_L_CONT, data[ML]); 
  }
  if((data[DIR]&DIR_RIGHT) == DIR_RIGHT) {
    digitalWrite(MOTOR_R_STAT, HIGH);
    analogWrite(MOTOR_R_CONT, (0xff - data[MR]));
  }
  else {
    digitalWrite(MOTOR_R_STAT, LOW);
    analogWrite(MOTOR_R_CONT, data[MR]);
  }
*/  
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
