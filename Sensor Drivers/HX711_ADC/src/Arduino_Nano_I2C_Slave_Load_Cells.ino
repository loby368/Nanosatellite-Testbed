/*
Title: Arduino Nano SLave to STM32 Master Sending Load Cell Data

Description: 
The Arduino Nano acts as a breakout board for 2x HX711 Load Cell Amps.
It collects the load cell data in 2x signed 32 bit intergers which are 4 bytes long.
Then it sends these 2 intergers (totoal of 8 bytes) to the STM32 when it recieves a master request over I2C.
The HX711 ADC has a resolution of 24 bits, but the HX711 Library ...
  takes care of converting raw voltage readings into signed 32 but interger values which we can use directly.

Resources:
// Load Cell Amp Webpage: https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide/all
// HX711 Library: https://github.com/bogde/HX711/tree/master

Author: Louí Byrne

Date: 03 August 2023

Project: Flexible CubeSat Testbed - Masters Thesis Internship with UCD Space Dynamics and Control Research Group
*/

#include "HX711.h"

// I2C define
#include <Wire.h>
#define slaveAddress 0x03 // Address of Arduino Nano
#define PACKET_SIZE 8 // Size of Packet to send over I2C

// Pins for data and clock
#define dataPin1  2
#define clockPin1 3
#define dataPin2  4
#define clockPin2 5

// Create Instance for Load Cells
HX711 loadCell1;
HX711 loadCell2;

// Define Storage Variables
long loadCellRead1; // signed int32_t (4 bytes)
long loadCellRead2; // signed int32_t (4 bytes)
byte TxBuffer[8];   // 8 byte array to send over I2C

void setup() {
  // Serial Debug
  Serial.begin(115200);
  
  // I2C
  Wire.begin(slaveAddress); // Begin in I2C slave mode on my address
  Wire.setWireTimeout(100); // Ensures the I2C line is free for other sensors in case of an error on the NANO after 100ms
  Wire.onRequest(requestEvent); // When HAL_I2C_Master_Receive() is called on STM32, it triggers this function by writing a W/R bit after the 7 bit address

  // Initialise Load Cells
  loadCell1.begin(dataPin1, clockPin1);
  loadCell2.begin(dataPin2, clockPin2);

  // Serial Debug Data
  Serial.print("\n-------- ARDUINO NANO Initialisation Complete -----\n\n");
}

void loop() {

  // Continuously sample load cells at 80Hz, and only transmit once we recieve a request from STM32
  loadCellRead1 = loadCell1.read(); // signed int32_t
  loadCellRead2 = loadCell2.read(); // signed int32_t

  //Serial Debug Data
  Serial.print("Raw Voltage - Load Cell 1: ");
  Serial.print(loadCellRead1);
  Serial.print(" Load Cell 2: ");
  Serial.println(loadCellRead2);

}

// Function to execute when the I2C master calls for a value
void requestEvent(){

  //Serial.println("\n**NANO: Request Event**");

  // Create a volatile buffer to store data we will send over I2C
  byte TxBuffer[PACKET_SIZE]; // array of 8 bytes (PACKET_SIZE = 8)

  // Fill TxBuffer with two four byte arrays, each representing the 32 bit raw voltage reading from the Load Cells
  // Load Cell 1 data stored in bytes 0:3 of the buffer
  TxBuffer[0] = loadCellRead1 & 0xFF;         //least LSB
  TxBuffer[1] = (loadCellRead1 >> 8) & 0xFF;  //
  TxBuffer[2] = (loadCellRead1 >> 16) & 0xFF; //
  TxBuffer[3] = (loadCellRead1 >> 24) & 0xFF; //msb
  // Load cell 2 data stored in bytes 4:7 of the buffer
  TxBuffer[4] = loadCellRead2 & 0xFF;         //lsb
  TxBuffer[5] = (loadCellRead2 >> 8) & 0xFF;  //
  TxBuffer[6] = (loadCellRead2 >> 16) & 0xFF; //
  TxBuffer[7] = (loadCellRead2 >> 24) & 0xFF; // most MSB
  
  // Transmit the 8 byte buffer to the master
  Wire.write(TxBuffer, PACKET_SIZE);

}

