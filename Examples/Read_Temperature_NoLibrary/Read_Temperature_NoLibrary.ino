/* BEFORE DOWNLOAD SKETCH TO BOARD PLEASE FILL-OUT CONFIGURATION DEFINITIONS BELOW!!!
*  MAX31865 RTD to digital converter sample code, v1.01
*  contact: mendresz at gmail dot com
*/

// CONFIGURATION
#define Rref 400       // Rfef = 400 if PT100 used, Rref = 4000 if PT1000 used
#define WIRE 4         // PT100/1000 has 2 or 3 or 4 wire connection
//#define DEBUG        // if defined, all register values printed out after every reading
// END OF CONFIGURATION 

#include <SPI.h>

void configureMAX31865(){
  SPI.transfer(0x80);
  if (WIRE==2 || WIRE==4) SPI.transfer(0xC2);
  if (WIRE==3) SPI.transfer(0xD2);
  delay(50);
} 

double CallendarVanDusen(double R){
  double a = 3.9083E-03;
  double b = -5.7750E-07;
  signed long R0=Rref/4;
//  double t;
  return (-R0*a+sqrt(R0*R0*a*a-4*R0*b*(R0-R)))/(2*R0*b);  
}

void printRegs(unsigned char *reg){
   Serial.println("Register values (all values in HEX format):");
  
  Serial.print("Configuration: ");
  Serial.println(reg[0], HEX);
  
  Serial.print("RTD MSBs: ");
  Serial.println(reg[1], HEX);
  
  Serial.print("RTD LSBs: ");
  Serial.println(reg[2], HEX);
  
  Serial.print("High Fault Treshold MSB: ");
  Serial.println(reg[3], HEX);
  
  Serial.print("High Fault Treshold LSB: ");
  Serial.println(reg[4], HEX);
  
  Serial.print("Low Fault Treshold MSB: ");
  Serial.println(reg[5]), HEX;
  
  Serial.print("Low Fault Treshold LSB: ");
  Serial.println(reg[6]), HEX;
  
  Serial.print("Fault Status: ");
  Serial.println(reg[7], HEX);
  
  Serial.println();
}

void setup(){
  Serial.begin(9600);
  SPI.begin();
  SPI.setClockDivider(200);
  SPI.setDataMode(SPI_MODE3);
  
  delay(100);
  configureMAX31865();
}

void loop(){
  unsigned char reg[8];          // array for all the 8 registers
  unsigned int i;
  unsigned int RTDdata;          // 16 bit value of RTD MSB & RTD LSB, reg[1] & reg[2]
  unsigned int ADCcode;          // 15 bit value of ADC, RTDdata >> 1, 0th bit of RTDdata is RTD connection fault detection bit
  double R;                      // actual resistance of PT100(0) sensor
  
  delay(10);
  SPI.transfer(0);                                       // start reading from address=0
  for (i=0; i<8; i++) reg[i]=SPI.transfer(0);            // read all the 8 registers
  delay(10);


  RTDdata = reg[1] << 8 | reg[2];
  Serial.print("RTD data: 0x");
  Serial.println(RTDdata,HEX);
  
  if (RTDdata & 1) {
    if (Rref==400) Serial.println("PT100 sensor connenction fault!");
    if (Rref==4000) Serial.println("PT1000 sensor connenction fault!");
    Serial.println();
    configureMAX31865();
  }  
  else{ 
    Serial.print("ADC code (decimal): ");
    ADCcode=RTDdata>>1;
    Serial.println(ADCcode,DEC);

    Serial.print("Resistance: ");
    R=(double)ADCcode*Rref/32768;
    Serial.print(R);
    Serial.println(" Ohms");
    
    Serial.print("Temperature: ");
    Serial.print(CallendarVanDusen(R));
    Serial.println(" deg. C");
    Serial.println();
  }

  #ifdef DEBUG
    printRegs(reg);
  #endif 

  delay(3000);
}
