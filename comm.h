#ifndef __comm_h
#define __comm_h

void log(String s);
void log(int data, int dataType);
void log(String s, uint8_t data);
void log(String s, char * data);
void csvAdd(String* target, String s);
void logCSV(String s);
void csvFirst(String* target, String s);

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

#endif