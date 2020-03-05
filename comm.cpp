#include <Arduino.h>
#include <Wire.h>


const int DEBUG = 1;//define as 1 for debug mode, 0 for operation
const int CSV = 1;//set to 1 for CSV formatted output
//if debug is 0, serial will not print. 

void log(String s)
{
	if(DEBUG)
		Serial.print(s);
}
void log(int data, int dataType)
{
	if(DEBUG)
		Serial.print(data, dataType);
}
void log(String s, uint8_t data)
{
	if(DEBUG)
	{
		Serial.print(s);
		Serial.println(data);
	}
}
void log(String s, char * data)
{
	if(DEBUG)
	{
		Serial.print(s);
		Serial.println(data);
	}
	
}
void logCSV(String s)
{
	if(CSV)
	{
		if(DEBUG)
			log("Writting to CSV file\n");
		
	}
}
void csvAdd(String* target, String s)
{
	String temp = ", ";
	temp.concat(s);
	target -> concat(temp);
}

void csvFirst(String* target, String s)
{
	String temp = "\n";
	temp.concat(s);
	target -> concat(temp);
}

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}