//Basic Arduino Functions
#include <Arduino.h>

//Sensor Comm
#include <Wire.h>
#include <VL53L0X.h>
//Servo Comm
#include <Servo.h>
//Custom Header Files
#include "comm.h"


//NOTE: Debug control is in the comm section

void move(int direction, int speed);
int16_t* getMagnetData();
float getAngle(int magx, int magy);

const int MPU9250_ADDR = 0x68;
const int MAG_ADDRESS = 0x0C;
const int RSIDE_LIDAR_ADDRESS = 0x16;
const int FRONT_LIDAR_ADDRESS = 0x19;

const int GYRO_FULL_SCALE_250_DPS = 0x00;
const int GYRO_FULL_SCALE_500_DPS = 0x08;
const int GYRO_FULL_SCALE_1000_DPS = 0x10;
const int GYRO_FULL_SCALE_2000_DPS = 0x18;

const int ACC_FULL_SCALE_2_G = 0x00;
const int ACC_FULL_SCALE_4_G = 0x08;
const int ACC_FULL_SCALE_8_G = 0x10;
const int ACC_FULL_SCALE_16_G = 0x18;

const int LED_PIN = 16;//D0
const int SDA_PIN = 14;//D7
const int SCL_PIN = 12;//D6
const int LEFT_MOTOR = 4;//D2
const int RIGHT_MOTOR = 5;//D1
const int EN_FRONT_LIDAR = 0;//D3
const int EN_RSIDE_LIDAR = 2;//D4

const int forward = 0;
const int backward = 1; 
const int left = 2; 
const int right = 3;
const int stay = 4;

const float angTargets[8] = {
	0,
0.11065722117389563*180/3.1415926535897,
-0.2592143715647132*180/3.1415926535897,
0.5409816690990277*180/3.1415926535897,
0.31781808077336693*180/3.1415926535897,
0.957663398466961*180/3.1415926535897,
1.5790063027733998*180/3.1415926535897,
3.114800941680322*180/3.1415926535897};

const int motionsSize = 8;

//Servo
int LSERVO_NULL = 90;
int RSERVO_NULL = 90;
Servo Left;
Servo Right;

//Laser Range Finder
VL53L0X FRONT;
VL53L0X RSIDE;

int brother_give_me_the_loops;
float currangle = 0;
float zeropos = 0;
void setup()
{
	//Enable Comm
	Serial.begin(115200);
	Wire.begin(SDA_PIN, SCL_PIN);
	delay(1000);
	Serial.flush();
	log("Captian, internal communications diagnostic starting!\n");
	
		//wifi communications	 
    for(uint8_t t = 4; t > 0; t--) 
	{
        Serial.printf("\t[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        delay(1000);
    }
    
	
	//motor attachment
	Left.attach(LEFT_MOTOR);
	Right.attach(RIGHT_MOTOR);
	log("Engines online.\n");
	
	//lasers
	pinMode(D3, OUTPUT);
	pinMode(D4, OUTPUT);
	//"enables" for LIDAR
	digitalWrite(EN_RSIDE_LIDAR, HIGH);
	delay(150);
	RSIDE.init(true);
	delay(100);
	RSIDE.setAddress(RSIDE_LIDAR_ADDRESS);
	digitalWrite(EN_FRONT_LIDAR, HIGH);
	delay(150);
	FRONT.init(true);
	delay(100);
	FRONT.setAddress(FRONT_LIDAR_ADDRESS);
	log("Laser array addressing complete.\n");
    delay(1000);
  	//I2C Verification 
	log ("Beginning Internal Communication verification\n");
	byte count = 0;
	for (byte i = 1; i < 120; i++)
	{
		Wire.beginTransmission (i);
		if (Wire.endTransmission () == 0)
		{
			log("Found address: ");
			log(i, DEC);
			log(" (0x");
			log(i, HEX);
			log(")\n");
			count++;
			delay(1); 
		} // end of good response
	} // end of for loop
	log ("Internal diagnostic has verified ");
	log(count, DEC);
	log (" device(s).\n");
	
	if(count == 4)
		log("Captain, all internal systems show green.\n");
	else
		log("Captain, we have an internal comm error.\n");
	delay(1000);
	
		
	
	//Enable bypass mode for the I2C on the MPU
	I2CwriteByte(MPU9250_ADDR,0x37,0x02);
  
	// Request first magnetometer single measurement
	I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
	log("Sensor Array online. \n");
	
	
	delay(1000);

	
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	log("Running Lights Active.\n");
	
	
	move(stay, 0);
	log("Engines at 0%, captain.\n");
	
	
	log("Attack formation.\n");
	brother_give_me_the_loops=0;
	
	delay(1000);
	
	digitalWrite(LED_PIN, HIGH);
	
	//collect data, and add to the csvLine
		//Magnetometer Data
		// Request first magnetometer single measurement
		
		
		I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
		
		// Read register Status 1 and wait for the DRDY: Data Ready
		  
		uint8_t ST1;
		do
		{
			I2Cread(MAG_ADDRESS,0x02,1,&ST1);
		}
		while (!(ST1&0x01));

		// Read magnetometer data  
		uint8_t Mag[7];  
		I2Cread(MAG_ADDRESS,0x03,7,Mag);

		// Create 16 bits values from 8 bits data
		  
		// Magnetometer
		int16_t mx=(Mag[1]<<8 | Mag[0]);
		int16_t my=(Mag[3]<<8 | Mag[2]);
		int16_t mz=(Mag[5]<<8 | Mag[4]);
		
		
	currangle = getAngle(mx, my);
	zeropos = currangle;
	currangle = currangle - zeropos;
	
	if(currangle < 0)
	{
		currangle = 360-abs(currangle);
	}
	if(currangle > 361)
	{
		currangle = currangle -360;
	}
	
	Serial.println(angTargets[0]);
	Serial.println(angTargets[1]);
	Serial.println(angTargets[2]);
	Serial.println(angTargets[3]);
	Serial.println(angTargets[4]);
	Serial.println(angTargets[5]);
	Serial.println(angTargets[6]);
}


void loop()
{
	brother_give_me_the_loops++;
	if(brother_give_me_the_loops > motionsSize)
	{
		while(true)
		{
				delay(10000);//spinzaku forever, since we have arrived in the target destination (we think)
		}
	}
	float magTarget = angTargets[brother_give_me_the_loops];////////////////////////////////load this with target for this iteration
	float absTarget = abs(magTarget);
	Serial.print("magnetic target: ");
	Serial.print(magTarget);
	Serial.print(" ");
	Serial.println(absTarget);
	float initAngle = currangle; //initial angle before rotation
	unsigned long initTime = millis();//initial time
	unsigned long targetTime = initTime + 500;//target time half a second
	//Serial.println(initAngle);
	//Determine Direction of Motion
	int direction;
	if(magTarget < initAngle)
	{
		//log("left turn");
		//while the current angle is greater than the fov target
		while(currangle < initAngle + absTarget)
		{
			//move
			move(right, 0);
			//read and update
			//collect data, and add to the csvLine
			//Magnetometer Data
			// Request first magnetometer single measurement
			
			
			I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
			
			// Read register Status 1 and wait for the DRDY: Data Ready
			  
			uint8_t ST1;
			do
			{
				I2Cread(MAG_ADDRESS,0x02,1,&ST1);
			}
			while (!(ST1&0x01));

			// Read magnetometer data  
			uint8_t Mag[7];  
			I2Cread(MAG_ADDRESS,0x03,7,Mag);

			// Create 16 bits values from 8 bits data
			  
			// Magnetometer
			int16_t mx=(Mag[1]<<8 | Mag[0]);
			int16_t my=(Mag[3]<<8 | Mag[2]);
			int16_t mz=(Mag[5]<<8 | Mag[4]);
		
		
			currangle = getAngle(mx, my) - zeropos;//calculate the current angle
			Serial.println(currangle);
		}
	}
	else
	{
		//log("right turn");
		//while the current angle is less than the fov target
		while(currangle > initAngle - absTarget)
		{
			//move
			move(left, 0);
			//read and update
			//collect data, and add to the csvLine
			//Magnetometer Data
			// Request first magnetometer single measurement
			
			
			I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
			
			// Read register Status 1 and wait for the DRDY: Data Ready
			  
			uint8_t ST1;
			do
			{
				I2Cread(MAG_ADDRESS,0x02,1,&ST1);
			}
			while (!(ST1&0x01));

			// Read magnetometer data  
			uint8_t Mag[7];  
			I2Cread(MAG_ADDRESS,0x03,7,Mag);

			// Create 16 bits values from 8 bits data
			  
			// Magnetometer
			int16_t mx=(Mag[1]<<8 | Mag[0]);
			int16_t my=(Mag[3]<<8 | Mag[2]);
			int16_t mz=(Mag[5]<<8 | Mag[4]);
			
			currangle = getAngle(mx, my) - zeropos;//calculate the current angle
			//Serial.println(currangle);

		}
	}
	
	float deltaTime = millis();
	move(stay, 0);
	while(deltaTime < targetTime)//hold for remainder of half second
	{
		deltaTime = millis();
	}

	//move forward for half a second
	move(forward, 0);
	delay(500);
	move(stay, 0);
	//delay(1000);
		
		
}


void move(int direction, int speed)
{
	switch(direction)
	{
		case forward: //forward
		{
			Left.write(0+speed);
			Right.write(180-speed);
			break;
		}
		case backward: //back
		{
			Left.write(180-speed);
			Right.write(0+speed);
			break;
		}
		case left: //left
		{
			Left.write(0+speed);
			Right.write(0+speed);
			break;
		}
		case right: //right
		{
			Left.write(180-speed);
			Right.write(180-speed);
			break;
		}
		default: //stay
		{
			Left.write(LSERVO_NULL);
			Right.write(RSERVO_NULL);
		}
	}
}

float getAngle(int magx, int magy)
{
	
	//zero degrees is -190x, -45 y
	//north is -145 x, -130 Y
	float heading = atan2(magx + 180, magy + 75) * 180/3.14159265;
	//Serial.println(heading);
	if(heading < 0)
	{
		heading = 360-abs(heading);
	}
	if(heading > 361)
	{
		heading = heading -360;
	}
	
	//zero position is 46*
	//heading -= 45; 
	
	//Serial.print("Heading: ");
	//Serial.println(heading);
	
	
    return heading;
}




