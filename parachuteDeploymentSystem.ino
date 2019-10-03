#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include "SparkFunBME280.h"
#include <TimeLib.h>

/*
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
*/
//----------------------- PIN LAYOUT --------------------------
//PWR -> 3.3v
//GND -> GND
//SCL -> A5
//SDA -> A4
//-------------------------------------------------------------

MPU6050 accelgyro;
BME280 mySensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;


//##TURN ON PARTS BY COMMENTING
//---------------------------------------

//----TESTS-----------------------
//#define blink_test
//#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_PRESSURE_TEMP
//#define chuteDeploymentTEST

//------Main Programs--------------------
#define mainSystem

//---------------------------------------

#define LED_PIN 13
bool blinkState = false;

//These are the pin out definitions for
//the deployment of the parachutes 
int drogue = 9;
int mainChute = 10;

//used in mainSystem to track rockets max height at a given moment
float altMax = 0;

time_t initTime;
time_t deltaTime;

int16_t accReading;

float curAlt;

int verifyStart = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

   //for setting up the altimeter
    mySensor.setI2CAddress(0x76);
    //Begin communication over I2C
    if (mySensor.beginI2C() == false) 
    {
      Serial.println("The sensor did not respond. Please check wiring.");
      while(1); //Freeze
    }

    pinMode(drogue, OUTPUT);
    pinMode(mainChute, OUTPUT);

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    initTime = now();
}

void loop() {
  // put your main code here, to run repeatedly:

  while (runCon ==0)
      {

      //(CONSIDER: Remove from loop, and create offset values)
      //set accelerometer axis variables
      accelgyro.getAcceleration(&ax, &ay, &az);
    
      curAlt = mySensor.readFloatAltitudeMeters();
      //double check orientation sign depending on implementation
      accReading = ((az/16384.0)*-9.81); 

      //verify the begining of rocket ascent to start tracking time
      if (accReading >= 20)
      {
        verifyStart = 1; 
      }
      if (curAlt >= 1000)
      {
        verifyStart = 1;
      }

      if (verifyStart == 1)
      {
        deltaTime = now() - initTime;
      }
      
      //Main deployment system 
      #ifdef mainSystem
          if (altMax < curAlt)
          {
            altMax = curAlt;
          }
          //deploy if 5m below and not before 20 seconds from take off 
          //this section code is an area that needs to be improved.
          //there need to be more checks and sensor fusion/scrolling value 
          //averages should be implemented to filter noise
          else if (altMax > (curAlt + 5) && (deltaTime > 20)) //make 'or' statement
          {
            initTime = now();
            digitalWrite(drogue, HIGH);
            delay(1000);
            digitalWrite(drogue, LOW);
            
            int runCon = 0; 
            while (runCon == 0)
            {
              
              curAlt = mySensor.readFloatAltitudeMeters();
              deltaTime = now() - initTime;
              
              if (altMax > (curAlt + 100) || (deltaTime > 10)) //deploy main chute when current altitude drops 100m below max or after 10s
              {
                digitalWrite(mainChute, HIGH);
                delay(1000);
                digitalWrite(mainChute, LOW);
                runCon = 1;
              }// end of if statment
              
            }//end while loop
            
          }//end else if 
      #endif

   	 //for testing accelerometer 
     #ifdef OUTPUT_READABLE_ACCELGYRO
         // display acceleration in z 
         Serial.print((az/16384.0)*-9.81);
	     Serial.print("\n");
	 #endif

	 #ifdef OUTPUT_PRESSURE_TEMP
	     //display altimeter Sensor Values
	     Serial.print(" Pressure: ");
	     Serial.print(mySensor.readFloatPressure(), 0);
	     Serial.print(" Alt: ");
	     Serial.print(mySensor.readFloatAltitudeMeters(), 1);
	     Serial.print(" Temp: ");
	     Serial.print(mySensor.readTempC(), 2);
	     Serial.println();
	     delay(50);
	 #endif
    
    //for testing parachute charge deployment 
     #ifdef chuteDeploymentTEST
	     Serial.print("Drogue will deploy in 10 seconds")
	     delay(10000)
         digitalWrite(drogue, HIGH);
         Serial.print("---------------Drogue Deployed ---------------")
         delay(1000);
         digitalWrite(drogue, LOW);

         Serial.print("Main will deploy in 10 seconds")
	     delay(10000)
         digitalWrite(mainChute, HIGH);
         Serial.print("---------------Main Deployed ---------------")
         delay(1000);
         digitalWrite(mainChute, LOW);
	 #endif

    
    //absolute basic arduino testing. Is your board working? 
     #ifdef blink_test
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
     #endif

      }
}
