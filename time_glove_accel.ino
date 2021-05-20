int light = 200;      // LIGHTS ON delay, microseconds
int dark;             // LIGHTS OFF delay
int min_dark = 1;     // min dark delay
int max_dark = 50;    // max dark delay
#define light_pin 2   // MOSFET pin
#define  potent_pin 6 // potentiometer pin
int angle;
boolean flag;
long lastchange;

#include <Wire.h>
#include "Kalman.h"
Kalman kalmanX;
Kalman kalmanZ;
Kalman kalmanY;
uint8_t IMUAddress = 0x68;
/* IMU Data */
float accZ, accY, accX;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
double accXangle; // Angle calculate using the accelerometer
double accZangle;
double accYangle;
double temp;
double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180; // Angle calculate using the gyro
double gyroZangle = 180;
double compAngleX = 180; // Calculate the angle using a Kalman filter
double compAngleZ = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;
double kalAngleZ;
float elapsedTime, currentTime, previousTime;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
uint32_t timer;
uint32_t myTimer;
int c = 0;

void setup() {
  Serial.begin(9600);
  pinMode(light_pin, OUTPUT);

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(IMUAddress);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  kalmanZ.setAngle(180);
  
  calculate_IMU_error();
  delay(20);
  
  timer = micros();
  
  flag = 0;

//  Serial.println("Startujemy!");
}

void loop() 
{
   measure();     
    
    if(flag == 0 && gyroX < -50 && gyroY < -220  && gyroZ > 50 ){        
         myTimer = millis();
//        Serial.print("OH!");
//        Serial.println(myTimer);
    }
    
    if(flag == 0 && millis() - myTimer < 1000 && gyroX > 150 && gyroY > 249  ) 
    {
      flag = 1;
      myTimer = millis(); 
//      Serial.println("odpalanko!");     
    }

    if(flag == 1 && millis() - myTimer > 700 && abs(gyroX) > 125 && gyroY > 220 && abs(gyroZ) > 55) 
    {
      flag = 0;
      myTimer = 0;
//      Serial.println("KONIEC odpalanka!");
      
    }


  
if (flag == 1) 
  {
    digitalWrite(light_pin, 1);    // lights up
    ADelay(200);      // wait
    digitalWrite(light_pin, 0);    // lights down
    uint32_t lol;
    ADelay(lol = (map(analogRead(potent_pin), 0, 1023, 27000, 800) + (kalAngleY * (-12)) ) );
//    Serial.println( lol );
  }

}

void ADelay(uint32_t czas)
{
  delay(czas / 1000);
  delayMicroseconds(czas % 1000); 
}

// oh my god, it's some tricky sh*t
void measure() 
{

  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  accX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  accY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  accZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  accXangle = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accYangle = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

  accZangle = (atan2(accX, accY) + PI) * RAD_TO_DEG;
  //accXangle = (atan2(accY, accX) + PI) * RAD_TO_DEG;

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  gyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  /*
  uint8_t* data = i2cRead(0x3B, 14);
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);
  tempRaw = ((data[6] << 8) | data[7]);
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);
  /* Calculate the angls based on the different sensors and algorithm */
  
  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = (double)gyroY / 131.0;
  double gyroZrate = -((double)gyroZ / 131.0);
  gyroXangle += kalmanX.getRate() * ((double)(micros() - timer) / 1000000); // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * ((double)(micros() - timer) / 1000000); // Calculate gyro angle using the unbiased rate
  gyroZangle += kalmanZ.getRate() * ((double)(micros() - timer) / 1000000);
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (double)(micros() - timer) / 1000000);
  timer = micros();
}

void i2cWrite(uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++)
    data [i] = Wire.read();
  return data;
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(IMUAddress);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(IMUAddress, 6, true);
    accX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    accY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    accZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(IMUAddress);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(IMUAddress, 6, true);
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (gyroX / 131.0);
    GyroErrorY = GyroErrorY + (gyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (gyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
