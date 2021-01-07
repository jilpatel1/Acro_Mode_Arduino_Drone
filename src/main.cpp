#include <Arduino.h>
#include <Wire.h>

#define MPU_POWER_REG 0x6B
#define MPU_POWER_CYCLE 0b00000000
#define MPU_SAMP_FREQ 250

#define MPU_GYRO_CONFIG_REG 0x1B
#define MPU_GYRO_READ_REG 0x43
#define MPU_GYRO_READ_REG_SIZE 6
#define MPU_GYRO_CONFIG_500DEG 0b00001000
#define MPU_GYRO_READINGSCALE_500DEG 65.5

#define MPU_CALIBRATE_READING_NUM 2000
#define MPU_ADDRESS 0b1101000

#define LOW_PASS_FILTER_REG 0x1A


//VARIABLES
float gyro_x, gyro_y, gyro_z;
float cal_x, cal_y, cal_z;
int cal_int;

void mpu_setup()
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_POWER_REG); //wake up so we can use gyro yaw pitch roll
  Wire.write(MPU_POWER_CYCLE); //prevents from going to sleep
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_GYRO_CONFIG_REG);
  Wire.write(MPU_GYRO_CONFIG_500DEG);
  Wire.endTransmission();

  //Check register to see if everything is configured correctly. If not, show warning LED and enter infinite loop to prevent from starting.
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_GYRO_CONFIG_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDRESS, 1);

  while(Wire.available() < 1);
  if(Wire.read() != MPU_GYRO_CONFIG_500DEG)
  {
    while(1)
    {
      digitalWrite(2,HIGH);
      delay(10000);
    }
  }

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(LOW_PASS_FILTER_REG);
  Wire.write(0x03);
  Wire.endTransmission();
}


void read_gyro()
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_GYRO_READ_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDRESS, MPU_GYRO_READ_REG_SIZE); 

    while(Wire.available() < MPU_GYRO_READ_REG_SIZE);
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();

    if(cal_int == MPU_CALIBRATE_READING_NUM)
    {
      gyro_x = gyro_x - cal_x;
      gyro_y = (gyro_y - cal_y) * -1;
      gyro_z = (gyro_z - cal_z) * -1; //Multiplied by negative one to invert the z-axis reading. It must correspond to the standard plane image
    }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(2, OUTPUT);
  mpu_setup();
  delay(3000); //delay helps settle down the mpu and work properly

  for(cal_int = 0; cal_int < MPU_CALIBRATE_READING_NUM; cal_int++)
  {
    read_gyro();
    cal_x = cal_x + gyro_x;
    cal_y = cal_y + gyro_y;
    cal_z = cal_z + gyro_z;
  }

  cal_x = cal_x/MPU_CALIBRATE_READING_NUM;
  cal_y = cal_y/MPU_CALIBRATE_READING_NUM;
  cal_z = cal_z/MPU_CALIBRATE_READING_NUM;

  Serial.print("X:  ");
  Serial.print(cal_x);

  Serial.print("Y:  ");
  Serial.print(cal_y);

  Serial.print("Z:  ");
  Serial.print(cal_z);

  delay(5000);

}

void loop()
{
  read_gyro();
  Serial.print("\t X: ");
  Serial.print(gyro_x/MPU_GYRO_READINGSCALE_500DEG);
  Serial.print("\t Y: ");
  Serial.print(gyro_y/MPU_GYRO_READINGSCALE_500DEG);
  Serial.print("\t  Z: ");
  Serial.print(gyro_z/MPU_GYRO_READINGSCALE_500DEG);
  Serial.println();
  delay(100);
}