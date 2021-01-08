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

unsigned long wait;

//Gyro Variables
float gyro_x, gyro_y, gyro_z;
float cal_x, cal_y, cal_z;
int cal_int;
int arm_indicator_led;

//Transmitter and Receiver Variables
byte channel_1_state, channel_2_state, channel_3_state, channel_4_state;
//int channel_1_pulse, channel_2_pulse, channel_3_pulse, channel_4_pulse;
unsigned long timer_1, timer_2, timer_3, timer_4;
unsigned long current_time;
unsigned long zero_timer;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long esc_loop_timer;

int receiver_pulse[4];
int channel_center_values_array[4] = {1500,1500,1500,1500};
int channel_high_values_array[4] = {2000,1988,1988,1984};
int channel_low_values_array[4] = {1020,1016,1012,988};

//Initializing Transmitter Receiver Function
ISR(PCINT2_vect)
{
  current_time = micros();

  //Channel 1
  if((channel_1_state == 0) && (PINK & B00000001))
  {
    channel_1_state = 1;
    timer_1 = current_time;
  }
  else if((channel_1_state == 1) && !(PINK & B00000001))
  {
    channel_1_state = 0;
    receiver_pulse[0] = current_time - timer_1;
  }

  //Channel 2
  if((channel_2_state == 0) && (PINK & B00000010))
  {
    channel_2_state = 1;
    timer_2 = current_time;
  }
  else if((channel_2_state == 1) && !(PINK & B00000010))
  {
    channel_2_state = 0;
    receiver_pulse[1] = current_time - timer_2;
  }

  //Channel 3
  if((channel_3_state == 0) && (PINK & B00000100))
  {
    channel_3_state = 1;
    timer_3 = current_time;
  }
  else if((channel_3_state == 1) && !(PINK & B00000100))
  {
    channel_3_state = 0;
    receiver_pulse[2] = current_time - timer_3;
  }

  //Channel 4
  if((channel_4_state == 0) && (PINK & B00001000))
  {
    channel_4_state = 1;
    timer_4 = current_time;
  }
  else if((channel_4_state == 1) && !(PINK & B00001000))
  {
    channel_4_state = 0;
    receiver_pulse[3] = current_time - timer_4;
  }
}

int convert_receiver_channel_pulse(byte channel_number)
{
  int center, high, low, actual;
  int difference;

  center = channel_center_values_array[channel_number-1];
  high = channel_high_values_array[channel_number-1];
  low = channel_low_values_array[channel_number-1];
  actual = receiver_pulse[channel_number-1];

  if(actual < center)
  {                                                                            //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 - difference;                                                  //If the channel is not reversed
  }
  else if(actual > center)
  {                                                                            //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    return 1500 + difference;                                                  //If the channel is not reversed
  }
  else return 1500;
}


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
      PORTE |= B00010000; //same thing as saying digitalWrite(2, HIGH)
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
      gyro_y = (gyro_y - cal_y) * -1; //Multiplied by negative one to invert the y-axis reading. It must correspond to the standard plane image
      gyro_z = (gyro_z - cal_z) * -1; //Multiplied by negative one to invert the z-axis reading. It must correspond to the standard plane image
    }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  DDRE |= B00011000; // same thing as saying pinMode(2, OUTPUT);  //PORTE |= B00010000; same thing as saying digitalWrite(2, HIGH);  //PORTE &= B11101111; same thing as saying digitalWrite(2, LOW); esc pin 5
  DDRG |= B00100000; // esc pin 4
  DDRH |= B00011000; // esc pin 6, esc pin 7

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

  //Setup Interrupt Pins for Transmitter and Receiver
  PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK0 scan
  PCMSK2 |= (1 << PCINT16);  // set PCINT16 (digital input 8) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT17);  // set PCINT17 (digital input 9)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT18);  // set PCINT18 (digital input 10)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT19);  // set PCINT18 (digital input 11)to trigger an interrupt on state change

  wait = micros();
  while(wait + 3000000 > micros());

  while(1)
  {
    if((convert_receiver_channel_pulse(3) > 1100) || (convert_receiver_channel_pulse(4) > 1100))
    {
      arm_indicator_led = arm_indicator_led + 1;
      PORTG |= B00100000;
      PORTE |= B00001000;
      PORTH |= B00011000;
      delayMicroseconds(1000);
      PORTG &= B11011111;
      PORTE &= B11110111;
      PORTH &= B11100111;
      delay(3);
      if(arm_indicator_led == 125)
      {
        digitalWrite(2, !digitalRead(2));
        arm_indicator_led = 0;
      }
    }

    if((convert_receiver_channel_pulse(3) < 1100) && (convert_receiver_channel_pulse(4) < 1100))
    {
      break;
    }
  }

  //while(convert_receiver_channel_pulse(3) > 1100);
  // {
  //   Serial.print("\t");
  //   Serial.print(convert_receiver_channel_pulse(3));
  //   Serial.print("\t");
  //   Serial.print(convert_receiver_channel_pulse(4));
  //   Serial.println();
  //   arm_indicator_led = arm_indicator_led + 1;
  //   PORTG |= B00100000;
  //   PORTE |= B00001000;
  //   PORTH |= B00011000;
  //   delayMicroseconds(1000);
  //   PORTG &= B11011111;
  //   PORTE &= B11110111;
  //   PORTH &= B11100111;
  //   delay(3);
  //   if(arm_indicator_led == 125)
  //   {
  //     digitalWrite(2, !digitalRead(2));
  //     arm_indicator_led = 0;
  //   }
  // }
  arm_indicator_led = 0;
  digitalWrite(2, LOW);
  zero_timer = micros();
}

void loop()
{
  // read_gyro();
  // Serial.print("\t X: ");
  // Serial.print(gyro_x/MPU_GYRO_READINGSCALE_500DEG);
  // Serial.print("\t Y: ");
  // Serial.print(gyro_y/MPU_GYRO_READINGSCALE_500DEG);
  // Serial.print("\t  Z: ");
  // Serial.print(gyro_z/MPU_GYRO_READINGSCALE_500DEG);
  // Serial.println();
  // delay(100);
  // Serial.print("\t");
  // Serial.print(convert_receiver_channel_pulse(1));
  // Serial.print("\t");
  // Serial.print(convert_receiver_channel_pulse(2));
  // Serial.print("\t");
  // Serial.print(convert_receiver_channel_pulse(3));
  // Serial.print("\t");
  // Serial.print(convert_receiver_channel_pulse(4));
  // Serial.println();

  while(zero_timer + 4000 > micros())
  {
    zero_timer = micros();
    PORTG |= B00100000;
    PORTE |= B00001000;
    PORTH |= B00011000;
    timer_channel_1 = convert_receiver_channel_pulse(3) + zero_timer;
    timer_channel_2 = convert_receiver_channel_pulse(3) + zero_timer;
    timer_channel_3 = convert_receiver_channel_pulse(3) + zero_timer;
    timer_channel_4 = convert_receiver_channel_pulse(3) + zero_timer;

    while((PORTG >= 223) && (PORTE >= 247) && (PORTH >= 231))
    {
      esc_loop_timer = micros();
      if(timer_channel_1 <= esc_loop_timer)
      {
        PORTG &= B11011111;
      }
      if(timer_channel_2 <= esc_loop_timer)
      {
        PORTE &= B11110111;
      }
      if(timer_channel_3 <= esc_loop_timer)
      {
        PORTH &= B11110111;
      }
      if(timer_channel_4 <= esc_loop_timer)
      {
        PORTH &= B11101111;
      }
    }
  }
}

