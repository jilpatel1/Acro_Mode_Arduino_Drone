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
#define ROLL_RATE_FACTOR 3.0 //lower this value for faster roll rates. use desmos to check. Do not go above (2000 pulse, 500 degrees/sec)

unsigned long wait;

//Gyro variables
float gyro_x, gyro_y, gyro_z;
float cal_x, cal_y, cal_z;
int cal_int;

//Startup variables
int status;

//Transmitter and receiver variables
byte channel_1_state, channel_2_state, channel_3_state, channel_4_state;
unsigned long timer_1, timer_2, timer_3, timer_4;
unsigned long current_time;
unsigned long loop_timer;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long esc_loop_timer;

int raw_receiver_pulse[4];
int channel_center_values_array[4] = {1500,1500,1500,1500};
int channel_high_values_array[4] = {2000,1988,1988,1984};
int channel_low_values_array[4] = {1020,1016,1012,988};

int throttle, battery_voltage;
int esc_1, esc_2, esc_3, esc_4;

//PID Variables
float pid_p_gain_roll = 1.30;   //1.3            //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;  //0.05            //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 15.00;    //15            //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.00;   //4.0             //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;  //0.02             //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.00;   //0.0             //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

//Interrupt service routine to measure the transmitter raw pulse length
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
    raw_receiver_pulse[0] = current_time - timer_1;
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
    raw_receiver_pulse[1] = current_time - timer_2;
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
    raw_receiver_pulse[2] = current_time - timer_3;
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
    raw_receiver_pulse[3] = current_time - timer_4;
  }
}

//Scaling the raw pulse length to a standardized 1000ms to 2000ms pulse
int convert_receiver_channel_pulse(byte channel_number)
{
  int center, high, low, actual;
  int difference;

  center = channel_center_values_array[channel_number-1];
  high = channel_high_values_array[channel_number-1];
  low = channel_low_values_array[channel_number-1];
  actual = raw_receiver_pulse[channel_number-1];

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

//Starting up the mpu and configuring its settings
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

//Read the raw gyro values and subtract the offset if gyro was calibrated
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

void calculate_pid()
{
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll = pid_i_mem_roll + (pid_error_temp * pid_i_gain_roll);
  if(pid_i_mem_roll > pid_max_roll)
  {
    pid_i_mem_roll = pid_max_roll;
  }
  else if(pid_i_mem_roll < pid_max_roll * -1)
  {
    pid_i_mem_roll = pid_max_roll * -1;
  }

  pid_output_roll = (pid_error_temp * pid_p_gain_roll) + pid_i_mem_roll + ((pid_error_temp - pid_last_roll_d_error)* pid_d_gain_roll);
  if(pid_output_roll > pid_max_roll)
  {
    pid_output_roll = pid_max_roll;
  }
  else if(pid_output_roll < pid_max_roll * -1)
  {
    pid_output_roll = pid_max_roll * -1;
  }

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch = pid_i_mem_pitch + (pid_error_temp * pid_i_gain_pitch);
  if(pid_i_mem_pitch > pid_max_pitch)
  {
    pid_i_mem_pitch = pid_max_pitch;
  }
  else if(pid_i_mem_pitch < pid_max_pitch * -1)
  {
    pid_i_mem_pitch = pid_max_pitch * -1;
  }

  pid_output_pitch = (pid_error_temp * pid_p_gain_pitch) + pid_i_mem_pitch + ((pid_error_temp - pid_last_pitch_d_error)* pid_d_gain_pitch);
  if(pid_output_pitch > pid_max_pitch)
  {
    pid_output_pitch = pid_max_pitch;
  }
  else if(pid_output_pitch < pid_max_pitch * -1)
  {
    pid_output_pitch = pid_max_pitch * -1;
  }
  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw = pid_i_mem_yaw + (pid_error_temp * pid_i_gain_yaw);
  if(pid_i_mem_yaw > pid_max_yaw)
  {
    pid_i_mem_yaw = pid_max_yaw;
  }
  else if(pid_i_mem_yaw < pid_max_yaw * -1)
  {
    pid_i_mem_yaw = pid_max_yaw * -1;
  }

  pid_output_yaw = (pid_error_temp * pid_p_gain_yaw) + pid_i_mem_yaw + ((pid_error_temp - pid_last_yaw_d_error)* pid_d_gain_yaw);
  if(pid_output_yaw > pid_max_yaw)
  {
    pid_output_yaw = pid_max_yaw;
  }
  else if(pid_output_yaw < pid_max_yaw * -1)
  {
    pid_output_yaw = pid_max_yaw * -1;
  }
  pid_last_yaw_d_error = pid_error_temp;
}

//Setup before flying
void setup()
{
  //Serial.begin(115200); 
  Wire.begin();
  
  DDRE |= B00010000; // same thing as saying pinMode(2, OUTPUT);  //PORTE |= B00010000; same thing as saying digitalWrite(2, HIGH);  //PORTE &= B11101111; same thing as saying digitalWrite(2, LOW); 
  DDRH |= B01111000; // esc pin 6,7,8,9

  mpu_setup();
  delay(3000); //delay helps settle down the mpu and work properly

  //Read 2000 raw gyro values to calibrate the sensor
  for(cal_int = 0; cal_int < MPU_CALIBRATE_READING_NUM; cal_int++)
  {
    read_gyro();
    cal_x = cal_x + gyro_x;
    cal_y = cal_y + gyro_y;
    cal_z = cal_z + gyro_z;
  }

  //Divide the accumulated raw gyro values by the number of readings to get the average offset
  cal_x = cal_x/MPU_CALIBRATE_READING_NUM;
  cal_y = cal_y/MPU_CALIBRATE_READING_NUM;
  cal_z = cal_z/MPU_CALIBRATE_READING_NUM;

  //Setup Interrupt Pins for Transmitter and Receiver
  PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK0 scan
  PCMSK2 |= (1 << PCINT16);  // set PCINT16 (analog pin 8) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT17);  // set PCINT17 (analog pin 9)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT18);  // set PCINT18 (analog pin 10)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT19);  // set PCINT18 (analog pin 11)to trigger an interrupt on state change

  //Wait a little for everything to settle down to prevent getting 0 transmitter pulses on startup.
  wait = micros();
  while(wait + 3000000 > micros());

  //This is the startup sequence for the drone. Keep waiting and blink the LED until the user has the throttle in the lowest position and the stick is moved to the left. 
  while(1)
  {
    if((convert_receiver_channel_pulse(3) > 1100) || (convert_receiver_channel_pulse(4) > 1100))
    {
      status = status + 1;
      PORTH |= B01111000;
      delayMicroseconds(1000);
      PORTH &= B10000111;
      delay(3);
      if(status == 125)
      {
        digitalWrite(2, !digitalRead(2));
        status = 0;
      }
    }

    if(convert_receiver_channel_pulse(3) < 1100)
    {
      break;
    }
  }

  status = 0;
  digitalWrite(2, LOW);
  battery_voltage = (analogRead(0) + 65) * 1.2317;
}

void loop()
{
  //Read the gyro and convert it to degrees/second. Use the complementary filter for non-jittery data.
  read_gyro();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_x/65.5) * 0.2);
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_y/65.5) * 0.2);
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_z/65.5) * 0.2);

  //In order to start the drone, throttle down and yaw left.
  if((convert_receiver_channel_pulse(3) < 1050) && (convert_receiver_channel_pulse(4) < 1050))
  {
    status = 1;
  }
  //Then bring your yaw to the middle. Erase all the PID I and D errors for a smooth start.
  if((status == 1) && (convert_receiver_channel_pulse(3) < 1050) && (convert_receiver_channel_pulse(4) > 1450))
  {
    status = 2;
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_roll_d_error = 0;
  }

  //To stop the drone, throttle down and yaw right.
  if((status == 2) && (convert_receiver_channel_pulse(3) < 1050) && (convert_receiver_channel_pulse(4) > 1950))
  {
    status = 0;
  }

  pid_roll_setpoint = 0;
  pid_pitch_setpoint = 0;
  pid_yaw_setpoint = 0;

 
  if(convert_receiver_channel_pulse(1) > 1508)
  {
    pid_roll_setpoint = (convert_receiver_channel_pulse(1) - 1508)/ROLL_RATE_FACTOR;
  }
  else if(convert_receiver_channel_pulse(1) < 1492)
  {
    pid_roll_setpoint = (convert_receiver_channel_pulse(1) - 1492)/ROLL_RATE_FACTOR;
  }

  if(convert_receiver_channel_pulse(2) > 1508)
  {
    pid_pitch_setpoint = (convert_receiver_channel_pulse(2) - 1508)/ROLL_RATE_FACTOR;
  }
  else if(convert_receiver_channel_pulse(2) < 1492)
  {
    pid_pitch_setpoint = (convert_receiver_channel_pulse(2) - 1492)/ROLL_RATE_FACTOR;
  }

  if(convert_receiver_channel_pulse(4) > 1508)
  {
    pid_yaw_setpoint = (convert_receiver_channel_pulse(4) - 1508)/ROLL_RATE_FACTOR;
  }
  else if(convert_receiver_channel_pulse(4) < 1492)
  {
    pid_yaw_setpoint = (convert_receiver_channel_pulse(4) - 1492)/ROLL_RATE_FACTOR;
  }

  calculate_pid();


  battery_voltage = ((battery_voltage * 0.92) + ((analogRead(0) + 65) * 1.2317) * 0.08);
  
  if((battery_voltage < 1050) && (battery_voltage > 600))
  {
    digitalWrite(2, HIGH);
  }

  throttle = convert_receiver_channel_pulse(3);

  if(status == 2)
  {
    if(throttle > 1800)
    {
      throttle = 1800;
    }

    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    
    if (battery_voltage < 1240 && battery_voltage > 800)                  //Check if the battery is connected
    {                   
      esc_1 = esc_1 + (esc_1 * ((1240 - battery_voltage)/(float)3500));              //Compensate the esc-1 pulse for voltage drop.
      esc_2 = esc_2 + (esc_2 * ((1240 - battery_voltage)/(float)3500));              //Compensate the esc-2 pulse for voltage drop.
      esc_3 = esc_3 + (esc_3 * ((1240 - battery_voltage)/(float)3500));              //Compensate the esc-3 pulse for voltage drop.
      esc_4 = esc_4 + (esc_4 * ((1240 - battery_voltage)/(float)3500));              //Compensate the esc-4 pulse for voltage drop.
    } 

    if(esc_1 < 1200)
    {
      esc_1 = 1200;
    }

    if(esc_2 < 1200)
    {
      esc_2 = 1200;
    }

    if(esc_3 < 1200)
    {
      esc_3 = 1200;
    }
  
    if(esc_4 < 1200)
    {
      esc_4 = 1200;
    }

    if(esc_1 > 2000)
    {
      esc_1 = 2000;
    }

    if(esc_2 > 2000)
    {
      esc_2 = 2000;
    }

    if(esc_3 > 2000)
    {
      esc_3 = 2000;
    }
  
    if(esc_4 > 2000)
    {
      esc_4 = 2000;
    }
  }

  else
  {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }
  
  while(micros() - loop_timer < 4000);
  loop_timer = micros();
  PORTH |= B01111000;
  timer_channel_1 = esc_1 + loop_timer;
  timer_channel_2 = esc_2 + loop_timer;
  timer_channel_3 = esc_3 + loop_timer;
  timer_channel_4 = esc_4 + loop_timer;

  while((PORTH & B01111000) != 0)
  {
    esc_loop_timer = micros();
    if(timer_channel_1 <= esc_loop_timer)
    {
      PORTH &= B11110111;
    }
    if(timer_channel_2 <= esc_loop_timer)
    {
      PORTH &= B11011111;
    }
    if(timer_channel_3 <= esc_loop_timer)
    {
      PORTH &= B10111111;
    }
    if(timer_channel_4 <= esc_loop_timer)
    {
      PORTH &= B11101111;
    }
  }
}



