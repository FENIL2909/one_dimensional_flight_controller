#include <Wire.h>
#include <Servo.h>

//for taking raw values from accelerometer:
double raw_accel_x,  raw_accel_y, raw_accel_z; 
//corrected values:
double gforce_x, gforce_y, gforce_z; 

//for taking raw values from gyro:
double raw_gyro_x,raw_gyro_y,raw_gyro_z;
//for corrected values:
double rot_x, rot_y, rot_z;

// variables for PID data
double p_error, i_error, d_error, current_value, set_value=0.0, total_value,kp, ki, kd, pid_error, last_p_error;


Servo left_motor, right_motor;
void setup() {
  // put your setup code here, to run once:
wire.begin();
wire.beginTransmission();
serial.begin(9600);
MPUsetup();
left_motor.attach(D10);     //assigning pins to the BLDc motor...
right_motor.attach(D11);
}

void loop() {
  //I2C address of MPU6050:
  wire.begin(0b110100);   
  //data register reading data from accelerometer:
  wire.write(0x3B);
  wire.endTransmission();


  wire.requestFrom(0b1101000,6)
  while(wire.available()<=6)
  {
    // reading data from the registers byte by byte as the data is 16 bit  long so taking 8 bits at a time:
        raw_accel_x=wire.read()<<8|wire.read();
        raw_accel_y=wire.read()<<8|wire.read();
        raw_accel_z=wire.read()<<8|wire.read();

  }

        gforce_x=raw_accel_x/16384.0;  //converting raw data into meaningful:
        gforce_y=raw_accel_y/16384.0;
        gforce_z=raw_accel_z/16384.0;



        Serial.println("Accelerometer Data");
        Serial.print("X axis");
        Serial.print(gforce_X);
        Serial.print("Y axis");
        Serial.print(gforce_Y);
        Serial.print("Z axis");
        Serial.print(gforce_Z);
  
  // put your main code here, to run repeatedly:

//I2C address of MPU6050:
 wire.begin(0b110100);
 //data register reading data from accelerometer:
  wire.write(0x43);
  wire.endTransmission();

  wire.requestFrom(0b1101000,6)
  while(wire.available()<=6)
  {
        raw_gyro_x=wire.read()<<8|wire.read();
        raw_gyro_y=wire.read()<<8|wire.read();
        raw_gyro_z=wire.read()<<8|wire.read();
    }

    rot_gyro_x=raw_gyro_x/131.0;     //finding angular velocity from raw data:
    rot_gyro_y=raw_gyro_y/131.0;
    rot_gyro_z=raw_gyro_z/131.0;


        Serial.println("Gyro Data");
        Serial.print("X axis");
        Serial.print(rot_gyro_x);
        Serial.print("Y axis");
        Serial.print(rot_gyro_Y);
        Serial.print("Z axis");
        Serial.print(rot_gyro_Z);


        pid_error = PIDData();

        pid_error = map(pid_error,value1, value2, -500, 500);       // value 1 and value 2 can be infered from the readings we are getting from the gyro and accelerometer through the serial monitor.
        // initially assuming pid_error to be positive when tilted in the right hand side direction.....
        left_motor = writeMicroseconds(1500-pid_error);
        right_motor = writeMicroseconds(1500+pid_error);


}



void MPUsetup()
{
  //power:
          wire.begin(0b1101000);
          wire.write(0x6B);
          wire.write(0b0000000);
          wire.endTransmission();

//gyro:
                wire.begin(0b1101000);
                wire.write(0x1B);
                wire.write(0b0000000);
                wire.endTransmission();

//accelerometer:
            wire.begin(0b1101000);
            wire.write(0x1C);
            wire.write(0b0000000);
            wire.endTransmission();

}




    double PIDData(){

      p_error = current_value - set_value;        // current value can be obtained from the calculations of angles from data obtained from MPU 6050...
      i_error = p_error + last_p_error;
      d_error = p_error - last_p_error;

      last_p_error = p_error;

      total_value = kp*p_error + ki*i_error + kd*d_error;     // calculating total error for PID...
      Serial.print(total_value);
      return total_value;
      
      
      }

