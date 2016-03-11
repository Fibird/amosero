#define USE_USBCON
#include <ros.h>
ros::NodeHandle nh;
#include <std_msgs/Int32.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1 and M2
Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);

void left_motor_cb(const std_msgs::Int32& msg) {
    if(msg.data > 0)
    {
    if(msg.data<256)
      {
        left_motor->setSpeed(msg.data);
        left_motor->run(FORWARD);
      }
      else
      {
        left_motor->setSpeed(msg.data-512);
        left_motor->run(BACKWARD);
      }
    }
    else
    {
      left_motor->run(RELEASE);
    }    
}

void right_motor_cb(const std_msgs::Int32& msg) {
    if(msg.data > 0)
    {
      if(msg.data<256)
      {
        right_motor->setSpeed(msg.data);
        right_motor->run(FORWARD);
      }
      else
      {
        right_motor->setSpeed(msg.data-512);
        right_motor->run(BACKWARD);
      }
    
      //left_motor.runSpeed();
      digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    }
    else
    {
      right_motor->run(RELEASE);
    }
    
}

ros::Subscriber<std_msgs::Int32> sub_left_motor("left_motor", &left_motor_cb);
ros::Subscriber<std_msgs::Int32> sub_right_motor("right_motor", &right_motor_cb);


void setup() {  
  
  nh.initNode();
  nh.subscribe(sub_left_motor);     
  nh.subscribe(sub_right_motor);   
  AFMS.begin();  // create with the default frequency 1.6KHz
}

void loop() {
  nh.spinOnce();
    delay(10);
}