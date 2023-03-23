#include <ESP8266WiFi.h>

#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"

#define LED 2

//Defining WiFi acces
const char* ssid     = "dlink";
const char* password = "";
IPAddress server(192, 168, 0, 161);
const uint16_t serverPort = 11412; //Unique for every omnidirectional


// Declaring physical pins
int M1A = 2;
int M1B = 14;
int ENA = 12;

int M2A = 4;
int M2B = 0;
int ENB = 13;

int M3A = 16;
int M3B = 5;
int ENC = 15;


// Defining control variables
int dottheta1;
int dottheta2;
int dottheta3;

int absdottheta1;
int absdottheta2;
int absdottheta3;

float u1;
float u2;
float u3;


// Factor of velocities 
float f;


// pwm floats
float pwm1;
float pwm2;
float pwm3;
float mapeo;

char hello[] = "Hello from Omni";
char initialazing[] = "Initializing...";
char done[] = "Ready for use";
char msg_data[] = "Moving omni";
 

// blinking built-in LED
void blink(int delay_time) {
  digitalWrite(LED, HIGH);
  delay(delay_time);
  digitalWrite(LED, LOW);
  delay(delay_time);
}


void motor1front()
{
//MOTOR1 adelante
digitalWrite (M1A, HIGH);
digitalWrite (M1B, LOW);
}

void motor2front()
{
//MOTOR2 adelante
digitalWrite (M2A, HIGH);
digitalWrite (M2B, LOW);
void velmot2();
}

void motor3front()
{
//MOTOR3 adelante
digitalWrite (M3A, HIGH);
digitalWrite (M3B, LOW);
void velmot3();
}


void motor1back()
{
//MOTOR1 back
digitalWrite (M1A, LOW);
digitalWrite (M1B, HIGH);
}


void motor2back()
{
//MOTOR2 back
digitalWrite (M2A, LOW);
digitalWrite (M2B, HIGH);
}


void motor3back()
{
//MOTOR3 back
digitalWrite (M3A, LOW);
digitalWrite (M3B, HIGH);
}


void motor1stop()
{
//MOTOR3 stop
digitalWrite (M1A, LOW);
digitalWrite (M1B, LOW);
}

void motor2stop()
{
//MOTOR3 stop
digitalWrite (M2A, LOW);
digitalWrite (M2B, LOW);
}

void motor3stop()
{
//MOTOR3 stop
digitalWrite (M3A, LOW);
digitalWrite (M3B, LOW);
}


void velmot1()
{
dottheta1= 44*(0.86*u1 + 0.5*u2 + 0.1*u3);
absdottheta1= abs(dottheta1);

pwm1 = map(absdottheta1 , 0, 130, 0, 255);
analogWrite(ENA,pwm1);

if(dottheta1>0)
{
motor1front();
}
else if(dottheta1==0)
{
motor1stop();
}
else if(dottheta1<0)
{
motor1back();
}
}


void velmot2()
{
dottheta2= 44*(-u2 + 0.1*u3);
absdottheta2= abs(dottheta2);
pwm2= map(absdottheta2, 0, 130, 0, 255);
analogWrite(ENB,pwm2);
if(dottheta2>0)
{
motor2front();
}

else if(dottheta2==0)
{
motor2stop();
}
else if(dottheta2<0)
{
motor2back();
}
}

void velmot3()
{
dottheta3= 44*(-0.86*u1 + 0.5*u2 + 0.1*u3);
absdottheta3= abs(dottheta3);
pwm3= map(absdottheta3, 0, 130, 0, 255);
analogWrite(ENC,pwm3);
if(dottheta3>0)
{
motor3front();
}
else if(dottheta3==0)
{
motor3stop();
}
else if(dottheta3<0)
{
motor3back();
}
}



//Creating ROS objects
ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Float64 float_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher float_publisher("float_publisher", &float_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("omni2_cmdVel", &cmdCB); // Name of omni robot


// forward [1,0,0] backward [-1, 0, 0], left [0, 1, 0], right [0, -1, 0], turn[0, 0, 1]
void cmdCB(const geometry_msgs::Twist& CVel)
{
Serial.println("x");
Serial.println(CVel.linear.x);
Serial.println("y");
Serial.println(CVel.linear.y);
Serial.println("z ang");
Serial.println(CVel.angular.z);

str_msg.data = hello;
chatter.publish( &str_msg );

u1= f*0.5*CVel.linear.x;  // x vel
u2 = f*0.4*CVel.linear.y;  // y vel
u3 = f*5*CVel.angular.z; // z ang



if (u1+u2+u3>1){
  str_msg.data = msg_data;
  chatter.publish( &str_msg );
}
velmot1();
velmot2();
velmot3();
mapeo= map(absdottheta1 , 0, 130, 0, 255);

//delay(2000);
}


void setupWiFi()
{
  WiFi.begin(ssid, password);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    while(1) delay(500);
  }
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  for(int i=0; i<15; i++)
    blink(50); 
}


void setup()
{
f = 1;

nh.getHardware()->setConnection(server, serverPort);
nh.initNode();

nh.subscribe(cmd_sub);
nh.advertise(chatter);
nh.advertise(float_publisher);

str_msg.data = hello;
chatter.publish(&str_msg);

str_msg.data = initialazing;
chatter.publish(&str_msg);

pinMode (ENA, OUTPUT);
pinMode (M1A, OUTPUT);
pinMode (M1B, OUTPUT);

pinMode (ENB, OUTPUT);
pinMode (M2A, OUTPUT);
pinMode (M2B, OUTPUT);

pinMode (ENC, OUTPUT);
pinMode (M3A, OUTPUT);
pinMode (M3B, OUTPUT);

pinMode(LED, OUTPUT);

Serial.begin(9600);

setupWiFi();

str_msg.data = done;
chatter.publish(&str_msg);

}


void loop()
{
//direct();

if (nh.connected()) {
  digitalWrite(LED, HIGH);
}
else {
  digitalWrite(LED, LOW);
}
nh.spinOnce();
delay(200);
digitalWrite(LED, LOW);
}
