#include<Servo.h>
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <Ethernet.h>

//Ethernet stuff

byte mac[] = {                        //MAC address 
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};
IPAddress ip(192, 168, 0, 10);       //Default IP address. DHCP will resolve the new IP address when connected
IPAddress gateway(192, 168, 0, 1);    //IP gateway
IPAddress subnet(255, 255, 255, 0);     //IP subnet mask

// telnet defaults to port 23
EthernetServer server(23);

//LED
const int LED_ARRAY_ONE = 53;                              // - Code: 51
const int LED_ARRAY_TWO = 49;                              // - Code: 52

//pressure sensor stuff                                 
//currently pressure sensor code is for MS5837-30BA sensor
MS5837 sensor;                             //- Code: 31
const int fluid_density = 997;

//Thermistor stuff                                      - Code: 21
#define THERMISTORPIN A0             //Thermistor analog pin 
#define THERMISTORNOMINAL 10000      //resistance at 25 degrees C
#define TEMPERATURENOMINAL 25        //temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 5                 //how many samples to take and average, more takes longer but is more 'smooth'
#define BCOEFFICIENT 3892            //The beta coefficient of the thermistor (usually 3000-4000) - 'US Sensor KS103JU'
#define SERIESRESISTOR 10000         //the value of the 'other' resistor
int samples[NUMSAMPLES];

//thruster values between 1100 and 1900
Servo THRUSTER_ONE;   //Forward-backward thruster 1    - Code: 10
Servo THRUSTER_TWO;   //Forward-backward thruster 2    - Code: 11
Servo THRUSTER_THREE;  //Left thruster                 - Code: 12
Servo THRUSTER_FOUR;   //Right thruster                - Code: 13
Servo THRUSTER_FIVE;   //Up-Down thruster 1            - Code: 14
Servo THRUSTER_SIX;    //Up-Down thruster 2            - Code: 15

//Camera servo values between 0 and 180degree
Servo CAM_1_1;  //CAMERA 1 SERVO PAN
Servo CAM_1_2;  //CAMERA 1 SERVO TILT
Servo CAM_2_1;  //CAMERA 2 SERVO PAN
Servo CAM_2_2;  //CAMERA 2 SERVO TILT

//camera servo pin declaration
const int C1S1_PIN = 36;  //CAMERA 1 SERVO PAN
const int C1S2_PIN = 37;  //CAMERA 1 SERVO TILT
const int C2S1_PIN = 38;  //CAMERA 2 SERVO PAN
const int C2S2_PIN = 39;  //CAMERA 2 SERVO TILT

//thruster pin declaration
const int THRUSTER_ONE_PIN = 14;
const int THRUSTER_TWO_PIN = 15; 
const int THRUSTER_THREE_PIN = 16; 
const int THRUSTER_FOUR_PIN = 17; 
const int THRUSTER_FIVE_PIN = 18; 
const int THRUSTER_SIX_PIN = 19; 

//motor pin declaration. Motor driver used is L298 which takes one EN pin and two input pins IN1 and IN2
//Input pin high/low combination decides direction
  //Robotic arm pan motor
const int MOTOR_RARM_PAN_EN_PIN = 2;
const int MOTOR_RARM_LEFT_PIN = 22;
const int MOTOR_RARM_RIGHT_PIN = 23;

  //robotic arm forward/backward motor
const int MOTOR_RARM_FWBW_EN_PIN = 3;
const int MOTOR_RARM_FW_PIN = 24; 
const int MOTOR_RARM_BW_PIN = 25; 

  //robotic arm up/down motor
const int MOTOR_RARM_UPDN_EN_PIN = 4;
const int MOTOR_RARM_UP_PIN = 26; 
const int MOTOR_RARM_DN_PIN = 27; 

  //robotic gripper 360a-clkwise/clkwise motor
const int MOTOR_RGRIPPER_360_EN_PIN = 5;
const int MOTOR_RGRIPPER_360ACLK_PIN = 28; 
const int MOTOR_RGRIPPER_360CLK_PIN = 29; 

  //robotic gripper 180up/down motor
const int MOTOR_RGRIPPER_180_EN_PIN = 6;
const int MOTOR_RGRIPPER_180UP_PIN = 30; 
const int MOTOR_RGRIPPER_180DN_PIN = 31; 

  //robotic gripper open/close motor
const int MOTOR_RGRIPPER_OPCL_EN_PIN = 7;
const int MOTOR_RGRIPPER_OPEN_PIN = 32; 
const int MOTOR_RGRIPPER_CLOSE_PIN = 33; 

  //robotic claw open/close motor
const int MOTOR_RCLAW_OPCL_EN_PIN = 8;  
const int MOTOR_RCLAW_OPEN_PIN = 34; 
const int MOTOR_RCLAW_CLOSE_PIN = 35; 

//all codes for systems
const int CODE_TH1 = 11;  //THRUSTER 1
const int CODE_TH2 = 12;  //THRUSTER 2
const int CODE_TH3 = 13;  //THRUSTER 3
const int CODE_TH4 = 14;  //THRUSTER 4
const int CODE_TH5 = 15;  //THRUSTER 5
const int CODE_TH6 = 16;  //THRUSTER 6
const int CODE_TS1 = 21;  //TEMPERATURE SENSOR 1
const int CODE_PS1 = 31;  //PRESSURE SENSOR 1
const int CODE_DCM1 = 41;  //DC MOTOR 1 - ROBOTIC ARM PAN LEFT/RIGHT
const int CODE_DCM2 = 42;  //DC MOTOR 2 - ROBOTIC ARM FW/BW 
const int CODE_DCM3 = 43;  //DC MOTOR 3 - ROBOTIC ARM DOWN/UP
const int CODE_DCM4 = 44;  //DC MOTOR 4 - ROBOTIC ARM GRIPPER 360DEGREE ROTATION  LEFT/RIGHT
const int CODE_DCM5 = 45;  //DC MOTOR 5 - ROBOTIC ARM GRIPPER 180DEGREE UP/DOWN   DOWN/UP
const int CODE_DCM6 = 46;  //DC MOTOR 6 - ROBOTIC ARM GRIPPER CLOSE/OPEN
const int CODE_DCM7 = 47;  //DC MOTOR 7 - ROBOTIC ARM CLAW CLOSE/OPEN
const int CODE_HL1 = 51;  //HEADLIGHT 1
const int CODE_HL2 = 52;  //HEADLIGHT 2
const int CODE_C1S1 = 61;  //CAMERA 1 SERVO PAN
const int CODE_C1S2 = 62;  //CAMERA 1 SERVO TILT
const int CODE_C2S1 = 63;  //CAMERA 2 SERVO PAN
const int CODE_C2S2 = 64;  //CAMERA 2 SERVO TILT

//temperature/pressure sensor value send timer
unsigned long data_transfer_timer;
const long data_transfer_time_gap = 1000;

//emergency shutoff timer
boolean stopped_bit = false;
unsigned long shutoff_timer;
const long shutoff_time_gap = 5000;

void setup() {
  Serial.begin(9600);
  //setup servo pins for all thrusters
  setup_thruster_servo();

  //setup servo pins for camera
  setup_camera_servo();

  //setup output pins for all dc motors
  setup_motors();

  //setup aref for thermistor
  analogReference(EXTERNAL);

  //setup pressure sensor
  setup_pressure_sensor();

  //setup LED pins
  pinMode(LED_ARRAY_ONE,OUTPUT);
  pinMode(LED_ARRAY_TWO,OUTPUT);
  digitalWrite(LED_ARRAY_ONE, LOW);
  digitalWrite(LED_ARRAY_TWO, LOW);

  //setup ethernet
  setup_ethernet();
}

void setup_ethernet()
{
  // start the Ethernet connection:
  Serial.println("Trying to get an IP address using DHCP");
 /* if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // initialize the ethernet device not using DHCP:
    Ethernet.begin(mac, ip, gateway, subnet);
  }*/
  
  Ethernet.begin(mac, ip, gateway, subnet);
  
  // print your local IP address:
  Serial.print("My IP address: ");
  ip = Ethernet.localIP();
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(ip[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
  // start listening for clients
  server.begin();
}

//setup pressure sensor, i2c wire and fluid density
void setup_pressure_sensor() 
{
  Wire.begin();
  sensor.init();
  sensor.setFluidDensity(fluid_density);    //kg/m^3 (freshwater, 1029 for seawater)
}

void setup_camera_servo()
{
  //attach each camera servo to its pwm pin
  //check lookup table for servo-pin pairing
  CAM_1_1.attach(C1S1_PIN);
  CAM_1_2.attach(C1S2_PIN);
  CAM_2_1.attach(C2S1_PIN);
  CAM_2_2.attach(C2S2_PIN);

  //set all servos to 0 degree
  reset_camera_servo();
}

//reset all camera servo to default position degree
void reset_camera_servo()
{
  CAM_1_1.write(90);
  CAM_1_2.write(90);
  CAM_2_1.write(90);
  CAM_2_2.write(90);
}

void setup_thruster_servo()
{
  //attach each thruster to its pwm pin
  //check lookup table for thruster-pin pairing
  THRUSTER_ONE.attach(THRUSTER_ONE_PIN);
  THRUSTER_TWO.attach(THRUSTER_TWO_PIN);
  THRUSTER_THREE.attach(THRUSTER_THREE_PIN);
  THRUSTER_FOUR.attach(THRUSTER_FOUR_PIN);
  THRUSTER_FIVE.attach(THRUSTER_FIVE_PIN);
  THRUSTER_SIX.attach(THRUSTER_SIX_PIN);
  
  //send "stop" signal to esc to prevent thrusters from starting
  stop_thrusters();
  ///////////////////////////////// delay - 1000. substitute with timer
}

//stop all thrusters
void stop_thrusters()
{
  THRUSTER_ONE.writeMicroseconds(1500);
  THRUSTER_TWO.writeMicroseconds(1500);
  THRUSTER_THREE.writeMicroseconds(1500);
  THRUSTER_FOUR.writeMicroseconds(1500);
  THRUSTER_FIVE.writeMicroseconds(1500);
  THRUSTER_SIX.writeMicroseconds(1500);
}

void setup_motors()
{
  //attach each motor to its IN1 IN2 and EN pin
  //check lookup table for motor-pin pairing

  //Robotic arm pan motor
  pinMode(MOTOR_RARM_PAN_EN_PIN, OUTPUT);
  pinMode(MOTOR_RARM_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RARM_RIGHT_PIN, OUTPUT);

  //robotic arm forward/backward motor
  pinMode(MOTOR_RARM_FWBW_EN_PIN, OUTPUT);
  pinMode(MOTOR_RARM_FW_PIN, OUTPUT);
  pinMode(MOTOR_RARM_BW_PIN, OUTPUT);

  //robotic arm up/down motor
  pinMode(MOTOR_RARM_UPDN_EN_PIN, OUTPUT);
  pinMode(MOTOR_RARM_UP_PIN, OUTPUT);
  pinMode(MOTOR_RARM_DN_PIN, OUTPUT);

  //robotic gripper 360a-clkwise/clkwise motor
  pinMode(MOTOR_RGRIPPER_360_EN_PIN, OUTPUT);
  pinMode(MOTOR_RGRIPPER_360ACLK_PIN, OUTPUT);
  pinMode(MOTOR_RGRIPPER_360CLK_PIN, OUTPUT);

  //robotic gripper 180up/down motor
  pinMode(MOTOR_RGRIPPER_180_EN_PIN, OUTPUT);
  pinMode(MOTOR_RGRIPPER_180UP_PIN, OUTPUT);
  pinMode(MOTOR_RGRIPPER_180DN_PIN, OUTPUT);

  //robotic gripper open/close motor
  pinMode(MOTOR_RGRIPPER_OPCL_EN_PIN, OUTPUT);
  pinMode(MOTOR_RGRIPPER_OPEN_PIN, OUTPUT);
  pinMode(MOTOR_RGRIPPER_CLOSE_PIN, OUTPUT);

  //robotic claw open/close motor
  pinMode(MOTOR_RCLAW_OPCL_EN_PIN, OUTPUT);
  pinMode(MOTOR_RCLAW_OPEN_PIN, OUTPUT);
  pinMode(MOTOR_RCLAW_CLOSE_PIN, OUTPUT);

  //stop all motors
  stop_motors();
}

//stops all motors
void stop_motors()
{
  int val = 0;
  analogWrite(MOTOR_RARM_PAN_EN_PIN, val);
  analogWrite(MOTOR_RARM_FWBW_EN_PIN, val);
  analogWrite(MOTOR_RARM_UPDN_EN_PIN, val);
  analogWrite(MOTOR_RGRIPPER_360_EN_PIN, val);
  analogWrite(MOTOR_RGRIPPER_180_EN_PIN, val);
  analogWrite(MOTOR_RGRIPPER_OPCL_EN_PIN, val);
  analogWrite(MOTOR_RCLAW_OPCL_EN_PIN, val);
}

//change position of camera servo
void camera_change_angle(int code, int val)
{
  if(code==CODE_C1S1) //camera1 servo pan
    CAM_1_1.write(val);
  else if(code==CODE_C1S2) //camera1 servo tilt
    CAM_1_2.write(val);
  else if(code==CODE_C2S1) //camera2 servo pan
    CAM_2_1.write(val);
  else if(code==CODE_C2S2) //camera2 servo tilt
    CAM_2_2.write(val);
}

//change speed of thruster using thruster code
void thruster_change_speed(int code,int val)
{
  Serial.println();
  Serial.print(code);
  Serial.print(":");
  Serial.print(val);
  //code is 2 digit integer mapped to a thruster
  if(code==CODE_TH1)
    THRUSTER_ONE.writeMicroseconds(val);
  else if(code==CODE_TH2)
    THRUSTER_TWO.writeMicroseconds(val);
  else if(code==CODE_TH3)
    THRUSTER_THREE.writeMicroseconds(val);
  else if(code==CODE_TH4)
    THRUSTER_FOUR.writeMicroseconds(val);
  else if(code==CODE_TH5)
    THRUSTER_FIVE.writeMicroseconds(val);
  else if(code==CODE_TH6)
    THRUSTER_SIX.writeMicroseconds(val);
}

//motor control code
void motor_control(int code, int val)
{
  long motorRunTime = 10;
  Serial.println();
  Serial.print(code);
  Serial.print(": ");
  Serial.print(val);
  //robotic arm pan movement
  if(code/100==CODE_DCM1)
  {
    analogWrite(MOTOR_RARM_PAN_EN_PIN, val);
    set_motor_val(MOTOR_RARM_LEFT_PIN, MOTOR_RARM_RIGHT_PIN, code%100);
    //delay(motorRunTime);
    //analogWrite(MOTOR_RARM_PAN_EN_PIN, 0);
  }
  //robotic arm fw/bw movement
  if(code/100==CODE_DCM2)
  {
    analogWrite(MOTOR_RARM_FWBW_EN_PIN, val);
    set_motor_val(MOTOR_RARM_FW_PIN, MOTOR_RARM_BW_PIN, code%100);
    //delay(motorRunTime);
    //analogWrite(MOTOR_RARM_FWBW_EN_PIN, 0);
  }
  //robotic arm up/down movement
  if(code/100==CODE_DCM3)
  {
    analogWrite(MOTOR_RARM_UPDN_EN_PIN, val);
    set_motor_val(MOTOR_RARM_UP_PIN, MOTOR_RARM_DN_PIN, code%100);
    //delay(motorRunTime);
    //analogWrite(MOTOR_RARM_UPDN_EN_PIN, 0);
  }
  //robotic gripper 360degree rotation
  if(code/100==CODE_DCM4)
  {
    analogWrite(MOTOR_RGRIPPER_360_EN_PIN, val);
    set_motor_val(MOTOR_RGRIPPER_360ACLK_PIN, MOTOR_RGRIPPER_360CLK_PIN, code%100);
    //delay(motorRunTime);
    //analogWrite(MOTOR_RGRIPPER_360_EN_PIN, 0);
  }
  //robotic gripper 180degree up/down
  if(code/100==CODE_DCM5)
  {
    analogWrite(MOTOR_RGRIPPER_180_EN_PIN, val);
    set_motor_val(MOTOR_RGRIPPER_180UP_PIN, MOTOR_RGRIPPER_180DN_PIN, code%100);
    //delay(motorRunTime);
    //analogWrite(MOTOR_RGRIPPER_180_EN_PIN, 0);
  }
  //robotic gripper open/close
  if(code/100==CODE_DCM6)
  {
    analogWrite(MOTOR_RGRIPPER_OPCL_EN_PIN, val);
    set_motor_val(MOTOR_RGRIPPER_OPEN_PIN, MOTOR_RGRIPPER_CLOSE_PIN, code%100);
    //delay(motorRunTime);
    //analogWrite(MOTOR_RGRIPPER_OPCL_EN_PIN, 0);
  }
  //robotic claw open/close
  if(code/100==CODE_DCM7)
  {
    analogWrite(MOTOR_RCLAW_OPCL_EN_PIN, val);
    set_motor_val(MOTOR_RCLAW_OPEN_PIN, MOTOR_RCLAW_CLOSE_PIN, code%100);
    //delay(motorRunTime);
    //analogWrite(MOTOR_RCLAW_OPCL_EN_PIN, 0);
  }
}

//set motor val
void set_motor_val(int pin1, int pin2, int mode)
{
  if(mode==0) //first direction
  {
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,LOW);
  }
  else //second direction
  {
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,HIGH);
  }
}

//read thermistor value and return reading
float read_thermistor()
{
  uint8_t i;
  double average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
  //Serial.print("Average analog reading "); 
  //Serial.println(average);
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(average);
 
  double steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
  //Serial.print("Temperature "); 
  //Serial.print(steinhart);
  //Serial.println(" *C");
  return steinhart;
 
  ///////////////////////////////// delay - 1000. substitute with timer
}

//read absolute pressure value and return depth
double read_depth()
{
  double pressure_abs;
  // Read pressure from the sensor in mbar.
  sensor.read();
  double depth = sensor.depth(); 
  //Serial.println("Abs Pressure: ");
  //Serial.println(pressure_abs);
  //Serial.println("mbar");
  return depth;
}

//turn led headlamps on off
void led_switch(int code,int val)
{
  if(code==CODE_HL1)
    digitalWrite(LED_ARRAY_ONE,val);
  else if(code==CODE_HL2)
    digitalWrite(LED_ARRAY_TWO,val);
}

//decode code and value
void decode_client_message(int code,int val)
{
  if(code>=1000 && code<2000)     //thruster code
  {
    thruster_change_speed(code/100,val);    //change speed of primary thruster
    if(code%100 != 0)                       //check if coupled thruster present
      thruster_change_speed(code%100,val);  //change speed of coupled thruster
  }
  else if(code>=2000 && code<3000)    //thermistor code
  {
    double thermistor_reading = 0;
    if(code/100 == CODE_TS1)                //code to read thermistor value
    {
      thermistor_reading = read_thermistor();   //call function to get temperature reading from thermistor
      put_tcp_message(code, thermistor_reading);  //send reading over tcp
    }
  }
  else if(code>=3000 && code<4000)    //pressure sensor code
  {
    double depth_reading;
    if(code/100 == CODE_PS1)                //code to read pressure value
    {
      depth_reading = read_depth();   //call function to get depth reading from pressure sensor
      put_tcp_message(code, depth_reading);    //send reading over tcp
    }
  }
  else if(code>=4000 && code<5000)    //dc motor control code
  {
    motor_control(code, val);    //change speed of primary motor
  }
  else if(code>=5000 && code<6000)    //LED code
  {
    led_switch(code/100, val);         //change state of primary led
    if(code%100 != 0)                //check if coupled led present
      led_switch(code%100, val);       //change state of coupled led
  }
  else if(code>=6000 && code<7000)    //camera servo code
  {
    camera_change_angle(code/100, val);    //control camera pan/tilt
  }
}

//reads tcp buffer
//concatenates message into string and returns int value
int get_tcp_message(EthernetClient client)
{
  char receivedChar;    
  String msg = "";                //Final message string
  while((receivedChar = client.read())!=':' && receivedChar!='e')  //read char from buffer. Check if 'separator/end' encountered
  {
    //if(isDigit(receivedChar))
      //Serial.print(receivedChar);
    if(isDigit(receivedChar))     //if received char is not digit ignore it
      msg += receivedChar;        //concatenate character with message string
    delay(1);                     //required delay
  }
  return msg.toInt();             //convert message to integer and return value
}

//send message on tcp to host
//format - "s"code:data"e"
//code is the code of the sensor whose data is being sent
//value is the data from the sensor being sent
void put_tcp_message(int code, double data)
{
  String message = "s" + String(code) + ":" + String(data) + "e";   //generate message string
  int len = message.length();
  char arr[len];
  message.toCharArray(arr,len);       //convert message to char array
  for(int i=0;i<len;i++)              //send message over tcp
  {
    server.write(arr[i]);
    delay(1);
  }
}

//check for incoming message
void loop() {
  //check if client is connected
  EthernetClient client = server.available();
  if(client && client.read()=='s')   //check if bytes available and first char is 's'
  {
    int code = get_tcp_message(client);
    int val = get_tcp_message(client);
    decode_client_message(code, val);
    Serial.println();
    Serial.print("Code: ");
    Serial.print(code);
    Serial.print("Val: ");
    Serial.print(val);
    
    stopped_bit = false;
    shutoff_timer = millis();
  }
  if(!stopped_bit && (millis()-data_transfer_timer)>data_transfer_time_gap)    //co-ordinate sending of pressure and temperature values when no data is available for read
  {
    //force send temperature and pressure data to controller
    decode_client_message(2100,0);  //read temperature value and send over tcp
    decode_client_message(3100,0);  //read pressure value and send over tcp
    Serial.print("in");
    data_transfer_timer = millis();
  }
  else if(!client && !stopped_bit && (millis()-shutoff_timer)>=shutoff_time_gap)  //no data available for read for more than shutoff_time_gap
  {
    //stop everything
    stop_thrusters();
    stop_motors();
    led_switch(CODE_HL1, 0);
    led_switch(CODE_HL2, 0);
    Serial.println();
    Serial.print("Everything stopped");
    stopped_bit = true;
    //client.stop();
  }
}
