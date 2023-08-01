#include <crazyflieComplementary.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include <ESP32Servo.h>

#define THRUST1 A0
#define THRUST2 A1
// #define THRUST1 A2
// #define THRUST2 A3

// min and max high signal of thruster PWMs
int minUs = 0;
int maxUs = 255*2;

// Wi-Fi access details
const char * ssid = "AIRLab-BigLab";
const char * password = "Airlabrocks2022";

// c.c. Edward
SensFusion sensorSuite;

AsyncUDP udp;
float joy_data[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
volatile bool joy_ready = false;
volatile unsigned long time_now, time_loop; 

// P.I.D. Values
float roll, pitch, yaw;
float rollrate, pitchrate, yawrate;
float estimatedZ, velocityZ, groundZ;
float abz = 0.0;
float kpz = 0.01*3.0; // N/meter
float kiz = 0.02;
float kdz = 0.2*1.0;
float kpx = 0.035;
float kdx = 0.2;
float kptz = 0.3;
float kdtz = -0.025;
float kptx = 0.01;
float kdtx = 0.01;
float lx = 0.25;
float m1 = 0.0;
float m2 = 0.0;
float heading = 0.0;
float massthrust = 0.9;
long dt, last_time, time_nw;

void setup() {
  Serial.begin(9600); 
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }

  // Allocate timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  pinMode(THRUST1,OUTPUT);
  pinMode(THRUST2,OUTPUT);
  
  // Access sensor suite c.c. Edward
  sensorSuite.initSensors();
  sensorSuite.updateKp(5, -1, 0.3); // 20, -1, 0
  groundZ = sensorSuite.returnZ();
  // sensorSuite.recordData();

  // magnetometer calibration
  float transformationMatrix[3][3] = {
    {1.0f, 9.693f, 0.6187f},
    {9.6624f, -0.6822f, 0.3864f},
    {-0.4155f, 0.6628f, -10.7386f}
  };
  float offsets[3] = {11.98f, 7.01f, 21.77f};
  sensorSuite.enterTransform(offsets, transformationMatrix);
  getSensorValues();

  
  time_now = millis();
  // time_loop = millis();
  if(udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());

    // setup callback functions of the udp
    udp.onPacket([](AsyncUDPPacket packet) {
      joy_ready = false;
      time_now = millis();
      unsigned char *buffer = packet.data();
      unpack_joystick(joy_data, buffer);
      joy_ready = true;
      //reply to the server
      // packet.printf("Got %u bytes of data", packet.length());
    });
  }

  // Yaw heading setup
  heading = sensorSuite.getYaw(); 

}

void loop() {
    //gyro, acc, mag, euler, z
  float cfx, cfy, cfz, ctx, cty, ctz;

  // Runs the sensor fusion loop
  sensorSuite.sensfusionLoop(false, 5);

  if (joy_data[7] != 1){

    Serial.println("Initialization") //debug;
    analogWrite(THRUST1, (int) minUs);
    analogWrite(THRUST2, (int) minUs);

    // Debug joystick input
    // thrust1.writeMicroseconds((int) (minUs + joy_data[0]));
    // thrust2.writeMicroseconds((int) (minUs + joy_data[0]));

  } else if (joy_ready && millis() - time_now <= 1000){ //&& millis() - time_loop > 50) {kdz
    // Call sensor suite to update 10-DOF values
    getSensorValues();
    getControllerInputs(&cfx, &cfy, &cfz, &ctx, &cty, &ctz, &abz);
    addFeedback(&cfx, &cfy, &cfz, &ctx, &cty, &ctz, abz);
    controlOutputs(cfx, cfy, cfz, ctx, cty, ctz);

    int m1us = 0;
    int m2us = 0;
    
    // Convert motor input to 1000-2000 Us values
    m1us = (minUs + (maxUs - minUs)*m1*2.0);
    m2us = (minUs + (maxUs - minUs)*m2*2.0);

    analogWrite(THRUST1, (int) m1us);
    analogWrite(THRUST2, (int) m2us);
    // Serial.print("m1us");
    // Serial.println((int) m1us); // Debug

    // Calls UDP Broadcast func
    // send_udp_feedback(m1us,m2us);

    // Else statement for if the Wi-Fi signal is lost
  } else {
]   // If no joystick B button command write min motor value
    analogWrite(THRUST1, (int) minUs);
    analogWrite(THRUST2, (int) minUs);
    
  }

}

void getSensorValues(){ 
  //all in radians or meters or meters per second
  yaw = sensorSuite.getYaw();
  yawrate = sensorSuite.getYawRate();
  estimatedZ = sensorSuite.returnZ();
  velocityZ = sensorSuite.returnVZ(); 
}

float valtz = 0;
void getControllerInputs(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float *abz){
  if (false) {
    *fx = 0;//joy_data[0];
    *fy = 0;//joy_data[1];
    *fz = 0;//joy_data[2];
    *tx = 0;//joy_data[3];
    *ty = 0;//joy_data[4];
    *tz = 0;//joy_data[5];
    *abz = 0;//joy_data[6];
    if (valtz > 1){
      valtz = -1;
    } else {
      valtz += .01;
    }
  } else{
  *fx = joy_data[0];
  *fy = joy_data[1];
  *fz = joy_data[2];
  *tx = joy_data[3];
  *ty = joy_data[4];
  *tz = joy_data[5];
  *abz = joy_data[6];
  }
}
void addFeedback(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float abz){
    // *fz = 0;
    float err = estimatedZ-groundZ;
    delta_time();
    float int_err =+ dt * err;
    // *fz = (*fz  - (estimatedZ-groundZ))*kpz - (int_err * kiz) - (velocityZ)*kdz + abz;//*fz = *fz + abz;//
    // *fz = (*fz  - (*tz))*kpz - (velocityZ)*kdz + abz;//*fz = *fz + abz;//
    *fz = *fz + *tz;
    // Serial.print(estimatedZ);
    // Serial.println(*tz);
    // *fz = (*fz  - (estimatedZ-groundZ))*kpz - (velocityZ)*kdz + abz;//*fz = *fz + abz;//

  // Serial.println(int_err*kiz);
}

long delta_time(){
  time_nw = millis();
  dt = (time_nw - last_time);
  last_time = time_nw;
  return dt;
}

float clamp(float in, float min, float max){
  if (in< min){
    return min;
  } else if (in > max){
    return max;
  } else {
    return in;
  }
}

void controlOutputs(float ifx, float ify, float ifz, float itx, float ity, float itz) {

  // // Heading added to the control loop
  // heading = heading + radians(ity * 180);
  
  // Serial.println(yaw);

  // Convert joystick input to theta and magnitude for cyclic input
  float joytheta = atan2(ify,-ifx) + PI;  
  float joymag  = sqrt(pow(ifx,2) + pow(ify,2));
  yaw = ity;

  // Cyclic pitch input
  float pitch = joymag * sin(joytheta) * cos(yaw);
  float roll =  joymag * cos(joytheta) * sin(yaw);

  // TODO: bring back x-y feedback
  // Mass Thrust it a proportional debug gain
  float mt = massthrust;

  float f1 = ifz + (mt * (pitch + roll)); // LHS motor
  float f2 = ifz - (mt * (pitch + roll)); // RHS motor


  // Clamp to ensure motor doesn't stop spinning at minimum
  // and motor doesn't draw too much current
  m1 = clamp(f1, 0.005, 0.3);
  m2 = clamp(f2, 0.005, 0.3);
  // Serial.println(m1);
}

// Having questions? Ask Jiawei!
void unpack_joystick(float *dat, const unsigned char *buffer) {
  int num_floats = 8;
  int num_bytes = 4;
  int i, j;

  for(i = 0; i < num_floats; i++) {
    char temp[4] = {0, 0, 0, 0};
    for(j = 0; j < num_bytes; j++) {
      temp[j] = buffer[num_bytes*i + j];
    }
    dat[i] = *((float*) temp);

  }
;
}

// Optional UDP feeback section
// ----------------------------

// // This function converts floats to strings and concatenates the strings for broadcastTo func
// void send_udp_feedback(int m1us, int m2us){
//   String motor1 = String(m1us);
//   String motor2 = String(m2us);
//   String comb = String("");
//   String comma = String(", ");
//   String semicol = String(";");
//   comb = motor1 + comma + motor2 + semicol;
//   // UDP Broadcast (string,port)
//   udp.broadcastTo(comb.c_str(),8003);
// // udp.broadcastTo("I Work",8001);
// }