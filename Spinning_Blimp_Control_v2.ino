#include "modBlimp.h"
#include <Adafruit_BNO055.h>
// #include <VL53L1X.h>


ModBlimp blimp;

String blimpcommand = "Custom";//"Automatic"; //"Custom"

// VL53L1X sensor;
AsyncUDP udp;

IBusBM IBus; 
//HardwareSerial MySerial0(0);
/*
flags to be used in the init 
-bool verbose: allows some debug print statments
-bool sensors: enables or disables the sensorsuite package: if false all values will be 0, and sensorReady =false in the sensor 
-bool UDP: starts up the UDP connection such that other UDP functions will be enabled
-int motor_type: determines if you are using brushless or brushed motors: 0 = brushless, 1 = brushed;
-int mode: sets which controller to listen to: 0 = UDP, 1 = IBUS,2 = espnow, -1 = None;
-int control: sets which type of controller to use: 0 = bicopter, 1 = spinning(TODO), -1 = None;
*/
init_flags_t init_flags = {
  .verbose = false,
  .sensors = false,
  .escarm = true,
  .UDP = true,
  .Ibus = true,
  .ESPNOW = true,
  .PORT = 1345,
  .motor_type = 1,
  .mode = 0,
  .control = 0,
};


/*
sensor values that control the sensors - if you want to turn off sensors use init_flags (sensors = false)
- float Kacc: kp value in implementation for accelerometer
- float Kgyro: kp value in implementation for gyroscope
- float Kmag: kp value in implementation for magnetometer
- bool baro: enables or disables barometer
- float eulerGamma: is the weight of the weighted average on euler angles (0 means that it always uses the newest value)
- float rateGamma: is the weight of the weighted average on gyroscope rates (0 means that it always uses the newest value)
- float zGamma: is the weight of the weighted average on estimatedZ (0 means that it always uses the newest value)
*/
init_sensors_t init_sensors = {
  .Kacc = 5,
  .Kgyro = -1,
  .Kmag = 0,
  .baro = true,
  .eulerGamma = 0,
  .rateGamma = 0.9f,
  .zGamma = 0.9f,
};

// BangBang
float tau = 0;
float yaw_calibrate = -1.6;

// Time of flight sensor init
float wall = 0.0;
float kwall = 0.02;
float last_time_flip = 0.0;
float yaw = 0.0;
float forcex = 0.15;
float forcey = 0.15;
float walk_heading  = atan2(forcey,forcex);
float joymag = sqrt(pow(forcex,2) + pow(forcey,2));
float random_reflect = 0;
float wall_min = 1500;
float yaw_min_dist = 0;
long dt, last_time, time_nw;

int wall_detect;
int wall_truly_short;
float mt = 0.3;
bool random_trip = false;

// min and max high signal of thruster PWMs
int minUs = 0;
int maxUs = 255;
int NUM_REGIONS = 8;



/*
PD terms for use in the feedback controller 
- bool roll, pitch, yaw, x, y, z, rotation: 
          enables that type of feedback (true means feedback is on for that state variable)
- float Croll, Cpitch, Cyaw, Cx, Cy, Cz, Cabsz: 
          KP term applied to the controller input
- float kproll, kdroll, kppitch, kdpitch, kpyaw, kdyaw: 
- float kpx, kdx, kpy, kdy, kpz, kdz;
          Kp and kd terms applied to each feedback mechanism using the sensors 
          (some do not have sensor availibility like x and y)
- float lx;
          a control variable used as the arm between the center of mass and the propellers
*/
feedback_t feedbackPD = {
  .roll = false,
  .pitch = false, 
  .yaw = true,
  .x = false,
  .y = false,
  .z = true,
  .rotation = false,

  .Croll = 1,
  .Cpitch = 0, 
  .Cyaw = 1,
  .Cx = 1,
  .Cy = 0,
  .Cz = 1,
  .Cabsz = 1,

  .kproll = 0,
  .kdroll = 0.0f,
  .kppitch = 0,
  .kdpitch = 0,
  .kpyaw = 3.0f,
  .kdyaw = -150.0f,//-70//5f,

  .kpx = 0,
  .kdx = 0,
  .kpy = 0,
  .kdy = 0,
  .kpz = 0.3f,
  .kdz = 0.1f,

  .lx = .15,
};
feedback_t * PDterms = &feedbackPD;
//storage variables
sensors_t sensors;
controller_t controls;
//rawInput_t rawInputs;TODO
actuation_t outputs;



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP280 bme; // I2C

float oldZ = 0;
float rawZ = 0;
float refZ = 0;
float barorate = 50;
bool baroOn = false;
time_t barotime;

bool bnoOn = false;

void setup() {
  
    
  //initializes systems based on flags and saves flags into the system
  blimp.init(&init_flags, &init_sensors, &feedbackPD);
    

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    
  } else{
    bnoOn = true;
  }
  /* Initialise the sensor */
  int countTries = 0;
  baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  while (!baroOn) {
      delay(100);
      if (countTries > 10) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                          "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        break;
      }
      countTries += 1;
      baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  }
  
  barotime = micros();
  if (baroOn){
    sensors.groundZ = bme.readAltitude();
    sensors.estimatedZ = sensors.groundZ;
    refZ = sensors.groundZ;
    rawZ = sensors.groundZ;
  } else {
    sensors.groundZ = 0;
    sensors.estimatedZ = 0;
    refZ = sensors.groundZ;
    rawZ = sensors.groundZ;
  }
  controls.snapshot = 0;

  delay(1000);


  // // Time of flight sensors setup
  // Wire.begin();
  // Wire.setClock(400000);

  // sensor.setTimeout(500);
  // if (!sensor.init()){
  //   Serial.println("Failed to detect and initialize sensor!");
  //   while (1);
  // }

  // sensor.setDistanceMode(VL53L1X::Long);
  // sensor.setMeasurementTimingBudget(15000);
  // sensor.startContinuous(13);
  // Serial.println("sensor ready");


}
float absoluteyawave = 0;
bool snapon = 0;
// float resyncPitch = 0.09;
// float resyncPitchTemp = 0;
// float resyncTimer = 0;
void loop() {
  // Serial.println(sensors.estimatedZ);
  /*  
  //    attempts to get the lastest information about the SENSORS and places them into the 
  //    sensor_t data structure
  //    contains: roll, pitch, yaw, rollrate, pitchrate, yawrate, estimatedZ, velocityZ, groundZ
  //    will return 0 for all sensors if sensors == false
  */
  getLatestSensorData(&sensors);
  delta_time();
  // wall = sensor.read();
  // Serial.println(wall);
  sensors.pitch =  sensors.pitch -3.1416 - 0.14;//hack to invert pitch due to orientation of the sensor
  while (sensors.pitch > 3.1416) {
    sensors.pitch -= 3.1416*2;
  }
  while (sensors.pitch < -3.1416) {
    sensors.pitch += 3.1416*2;
  }
  


  /*
  //    attempts to get the lastest information about the CONTROLLER and places them into the 
  //    controller_t data structure
  //    contrains: fx, fy, fz, absz, tx, ty, tz, ready
  */
  blimp.getControllerData(&controls);

  // controlAlgorithm(&controls, &sensors);
  

  /* TODO- NOT IMPLEMENTED
  //    optionally you can get the lastest information about the controller as raw values labeled as I1, I2, I3...
  */
  //rawInputs = blimp.getRawInputs();


  /*
  //    adds feedback directly into the controller terms using sensor feedback
  //    replace this with your own custom function for more customization
  //        example is placed below
  */
  //blimp.addFeedback(&controls, &sensors);
  addFeedback(&controls, &sensors); //this function is implemented here for you to customize


  // Serial.print(controls.fz);
  // Serial.print(", ");
  // Serial.print(controls.tz);
  // Serial.print(", ");
  // Serial.println(sensors.yawrate*100);

  // Serial.print(sensors.yawrate*100);
  // Serial.print(", ");
  // Serial.print(sensors.roll);
  // Serial.print(", ");
  // Serial.println(sensors.pitch);

  /*
  //    uses the mode to determine the control scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    replace this with your own custom function for more customization
  //    actuation_t data type contains: m1, m2, s1, s2 for each motor and servo
  //        example is placed below
  */
  getOutputs(&controls, &sensors, &outputs);
  //getOutputs(&controls, &sensors, &outputs); //this function is implemented here for you to customize
  
  /*
  //    uses the mode to determine the ouput scheme for the motor/servo outputs
  //    currently only implementation is for the bicopter blimp
  //    outputs should be floats between 0 and 1
  */
  blimp.executeOutputs(&outputs);
  // delay(4);
  


}

time_t snaptime;

float aveyaw = 0;
float tempyaw = 0;
int oldsnap = 0;
float ballz = 0;

void controlAlgorithm(controller_t *controls, sensors_t *sensors) {
  blimp.IBus.loop();
  int cx = blimp.IBus.readChannel(0);
  int cy = blimp.IBus.readChannel(1);
  int sig = blimp.IBus.readChannel(2);
  
  if (controls->snapshot != 0 ) {
    controls->snapshot = sig;
    controls->tz  = (float)((cx-120)/3)*3.14159f/180.0f;
    ballz += (float)((cx-120)/120*2);
    if (abs(ballz) > 15){
      ballz = 15 * ballz/abs(ballz);
    }
    controls->fz = ballz;
    if (controls->snapshot != oldsnap){
      snaptime = millis();
      tempyaw = sensors->yaw + controls->tz;//controls->tz;//
      oldsnap = controls->snapshot;
      while (tempyaw > 3.1416f){
        tempyaw -= 6.283f;
      }
      while (tempyaw < -3.1416f){
        tempyaw += 6.283f;
      }
    } else if (millis() - snaptime > 5000){// if time since last snap > 5 seconds do patterned walk on yaw
      tempyaw = sensors->yaw + 3.1415/4.0f;
      snaptime = millis();

    }
    controls->tz = 0;
    float tempdiff = -1*(tempyaw - sensors->yaw);
    while (tempdiff > 3.1416f){
      tempdiff -= 6.283f;
    }
    while (tempdiff < -3.1416f){
      tempdiff += 6.283f;
    }
    // float dyaw = tempdiff - oldyaw;
    // oldyaw = tempdiff;
    // float kdabsyaw = 1;
    float actyaw = clamp(tempdiff,-1,1);//- dyaw*kdabsyaw;
    
    // actyaw = actyaw;
    aveyaw = aveyaw * 0 + (actyaw) * 1;
  } else {
    tempyaw = sensors->yaw;
    aveyaw = 0;
    oldsnap = 0;
  }

  Serial.print(controls->snapshot);
  Serial.print(", ");
  Serial.print(ballz);
  Serial.print(", ");
  Serial.print(aveyaw);
  Serial.print(", ");
  Serial.print(sensors->yaw);
  Serial.print(", ");
  Serial.println(tempyaw);
  
  
}

/*
  -----------------------------------------------------------------------------------------------------
  EXAMPLE FUNCTIONS for full customization on outputs
  If you want to add a new sensor, you can try to go into firmware (crazyflieComplementary.cpp)
      or just implement it in this program
  -----------------------------------------------------------------------------------------------------
*/

void getLatestSensorData(sensors_t *sensors) {
  if (bnoOn){
    sensors_event_t orientationData, angVelocityData;//, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    sensors->yaw = orientationData.orientation.x* 3.1416f/180.0f;
    if (sensors->yaw > 3.1416f){
      sensors->yaw -= 3.1416f*2;
    }
    sensors->yaw = -sensors->yaw;
    sensors->roll = orientationData.orientation.y* 3.1416f/180.0f;
    sensors->pitch = orientationData.orientation.z* 3.1416f/180.0f;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    sensors->yawrate = sensors->yawrate *.7 + angVelocityData.gyro.z* 3.1416f/180.0f * .3;
    sensors->rollrate = sensors->rollrate *.7+ angVelocityData.gyro.y* 3.1416f/180.0f* .3;
    sensors->pitchrate = sensors->pitchrate *.7+  angVelocityData.gyro.x* 3.1416f/180.0f* .3;
    //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  } else {
    sensors->yaw = 0;
    sensors->roll = 0;
    sensors->pitch = 0;
    
    sensors->yawrate = 0;
    sensors->rollrate = 0;
    sensors->pitchrate = 0;
  }
  oldZ = rawZ;
  time_t newtime = micros();
  int barotimer = newtime - barotime; 
  if (barotimer > 1/barorate * 1000000) {
    if (baroOn) {
      float newHeight = bme.readAltitude();
      if (newHeight > 400 or newHeight < 100){
        baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
      } else {
        rawZ = newHeight;
        // Serial.print(sensors->velocityZ);
        // Serial.print(",");
        // Serial.println(sensors->estimatedZ);
        refZ += (rawZ - refZ) * 0.02;     
        sensors->estimatedZ = sensors->estimatedZ * .2 +  refZ* .8;
        sensors->velocityZ = sensors->velocityZ *.8 + (sensors->estimatedZ - oldZ)*.2;
      }
    } else {
      baroOn = bme.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

    }
    barotime = newtime;
    //baroHeightave = baroHeightave*.95 + baroHeight*.05;

  }
  //sensors->groundZ = 0;
}

float fzave = 0;
float tzave = 0;
// float tempyaw = 0;
// float oldyaw = 0;
// float aveyaw = 0;
// float oldsnap = 0;

//adds sensor feedback into the control values
//this set is specifically made for bicopter
void addFeedback(controller_t *controls, sensors_t *sensors) {
    //controller weights
    controls->fx *= PDterms->Cx;
    controls->fy *= PDterms->Cy;
    controls->fz *= PDterms->Cz;
    controls->tx *= PDterms->Croll;
    controls->ty *= PDterms->Cpitch;
    controls->tz *= PDterms->Cyaw;
    controls->absz *= PDterms->Cabsz;

    // Serial.print(controls->fx);
    // Serial.print(",");
    // Serial.println(controls->absz);

    //z feedback 
    if (PDterms->z) { 
    // controls->fz = (controls->fz + controls->absz - (sensors->estimatedZ-sensors->groundZ))*PDterms->kpz 
    //                 - (sensors->velocityZ)*PDterms->kdz;
    controls->fz = (controls->fz + controls->absz - (sensors->estimatedZ-sensors->groundZ))*controls->tx 
                    - (sensors->velocityZ)*PDterms->kdz;
    // Serial.println(controls->fz);
    // fzave = fzave * .9 + controls->fz * .1;
    // controls->fz = fzave;
    }
    
    // //yaw feedback
    // if (PDterms->yaw) { 
      
    //   controls->tz = controls->tz + aveyaw * PDterms->kpyaw - sensors->yawrate*PDterms->kdyaw;
      
    // }
    
    // //roll feedback
    // if (PDterms->roll) { 
    //   controls->tx = controls->tx - sensors->roll* PDterms->kproll - sensors->rollrate * PDterms->kdroll;
    // }

    // //roll and pitch rotation state feedback
    // if (PDterms->rotation) { 
    //   float cosp = (float) cos(sensors->pitch);
    //   float sinp = (float) sin(sensors->pitch);
    //   float cosr = (float) cos(sensors->roll);
    //   float ifx = controls->fx;
    //   controls->fx = ifx*cosp + controls->fz*sinp;
    //   controls->fz = (-1*ifx*sinp + controls->fz* cosp)/cosr;
    // }
}


//creates the output values used for actuation from the control values
void getOutputs(controller_t *controls, sensors_t *sensors, actuation_t *out ){
    //set up output
    float t1 = 0; 
    float t2 = 0;
    float f1 = 0;
    float f2 = 0;

    //set output to default if controls not ready
    if (controls->ready == false){
      out->s1 = .5f;
      out->s2 = .5f;
      out->m1 = 0;
      out->m2 = 0;
      out->ready = false;
      return;
    }

    out->ready = true;


    // Regular Control
      // // Convert joystick input to theta and magnitude for cyclic input
      // float joytheta = atan2(controls->fy,-controls->fx);  
      // float joymag  = sqrt(pow(controls->fx,2) + pow(controls->fy,2));
      yaw = sensors->yaw + yaw_calibrate;

      // float pitch = joymag * sin(joytheta) * cos(sensors->yaw);
      // float roll =  joymag * cos(joytheta) * sin(sensors->yaw);

      // // Mass Thrust it a proportional debug gain
      // f1 = controls->fz + (mt * (pitch + roll)); // LHS motor
      // f2 = controls->fz - (mt * (pitch + roll)); // RHS motor
      // Serial.println(sensors->yaw);

    //BangBang control
      float joytheta = atan2(controls->fy,-controls->fx);  
      float joymag  = sqrt(pow(controls->fx,2) + pow(controls->fy,2));
      // wall = sensor.read();
      // Serial.println(wall);
      // randomwalk(sensors->yaw);
      // walk_heading = 1.0;

      if ((yaw + joytheta) > PI + yaw_calibrate){
      joytheta = joytheta - 2*PI;
      } 
      else
        {if ((yaw + joytheta) < -1.0*PI + yaw_calibrate){
              joytheta = joytheta + 2*PI;
            }
        }
    
      // if (0 <= ((sensors->yaw + walk_heading)) && ((sensors->yaw + walk_heading)) < PI){
      if (0 + yaw_calibrate <= ((yaw + joytheta)) && ((yaw + joytheta)) < PI + yaw_calibrate){
        tau = joymag;
      } else{
        tau = -joymag;
      }
      f1 = controls->fz + (tau); // LHS motor
      f2 = controls->fz - (tau); // RHS motor


    // Random walk control
      // walk_heading = randomwalk();
      // float pitch = joymag * sin(walk_heading) * cos(sensors->yaw);
      // float roll =  joymag * cos(walk_heading) * sin(sensors->yaw); 
      // f1 = ifz + (mt * (pitch + roll)); // LHS motor
      // f2 = ifz - (mt * (pitch + roll)); // RHS motor

    //converting values to a more stable form
    out->s1 = blimp.clamp(t1, 0, 0)/(PI);// cant handle values between PI and 2PI
    out->s2 = blimp.clamp(t2, 0, 0)/(PI);
    out->m1 = blimp.clamp(f1, 0.0, 1.0);
    out->m2 = blimp.clamp(f2, 0.0, 1.0);
    send_udp_feedback(sensors->estimatedZ, sensors->groundZ, yaw, f1, f2);

    // Serial.println(out->m1);



    // if (out->m1 < 0.02f ){
    //   out->s1 = 0.5f; 
    // }
    // if (out->m2 < 0.02f ){
    //   out->s2 = 0.5f; 
    // }
    // return;
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

long delta_time(){
  time_nw = millis();
  dt = (time_nw - last_time);
  last_time = time_nw;
  // Serial.println()
  return dt;
}
// void randomwalk(float yaw){
//   // Serial.print(time_nw);
//   // Serial.print(", ");
//   // Serial.println(last_time_flip);
//   int distance = 1000;
//   //yaw = ity;

//   // Convert joystick input to theta and magnitude for cyclic input
//   // float joytheta = atan2(ify,-ifx) + PI;  

//   // Flipping fx-direction based on time since fx was last flipped
//   if ((time_nw - last_time_flip)>3000.0){
//     joymag = sqrt(pow(forcex,2) + pow(forcey,2));
//     wall = sensor.read();
//     // Serial.println(wall);
//     // if (random_trip == true){
//     //   walk_heading = walk_heading + (PI*0.0025 * random(-25,25));
//     //   random_trip = false;
//     // }

//     if (wall<distance) {
//       wall_detect ++;
//       if (wall_min > wall){
//         wall_min = wall;
//         yaw_min_dist = yaw;
//       }

//       if (wall_detect > 1){
//         // joymag = 0;

//         if (wall>wall_min){
//           wall_truly_short ++;
//         }

//         if (wall_truly_short > 2){
//           // walk_heading = walk_heading + 2.0*(PI*0.5 - (walk_heading - yaw_min_dist)); // 
//           walk_heading = yaw_min_dist - PI;

//           if (walk_heading > PI){
//             walk_heading = walk_heading - 2*PI;
//           } else{if (walk_heading < -1.0*PI){
//             walk_heading = walk_heading + 2*PI;
//           }
//           }
//           last_time_flip = time_nw;
//           wall_detect = 0;
//           wall_truly_short = 0;
//           joymag = 2.0*sqrt(pow(forcex,2) + pow(forcey,2));
//           wall_min = 1500;
//           random_trip = true;
//         }
//       }
//     }else{if(wall_detect > 0){wall_detect --;}}
//   }




































  // int region;
  // int maxVal = 0;
  // int distance = 1300;
  // int regions[8] = {0,1,2,3,4,5,6,7};
  // float yaw_store[8] = {0,PI*0.25,PI*0.5,PI*0.75,PI,PI*1.25,PI*1.5,PI*1.75};
  // int wall_counter[8] = {0,0,0,0,0,0,0,0};
  // int maxIndex = 0;
  // // float yaw = sensorsyaw;

  // if ((time_nw - last_time_flip)>3000.0){
  //   wall = sensor.read();
  //   // Serial.println("IN");
  //   // if (random_trip == true){
  //   //   walk_heading = walk_heading + (PI*0.0025 * random(-25,25));
  //   //   random_trip = false;
  //   // }

  //   if (wall<distance) {
  //     // wall_detect ++;    
  //     region = int((yaw + PI) / (PI*0.25));
  //     yaw_min_dist = yaw_store[region];
  //     wall_counter[region] ++;


  //     for (int i = 0; i < (sizeof(wall_counter) / sizeof(wall_counter[0])); i++) {
  //       if (wall_counter[i] > maxVal){
  //         maxVal = wall_counter[i];
  //         maxIndex = i;
  //       }
  //     }

  //     // if (wall_detect > 2){
  //     if (maxVal > 2){
  //       walk_heading = yaw - PI;
  //       // walk_heading = yaw_store[maxIndex] - PI;  
  //       if (walk_heading > PI){
  //         walk_heading = walk_heading - 2*PI;
  //       } else{if (walk_heading < -1.0*PI){
  //         walk_heading = walk_heading + 2*PI;
  //       }
  //       }
  //       last_time_flip = time_nw;
  //       wall_detect = 0;
  //       wall_truly_short = 0;
  //       joymag = sqrt(pow(forcex,2) + pow(forcey,2));
  //       wall_min = 1600;
  //       random_trip = true;

  //       for (int j = 0; j < sizeof(wall_counter) / sizeof(wall_counter[0]); j++){
  //         wall_counter[j] = 0;
  //       }
  //       maxIndex = 0;
  //     }
  //   }else{if(wall_detect > 0){wall_detect --;}}
  // }
// }

// // This function converts floats to strings and concatenates the strings for broadcastTo func
void send_udp_feedback(float wall, float walk_heading, float yaw, float m1, float m2){
  
  String motor1 = String(wall);
  String motor2 = String(walk_heading);
  String motor3 = String(yaw);
  String motor4 = String(m1);
  String motor5 = String(m2);
  String comb = String("");
  String comma = String(", ");
  String semicol = String(";");
  comb = motor1 + comma + motor2 + comma + motor3 + comma + motor4 + comma + motor5;
  // UDP Broadcast (string,port)
  udp.broadcastTo(comb.c_str(),8003);
// udp.broadcastTo("I Work",8001);
}

