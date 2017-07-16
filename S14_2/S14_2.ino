#include <NewPing.h>
#include <Wire.h>
#include <HMC5883L.h>

#define TRIGGER_PIN_H       4  
#define ECHO_PIN_H          5 
#define MAX_DISTANCE_H      200 

#define TRIGGER_PIN_V       13  
#define ECHO_PIN_V          17 
#define MAX_DISTANCE_V      100 

#define LEFT_MOTOR_ENABLE   9
#define LEFT_MOTOR_FORWARD  8
#define LEFT_MOTOR_REVERSE  7

#define RIGHT_MOTOR_ENABLE  10
#define RIGHT_MOTOR_FORWARD 12
#define RIGHT_MOTOR_REVERSE 11

#define LEFT_LED            14
#define RIGHT_LED           15

volatile long leftCount;
volatile long rightCount;

const float rotation_factor_cw = 9.0;
const float rotation_factor_ccw = 8.0;

//##################################################################################################################################################################################
//### MOTOR CLASS ##################################################################################################################################################################
//##################################################################################################################################################################################

class Motor {

  public:
  
    Motor();
    Motor(int ENB, int FWD, int RVS); //Enable pin number, forward pin number, reverse pin number
  
    void enable();                    //Enables the motor
    void disable();                   //Disables the motor
    void toggle();                    //Toggles the motors enable output: if the motor was enabled it disables and vice versa
    void halt();                      //Quickly stops the motor
    void kill();                      //Sets every output to the motor to LOW, basically disables and halts it simultaneously
    void forward();                   //Makes the track rotate forwards
    void reverse();                   //Makes the track rotate backwards

    void pwm(byte duty);              //Motor speed control; 0 = off, 127 = 50% speed & 255 = 100% speed

    bool isEnabled();                 //Returns a boolean testing whether the motor is enabled
    bool isRunning();                 //Returns a boolean testing whether the motor is running

  private:
  
    bool enabled;
    bool running_;
  
    int enable_;
    int forward_;
    int reverse_;
  
};

Motor::Motor() {}

Motor::Motor(int ENB, int FWD, int RVS) {

  enable_ = ENB;
  forward_ = FWD;
  reverse_ = RVS;

  pinMode(ENB, OUTPUT);
  pinMode(FWD, OUTPUT);
  pinMode(RVS, OUTPUT);
  
}

void Motor::enable() {
  
  digitalWrite(enable_, HIGH);

  enabled = true;
  
}

void Motor::disable() {
  
  digitalWrite(enable_, LOW);

  enabled = false;
  running_ = false;
  
}

void Motor::toggle() {

  if(enabled) {
    
    digitalWrite(enable_, LOW);

    running_ = false;
    
  } else {
    digitalWrite(enable_, HIGH);
  } 
}

void Motor::halt() {

  digitalWrite(forward_, LOW);
  digitalWrite(reverse_, LOW);

  running_ = false;
  
}

void Motor::kill() {

  digitalWrite(enable_, LOW);
  digitalWrite(forward_, LOW);
  digitalWrite(reverse_, LOW);

  running_ = false;
  
}

void Motor::forward() {

  digitalWrite(forward_, HIGH);
  digitalWrite(reverse_, LOW);

  running_ = true;
  
}

void Motor::reverse() {

  digitalWrite(reverse_, HIGH);
  digitalWrite(forward_, LOW);

  running_ = true;
  
}

void Motor::pwm(byte duty) {
  
  enabled = true;

  analogWrite(enable_, duty);
  
}

bool Motor::isEnabled() {
  return enabled;
}

bool Motor::isRunning() {
  return running_;
}

//##################################################################################################################################################################################
//### HEADLIGHT CLASS ##############################################################################################################################################################
//##################################################################################################################################################################################

class Headlights {

    public:
    
      Headlights(int left_output, int right_output);                                        //Left LED output pin, right LED output pin

      void illuminate(bool left, bool right);                                               //Illuminates the selected LED's; left = left, right = right
      void extinguish(bool left, bool right);                                               //Extinguishes the selected LED's; left = left, right = right
      void toggle(bool left, bool right);                                                   //Toggles the selected LED's to the opposite state
      
      void flash(bool left, bool right, bool invert, long duration, int duty, int period);  //Flashes the selected LED's; left = left LED, right = right LED, invert inverts the 
                                                                                            //right LED to illuminate when the left LED is off and vice versa, duration is the 
                                                                                            //duration of the flashing sequence in milliseconds, duty is the duty cycle of the 
                                                                                            //left LED, period is the duration of 1 cycle
      bool left_illuminated();                                                              //Returns a boolean testing whether the left LED is on
      bool right_illuminated();                                                             //Returns a boolean testing whether the right LED is on
      bool both_illuminated();                                                              //Returns a boolean testing whether the both LED's are on

    private:

      int LO;
      int RO;

      long start;

      bool left_on;
      bool right_on;

};

Headlights::Headlights(int left_output, int right_output) {
  
  LO = left_output;
  RO = right_output;

  pinMode(left_output, OUTPUT);
  pinMode(right_output, OUTPUT);
 
}

void Headlights::illuminate(bool left, bool right) {

  if(left) {
    
    left_on = true;  
    digitalWrite(LO, HIGH);
    
  }
  
  if(right) {
    
    right_on = true;   
    digitalWrite(RO, HIGH);
    
  }
}

void Headlights::extinguish(bool left, bool right) {

  if(left) {
    
    left_on = false;  
    digitalWrite(LO, LOW);
    
  }
  
  if(right) {
    
    right_on = false;   
    digitalWrite(RO, LOW);
    
  }
}

void Headlights::toggle(bool left, bool right) {

  if(left) {
    if(left_on) {

      left_on = false;
      digitalWrite(LO, LOW);
      
    } else {

      left_on = true;
      digitalWrite(LO, HIGH);
      
    }
  }
  
  if(right) {
    if(left_on) {

      right_on = false;
      digitalWrite(RO, LOW);
      
    } else {

      right_on = true;
      digitalWrite(RO, HIGH);
      
    }
  } 
}

bool Headlights::left_illuminated() {
  return left_on;
}

bool Headlights::right_illuminated() {
  return right_on;
}

bool Headlights::both_illuminated() {
  return left_on && right_on;
}

void Headlights::flash(bool left, bool right, bool invert, long duration, int duty, int period) {

  start = millis();

  while(duration + start > millis()) {

    if(left) {
      if(invert) {
        digitalWrite(LO, LOW);
      } else {
        digitalWrite(LO, HIGH);
      } 
    }

    if(right) {
      if(invert) {
        digitalWrite(RO, HIGH);
      } else {
        digitalWrite(RO, HIGH);
      }   
    }

    delay(duty);

    if(left) {
      if(invert) {
        digitalWrite(LO, HIGH);
      } else {
        digitalWrite(LO, LOW);
      }   
    }

    if(right) {
      if(invert) {
        digitalWrite(RO, LOW);
      } else {
        digitalWrite(RO, LOW);
      }   
    }

    delay(period - duty);
    
  }
  
  digitalWrite(LO, LOW);
  digitalWrite(RO, LOW);
  
}

//##################################################################################################################################################################################
//### VEHICLE CLASS ################################################################################################################################################################
//##################################################################################################################################################################################

class Vehicle {

  public:
  
    Vehicle(Motor L, Motor R);                                                  //Left motor, right motor
  
    void travel(bool dir, byte spd);                                            //Makes the vehicle travel; dir = direction (true = forward, false = reverse), spd = speed (0 - 255)

    void stop_();                                                               //Stops the motors providing power to the vehicle
    void brake();                                                               //Quickly stops the vehicle

    void turn(bool ForR, bool LorR, byte arc, byte velocity);                   //Turns the vehicle; ForR = foward or reverse (true = forward, false = reverse), LorR = left or right 
                                                                                //(true = left, false = right), arc = how quickly the vehicle turns (0 - 255), velocity = speed of the 
                                                                                //vehicle during the turn (0 - 255)
    void rotate(bool direction_, long duration, float angle, byte velocity);    //Rotates the vehicle on the spot; direction_ = direction (true = clockwise, false = counterclockwise),
                                                                                //duration = the number of milliseconds the vehicle should turn for (leave 0 is using angle instead),
                                                                                //angle = the angle the vehicle should rotate (leave 0.0 if using duration instead), velocity = speed 
                                                                                //of the vehicle during the turn (0 - 255)

    void enableCourseCorrection(long duration, long pulses);                    //Enables the automatic coarse correction, an end point must be specified! Duration = the number of 
                                                                                //milliseconds the vehicle will travel for (leave 0 if using pulses instead), pulses = the number of 
                                                                                //rising edges of the signal output of the motor hall effect sensor (leave 0 if using duration instead)
    void disableCourseCorrection();                                             //Disables the automatic coarse correction

    bool courseCorrectionEnabled();                                             //Returns a boolean testing whether the automatic coarse correction is enabled

  private:

    Motor LEFT;
    Motor RIGHT;

    bool course_correction;

    long cc_duration;
    long cc_pulses;

    long finish;

};

Vehicle::Vehicle(Motor L, Motor R) {

  LEFT = L;
  RIGHT = R;
  
}

void Vehicle::travel(bool dir, byte spd) {

  if(dir) {
    
    LEFT.forward();
    RIGHT.forward();
    
  } else {
    
    LEFT.reverse();
    RIGHT.reverse(); 
      
  }

  if(course_correction) {

    ::leftCount = 0;
    ::rightCount = 0;
      
    if(cc_pulses == 0) {

      finish = millis() + cc_duration;
      
      while(finish > millis()) {
        
        Serial.println(::leftCount);
        Serial.println(::rightCount);

        if(::leftCount == ::rightCount) {
          
          LEFT.pwm(spd);
          RIGHT.pwm(spd); 
          
        }
     
        if(::leftCount < ::rightCount) {
          
          LEFT.pwm(spd);
          RIGHT.pwm(0); 
          
        }
     
        if(::leftCount > ::rightCount) {
          
          LEFT.pwm(0);
          RIGHT.pwm(spd); 
          
        }    
      }
   
      LEFT.halt();
      RIGHT.halt();
      
    } else {

      while(::leftCount + ::rightCount < cc_pulses * 2) {

        if(::leftCount == ::rightCount) {
          
          LEFT.pwm(spd);
          RIGHT.pwm(spd); 
          
        }
     
        if(::leftCount < ::rightCount) {
          
          LEFT.pwm(spd);
          RIGHT.pwm(0); 
          
        }
     
        if(::leftCount > ::rightCount) {
          
          LEFT.pwm(0);
          RIGHT.pwm(spd); 
          
        }    
      }   
    }
  
  } else {

    LEFT.pwm(spd);
    RIGHT.pwm(spd);
    
  }
}

void Vehicle::stop_() {

  LEFT.disable();
  RIGHT.disable();
  
}

void Vehicle::brake() {

  LEFT.halt();
  RIGHT.halt();
  
}

void Vehicle::turn(bool ForR, bool LorR, byte arc, byte velocity) {

  if(ForR) {
    if(LorR) {

      RIGHT.forward();
      RIGHT.pwm(velocity);
          
      if(arc < 128) {

        LEFT.reverse();
        LEFT.pwm(velocity - map(arc, 0, 127, 0, velocity));
        
      } else {

        LEFT.forward();
        LEFT.pwm(map(arc, 128, 255, 0, velocity));
        
      }   
        
    } else {

      LEFT.forward();
      LEFT.pwm(velocity);
          
      if(arc < 128) {

        RIGHT.reverse();
        RIGHT.pwm(velocity - map(arc, 0, 127, 0, velocity));
        
      } else {

        RIGHT.forward();
        RIGHT.pwm(map(arc, 128, 255, 0, velocity));
        
      }         
    }
    
  } else {
    if(LorR) {

      RIGHT.reverse();
      RIGHT.pwm(velocity);
          
      if(arc < 128) {

        LEFT.forward();
        LEFT.pwm(velocity - map(arc, 0, 127, 0, velocity));
        
      } else {

        LEFT.reverse();
        LEFT.pwm(map(arc, 128, 255, 0, velocity));
        
      }   
          
    } else {
      
      LEFT.reverse();
      LEFT.pwm(velocity);
          
      if(arc < 128) {

        RIGHT.forward();
        RIGHT.pwm(velocity - map(arc, 0, 127, 0, velocity));
        
      } else {

        RIGHT.reverse();
        RIGHT.pwm(map(arc, 128, 255, 0, velocity));
        
      }       
    }
  } 
}

void Vehicle::rotate(bool direction_, long duration, float angle, byte velocity) {

  LEFT.pwm(velocity);
  RIGHT.pwm(velocity);

  if(duration == 0) {
    if(direction_) {

      duration = (long) angle * ::rotation_factor_cw;
      
    } else {

      duration = (long) angle * ::rotation_factor_ccw;
      
    }
  }

  long finish = millis() + duration;

  if(direction_) {
    while(finish > millis()) {
        
      LEFT.forward();
      RIGHT.reverse();
          
    }
  } else {
    while(finish > millis()) {
        
      LEFT.reverse();
      RIGHT.forward();
          
    }    
  }

  LEFT.halt();
  RIGHT.halt();
  
}

void Vehicle::enableCourseCorrection(long duration, long pulses) {

  course_correction = true;
  
  cc_duration = duration;
  cc_pulses = pulses;
  
}

void Vehicle::disableCourseCorrection() {
  course_correction = false;
}

//##################################################################################################################################################################################
//### COMPASS CLASS ################################################################################################################################################################
//##################################################################################################################################################################################

class Compass {
  
  public:
  
    Compass(HMC5883L compass);              //HMC5883L compass object
    
    void calibrate(int duration);           //Calibrates the compass; duration = the amount of time the compass should be gyrated for
    void setStartupBearing(float bearing);  //Sets the bearing to the specified bearing on startup, basically offsets them; bearing = the bearing the compass will start at
    void initialize();                      //Initializes the compass. This must be called before using any other of the compass' functions
    
    void enableDataStream();                //Enables the serial stream of both raw and processed compass values every time it takes a reading
    void disableDataStream();               //Disables the serial data stream
    
    double bearing();                       //Returns the current compass bearing in degrees 

    bool dataStreamEnabled();               //Tests to see if the serial data stream is enabled
    
  private:
  
    HMC5883L compass;

    bool stream;
    bool startup;
    
    long finish;
    
    double angle;
    
    double x;
    double y;
    
    int currentX;
    int currentY;
    
    float xOffset;
    float yOffset;
    float totalOffset;
    float startupBearing;
     
    float xMax = -8192.0;
    float xMin = 8192.0;
    float yMax = -8192.0;
    float yMin = 8192.0;   

};

Compass::Compass(HMC5883L compass) { 
  compass = compass;
}

void Compass::calibrate(int duration) {
  
  finish = millis() + duration;
  
  while(finish > millis()) {
    
    currentX = compass.readRaw().XAxis;
    currentY = compass.readRaw().YAxis;
    
    if(currentX > xMax) xMax = currentX;  
    if(currentX < xMin) xMin = currentX;  
    if(currentY > yMax) yMax = currentY; 
    if(currentY < yMin) yMin = currentY;
      
  }  
  
  xOffset = (xMax - xMin) / 2 - xMax;
  yOffset = (yMax - yMin) / 2 - yMax;
  
  xMax += xOffset;
  xMin += xOffset;
  yMax += yOffset;
  yMin += yOffset;

  if(stream) {

    Serial.print("Max X: ");   
    Serial.print(xMax);
    Serial.print("   Min X: ");
    Serial.print(xMin);
    Serial.print("   Max Y: ");
    Serial.print(yMax);
    Serial.print("   Min Y: ");
    Serial.print(yMin);
    Serial.print("   X Offset: ");
    Serial.print(xOffset);
    Serial.print("   Y Offset: ");
    Serial.println(yOffset);
    Serial.println();

  }
}

void Compass::setStartupBearing(float bearing) {

  startup = true;
  startupBearing = bearing;
  
}

void Compass::initialize() {
  
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.begin();
  
}

double Compass::bearing() {
  
  int xRaw = compass.readRaw().XAxis;
  int yRaw = compass.readRaw().YAxis;

  x = map(xRaw  + xOffset, xMin, xMax, yMin, yMax);
  y = yRaw + yOffset;

  if(stream) {
    
    Serial.print("Raw X: ");
    Serial.print(xRaw);
    Serial.print("   Raw Y:");
    Serial.print(yRaw);
    Serial.print("   X: ");
    Serial.print(x);
    Serial.print("   Y: ");
    Serial.print(y);
    Serial.print("      Angle (Degrees): "); 

  }
  
  float heading = atan2(y, x) + totalOffset;

  if (heading < 0)  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }

  angle = heading * 180 / M_PI; 

  if(startup) {

    startup = false;
    
    totalOffset = ((startupBearing - angle) * PI) / 180;    
    
  }
  
  return angle;
  
}

void Compass::enableDataStream() {
  stream = true;
}

void Compass::disableDataStream() {
  stream = false;
}

bool Compass::dataStreamEnabled() {
  return stream;
}

//##################################################################################################################################################################################
//### PROGRAM ######################################################################################################################################################################
//##################################################################################################################################################################################

Motor LEFT(LEFT_MOTOR_ENABLE, LEFT_MOTOR_FORWARD, LEFT_MOTOR_REVERSE);
Motor RIGHT(RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_REVERSE);

Vehicle ERNIE(LEFT, RIGHT);

Headlights LEDS(LEFT_LED, RIGHT_LED);

HMC5883L compass;

Compass COMPASS(compass);

NewPing sonar_horizontal(TRIGGER_PIN_H, ECHO_PIN_H, MAX_DISTANCE_H);
NewPing sonar_vertical(TRIGGER_PIN_V, ECHO_PIN_V, MAX_DISTANCE_V);

void incrementLeft() {
  leftCount++; 
}

void incrementRight() {
  rightCount++;
}

const byte straightSpeed = 255;

const int limit_h = 20;
const int limit_v = 13;

const int calibrationTime = 30000;
const int setupTime = 3000;
const int rechargeDelay = 2000;
const int pauseDelay = 200;
const int delayForHalfMeter = 1350;
const int startBearing = 270;
const int forwardBearing = 0;
const int margin = 4;
const int numberOfCycles = 4;

const int bearingList[numberOfCycles] = {135, 270, 45, 90};

int itterate;
int bearing;

void rotateUntilAt(int angle, int margin) {

  bearing = COMPASS.bearing();

  while(!(bearing > angle - margin && bearing < angle + margin)) {

    if(((((bearing - angle) % 360) + 540) % 360) - 180 < 0) {
      
      ERNIE.rotate(true, 0, 1.0, 255);
      LEDS.illuminate(true, false);
      
    } else {
      
      ERNIE.rotate(false, 0, 1.0, 255);
      LEDS.illuminate(false, true);
      
    }

    delay(30);
    
    bearing = COMPASS.bearing();
   
  }

  ERNIE.brake();
  LEDS.extinguish(true, true);
  
}

void cycle(int rotateBearing) {

  ERNIE.travel(true, straightSpeed);
  Serial.println("Forward 50cm");

  LEDS.illuminate(true, true);
  
  delay(delayForHalfMeter);

  ERNIE.brake();
  Serial.println("Stopping");

  LEDS.extinguish(true, true);

  delay(pauseDelay);

  Serial.print("Rotating ");
  Serial.print(rotateBearing);
  Serial.println(" Degrees");
  rotateUntilAt(rotateBearing, margin);

  Serial.println("Recharging...");
  delay(rechargeDelay);

  Serial.println("Continuing");
  rotateUntilAt(forwardBearing, margin);
  
}

void setup() {

  Serial.begin(115200);

  COMPASS.initialize();
  COMPASS.setStartupBearing(0.0);

  Serial.print("Gyrate the vehicle to calibrate the compass, you have ");
  Serial.print(calibrationTime / 1000);
  Serial.println(" seconds to do this after which the headlights will flash and the vehicle should be placed in its starting position.");
  Serial.println();
  
  COMPASS.calibrate(calibrationTime);

  LEDS.flash(true, true, false, 2000, 200, 400);

  delay(setupTime);

  rotateUntilAt(forwardBearing, margin);

  for(int i = 0; i < numberOfCycles; i++) {
    cycle(bearingList[i]);   
  }

  Serial.println("Program executed successfully!");
  
}

void loop() {}


