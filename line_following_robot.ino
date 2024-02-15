#include <AFMotor.h>
#include <Servo.h>
#include <PID_v1.h>

#define MOTOR 1 //Driving motor pin (Adafruit motor shield)
#define MOTOR_TRANSPORT 2 //Transport motor pin (Adafruit motor shield)

#define IR_LEFT A9 //Left steering ir sensor pin
#define IR_RIGHT A8 //Right steering ir sensor pin
#define LINE_DETECTOR A10 //Line detector steering ir sensor pin
#define SERVO 10 //Servo pin
#define SATURATION 10 //PID output saturation
#define SERVO_SET_ANGLE 82 //Servo set angle to drive straight
#define OUTPUT_MAX 80 //Maximum steering angle, adds to SERVO_SET_ANGLE
//PID gains
#define KP 0.000225 //P gain
#define KI 0.000000 //I gain
#define KD 0.0000001 ///D gain
#define DELAY 0 //Main loop delay
#define SPEED 255 //Driving speed
#define LINE_THRESHOLD 28 //Line threshold for line detection
//Set CALIBRATE equal to 1 to enable serial print
#define CALIBRATE 0 //Calibrate state
#define PICKUP_DELAY 5800 //Duration in which transport motor is run in pickup mode
#define UNLOAD_DELAY 3600 //Duration in which transport motor is run in unload mode
#define RETRACT_DELAY 4000 //Duration in which transport motor is run in retract mode
#define OFFSET_SENSOR_RIGHT 0 //Sensor offset 

//enum that holds states of state machine 
enum {STANDBY, DRIVE_STEER, PICKUP, UNLOAD, RETRACT, STOP};
//class IRSensor handles communication with infrared sensor, reads from analog pin
class IRSensor 
{
  public:
    IRSensor() {}
    IRSensor(int port1):p1_(port1), offset_(0) {} //Constructor, takes sensor pin as input

    void setOffset(int offset) //Offset function, sets offset_ variable
    { 
      offset_ = offset;
    }
    /*
    Reads sensor value from analog port and adds offset to it. 
    Sensor values are mapped to range between 0 and 100 to reduce noise in sensor readings
    */
    void read_raw(int &val) 
    {
      val = map(analogRead(p1_), 0, 1024, 0, 100) + offset_;
    }

  private:
    int p1_;
    int  offset_;

};

AF_DCMotor motor(MOTOR); //Instantiate driving motor object
AF_DCMotor transport_motor(MOTOR_TRANSPORT); //Instantiate transport motor object
Servo servo; //Instantiate servo object
//Instantiate ir sensors
IRSensor ir_left_(IR_LEFT);
IRSensor ir_right_(IR_RIGHT);
IRSensor line_detector(LINE_DETECTOR);
//Instantiate variables to hold sensor readings
int val_line;
int val_l;
int val_r;
//Instantiate pid variables and PID object
double setPoint, outputVal, error, abs_error, angle;
PID pid(&abs_error, &outputVal, &setPoint, KP, KI, KD, REVERSE); 
//Instantiate logical variables 
bool left, right; //Direction
bool line_pass = false; //Line threshold passed
bool setTimer = true; //Logical variable for skewer retraction timer
unsigned long offset; //Holds current time
int line_count = 0;
int state = DRIVE_STEER; //Set initial state



//Reads value of line sensor, counts line crossings and sets state according to line count
void line_check(const int &val)
{   
  /*
  Check if sensor reading exceeds line threshold and 
  if line threshold was exceeded in previous iteration
  */
  if (val > LINE_THRESHOLD && !line_pass) //
  { 
    line_pass = true; //Set line_pass to true to ensure only a change in value is counted and not each iteration
    line_count++;
    if(line_count == 3) //Third line corresponds to pick-up zone
    {
      state = PICKUP;
    }
    if(line_count ==4) //As soon as fourth line is detected, partial retraction of skewer starts
    {
      state = RETRACT;
    }
    if(line_count ==5) //Full retraction, unload object
    {
      state = UNLOAD;
    }
    
  }
  /*
  Set line_pass to false as soon as line is out of 
  field-of-view of sensor and sensor reading is below threshold
  */
  if(val < LINE_THRESHOLD && line_pass)
  {
    line_pass = false;
  }
    
}

//Setup function, initializes program
void setup() 
{

  Serial.begin(9600);
  Serial.println(F("Starting Line Following Transport Robot"));
  ir_right_.setOffset(OFFSET_SENSOR_RIGHT); //If range of sensor readings does not match, the range can be adjusted
  servo.attach(SERVO); //Set ports
  delay(1000);
  servo.write(SERVO_SET_ANGLE); //Align wheel to steer straight
  setPoint = 0;
  transport_motor.setSpeed(0); 
  transport_motor.run(FORWARD);
  pid.SetMode(AUTOMATIC); //Initialize PID controller
  angle = SERVO_SET_ANGLE;
  delay(2000);
  left = false;
  right = false;

}


//Main execution loop with state machine
void loop() 
{
  /*
  Dirve and retact state, vehicle will follow 
  the line and if RETRACT is true, retract the skewer half way
  */
  if(state == DRIVE_STEER || state ==RETRACT) 
  {
    line_detector.read_raw(val_line); //Get line sensor reading
    line_check(val_line); //Check for line crossing and set state
    int speed = map(abs(-SERVO_SET_ANGLE+angle), -OUTPUT_MAX, OUTPUT_MAX, 0,30); //Map speed proportional to steering angle
    motor.setSpeed(SPEED-speed);//Subtract speed from main velocity set in #define SPEED
    motor.run(FORWARD);
    ir_left_.read_raw(val_l); //Read left steering sensor
    ir_right_.read_raw(val_r);//Read right steering sensor
    error = 20*(val_l - val_r);//Multiply error with factor to resolve finer errors. Accounts for information loss to double->int cast
    abs_error = abs(round(error)); //Round and compute absolute error
    
    if (error >= 0) //Set direction variables left, and right so that vehicle steers left
    {
      left = true;
      right = false;
    }
  
    if (error < 0) //Set direction variables left, and right so that vehicle steers right
    {
      right = true;
      left = false;
    }
  
    pid.Compute(); //Compute pid output
  
    if (left) //Steer left
    { /*
      Check if pid output exceeds saturation, 
      prevents large changes in steering angle
      */
      if (outputVal > SATURATION) //
      {
        outputVal = SATURATION;
      }
  
      angle = angle - outputVal; //Adjust steering anlge with pid output
      /*
      Bound angle to prevent steering angles 
      smaller than SERVO_SET_ANGLE - OUTPUT_MAX
      */
      if (angle < SERVO_SET_ANGLE - OUTPUT_MAX) 
      {
        angle = SERVO_SET_ANGLE - OUTPUT_MAX;
      }
      /*
      Map agnle to range between 600 - 2300. Required for writeMicroseconds function
      */
      int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 600, 2300); 
      servo.writeMicroseconds(angle_); //set servo 
  
    }
  
    if (right) 
    {
      /*
      Check if pid output exceeds saturation, 
      prevents large changes in steering angle
      */
      if (outputVal > SATURATION) 
      {
        outputVal = SATURATION;
      }
  
      angle = angle + outputVal;  //Adjust steering anlge with pid output
      /*
      Bound angle to prevent steering angles 
      larger than SERVO_SET_ANGLE + OUTPUT_MAX
      */
      if (angle > OUTPUT_MAX + SERVO_SET_ANGLE) 
      {
        angle = SERVO_SET_ANGLE + OUTPUT_MAX;
      }
      /*
      Map agnle to range between 600 - 2300. Required for writeMicroseconds function
      */
      int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 600, 2300);
      servo.writeMicroseconds(angle_);//set servo 
  
    }
    /*
    If retract is true, skewer will be retracted 
    while driving until RETRACT_DELAY has passed 
    */
    if (state == RETRACT) 
    { 
      if(setTimer == true) //Start timer once
      {
        offset = millis();
        setTimer=false; 
      }
      if(millis()-offset<RETRACT_DELAY) //Run transport motor for RETRACT_DELAY seconds 
      {
        
        transport_motor.setSpeed(255);
        transport_motor.run(BACKWARD);
      }
      else  //Stop transport motor if RETRACT_DELAY has passed
      {
        transport_motor.setSpeed(0);
        transport_motor.run(BACKWARD);
        state = DRIVE_STEER;
      }
    }
    /*
    Calibration state.Prints line sensor readings and 
    is only required to set LINE_THRESHOLD before operation.
    */
    if (CALIBRATE) 
    {
      String mot = String(val_line);
      Serial.println(  "line: "+mot);
    }

  }
  /*
  PICKUP state stops driving motor and 
  runs transport motor for PICKUP_DELAY amount of time. 
  */
  if(state == PICKUP) 
  {
    motor.setSpeed(255);
    motor.run(BACKWARD); //Break
    delay(100);
    motor.setSpeed(0);
    motor.run(FORWARD);
    transport_motor.setSpeed(255); //Start transport motor pickup
    transport_motor.run(FORWARD);
    delay(PICKUP_DELAY);
    transport_motor.setSpeed(0); //Stop transport motor pickup
    transport_motor.run(FORWARD);
    state = DRIVE_STEER; //Re-enter DRIVE_STEER state
    
  }
  /*
  UNLOAD state runs transport motor in reverse direction for 
  UNLOAD_DELAY amount of time until object is unloaded.
  */
  if(state==UNLOAD) 
  {
    motor.setSpeed(255);
    motor.run(BACKWARD); //Break
    delay(100);
    motor.setSpeed(0);
    motor.run(FORWARD);
    transport_motor.setSpeed(255);  //Start transport motor unload
    transport_motor.run(BACKWARD);
    delay(UNLOAD_DELAY);
    transport_motor.setSpeed(0);
    transport_motor.run(BACKWARD); //Stop transport motor unload
    state=STOP; //Set STOP state 
   }
  if(state==STOP) //Final state, stops vehicle
  {
    motor.setSpeed(0);
    motor.run(FORWARD);
  }

}
