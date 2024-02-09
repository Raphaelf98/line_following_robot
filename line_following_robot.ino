#include <AFMotor.h>
#include <Servo.h>
#include <PID_v1.h>

#define MOTOR 1 //driving motor pin (Adafruit motor shield)
#define MOTOR_TRANSPORT 2 //transport motor pin (Adafruit motor shield)

#define IR_LEFT A9 //left steering ir sensor pin
#define IR_RIGHT A8 //right steering ir sensor pin
#define LINE_DETECTOR A10 //line detector steering ir sensor pin
#define SERVO 10 //servo pin
#define SATURATION 10 //pid output saturation
#define SERVO_SET_ANGLE 82 //servo set angle to drive straight
#define OUTPUT_MAX 80 //maximum steering angle, adds to SERVO_SET_ANGLE
//PID gains
#define KP 0.000225 //P gain
#define KI 0.000000 //I gain
#define KD 0.0000001 ///D gain
#define DELAY 0 //main loop delay
#define SPEED 255 //drive speed
#define LINE_THRESHOLD 40 //line threshold for line detection
//set CALIBRATE equal to 1 to enable serial print
#define CALIBRATE 0 //calibrate state
#define PICKUP_DELAY 5800 //time span in which transport motor is run in pickup mode
#define UNLOAD_DELAY 3600 //time span in which transport motor is run in unload mode
#define RETRACT_DELAY 4000 //time span in which transport motor is run in retract mode
#define OFFSET_SENSOR_RIGHT 0 //sensor offset 

//enum that holds states of state machine 
enum {STANDBY, DRIVE_STEER, PICKUP, UNLOAD, RETRACT, STOP};
//class IRSensor handles communication with infrared sensor, reads from analog pin
class IRSensor 
{
  public:
    IRSensor() {}
    IRSensor(int port1):p1_(port1), offset_(0) {} //constructor, takes sensor pin as input

    void setOffset(int offset) //offset function, sets offset_ variable
    { 
      offset_ = offset;
    }
    void read_raw(int &val) //reads sensor value from analog port and adds offset to it. sensor values are mapped to rangle between 0 and 100 to reduce noise in sensor readings
    {
      val = map(analogRead(p1_), 0, 1024, 0, 100) + offset_;
    }

  private:

    int p1_;
    int  offset_;

};

AF_DCMotor motor(MOTOR); //instantiate driving motor object
AF_DCMotor transport_motor(MOTOR_TRANSPORT); //instantiate transport motor object
Servo servo; //instantiate servo object
//instantiate ir sensors
IRSensor ir_left_(IR_LEFT);
IRSensor ir_right_(IR_RIGHT);
IRSensor line_detector(LINE_DETECTOR);
//instantiate variables to hold sensor readings
int val_line;
int val_l;
int val_r;
//instantiate pid variables and PID object
double setPoint, outputVal, error, abs_error, angle;
PID pid(&abs_error, &outputVal, &setPoint, KP, KI, KD, REVERSE); 
//instantiate logical variables 
bool left, right; //direction
bool line_pass = false; //line threshold passed
bool setTimer = true; //logical variable for skewer retraction timer
unsigned long offset; //holds current time
int line_count = 0;
int state = DRIVE_STEER; //set initial state



//Reads value of line sensor, counts line crossings and sets state according to line count
void line_check(const int &val)
{
    if (val > LINE_THRESHOLD && !line_pass) //check if sensor reading exceeds line threshold and if line threshold was exceeded in previous iteration
    { 
      line_pass = true; //set line_pass to true to ensure only a change in value is counted and not each iteration
      line_count++;
      if(line_count == 3) //third line corresponds to pick-up zone
      {
        state = PICKUP;
      }
      if(line_count ==4) //as soon as fourth line is detected, partial retraction of skewer starts
      {
        state = RETRACT;
      }
      if(line_count ==5) //full retraction, unload object
      {
        state = UNLOAD;
      }
      
    }
    if(val < LINE_THRESHOLD && line_pass)//set line_pass to false as soon as line is out of field-of-view of sensor and sensor reading is below threshold
    {
      line_pass = false;
    }
    
}

//setup function, initializes program
void setup() 
{

  Serial.begin(9600);
  Serial.println(F("Starting Line Follwing Transport Robot"));
  ir_right_.setOffset(OFFSET_SENSOR_RIGHT); //if range of sensor readings does not match, the range can be adjusted
  servo.attach(SERVO); //set ports
  delay(1000);
  servo.write(SERVO_SET_ANGLE); //align wheel to steer straight
  setPoint = 0;
  transport_motor.setSpeed(0); 
  transport_motor.run(FORWARD);
  pid.SetMode(AUTOMATIC); //initialize PID controller
  angle = SERVO_SET_ANGLE;
  delay(2000);
  left = false;
  right = false;

}


//main execution loop with state machine
void loop() 
{
 
  if(state == DRIVE_STEER || state ==RETRACT) //dirve and retact state, vehicle will follow the line and if RETRACT is true, retract the skewer half way
  {
    line_detector.read_raw(val_line); //get line sensor reading
    line_check(val_line); //check for line crossing and set state
    int speed = map(abs(-SERVO_SET_ANGLE+angle), -OUTPUT_MAX, OUTPUT_MAX, 0,30); //map speed proportional to steering angle
    motor.setSpeed(SPEED-speed);//subtract speed from main velocity set in #define SPEED
    motor.run(FORWARD);
    ir_left_.read_raw(val_l); //read left steering sensor
    ir_right_.read_raw(val_r);//read right steering sensor
    error = 20*(val_l - val_r);//multiply error with factor to resolve finer errors. Accounts for information loss to double->int cast
    abs_error = abs(round(error)); //round and compute absolute error
    
    if (error >= 0) //set direction variables left, and right so that vehicle steers left
    {
      left = true;
      right = false;
    }
  
    if (error < 0) //set direction variables left, and right so that vehicle steers right
    {
      right = true;
      left = false;
    }
  
    pid.Compute(); //compute pid output
  
    if (left) //steer left
    {
      if (outputVal > SATURATION) //check if pid output exceeds saturation, prevents large changes in steering angle
      {
        outputVal = SATURATION;
      }
  
      angle = angle - outputVal; //adjust steering anlge with pid output
  
      if (angle < SERVO_SET_ANGLE - OUTPUT_MAX) //bound angle to prevent steering angles smaller than SERVO_SET_ANGLE - OUTPUT_MAX
      {
        angle = SERVO_SET_ANGLE - OUTPUT_MAX;
      }
  
      int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 600, 2300); //map agnle to range between 600 - 2300. Required for writeMicroseconds function
      servo.writeMicroseconds(angle_); //set servo 
  
    }
  
    if (right) 
    {
  
      if (outputVal > SATURATION) //check if pid output exceeds saturation, prevents large changes in steering angle
      {
        outputVal = SATURATION;
      }
  
      angle = angle + outputVal;  //adjust steering anlge with pid output
  
      if (angle > OUTPUT_MAX + SERVO_SET_ANGLE) //bound angle to prevent steering angles larger than SERVO_SET_ANGLE + OUTPUT_MAX
      {
        angle = SERVO_SET_ANGLE + OUTPUT_MAX;
      }
  
      int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 600, 2300); //map agnle to range between 600 - 2300. Required for writeMicroseconds function
      servo.writeMicroseconds(angle_);//set servo 
  
    }
    
    if (state == RETRACT) //if retract is true, skewer will be retracted while driving until RETRACT_DELAY has passed 
    {
      if(setTimer == true)
      {
        offset = millis();
        setTimer=false;
      }
      if(millis()-offset<RETRACT_DELAY)
      {
        
        transport_motor.setSpeed(255);
        transport_motor.run(BACKWARD);
      }
      else{
        transport_motor.setSpeed(0);
        transport_motor.run(BACKWARD);
        state = DRIVE_STEER;
        }
    }
    
    if (CALIBRATE) //calibration state.Prints line sensor readings and is only required to set LINE_THRESHOLD before operation.
    {
      String mot = String(val_line);
      Serial.println(  "line: "+mot);
    }

  }
  if(state == PICKUP) //PICKUP state stops driving motor and runs skewer motor for PICKUP_DELAY amount of time. 
  {
    motor.setSpeed(255);
    motor.run(BACKWARD); //break
    delay(100);
    motor.setSpeed(0);
    motor.run(FORWARD);
    transport_motor.setSpeed(255); //start skewer motor pickup
    transport_motor.run(FORWARD);
    delay(PICKUP_DELAY);
    transport_motor.setSpeed(0); //stop skewer motor pickup
    transport_motor.run(FORWARD);
    state = DRIVE_STEER; //re-enter DRIVE_STEER state
    
  }
  if(state==UNLOAD) //UNLOAD state runs skewer motor in reverse direction for UNLOAD_DELAY amount of time until object is unloaded.
  {
    motor.setSpeed(255);
    motor.run(BACKWARD); //break
    delay(100);
    motor.setSpeed(0);
    motor.run(FORWARD);
    transport_motor.setSpeed(255);  //start skewer motor unload
    transport_motor.run(BACKWARD);
    delay(UNLOAD_DELAY);
    transport_motor.setSpeed(0);
    transport_motor.run(BACKWARD); //stop skewer motor unload
    state=STOP; //set STOP state 
   }
  if(state==STOP) //final state, stops vehicle
  {
    motor.setSpeed(0);
    motor.run(FORWARD);
  }

}
