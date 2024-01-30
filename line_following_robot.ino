#include <AFMotor.h>
#include <Servo.h>
#include <MovingAverage.h>
#include <movingAvg.h>
#include <PID_v1.h>

#define MOTOR 1
#define MOTOR_TRANSPORT 2

#define IR_LEFT A9
#define IR_RIGHT A8
#define LINE_DETECTOR A10
#define SERVO 10
#define SATURATION 10
#define SERVO_SET_ANGLE 82
#define OUTPUT_MIN 0
#define OUTPUT_MAX 80
//#define KP 0.0000355
//#define KI 0.000000
//#define KD 0.000000115
#define KP 0.00024
#define KI 0.000000
#define KD 0.0000001
#define DELAY 0
#define SPEED 255
#define LINE_THRESHOLD 16
#define BREAK_DELAY 0
#define PICKUP_DELAY 7000
#define UNLOAD_DELAY 6000
#define RETRACT_DELAY 4000
#define PICK 1
#define OFFSET_SENSOR -3
#define MVG_AVG 1

enum {STANDBY, DRIVE_STEER, PICKUP, UNLOAD, RETRACT, STOP};

/*#define KP 0.15

  #define KI 0.01
  #define KD 0.0015*/
bool line_pass = false;
int line_count = 0;
int state = DRIVE_STEER;
class IRSensor {

  public:

    IRSensor() {}

    IRSensor(int port1)

      : p1_(port1), low_(0), high_(1024), offset_(0) {

    }

    void setHighLow(int low, int high) {

      low_ = low;

      high_ = high;

    }

    void setOffset(int offset) {

      offset_ = offset;

    }

    void read(int &val) {

      filter_.add(analogRead(p1_));



      int tmp = filter_.get();

      if (tmp <= low_) {

        val = 0;

      } else if (tmp >= high_) {

        val = 100;

      } else {

        val = map(tmp, low_, high_, 0, 90) + offset_;

      }

    }

    void read_raw(int &val) {

      //filter_.add(analogRead(p1_));

      //val = filter_.get();

      
      val = analogRead(p1_) ;
      val = map(val, 0, 1024, 0, 100) + offset_;

    }

  private:

    int p1_;

    int low_, high_, offset_;

    MovingAverage<int, 3> filter_;

};

class ServoDriver {



  public:

    ServoDriver(int channel)

      : state_(0) {

      servo_.attach(channel);

    }



    void set(int p) {

      state_ = p;



      servo_.write(p);

    }


    int get() {

      return state_;

    }

  private:

    int state_;

    Servo servo_;

};
//Funktioin setzt status bei Linienüberfahrt
void line_check(const int &val)
{
    if (val > LINE_THRESHOLD && !line_pass)
    { 
      line_pass = true;
      line_count++;
      if(line_count == 3)
      {
        delay(BREAK_DELAY);
        state = PICKUP;
      }
      if(line_count ==4)
      {
        state = RETRACT;
      }
      if(line_count ==5)
      {
        state = UNLOAD;
      }
      
    }
    if(val < LINE_THRESHOLD && line_pass)
    {
      line_pass = false;
    }
    
}

AF_DCMotor motor(MOTOR);
AF_DCMotor transport_motor(MOTOR_TRANSPORT);

//ServoDriver servo(10);

Servo servo;

IRSensor ir_left_(IR_LEFT);
IRSensor ir_right_(IR_RIGHT);

IRSensor line_detector(LINE_DETECTOR);

int val_line;
int val_l;

int val_r;

double setPoint, outputVal, error, abs_error;

movingAvg movingAverage(MVG_AVG);


PID pid(&abs_error, &outputVal, &setPoint, KP, KI, KD, REVERSE);

double angle;

bool left, right;

void setup() {

  Serial.begin(9600);

  Serial.println(F("In Setup function"));

  //Sensorkalibrierung


  
  movingAverage.begin();  
  ir_right_.setOffset(0);

  servo.attach(SERVO);

  delay(1000);

  servo.write(SERVO_SET_ANGLE);

  // delay(5000);

  //setpoint

  setPoint = 0;


transport_motor.setSpeed(0);
    transport_motor.run(FORWARD);


  pid.SetMode(AUTOMATIC);

  angle = SERVO_SET_ANGLE;

  delay(2000);

  left = false;

  right = false;
ir_right_.setOffset(OFFSET_SENSOR);
}



void loop() {

  if(state == DRIVE_STEER)
  {
  line_detector.read_raw(val_line);
  line_check(val_line);
  int speed = map(abs(-SERVO_SET_ANGLE+angle), -OUTPUT_MAX, OUTPUT_MAX, 0,30);
   
  motor.setSpeed(SPEED-speed);
  motor.run(FORWARD);

  ir_left_.read_raw(val_l);
  
  ir_right_.read_raw(val_r);
  
  //delay(DELAY);



  error = 20*(val_l - val_r);
  
  int errorMovingAvg = round(error);//movingAverage.reading(error);

  abs_error = abs(errorMovingAvg);
  
  

  // Serial.println(error);



  //Regler



  if (abs(error) > 0 && error >= 0) {

    left = true;

    right = false;

    // Serial.println("links");

  }

  if (abs(error) > 0 && error < 0) {

    right = true;

    left = false;

    // Serial.println("rechts");

  }

  pid.Compute();



  //String direction = "striaght";

  if (left) {

    // Serial.println("links");

    if (outputVal > SATURATION) {
      outputVal = SATURATION;
    }

    //if(outputVal_left > 0.5 && outputVal_left <1)outputVal_left=1;

    angle = angle - outputVal;

    if (angle < SERVO_SET_ANGLE - OUTPUT_MAX) 
    {
      angle = SERVO_SET_ANGLE - OUTPUT_MAX;
    }



    //Serial.println("ANGLE" + String(angle));

    int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 600, 2300);

    servo.writeMicroseconds(angle_);

    //direction = "left";

  }

  if (right) {

    // Serial.println("rechts");

    if (outputVal > SATURATION) {
      outputVal = SATURATION;
    }

    //if(outputVal_right > 0.5 && outputVal_right <1)outputVal_right=1;

    angle = angle + outputVal;



    if (angle > OUTPUT_MAX + SERVO_SET_ANGLE) {
      angle = SERVO_SET_ANGLE + OUTPUT_MAX;
    }



    int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 600, 2300);

    servo.writeMicroseconds(angle_);

    //direction = "right";

  }

 
  

  //String s1 = String(val_l);
  //String mot = String(val_line);Serial.println(  "line: "+mot);
  //String s2 = String(val_r);

  //Serial.println("DIRECTION: " + direction + "   LEFT: " + s1 + "    RIGHT: " + s2 + "   u: " + String(outputVal) + "   angle:  " + String(angle) +"  line: "+mot);
  //
  }
  if(state == PICKUP)
  {
    motor.setSpeed(255);
    motor.run(BACKWARD);
    delay(100);
    motor.setSpeed(0);
    motor.run(FORWARD);
    transport_motor.setSpeed(255);
    transport_motor.run(FORWARD);
    delay(PICKUP_DELAY);
    transport_motor.setSpeed(0);
    transport_motor.run(FORWARD);
    state = DRIVE_STEER;
    
  }
  if(state == RETRACT)
  {
    motor.setSpeed(100);
    motor.run(BACKWARD);
    delay(100);
    motor.setSpeed(0);
    motor.run(FORWARD);
    transport_motor.setSpeed(255);
    transport_motor.run(BACKWARD);
    delay(RETRACT_DELAY);
    transport_motor.setSpeed(0);
    transport_motor.run(FORWARD);
    state = DRIVE_STEER;
    
  }
  if(state==UNLOAD)
  {
    motor.setSpeed(255);
    motor.run(BACKWARD);
    delay(100);
    motor.setSpeed(0);
    motor.run(FORWARD);
    transport_motor.setSpeed(255);
    transport_motor.run(BACKWARD);
    delay(UNLOAD_DELAY);
    transport_motor.setSpeed(0);
    transport_motor.run(BACKWARD);
    state=STOP;
   }
  if(state==STOP){
    motor.setSpeed(0);
    motor.run(FORWARD);}

}
