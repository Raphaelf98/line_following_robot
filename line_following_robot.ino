#include <AFMotor.h>

#include <Servo.h>

#include <MovingAverage.h>

#include <PID_v1.h>

#define MOTOR 1

#define IR_LEFT A9

#define IR_RIGHT A8

#define SERVO 10

#define SATURATION 100

#define SERVO_SET_ANGLE 82

#define OUTPUT_MIN 0

#define OUTPUT_MAX 80

#define KP 0.04

#define KI 0.00005

#define KD 0.0000005

#define DELAY 8

#define SPEED 255

/*#define KP 0.15

#define KI 0.01

#define KD 0.0015*/

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

    //filter_.add(analogRead(p1_)+offset_);

    //val = filter_.get();

    val = map(analogRead(p1_), 0, 1024, 0, 80) + offset_;

  }

private:

  int p1_;

  int low_, high_, offset_;

  MovingAverage<int, 1> filter_;

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

 

AF_DCMotor motor(MOTOR);

//ServoDriver servo(10);

Servo servo;

IRSensor ir_left_(IR_LEFT);

IRSensor ir_right_(IR_RIGHT);

 

int val_l;

int val_r;

double setPoint, outputVal, error, abs_error;

 

PID pid(&abs_error, &outputVal, &setPoint, KP, KI, KD, REVERSE);

int angle;

bool left, right;

void setup() {

  Serial.begin(9600);

  Serial.println(F("In Setup function"));

  //Sensorkalibrierung

 

  ir_right_.setOffset(0);

  servo.attach(SERVO);

  delay(1000);

  servo.write(SERVO_SET_ANGLE);

  // delay(5000);

  //setpoint

  setPoint= 0;

 

 

  pid.SetMode(AUTOMATIC);

  angle = SERVO_SET_ANGLE;

  delay(2000);

  left = false;

  right = false;

}

 

void loop() {

  motor.setSpeed(SPEED);

  motor.run(FORWARD);

  ir_left_.read_raw(val_l);

  ir_right_.read_raw(val_r);

  delay(DELAY);

 

  error = val_l - val_r;

  abs_error = abs(error);

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

 

  String direction = "striaght";

  if (left) {

    // Serial.println("links");

    if (outputVal > SATURATION) { outputVal = SATURATION; }

    //if(outputVal_left > 0.5 && outputVal_left <1)outputVal_left=1;

    angle = angle - outputVal;

    if (angle < SERVO_SET_ANGLE - OUTPUT_MAX) { angle = SERVO_SET_ANGLE - OUTPUT_MAX; }

 

    //Serial.println("ANGLE" + String(angle));

    int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 700, 2300);

    servo.writeMicroseconds(angle_);

    direction = "left";

  }

  if (right) {

    // Serial.println("rechts");

    if (outputVal > SATURATION) { outputVal = SATURATION; }

    //if(outputVal_right > 0.5 && outputVal_right <1)outputVal_right=1;

    angle = angle + outputVal;

 

    if (angle > OUTPUT_MAX + SERVO_SET_ANGLE) { angle = SERVO_SET_ANGLE + OUTPUT_MAX; }

 

    int angle_ = map(angle, SERVO_SET_ANGLE - OUTPUT_MAX, OUTPUT_MAX + SERVO_SET_ANGLE, 700, 2300);

    servo.writeMicroseconds(angle_);

    direction = "right";

  }

 

  String s1 = String(val_l);

  String s2 = String(val_r);

  //Serial.println("DIRECTION: " + direction + "          LEFT: " + s1 + "    RIGHT: " + s2 + "   u_l: " + String(outputVal) + "   u_r: " + String(outputVal) + "   angle:  " + String(angle));

}
