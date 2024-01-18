
#include <AFMotor.h>
#include<Servo.h>
#include<MovingAverage.h>
#include <PID_v1.h>
#define MOTOR 1
#define IR_LEFT A9
#define IR_RIGHT A8
#define SERVO 10
#define SATURATION 100
#define SERVO_SET_ANGLE 82
#define OUTPUT_MIN 0
#define OUTPUT_MAX 50
#define KP 0.15
#define KI 0.013
#define KD 0.0015
/*#define KP 0.15
#define KI 0.01
#define KD 0.0015*/
class IRSensor
{
  public:
  IRSensor(){}
  IRSensor(int port1):p1_(port1),low_(0),high_(1024),offset_(0)
  {
     
    
  }
  void setHighLow(int low, int high)
  {
    low_=low;
    high_=high;
  }
  void setOffset(int offset){offset_=offset;}
  void read(int &val)
  {
    filter_.add(analogRead(p1_));
    
    int tmp = filter_.get();
    if(tmp <= low_){
      val = 0;
    }
    else if(tmp >= high_){
      val = 100;
    }
    else{
     val =map(tmp,low_,high_,0,100)+offset_;
    }
  }
  void read_raw(int &val)
  {
    //filter_.add(analogRead(p1_)+offset_);
    //val = filter_.get();
    val = map(analogRead(p1_),0,1024,0,100)+offset_;
  }
  private:
  int p1_;
  int low_, high_,offset_;
  MovingAverage<int, 1> filter_;
};
class ServoDriver{
  
  public:
  ServoDriver(int channel):state_(0)
  {
    servo_.attach(channel);
  }
  
  void set(int p)
  {
    state_ = p;
    
    servo_.write(p);
  }
  int get(){
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
double  setPoint_left, outputVal_left, setPoint_right, outputVal_right,error_left,error_right;

PID pid_left(&error_left, &outputVal_left, &setPoint_left, KP, KI, KD,REVERSE);
PID pid_right(&error_right, &outputVal_right, &setPoint_right, KP, KI, KD,REVERSE);
 int angle;
bool left, right;
void setup()
{
  Serial.begin(9600);
  Serial.println(F("In Setup function"));
  //Sensorkalibrierung
 
  ir_right_.setOffset(0);
  servo.attach(SERVO);
  delay(1000); 
  servo.write(SERVO_SET_ANGLE);
  //setpoint
  setPoint_right = 0;
  setPoint_left= 0;
  
  pid_left.SetMode(AUTOMATIC);
  pid_right.SetMode(AUTOMATIC);
  angle=SERVO_SET_ANGLE;
  delay(2000); 
  left=false;
  right=false;
}

void loop()
{
  //servo.set(100);

  motor.setSpeed(200);
  motor.run(FORWARD);
  ir_left_.read_raw(val_l);
  ir_right_.read_raw(val_r);
  
  int e =val_l-val_r;
  
   //Steer LEFT
  if(abs(val_l - val_r)>1 && e>=0)
  {
    error_left = abs(e);
    error_right =0;
    left = true;
  }

  //Steer RIGHT
  if(abs(val_l - val_r)>0 && e<0)
  {
    error_right = abs(e);
    error_left = 0;
    right = true;
  }
  //Serial.println("error_left:  "+ String(error_left)+"   error_right: "+ String(error_right));
  
  pid_left.Compute();
  pid_right.Compute();
  String direction = "striaght";
  if(left)
  {
    if(outputVal_left > SATURATION){outputVal_left=SATURATION;}
    //if(outputVal_left > 0.5 && outputVal_left <1)outputVal_left=1;
    angle = angle-outputVal_left;
     if(angle < SERVO_SET_ANGLE-OUTPUT_MAX){angle=SERVO_SET_ANGLE-OUTPUT_MAX;}
    //Serial.println("ANGLE" + String(angle));
    
    servo.write((int)angle);
    //delay(1);           
    direction = "left";
    //Serial.println("Steer left  "+ String(angle));
  }
  if(right)
  {
    if(outputVal_right > SATURATION){outputVal_right=SATURATION;}
    //if(outputVal_right > 0.5 && outputVal_right <1)outputVal_right=1;
    angle = angle + outputVal_right;
    
    if(angle > OUTPUT_MAX+SERVO_SET_ANGLE){angle=SERVO_SET_ANGLE+OUTPUT_MAX;}
   
    servo.write((int)angle);
    //delay(10);           
    //Serial.println("Steer right  "+ String(angle));
    direction = "right";
  }
  

  left = false;
  right = false;
 
 
  
  //msg = map(val_l, 0, 1023, 45, 135);
  //msg = map(val_l, 0, 1023, 45, 135);
      //msg = 90;
  String s1 = String(val_l);
  String s2 = String(val_r); 
  Serial.println("DIRECTION: " + direction+    "          LEFT: "+s1 + "    RIGHT: "+s2 + "   u_l: "+String(outputVal_left)+ "   u_r: "+String(outputVal_right) + "   angle:  "+ String(angle)  );
  
}
