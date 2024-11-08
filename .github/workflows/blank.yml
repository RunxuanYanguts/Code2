#include <Servo.h>


/*connectino method is below*/
#define Sensor_Left_PIN1 A0
#define Sensor_Left_PIN2 A1
#define Sensor_Middle_Pin A2
#define Sensor_Right_PIN1 A3
#define Sensor_Right_PIN2 A4

#define Motor_Left_Pin1 1
#define Motor_Left_Pin2 2
#define Motor_Right_Pin1 3
#define Motor_Right_Pin2 4

#define left_front_PWM 5
#define right_front_PWM 6

#define left_behind_PWM 10
#define right_behind_PWM 11


/*variable,array,object are initialized below*/

/*array used to store sensor data*/
float sensors_data[] = {0, 0, 0, 0, 0};

/*variables used in PID algorithm*/
float Kp = 3.1;
float Ki = 0.04;
float Kd = 6;
float error = 0;
float proportional = 0;
float integral = 0;
float derivative = 0;
float last_proportional;
int error_value;

/*variables used in motor drive*/
int left_speed, right_speed;
float init_speed =80.0;

/*variables and object used in servo drive*/
int pos = 90;
int turn_angle = 0;
Servo myservo;

/*function used to read and store sensor data*/
void sensors_data_read()
{
  sensors_data[0] = analogRead(Sensor_Left_PIN1);
  sensors_data[1] = analogRead(Sensor_Left_PIN2);
  sensors_data[2] = analogRead(Sensor_Middle_Pin);
  sensors_data[3] = analogRead(Sensor_Right_PIN1);
  sensors_data[4] = analogRead(Sensor_Right_PIN2);

  /*turn analog data into digital data*/
  for (int i = 0; i < 5; i++)
  {
    if(sensors_data[i] >= 0.8) 
        sensors_data[i] = 1;
    else if(sensors_data[i] <= 0.5) 
        sensors_data[i] = 0; 
  }
  
  for (int i = 0; i < 5; i++)
  {
    sensors_data[i] = 1 - sensors_data[i];
  }
}

/*function implement localization algorithm*/
void judge()
{
  if ((sensors_data[0] == 0) && (sensors_data[1] == 0) && (sensors_data[2] == 0) && (sensors_data[3] == 0) && (sensors_data[4] == 1)) {
    error = 12.5;//          0 0 0 0 1
  } else if ((sensors_data[0] == 0) && (sensors_data[1] == 0) && (sensors_data[2] == 0) && (sensors_data[3] == 1) && (sensors_data[4] == 0)) {
    error = 4;//          0 0 0 1 0
  } else if ((sensors_data[0] == 0) && (sensors_data[1] == 0) && (sensors_data[2] == 1) && (sensors_data[3] == 0) && (sensors_data[4] == 0)) {
    error = 0;//          0 0 1 0 0
  } else if ((sensors_data[0] == 0) && (sensors_data[1] == 1) && (sensors_data[2] == 0) && (sensors_data[3] == 0) && (sensors_data[4] == 0)) {
    error = -4;//         0 1 0 0 0
  } else if ((sensors_data[0] == 1) && (sensors_data[1] == 0) && (sensors_data[2] == 0) && (sensors_data[3] == 0) && (sensors_data[4] == 0)) {
    error = -12.5;//         1 0 0 0 0
  } else if ((sensors_data[0] == 0) && (sensors_data[1] == 0) && (sensors_data[2] == 0) && (sensors_data[3] == 0) && (sensors_data[4] == 0)) {
    if (error == -12.5) {error = -27;} 
    else if(error == 12.5){error = 27;}
  }
}

/*function implement PID algorithm*/
void pid_calc()
{
  proportional = error; 
  if(integral>500||integral<-500) integral=0;
  integral += error;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = int(proportional * Kp + integral * Ki + derivative * Kd);
}


/*function used to calculate left,right speed of motorand the angle of servo*/
void calc_turn() {
    right_speed = init_speed + error_value*0.5;
    left_speed = init_speed  - error_value*0.5;
  if (right_speed > 255)  right_speed = 255;
  if (left_speed > 255)  left_speed = 255;
    
    turn_angle = pos + error_value*2.5;
  if (turn_angle > 180)  turn_angle = 180;
  if (turn_angle < 0)  turn_angle = 0;
}


/*function used to drive motor*/
void motor_drive()
{
  analogWrite(right_front_PWM, init_speed);
  analogWrite(left_front_PWM, init_speed);
  analogWrite(right_behind_PWM, right_speed);
  analogWrite(left_behind_PWM, left_speed);
}


/*function used to drive servo*/
void servo_drive()
{
  myservo.write(turn_angle);
}


void setup() {
  pinMode(Motor_Left_Pin1, OUTPUT);
  pinMode(Motor_Left_Pin2, OUTPUT);
  pinMode(Motor_Right_Pin1, OUTPUT);
  pinMode(Motor_Right_Pin2, OUTPUT);
  
  pinMode(left_front_PWM, OUTPUT);
  pinMode(right_front_PWM, OUTPUT);
  pinMode(left_behind_PWM, OUTPUT);
  pinMode(right_behind_PWM, OUTPUT);

  pinMode(Sensor_Left_PIN1, INPUT);
  pinMode(Sensor_Left_PIN2, INPUT);
  pinMode(Sensor_Middle_Pin, INPUT);
  pinMode(Sensor_Right_PIN1, INPUT);
  pinMode(Sensor_Right_PIN2, INPUT);

  
  /*make motor turn in forward direction*/
  digitalWrite(Motor_Left_Pin1, HIGH);
  digitalWrite(Motor_Left_Pin2, LOW);
  digitalWrite(Motor_Right_Pin1, HIGH);
  digitalWrite(Motor_Right_Pin2, LOW);
  
  /*attach servo object with Pin9*/
  myservo.attach(9, 500, 2500);
}

void loop() {
  sensors_data_read(); 
  judge();
  pid_calc(); 
  calc_turn();
  motor_drive(); 
  servo_drive();
}

