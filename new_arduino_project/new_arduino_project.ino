#include <SoftwareSerial.h>


//Define Bluetooth variables
#define LED 13
#define BT_TX 8
#define BT_RX 9
#define BT_Baudrate 9600
#define GYRO_PIN 0
//Motor variables
#define LEFT_DIR_PIN 4
#define RIGHT_DIR_PIN 7
#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 6
//#define BT_DEBUG
//#define GYRO_DEBUG
//#define PID_DEBUG
//#define PROCESSING_PLOT
int sensorValue = 0;
int x = 0;


float left_motor_speed = 0;
float right_motor_speed = 0;

SoftwareSerial BT(BT_TX, BT_RX);
char speed1 = 1; //0 means backwards, 2 is front, 1 means stop
char dir1 = 1; //0 is left, 2 is right. 1 means no direction.
const char UP = 'w';
const char DOWN = 's';
const char LEFT = 'a';
const char RIGHT = 'd';
const char STOP = 'x';
char led = 0;
// Loop variables
const int LOOPTIME = 10; //In milliseconds
long loop_start_time;
long loop_used_time;

// PID -controller
float PID_integrator_sum = 0.0;

// Some values: K_p = 0.0336, K_i = 0.2688
float PID[] = {2, 0};  // K_p, K_i
float feedback;

//float gv[7];
float initial_angle;
float initial_angle_sum = 0;
float set_point = 0.0;  // default set_point is 0 degrees, _|_

int minVal = 265;
int maxVal = 402;



void setup() {
    //analogReference(EXTERNAL);
    analogWrite(RIGHT_MOTOR_PIN, 0);
    analogWrite(LEFT_MOTOR_PIN, 0);
    led = 0;
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    digitalWrite(LEFT_DIR_PIN, HIGH);
    digitalWrite(LED, LOW);
    delay(20);
    pinMode(LED, OUTPUT);
    BT.begin(BT_Baudrate);

    
    Serial.begin(57600);
  set_point = 0;
}

void read_bluetooth() {
    char val = '0';
    dir1 = 1;
    char old_speed = speed1;
    unsigned int N = 10;
    for(unsigned int i = 0; i < N; i++){
    if (BT.available()) {
        val = BT.read();
    }
    if(isalpha(val) && val != 'b'){
      
      //Serial.println("Is alpha!");
      if (val == UP) {
          if(old_speed == 0){
            speed1 = 1;
          }
         // Serial.println("UP");
          else speed1 = 2;
      }
      
      else if (val == LEFT) {
          //Serial.println("LEFT");
          dir1 = 0;
      }
      else if (val == RIGHT) {
          //Serial.println("RIGHT");
          dir1 = 2;
      }
      else if (val == DOWN) {
        if(old_speed == 2){
            speed1 = 1;
          }
          //Serial.println("DOWN");
          else speed1 = 0;
      }
      else if (val == STOP) {
        
            speed1 = 1;
          
          //Serial.println("DOWN");
          
      }
          
    }
    }
      #ifdef BT_DEBUG
          Serial.print("Val: ");
          Serial.print(val);
          Serial.print("\tSpeed: ");
          Serial.print(speed1, DEC);
          Serial.print("\tDir: ");
          Serial.println(dir1, DEC);
      #endif
    
}

void read_gyro() {
  int xRead = analogRead(GYRO_PIN);
  x = map(xRead, minVal, maxVal, -90, 90);
  #ifdef GYRO_DEBUG
    Serial.print("x: ");
    Serial.println(x);
  #endif
  #ifdef PROCESSING_PLOT
    
    //Serial.println(sensorValue);
  #endif
}

void set_led() {
    if (led) digitalWrite(LED, HIGH);
    else digitalWrite(LED, LOW);
    led = !led;
}

void pid() {
  float error = set_point - x;
  PID_integrator_sum = PID_integrator_sum + error;  // Add the angle to PID_integrator_sum
  if(PID_integrator_sum > 10000){
    PID_integrator_sum = 10000;
  }
  feedback = PID[0] * error + PID[1] * PID_integrator_sum;
  #ifdef PID_DEBUG
        if((debug_index % 10) == 0){
        Serial.print("Setpoint\t");
        Serial.print(set_point);
        Serial.print("\t");
        Serial.print("Output\t");
        Serial.print(x);
        Serial.print("\t");
        Serial.print("Error\t");
        Serial.print(error);
        Serial.print("Feedback\t");
        Serial.print("\t");
        Serial.println(feedback);
        debug_index = 0;
      }
        debug_index++;
#endif
}

void doCalculations() {
  pid();
  
  left_motor_speed = feedback;
  right_motor_speed = feedback;
}

void setMotors() {

    //Sets direction to motors 
    if(right_motor_speed < 0) {digitalWrite(RIGHT_DIR_PIN, LOW);
    //Serial.println("Taakke");
    }
    else{ digitalWrite(RIGHT_DIR_PIN, HIGH);
    //Serial.println("Ettee");
    }
    if(left_motor_speed < 0) digitalWrite(LEFT_DIR_PIN, LOW);
    else digitalWrite(LEFT_DIR_PIN, HIGH);
    
    //writes PWM value to motors
    analogWrite(LEFT_MOTOR_PIN, (int)abs(left_motor_speed));
    analogWrite(RIGHT_MOTOR_PIN, (int)abs(right_motor_speed));
}

void loop() {
    loop_start_time = micros();
    read_bluetooth();
    read_gyro();
    doCalculations();
    setMotors();
    set_led();
    loop_used_time = micros() - loop_start_time;
    Serial.println(x);
    if (loop_used_time < LOOPTIME) {
        delay(LOOPTIME - loop_used_time);
    }
}




