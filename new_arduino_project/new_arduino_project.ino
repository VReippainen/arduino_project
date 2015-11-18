#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>


//Define Bluetooth variables
#define RESET_PIN 12
#define LED 13
#define BT_TX 8
#define BT_RX 9
#define BT_Baudrate 9600
//#define BT_DEBUG
//#define MPU6050_DEBUG
#define PID_DEBUG
int debug_index = 0;

//Motor variables
#define LEFT_DIR_PIN 4
#define RIGHT_DIR_PIN 7
#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 6

float left_motor_speed = 0;
float right_motor_speed = 0;

SoftwareSerial BT(BT_TX, BT_RX);
char speed1 = 1; //0 means backwards, 2 is front, 1 means stop
char dir1 = 1; //0 is left, 2 is right. 1 means no direction.
const char UP = 'w';
const char DOWN = 's';
const char LEFT = 'a';
const char RIGHT = 'd';
char led;
// Loop variables
const int LOOPTIME = 15; //In milliseconds
long loop_start_time;
long loop_used_time;
//MPU variables, copied from
MPU6050 mpu(0x69); // AD0 low = 0x68
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
int16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//MPU orientation
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];

// PID -controller
float PID_integrator_sum = 0.0;

// Some values: K_p = 0.0336, K_i = 0.2688
float PID[] = {5, 0};  // K_p, K_i
float feedback;

//float gv[7];
float initial_angle;
float initial_angle_sum = 0;
float set_point = 0.0;  // default set_point is 0 degrees, _|_

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    analogWrite(RIGHT_MOTOR_PIN, 0);
    analogWrite(LEFT_MOTOR_PIN, 0);
    led = 0;
    //digitalWrite(RESET_PIN, HIGH);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    digitalWrite(LEFT_DIR_PIN, HIGH);
    digitalWrite(LED, LOW);
    delay(20);
    pinMode(RESET_PIN, OUTPUT);
    pinMode(LED, OUTPUT);
    BT.begin(BT_Baudrate);

    Wire.begin(); //join I2C bus
    TWBR = 24;

    Serial.begin(57600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));



    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-10);
    mpu.setYGyroOffset(2);
    mpu.setZGyroOffset(0.7);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        //Serial.println(packetSize);
    }else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    unsigned int n = 7;
    for (unsigned int j=0; j<n; j++) {
    read_gyro();
    initial_angle_sum = (float) initial_angle_sum  + ypr[0]; //sum of the n readings left/right steer gyro
    //delay to do accel/gyro reads.
    delay (10); //10ms
  }
  initial_angle = (float) initial_angle_sum / n;  //initial front/back tilt gyro
  set_point = initial_angle;
}

void read_bluetooth() {
    char val = '0';
    if (BT.available()) {
        val = BT.read();
    }
    if(isalpha(val) && val != 'b'){
      
      Serial.println("Is alpha!");
      if (val == UP) {
         // Serial.println("UP");
          speed1 = 2;
          dir1 = 1;
      }
      
      else if (val == LEFT) {
          //Serial.println("LEFT");
          speed1 = 1;
          dir1 = 0;
      }
      else if (val == RIGHT) {
          //Serial.println("RIGHT");
          speed1 = 1;
          dir1 = 2;
      }
      else if (val == DOWN) {
          //Serial.println("DOWN");
          speed1 = 0;
          dir1 = 1;
      }
          
          

      else {
          speed1 = 1;
          dir1 = 1;
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
}

void read_gyro() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {  //WARNING: Comparison between int and uint
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
        // otherwise, check for DMP data ready interrupt (this should happen frequently)}
     else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();  //WARNING: Comparison between int and uint
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef MPU6050_DEBUG
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
#endif
        mpu.resetFIFO();
    }
}

void set_led() {
    if (led) digitalWrite(LED, HIGH);
    else digitalWrite(LED, LOW);
    led = !led;
}

void pid() {
  float error = set_point - (ypr[1]* 180 / M_PI);
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
        Serial.print(ypr[1] * 180 / M_PI);
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
//Serial.println(right_motor_speed < 0);
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
    loop_start_time = millis();
    read_bluetooth();
    read_gyro();
    doCalculations();
    setMotors();
    set_led();
    loop_used_time = millis() - loop_start_time;
    if (loop_used_time < LOOPTIME) {
        delay(LOOPTIME - loop_used_time);
    }
}
