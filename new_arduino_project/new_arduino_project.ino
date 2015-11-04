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
#define BT_DEBUG
//#define MPU6050_DEBUG


//Motor variables
#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 6
int left_motor_speed = 0;
int right_motor_speed = 0;

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

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    analogWrite(RIGHT_MOTOR_PIN, 0);
    analogWrite(LEFT_MOTOR_PIN, 0);
    led = 0;
    //digitalWrite(RESET_PIN, HIGH);
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
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
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
    loop_start_time = millis();
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
    while (!mpuInterrupt && fifoCount < packetSize) {
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
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
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

void doCalculations() {
}

void setMotors() {
    
    //Maps variables left_motor_speed and right_motor speed to correct
    //range and writes them through PWM pins to motor controllers. Positive values
    //means the positive direction and opposite.
    int write_to_left = map(left_motor_speed,-127,128,0,255);
    int write_to_right = map(right_motor_speed, -127, 128, 0, 255);
    analogWrite(LEFT_MOTOR_PIN, write_to_left);
    analogWrite(RIGHT_MOTOR_PIN, write_to_right);
}

void loop() {
    loop_start_time = millis();
    read_bluetooth();
    read_gyro();    
    //doCalculations();
    setMotors();
    set_led();
    loop_used_time = millis() - loop_start_time;
    if (loop_used_time < LOOPTIME) {
        delay(LOOPTIME - loop_used_time);
    }
    

}
