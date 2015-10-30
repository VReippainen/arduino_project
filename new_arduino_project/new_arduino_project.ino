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
#define MPU6050_DEBUG

SoftwareSerial BT(BT_TX, BT_RX);
char speed; //0 means backwards, 2 is front, 1 means stop
char dir; //0 is left, 2 is right. 1 means no direction.
char UP = '2';
char DOWN = '0';
char UPLEFT = 'B';
char UPRIGHT = 'C';
char DOWNLEFT = 'E';
char DOWNRIGHT = 'F';
char RESET = '6';
char led;
// Loop variables
int LOOPTIME = 10; //In milliseconds
int loop_start_time;
int loop_used_time;
//MPU variables, copied from
MPU6050 mpu(0x69); // AD0 low = 0x68
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
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
    led = 0;
    digitalWrite(RESET_PIN, HIGH);
    digitalWrite(LED, LOW);
    delay(20);
    pinMode(RESET_PIN, OUTPUT);
    pinMode(LED, OUTPUT);
    BT.begin(BT_Baudrate);

    Wire.begin(); //join I2C bus
    TWBR = 24;

    Serial.begin(115200);
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
    } else {
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
    char val;
    if (BT.available()) {
        val = BT.read();
    }
    if (val == UP) {
        speed = 2;
        dir = 1;
    }
    if (val == DOWN) {
        speed = 0;
        dir = 1;
    }
    if (val == UPLEFT) {
        speed = 2;
        dir = 0;
    }
    if (val == UPRIGHT) {
        speed = 2;
        dir = 2;
    }
    if (val == DOWNLEFT) {
        speed = 0;
        dir = 0;
    }
    if (val == DOWNRIGHT) {
        speed = 0;
        dir = 2;
    if (val == RESET) {
        Serial.println("Reset!");
        pinMode(RESET_PIN, LOW);
    }
    } else {
        speed = 1;
        dir = 1;
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

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

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
    }
}
void set_led(){
    if(led) digitalWrite(LED, HIGH);
    else digitalWrite(LED,LOW);
    led = !led;
}

    void loop() {
        read_bluetooth();
        //read_gyro();
        set_led();
        loop_used_time = millis()-loop_start_time;
        if(loop_used_time < LOOPTIME){
            delay(LOOPTIME-loop_used_time);
        }
        loop_start_time = millis();
        
    }
