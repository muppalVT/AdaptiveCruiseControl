#include <Stepper.h>
#include <Wire.h>
#include <TFMini.h>
#include <FastPID.h>
#include <SoftwareSerial.h>

#define PIN_INPUT     A0
#define PIN_SETPOINT  A1   
#define PIN_OUTPUT    9
#define STEPS 200   

float Kp = 0.1, Ki = 0.5, Kd = 0.1, Hz = 10;
int output_bits = 8;
bool output_signed = false;

byte number=0;

SoftwareSerial mySerial(10, 11);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;
int vhCH = 1143;
int vhAP = 1016;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

Stepper stepper(STEPS, 2, 3); // Pin 2 connected to DIRECTION & Pin 3 connected to STEP Pin of Driver
#define motorInterfaceType 1

void setup() {
  // put your setup code here, to run once:
  
  // TFMini setup
  Serial.begin(115200);
  while (!Serial);
  Serial.println ("Initializing...");
  mySerial.begin(TFMINI_BAUDRATE);
  tfmini.begin(&mySerial);
  //Accelerometer setup
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  stepper.setSpeed(1000);            // Set motor speed 
}

void loop() {
  //TF Mini reading code here:
  uint16_t vDist = tfmini.getDistance();
  uint16_t tfStr = tfmini.getRecentSignalStrength();
  if (vDist < vhCH) {
    // Send command to rasp pi to check if vehicel
    if (ADpinfor Rasp == vehDetected) {
      tol = .1;
      while (tol >= diff) {
        int feedback = readAccel(); // read accelerometer;
        int setpoint = calcAccel(); //Calc acceleration of lead vehicle
        if (feedback < (setpoint * 0.9)) {
          stepper.step(s1);
          myPID.clear();
        }
        else {
          stepper.step(myPID.step(setpoint, feedback));
        }
      }
    }
  }
}

int readAccel(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_Hs)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  return AccX
}      

int calcAccel(){
  uint16_t dist0 = tfmini.getDistance();
  delay(100)
  uint16_t dist1 = tfmini.getDistance();
  vLead1=(dist1-dist0)/.1;
  delay(100)
  uint16_t dist2 = tfmini.getDistance();
  delay(100)
  uint16_t dist3 = tfmini.getDistance();
  vLead2=(dist3-dist2)/.1;

  a=(vLead2-vLead1)/.1;
  return a
}
