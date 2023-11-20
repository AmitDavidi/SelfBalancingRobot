#include <Wire.h>
#include <Encoder.h>
#include <MPU6050_tockn.h>

const int UPDATE_INTERVAL_MS = 10;

#define COUNTS_PER_REVOLUTION 11   // 11 ticks per motro rev
#define GEAR_RATIO 85.6              // IDK really - roughly measured by counting the ticks after rotating the wheel one meter and X = NUMBERMEASURED / (COUNTS_PER_REVOLUTION*ROTATIONS_PER_METER)
#define WHEEL_DIAMETER_IN_mm 100         // 100mm = 10cm diameter
#define WHEELS_DISTANCE 160        // 16 cm distance between wheels = 160mm
#define ROTATIONS_PER_METER 3.1847  // 1 / (pi)*(WHEEL_DIAMETER_IN_mm/1000)
#define ENCODER_TICKS_PER_ROTATION GEAR_RATIO*COUNTS_PER_REVOLUTION*
#define ENCODER_TICKS_PER_METER ENCODER_TICKS_PER_ROTATION*ROTATIONS_PER_METER

float encoder2dist = (WHEEL_DIAMETER_IN_mm*3.14/(COUNTS_PER_REVOLUTION*GEAR_RATIO)) / 1000.0;  // conversition of encoder pulses to distance in m


Encoder EncoderA(3, 5);
Encoder EncoderB(2, 4);
MPU6050 mpu6050(Wire);

int IN1_FIRST = 10;
int IN1_SEC = 11;

int IN2_FIRST = 6;
int IN2_SEC = 9;

// time
unsigned long prevTime = 0, currTime = 0;
float deltaTime;

// todo remove
double desiredSpeed = 0.0;

// control variables - todo - move to object
double errorLeft = 0.0, prevErrorLeft = 0.0, integralLeft = 0.0, controlLeft = 0.0;
double errorRight = 0.0, prevErrorRight = 0.0, integralRight = 0.0, controlRight = 0.0;

// Wheel variables Todo move to object
float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;
float lastleftWheelSpeed = 0.0, lastrightWheelSpeed = 0.0;

// Robot position estimation
float posx;
float posy;
float theta = 0.0; 


void odometry(){
    //encoder read
    int16_t countsRight = EncoderA.readAndReset();
    int16_t countsLeft = EncoderB.readAndReset();
    Serial.print("CountsLeft/sec:");
    Serial.print(countsLeft/deltaTime);
    Serial.print(",");


    // 1 m/s -> 3000 ticks/sec ?
    float dx_1 = countsRight*encoder2dist; // [m]
    float dx_2 = countsLeft*encoder2dist; // [m]
    
    float d_theta = float(dx_1-dx_2)/(WHEELS_DISTANCE / 1000.0);
    posx += cos(theta+d_theta/2)*(dx_1+dx_2)/2;
    posy += sin(theta+d_theta/2)*(dx_1+dx_2)/2;
    
    theta += d_theta;

    leftWheelSpeed = float(dx_2 / deltaTime); 
    rightWheelSpeed = float(dx_1 / deltaTime); 


    leftWheelSpeed = LowPassFilter(leftWheelSpeed, lastleftWheelSpeed, 0.5);
    rightWheelSpeed = LowPassFilter(rightWheelSpeed, lastrightWheelSpeed, 0.5);

    lastrightWheelSpeed = rightWheelSpeed;
    lastleftWheelSpeed = leftWheelSpeed;
}


double pidController(double error, double prevError, double dt) {
  double Kp = 110.0, Ki = 10.0, Kd = 0.0, outMin = -255.0, outMax = 255.0;

  integralLeft += Ki * error * dt;
  double proportional = Kp * error;
  double derivative = Kd * (error - prevError) / dt;
  double output = proportional + integralLeft + derivative;

  // Limit the output to the minimum and maximum values
  if (output > outMax) {
    output = outMax;
  } else if (output < outMin) {
    output = outMin;
  }

  return output;  // Return the output
}

double LowPassFilter(double currentReading, double previousReading, double alpha) {
  if (abs(currentReading - previousReading) > 60000) {
    currentReading = previousReading;  // bad ENCODER_COUNTS read, integer overflow, skip this reading.
  }
  double filteredReading = double(alpha * currentReading + (1.0 - alpha) * previousReading);
  return filteredReading;
}

// TODO revisit
void SetLeftMotorSpeed(int speed) {
    // Adjust motor speed and direction
    if(speed > 0 ) {
      analogWrite(IN2_FIRST, speed);
      analogWrite(IN2_SEC, LOW);
    } else {
      analogWrite(IN2_FIRST, LOW);
      analogWrite(IN2_SEC, -1*speed);
    }
}

// TODO revisit
void SetRightMotorSpeed(int speed) {
    // Adjust motor speed and direction
    if(speed > 0 ) {
      analogWrite(IN1_FIRST, speed);
      analogWrite(IN1_SEC, LOW);
    } else {
      analogWrite(IN1_FIRST, LOW);
      analogWrite(IN1_SEC, -1*speed);
    }
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  // // init DC motors
  pinMode(IN1_FIRST, OUTPUT);
  pinMode(IN2_FIRST, OUTPUT);

  pinMode(IN1_SEC, OUTPUT);
  pinMode(IN2_SEC, OUTPUT);

  digitalWrite(IN1_FIRST, LOW);
  digitalWrite(IN2_FIRST, LOW);

  digitalWrite(IN1_SEC, LOW);
  digitalWrite(IN2_SEC, LOW);

  mpu6050.begin();

  Serial.println("start...");
  delay(1);
}

void loop() {

  // // ------ Plot wheel position ------
  // // Calculate time difference
  currTime = millis();
  deltaTime = (currTime - prevTime) / 1000.0;  // convert to seconds

  if (deltaTime > 0.01) {
    // get desired position if available
    if (Serial.available() > 0)
    {
      String inputString = Serial.readStringUntil('\n');
      desiredSpeed = inputString.toDouble();
    }
  
    // // MPU 6050
    // mpu6050.update();
    // Serial.print(mpu6050.getAngleX());
    // Serial.print(" ");
    // Serial.print(mpu6050.getAngleY());
    // Serial.print(" ");
    // Serial.print(mpu6050.getAngleZ());
    // Serial.println();


    odometry();
    // desiredSpeed from P2P \ balancer
    errorLeft = double(desiredSpeed - leftWheelSpeed);

    controlLeft = pidController(errorLeft, prevErrorLeft, deltaTime); // revisit turn into an Object 

    //Adjust motor speed and direction
    SetLeftMotorSpeed(controlLeft);
    

    // printsssss
    Serial.print("Signal to Left:");
    Serial.print(controlLeft);
    Serial.print(",");
    Serial.print("error Left:");
    Serial.print(errorLeft);
    Serial.print(",");
    Serial.print("Speed Right:");
    Serial.print(rightWheelSpeed);
    Serial.print(",");
    Serial.print("Speed Left:");
    Serial.print(leftWheelSpeed);
    Serial.print(",");
    Serial.print("desired Speed:");
    Serial.println(desiredSpeed);

    // Update previous values
    prevErrorLeft = errorLeft; // mvoe to object
    prevTime = currTime;
  }
}
