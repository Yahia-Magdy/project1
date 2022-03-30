#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "math.h"

#define IN1 33
#define IN2 25
#define IN3 26
#define IN4 27
#define ENA 32
#define ENB 14

#define IR1 16
#define IR2 17
#define IR3 18
#define IR4 19
#define IR5 21
#define BTN 4

int tryNumber = 1;
String path = "";
String simplePath = "";
int j = 0;

const int freq = 400;
const int res = 8;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float kP = 4;
float kD = 0.1;
float kI = 0;

int milliOld;
int milliNew;
float dt;

float yawTarget = 0;
float yawActual;
float yawError = 0;
float yawErrorOld;
float yawErrorChange;
float yawErrorSlope = 0;
float yawErrorArea = 0;
float correction;
float angle;
float desired_angle;
float set_point = 0;

int milli;

float rscale = 4;
float lscale = 4;
float spd = 72;

void setup() {
  ledcSetup(0, freq, res);
  ledcSetup(1, freq, res);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(BTN, INPUT_PULLUP);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(23, 22);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

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
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.println(F(")..."));
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

  //motion part
  ledcSetup(0, freq, res);
  ledcSetup(1, freq, res);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  milliNew = millis();
}

void loop() {
  if (tryNumber == 1) { //check for the try number, then run the appropriate code
    firstTry();
  }
  else {
    secondTry();
  }
}

bool cycle(float x, float y)
{
  x += 360, y += 360;
  if (abs(x - y) > (360 - max(x, y)) + min(x, y))
    return true;
  return false;
}

void turnRight()
{
  set_point += 90;
  angle = imu_6050();
  if (set_point > 180)
  {
    set_point -= 360;
  }
  while (cycle(angle, set_point) ? angle > set_point - 15 : angle < set_point - 15)
  {
    right();
    angle = imu_6050();
  }
  milliNew = millis();
  stopp();
}

void turnLeft()
{
  angle = imu_6050();
  set_point -= 90;
  if (set_point < -180)
  {
    set_point += 360;
  }
  while (cycle(angle, set_point) ? angle < set_point + 15 : angle > set_point + 15)
  {
    left();
    angle = imu_6050();
  }
  milliNew = millis();
  stopp();
}

void right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(0, spd);
  ledcWrite(1, spd);
}

void left()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(0, spd);
  ledcWrite(1, spd);
}

void forward()
{
  angle = imu_6050();
  float rcorrection = min(abs(correction / rscale), spd) * (correction / abs(correction ? correction : 1));
  float lcorrection = min(abs(correction / lscale), spd) * (correction / abs(correction ? correction : 1));
  Serial.print("Set Point: "); Serial.print(set_point);
  Serial.print(", Right Correction: "); Serial.print(rcorrection);
  Serial.print(", Left Correction: "); Serial.print(lcorrection);
  Serial.print(", Angle: "); Serial.print(angle);
  Serial.print(", Right Speed: "); Serial.print(spd - rcorrection);
  Serial.print(", Left Speed: "); Serial.print(spd + lcorrection);
  Serial.println();
  ledcWrite(0, spd - rcorrection); //right
  ledcWrite(1, spd + lcorrection); //left
}

void stopp()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

//returns the state of the line that is currently followed
String lineState() {
  //reading ir sensor array
  int ir5 = !digitalRead(IR1);
  int ir4 = !digitalRead(IR2);
  int ir3 = !digitalRead(IR3);
  int ir2 = !digitalRead(IR4);
  int ir1 = !digitalRead(IR5);

  //if only the left-most sensor detects blackness, then the robot is going off-course to the left & must be corrected
  if (!ir2 && !ir3 && !ir4 && ir1) {
    return "offLEFT";
  }
  //if only the right-most sensor detects blackness, then the robot is going off-course to the right & must be corrected
  if (!ir2 && !ir3 && !ir4 && ir5) {
    return "offRIGHT";
  }
  //if no sensors detect blackness, then the robot has either reached a dead end and must go back, or has gone off-course and can no longer be corrected
  //SPECIAL CASE: if this was the case right after "RIGHT" or "LEFT" is returned, then the robot is at a 90-degrees turn
  //SPECIAL CASE: if this was the case right after "LEFTandRIGHT" is returned, then the robot is at a T-intersection
  if (!ir2 && !ir3 && !ir4) {
    return "NONE";
  }
  //if both the left-most & right-most sensors detect blackness, then then are branches to the left & to the right
  //IMPORTANT: must move a little forward and call the function again to check for T-intersection if "NONE" is returned
  //SPECIAL CASE: if this was the case right after "LEFTandRIGHT", then the robot is at a solid rectangle & has reached the end of the maze
  if (ir1 && ir5) {
    return "LEFTandRIGHT";
  }
  //if the left-most detects blackness, then then is a branch to the left
  //IMPORTANT: must move a little forward and call the function again to check for 90-degrees turn if "NONE" is returned
  if (ir1) {
    return "LEFT";
  }
  //if the right-most detects blackness, then then is a branch to the right
  //IMPORTANT: must move a little forward and call the function again to check for 90-degrees turn if "NONE" is returned
  if (ir5) {
    return "RIGHT";
  }
  //if none of the above cases are satisfied, then the robot is moving correctly along a straight line
  return "STRAIGHT";
}

//takes an string of a sequence of movements and returns an string with a simplified sequence, convert string to char array before passing to the function
char* simplify(char* sequence) {
  //calculate size of the string
  int n = 0;
  while (sequence[n] != '\0') {
    n++;
  }

  //new string variable for simplified sequence
  char* simple = (char*) malloc(n);
  int c = 0; //increment for the new string

  int done = 1; //will stay = 1 if the sequence cannot be simplified more, else = 0
  int i; //increment variable for the old string
  for (i = 1; i < n - 1; i++) { //start from 1 till n-1 because a 'B' cannot be the first or last movement
    if (sequence[i] == 'B') {
      done = 0; //simplifications exist for the sequence
      if (sequence[i - 1] == 'L' && sequence[i + 1] == 'L') { //replace "LBL" with 'S'
        simple[c] = 'S';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'L' && sequence[i + 1] == 'R') { //replace "LBR" with 'B'
        simple[c] = 'B';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'L' && sequence[i + 1] == 'S') { //replace "LBS" with 'R'
        simple[c] = 'R';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'R' && sequence[i + 1] == 'L') { //replace "RBL" with 'B'
        simple[c] = 'B';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'S' && sequence[i + 1] == 'L') { //replace "SBL" with 'R'
        simple[c] = 'R';
        i += 2;
        c++;
        break;
      }
      else if (sequence[i - 1] == 'S' && sequence[i + 1] == 'S') { //replace "SBS" with 'B'
        simple[c] = 'B';
        i += 2;
        c++;
        break;
      }
      else { //add the original movement if cannot simplify this segment
        simple[c] = sequence[i - 1];
        c++;
      }
    }
    else { //add the original movement if cannot simplify this segment
      simple[c] = sequence[i - 1];
      c++;
    }
  }

  //return old string if it was not simplified
  if (done == 1) {
    return sequence;
  }

  //add trailing original movements
  for (; i < n; i++) {
    simple[c] = sequence[i];
    c++;
  }
  simple[c] = '\0'; //end the string with a null character

  return simplify(simple);
}

float imu_6050()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return 1.0;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  return ypr[0] * 180 / M_PI;
}

float PID_CONTROL(float yawActual, float yawTarget)
{
  milliOld = milliNew;
  milliNew = millis();
  dt = (milliNew - milliOld) / 1000.0;
  yawErrorOld = yawError;
  if(cycle(yawTarget, yawActual))
        yawError = 360 - yawTarget + yawActual;
    else
        yawError = yawTarget - yawActual;
  yawErrorChange = yawError - yawErrorOld;
  yawErrorSlope = yawErrorChange / dt;
  yawErrorArea = yawErrorArea + yawError * dt;

  correction = kP * yawError + kD * yawErrorSlope + kI * yawErrorArea;
 
  return correction;
}

void firstTry() {
  String state = lineState();
  if (state ==  "offLEFT") {
    //((correct course to the right))
    right();
  }
  else if (state ==  "offRIGHT") {
    //((correct course to the left))
    left();
  }
  else if (state ==  "NONE") {
    //((move forward to check for special cases))
    milli = millis(); while(millis() < milli + 1000) forward();
    String state2 = lineState();
    if (state2 == "NONE") { //Dead End
      //((turn back because dead end reached))
      turnRight();
      turnRight();
      path += "B";
    }
    else if (state2 == "STRAIGHT") { //Split Line
      //((keep moving forward as line is only split and will continue))
      forward();
    }
  }
  else if (state ==  "LEFTandRIGHT") {
    //((move a little bit forward to check for special cases))
    milli = millis(); while(millis() < milli + 1000) forward();
    String state2 = lineState();
    if (state2 == "NONE") { //T-Intersection
      //((turn left as priority is to the left way))
      turnLeft();
      path += "L";
    }
    else if (state2 == "STRAIGHT") { //Cross Intersection
      //((turn left as priority is to the left way))
      turnLeft();
      path += "L";
    }
    else if (state2 == "LEFTandRIGHT") { //Solid Rectangle (end of the maze)
      //((stop all movements))
      stopp();
      //first try finished, now to the second try
      tryNumber = 2;
      int len = path.length() + 1;
      char arr[len];
      for (int i = 0; i < len; i++) {
        arr[i] = path[i];
      }
      char* buff = (char*) malloc(32);
      buff = simplify(arr);
      simplePath = String(buff);
      while (!digitalRead(BTN)) {}
    }
  }
  else if (state ==  "LEFT") {
    //((move a little bit forward to check for special cases))
    milli = millis(); while(millis() < milli + 1000) forward();
    String state2 = lineState();
    if (state2 == "NONE") { //90-Degrees Turn
      //((turn left as there are no other way))
      turnLeft();
      path += "L";
    }
    else if (state2 == "STRAIGHT") { //T-Intersection
      //((turn left as priority is to the left way))
      turnLeft();
      path += "L";
    }
  }
  else if (state ==  "RIGHT") {
    //((move a little bit forward to check for special cases))
    milli = millis(); while(millis() < milli + 1000) forward();
    String state2 = lineState();
    if (state2 == "NONE") { //90-Degrees Turn
      //((turn right as there are no other way))
      turnRight();
      path += "R";
    }
    else if (state2 == "STRAIGHT") { //T-Intersection
      //((keep moving forward as priority is to the left way))
      turnRight();
      path += "S";
    }
  }
  else if (state ==  "STRAIGHT") {
    //((keep moving forward))
    forward();
  }
}

void secondTry() {
  String state = lineState();
  if (state ==  "offLEFT") {
    //((correct course to the right))
    right();
  }
  else if (state ==  "offRIGHT") {
    //((correct course to the left))
    left();
  }
  else if (state ==  "NONE") {
    //((keep moving forward as line is only split and will continue))
    forward();
  }
  else if (state ==  "STRAIGHT") {
    //((keep moving forward))
    forward();
  }
  else {
    //((move a little bit forward to position the robot in the center of the line after turning))
    char currentMove = simplePath[j];
    if (currentMove == 0) { //if we reached the end of the path array, then the end of the maze is reached
      //((stop all movements))
      stopp();
    }
    j++;
    switch (currentMove) {
      case 'L':
        turnLeft();
        break;
      case 'R':
        turnRight();
        break;
      case 'S':
        milli = millis(); while(millis() < milli + 1000) forward();
        break;
    }
  }
}
