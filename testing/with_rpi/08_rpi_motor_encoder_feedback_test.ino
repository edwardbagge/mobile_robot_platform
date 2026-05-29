// 08_rpi_motor_encoder_feedback_test.ino
// Adafruit ESP32 Feather HUZZAH + L298N motor driver + quadrature encoders
// Raspberry Pi 5 sends motor commands to ESP32 over USB serial
// Serial baud: 115200

// ---------- Motor pins ----------
// Same mapping as the working 03_motor_encoder_combined_test.ino:
//
// Right motor = L298N OUT1 / OUT2, controlled by IN1 / IN2 / ENA
// Left motor  = L298N OUT3 / OUT4, controlled by IN3 / IN4 / ENB

const int RIGHT_IN1 = 13;
const int RIGHT_IN2 = 21;
const int RIGHT_EN  = 14;

const int LEFT_IN1  = 22;
const int LEFT_IN2  = 23;
const int LEFT_EN   = 32;

// ---------- Encoder pins ----------
const int LEFT_A  = 27;
const int LEFT_B  = 33;
const int RIGHT_A = 25;
const int RIGHT_B = 26;

// ---------- PWM settings ----------
const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;
const int TEST_SPEED = 180;
const int BRAKE_PWM = 255;

// ---------- Timing ----------
const unsigned long MOTOR_TIMEOUT = 1000;     // ms
const unsigned long FEEDBACK_INTERVAL = 250;  // ms
const unsigned long AUTO_RUN_TIME = 1000;     // ms
const unsigned long AUTO_SETTLE_TIME = 2000;  // ms

// ---------- Encoder counters ----------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

String lastCommand = "none";

unsigned long lastMotorCommandTime = 0;
unsigned long lastFeedbackTime = 0;

bool motorsRunning = false;

// ---------- Encoder interrupt functions ----------
void IRAM_ATTR onLeftA()
{
  int a = digitalRead(LEFT_A);
  int b = digitalRead(LEFT_B);

  if (a == b)
  {
    leftTicks++;
  }
  else
  {
    leftTicks--;
  }
}

void IRAM_ATTR onRightA()
{
  int a = digitalRead(RIGHT_A);
  int b = digitalRead(RIGHT_B);

  if (a == b)
  {
    rightTicks++;
  }
  else
  {
    rightTicks--;
  }
}

// ---------- Motor control ----------
void setLeftMotor(int speedValue)
{
  // Left motor uses IN3 / IN4 / ENB in the working Test 3 wiring.

  if (speedValue > 0)
  {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    ledcWrite(LEFT_EN, speedValue);
  }
  else if (speedValue < 0)
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    ledcWrite(LEFT_EN, -speedValue);
  }
  else
  {
    ledcWrite(LEFT_EN, 0);
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
}

void brakeLeftMotor()
{
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, HIGH);
  ledcWrite(LEFT_EN, BRAKE_PWM);
}

void setRightMotor(int speedValue)
{
  // Right motor uses IN1 / IN2 / ENA in the working Test 3 wiring.

  if (speedValue > 0)
  {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    ledcWrite(RIGHT_EN, speedValue);
  }
  else if (speedValue < 0)
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    ledcWrite(RIGHT_EN, -speedValue);
  }
  else
  {
    ledcWrite(RIGHT_EN, 0);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
}

void brakeRightMotor()
{
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, HIGH);
  ledcWrite(RIGHT_EN, BRAKE_PWM);
}

// ---------- Encoder helper functions ----------
void getEncoderCounts(long &leftCount, long &rightCount)
{
  noInterrupts();
  leftCount = leftTicks;
  rightCount = rightTicks;
  interrupts();
}

void resetEncoderCounts()
{
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  Serial.println("Encoder counts reset.");
}

void printEncoderFeedback()
{
  long currentLeftTicks;
  long currentRightTicks;

  getEncoderCounts(currentLeftTicks, currentRightTicks);

  Serial.print("FEEDBACK | LEFT ticks = ");
  Serial.print(currentLeftTicks);
  Serial.print(" | RIGHT ticks = ");
  Serial.println(currentRightTicks);
}

// ---------- Movement functions ----------
void stopMotors()
{
  brakeLeftMotor();
  brakeRightMotor();

  motorsRunning = false;
  lastCommand = "stop";

  Serial.println("Motors stopped with active brake.");
  printEncoderFeedback();
}

void releaseMotors()
{
  setLeftMotor(0);
  setRightMotor(0);
  Serial.println("Motor outputs released.");
}

void driveForward()
{
  setLeftMotor(TEST_SPEED);
  setRightMotor(TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastFeedbackTime = millis();
  lastCommand = "forward";

  Serial.println("Motor command: forward");
  printEncoderFeedback();
}

void driveBackward()
{
  setLeftMotor(-TEST_SPEED);
  setRightMotor(-TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastFeedbackTime = millis();
  lastCommand = "backward";

  Serial.println("Motor command: backward");
  printEncoderFeedback();
}

void turnLeft()
{
  setLeftMotor(-TEST_SPEED);
  setRightMotor(TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastFeedbackTime = millis();
  lastCommand = "turn left";

  Serial.println("Motor command: turn left");
  printEncoderFeedback();
}

void turnRight()
{
  setLeftMotor(TEST_SPEED);
  setRightMotor(-TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastFeedbackTime = millis();
  lastCommand = "turn right";

  Serial.println("Motor command: turn right");
  printEncoderFeedback();
}

void runTimedMotion(const char *label, int leftSpeed, int rightSpeed)
{
  long startLeftTicks;
  long startRightTicks;
  long endLeftTicks;
  long endRightTicks;
  unsigned long startTime;
  unsigned long lastPrintTime;

  Serial.println();
  Serial.print("AUTO STEP | ");
  Serial.println(label);

  stopMotors();
  delay(AUTO_SETTLE_TIME);
  resetEncoderCounts();
  delay(100);

  getEncoderCounts(startLeftTicks, startRightTicks);

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
  motorsRunning = true;
  lastCommand = label;
  startTime = millis();
  lastPrintTime = startTime;

  Serial.print("AUTO RUN | ");
  Serial.print(label);
  Serial.print(" | left PWM = ");
  Serial.print(leftSpeed);
  Serial.print(" | right PWM = ");
  Serial.println(rightSpeed);

  while (millis() - startTime < AUTO_RUN_TIME)
  {
    if (millis() - lastPrintTime >= FEEDBACK_INTERVAL)
    {
      printEncoderFeedback();
      lastPrintTime = millis();
    }
    delay(5);
  }

  stopMotors();
  delay(500);
  getEncoderCounts(endLeftTicks, endRightTicks);

  Serial.print("RESULT | ");
  Serial.print(label);
  Serial.print(" | LEFT delta ticks = ");
  Serial.print(endLeftTicks - startLeftTicks);
  Serial.print(" | RIGHT delta ticks = ");
  Serial.println(endRightTicks - startRightTicks);
}

void runAutomaticRepeatabilityTest()
{
  Serial.println();
  Serial.println("=== AUTOMATIC MOTOR ENCODER REPEATABILITY TEST ===");
  Serial.println("Robot must be lifted. Do not touch shafts during this test.");

  runTimedMotion("forward", TEST_SPEED, TEST_SPEED);
  runTimedMotion("backward", -TEST_SPEED, -TEST_SPEED);
  runTimedMotion("turn left", -TEST_SPEED, TEST_SPEED);
  runTimedMotion("turn right", TEST_SPEED, -TEST_SPEED);

  stopMotors();
  Serial.println("=== AUTOMATIC TEST COMPLETE ===");
  Serial.println();
}

// ---------- Status ----------
void printStatus()
{
  long currentLeftTicks;
  long currentRightTicks;

  getEncoderCounts(currentLeftTicks, currentRightTicks);

  Serial.println();
  Serial.println("=== STATUS ===");

  Serial.print("Last command = ");
  Serial.println(lastCommand);

  Serial.print("Motors running = ");
  Serial.println(motorsRunning ? "YES" : "NO");

  Serial.print("Test speed PWM = ");
  Serial.println(TEST_SPEED);

  Serial.print("Motor timeout ms = ");
  Serial.println(MOTOR_TIMEOUT);

  Serial.print("Feedback interval ms = ");
  Serial.println(FEEDBACK_INTERVAL);

  Serial.print("LEFT ticks = ");
  Serial.println(currentLeftTicks);

  Serial.print("RIGHT ticks = ");
  Serial.println(currentRightTicks);

  Serial.println();
}

// ---------- Serial command handler ----------
void handleCommand(String command)
{
  command.trim();

  if (command.length() == 0)
  {
    return;
  }

  Serial.print("Received command: ");
  Serial.println(command);

  if (command == "f")
  {
    driveForward();
  }
  else if (command == "b")
  {
    driveBackward();
  }
  else if (command == "l")
  {
    turnLeft();
  }
  else if (command == "r")
  {
    turnRight();
  }
  else if (command == "x")
  {
    stopMotors();
  }
  else if (command == "s")
  {
    printStatus();
  }
  else if (command == "z")
  {
    resetEncoderCounts();
  }
  else if (command == "a")
  {
    runAutomaticRepeatabilityTest();
  }
  else
  {
    Serial.println("Unknown command.");
  }
}

// ---------- Periodic checks ----------
void checkMotorTimeout()
{
  if (motorsRunning)
  {
    if (millis() - lastMotorCommandTime >= MOTOR_TIMEOUT)
    {
      Serial.println("Motor timeout reached.");
      stopMotors();
    }
  }
}

void checkFeedbackPrint()
{
  if (motorsRunning)
  {
    if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL)
    {
      printEncoderFeedback();
      lastFeedbackTime = millis();
    }
  }
}

// ---------- Setup ----------
void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), onRightA, CHANGE);

  ledcAttach(LEFT_EN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(RIGHT_EN, PWM_FREQUENCY, PWM_RESOLUTION);

  stopMotors();

  Serial.println();
  Serial.println("=== RPI MOTOR ENCODER FEEDBACK TEST ===");
  Serial.println("ESP32 ready.");
  Serial.println("Keep robot lifted from ground for first test.");
  Serial.println("Commands:");
  Serial.println("f = forward");
  Serial.println("b = backward");
  Serial.println("l = turn left");
  Serial.println("r = turn right");
  Serial.println("x = stop");
  Serial.println("s = status");
  Serial.println("z = reset encoder counts");
  Serial.println("a = automatic repeatability test");
  Serial.println();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }

  checkFeedbackPrint();
  checkMotorTimeout();
}
