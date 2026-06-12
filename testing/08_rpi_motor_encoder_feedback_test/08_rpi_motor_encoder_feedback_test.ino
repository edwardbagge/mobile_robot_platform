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
volatile long leftInvalidTransitions = 0;
volatile long rightInvalidTransitions = 0;
volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;

String lastCommand = "none";

unsigned long lastMotorCommandTime = 0;
unsigned long lastFeedbackTime = 0;

bool motorsRunning = false;

// ---------- Encoder interrupt functions ----------
uint8_t IRAM_ATTR readEncoderState(int pinA, int pinB)
{
  uint8_t a = digitalRead(pinA) ? 1 : 0;
  uint8_t b = digitalRead(pinB) ? 1 : 0;

  return (a << 1) | b;
}

int8_t IRAM_ATTR decodeQuadratureTransition(uint8_t previousState, uint8_t currentState)
{
  switch ((previousState << 2) | currentState)
  {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      return 1;

    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      return -1;

    default:
      return 0;
  }
}

void IRAM_ATTR onLeftEncoder()
{
  uint8_t currentState = readEncoderState(LEFT_A, LEFT_B);
  int8_t step = decodeQuadratureTransition(leftEncoderState, currentState);

  if (step != 0)
  {
    leftTicks += step;
  }
  else if (currentState != leftEncoderState)
  {
    leftInvalidTransitions++;
  }

  leftEncoderState = currentState;
}

void IRAM_ATTR onRightEncoder()
{
  uint8_t currentState = readEncoderState(RIGHT_A, RIGHT_B);
  int8_t step = decodeQuadratureTransition(rightEncoderState, currentState);

  if (step != 0)
  {
    rightTicks += step;
  }
  else if (currentState != rightEncoderState)
  {
    rightInvalidTransitions++;
  }

  rightEncoderState = currentState;
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

void getInvalidTransitionCounts(long &leftInvalid, long &rightInvalid)
{
  noInterrupts();
  leftInvalid = leftInvalidTransitions;
  rightInvalid = rightInvalidTransitions;
  interrupts();
}

void resetEncoderCounts()
{
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  leftInvalidTransitions = 0;
  rightInvalidTransitions = 0;
  leftEncoderState = readEncoderState(LEFT_A, LEFT_B);
  rightEncoderState = readEncoderState(RIGHT_A, RIGHT_B);
  interrupts();

  Serial.println("Encoder counts reset.");
}

void printEncoderFeedback()
{
  long currentLeftTicks;
  long currentRightTicks;
  long currentLeftInvalid;
  long currentRightInvalid;

  getEncoderCounts(currentLeftTicks, currentRightTicks);
  getInvalidTransitionCounts(currentLeftInvalid, currentRightInvalid);

  Serial.print("FEEDBACK | LEFT ticks = ");
  Serial.print(currentLeftTicks);
  Serial.print(" | RIGHT ticks = ");
  Serial.print(currentRightTicks);
  Serial.print(" | LEFT invalid = ");
  Serial.print(currentLeftInvalid);
  Serial.print(" | RIGHT invalid = ");
  Serial.println(currentRightInvalid);
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
  long runEndLeftTicks;
  long runEndRightTicks;
  long brakeEndLeftTicks;
  long brakeEndRightTicks;
  long startLeftInvalid;
  long startRightInvalid;
  long runEndLeftInvalid;
  long runEndRightInvalid;
  long brakeEndLeftInvalid;
  long brakeEndRightInvalid;
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
  getInvalidTransitionCounts(startLeftInvalid, startRightInvalid);

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

  getEncoderCounts(runEndLeftTicks, runEndRightTicks);
  getInvalidTransitionCounts(runEndLeftInvalid, runEndRightInvalid);
  stopMotors();
  delay(500);
  getEncoderCounts(brakeEndLeftTicks, brakeEndRightTicks);
  getInvalidTransitionCounts(brakeEndLeftInvalid, brakeEndRightInvalid);

  Serial.print("RESULT | ");
  Serial.print(label);
  Serial.print(" | LEFT run = ");
  Serial.print(runEndLeftTicks - startLeftTicks);
  Serial.print(" | RIGHT run = ");
  Serial.print(runEndRightTicks - startRightTicks);
  Serial.print(" | LEFT brake = ");
  Serial.print(brakeEndLeftTicks - runEndLeftTicks);
  Serial.print(" | RIGHT brake = ");
  Serial.print(brakeEndRightTicks - runEndRightTicks);
  Serial.print(" | LEFT total = ");
  Serial.print(brakeEndLeftTicks - startLeftTicks);
  Serial.print(" | RIGHT total = ");
  Serial.print(brakeEndRightTicks - startRightTicks);
  Serial.print(" | LEFT invalid run = ");
  Serial.print(runEndLeftInvalid - startLeftInvalid);
  Serial.print(" | RIGHT invalid run = ");
  Serial.print(runEndRightInvalid - startRightInvalid);
  Serial.print(" | LEFT invalid brake = ");
  Serial.print(brakeEndLeftInvalid - runEndLeftInvalid);
  Serial.print(" | RIGHT invalid brake = ");
  Serial.println(brakeEndRightInvalid - runEndRightInvalid);
}

void runAutomaticRepeatabilityTest()
{
  Serial.println();
  Serial.println("=== AUTOMATIC MOTOR ENCODER REPEATABILITY AND ISOLATION TEST ===");
  Serial.println("Robot must be lifted. Do not touch shafts during this test.");
  Serial.println("RESULT fields separate run ticks from post-brake ticks.");

  runTimedMotion("forward", TEST_SPEED, TEST_SPEED);
  runTimedMotion("backward", -TEST_SPEED, -TEST_SPEED);
  runTimedMotion("turn left", -TEST_SPEED, TEST_SPEED);
  runTimedMotion("turn right", TEST_SPEED, -TEST_SPEED);
  runTimedMotion("left forward only", TEST_SPEED, 0);
  runTimedMotion("left backward only", -TEST_SPEED, 0);
  runTimedMotion("right forward only", 0, TEST_SPEED);
  runTimedMotion("right backward only", 0, -TEST_SPEED);

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

  leftEncoderState = readEncoderState(LEFT_A, LEFT_B);
  rightEncoderState = readEncoderState(RIGHT_A, RIGHT_B);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), onRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), onRightEncoder, CHANGE);

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
