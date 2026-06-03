// robot_base_esp32.ino
// ESP32 base firmware for differential-drive control from ROS 2 over USB serial.
//
// Serial protocol, 115200 baud:
//   M <left_pwm> <right_pwm>   signed PWM command, range -255..255
//   X                          brake/stop immediately
//   Z                          reset encoder counts
//   S                          print status
//
// Feedback format is kept compatible with earlier tests:
//   FEEDBACK | LEFT ticks = ... | RIGHT ticks = ... | LEFT invalid = ... | RIGHT invalid = ...

// ---------- Motor pins ----------
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
const int BRAKE_PWM = 255;
const int MAX_PWM = 255;

// ---------- Timing ----------
const unsigned long COMMAND_TIMEOUT_MS = 300;
const unsigned long FEEDBACK_INTERVAL_MS = 50;

// ---------- Encoder counters ----------
volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile long leftInvalidTransitions = 0;
volatile long rightInvalidTransitions = 0;
volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;

int lastLeftPwm = 0;
int lastRightPwm = 0;
bool commandActive = false;

unsigned long lastCommandTime = 0;
unsigned long lastFeedbackTime = 0;

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
int clampPwm(int pwm)
{
  if (pwm > MAX_PWM)
  {
    return MAX_PWM;
  }
  if (pwm < -MAX_PWM)
  {
    return -MAX_PWM;
  }
  return pwm;
}

void setLeftMotor(int speedValue)
{
  speedValue = clampPwm(speedValue);

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

void setRightMotor(int speedValue)
{
  speedValue = clampPwm(speedValue);

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

void brakeLeftMotor()
{
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, HIGH);
  ledcWrite(LEFT_EN, BRAKE_PWM);
}

void brakeRightMotor()
{
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, HIGH);
  ledcWrite(RIGHT_EN, BRAKE_PWM);
}

void releaseMotors()
{
  setLeftMotor(0);
  setRightMotor(0);
  lastLeftPwm = 0;
  lastRightPwm = 0;
  commandActive = false;
}

void brakeMotors()
{
  brakeLeftMotor();
  brakeRightMotor();
  lastLeftPwm = 0;
  lastRightPwm = 0;
  commandActive = false;

  Serial.println("Motors stopped with active brake.");
}

void applyMotorCommand(int leftPwm, int rightPwm)
{
  leftPwm = clampPwm(leftPwm);
  rightPwm = clampPwm(rightPwm);
  bool commandChanged = (leftPwm != lastLeftPwm || rightPwm != lastRightPwm);

  setLeftMotor(leftPwm);
  setRightMotor(rightPwm);

  lastLeftPwm = leftPwm;
  lastRightPwm = rightPwm;
  commandActive = (leftPwm != 0 || rightPwm != 0);
  lastCommandTime = millis();

  if (commandChanged)
  {
    Serial.print("CONTROL | LEFT pwm = ");
    Serial.print(leftPwm);
    Serial.print(" | RIGHT pwm = ");
    Serial.println(rightPwm);
  }
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

void printStatus()
{
  Serial.println();
  Serial.println("=== ROBOT BASE STATUS ===");
  Serial.print("LEFT pwm = ");
  Serial.println(lastLeftPwm);
  Serial.print("RIGHT pwm = ");
  Serial.println(lastRightPwm);
  Serial.print("Command active = ");
  Serial.println(commandActive ? "YES" : "NO");
  Serial.print("Command timeout ms = ");
  Serial.println(COMMAND_TIMEOUT_MS);
  Serial.print("Feedback interval ms = ");
  Serial.println(FEEDBACK_INTERVAL_MS);
  printEncoderFeedback();
  Serial.println();
}

// ---------- Serial command handler ----------
bool parseMotorCommand(const String &command, int &leftPwm, int &rightPwm)
{
  int firstSpace = command.indexOf(' ');
  if (firstSpace < 0)
  {
    return false;
  }

  int secondSpace = command.indexOf(' ', firstSpace + 1);
  if (secondSpace < 0)
  {
    return false;
  }

  String leftPart = command.substring(firstSpace + 1, secondSpace);
  String rightPart = command.substring(secondSpace + 1);
  leftPart.trim();
  rightPart.trim();

  leftPwm = leftPart.toInt();
  rightPwm = rightPart.toInt();
  return true;
}

void handleCommand(String command)
{
  command.trim();

  if (command.length() == 0)
  {
    return;
  }

  int commandStart = -1;
  for (int i = 0; i < command.length(); i++)
  {
    char c = command.charAt(i);
    if (c == 'M' || c == 'm' || c == 'X' || c == 'x' ||
        c == 'Z' || c == 'z' || c == 'S' || c == 's')
    {
      commandStart = i;
      break;
    }
  }

  if (commandStart < 0)
  {
    Serial.print("Ignored serial noise: ");
    Serial.println(command);
    return;
  }

  if (commandStart > 0)
  {
    command = command.substring(commandStart);
    command.trim();
  }

  char commandType = command.charAt(0);

  if (commandType == 'M' || commandType == 'm')
  {
    int leftPwm = 0;
    int rightPwm = 0;

    if (parseMotorCommand(command, leftPwm, rightPwm))
    {
      applyMotorCommand(leftPwm, rightPwm);
    }
    else
    {
      Serial.print("Invalid motor command: ");
      Serial.println(command);
    }
  }
  else if (commandType == 'X' || commandType == 'x')
  {
    brakeMotors();
    printEncoderFeedback();
  }
  else if (commandType == 'Z' || commandType == 'z')
  {
    resetEncoderCounts();
  }
  else if (commandType == 'S' || commandType == 's')
  {
    printStatus();
  }
  else
  {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

// ---------- Periodic checks ----------
void checkCommandTimeout()
{
  if (commandActive && millis() - lastCommandTime >= COMMAND_TIMEOUT_MS)
  {
    Serial.println("Command timeout reached.");
    brakeMotors();
    printEncoderFeedback();
  }
}

void checkFeedbackPrint()
{
  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL_MS)
  {
    printEncoderFeedback();
    lastFeedbackTime = millis();
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

  releaseMotors();
  resetEncoderCounts();

  Serial.println();
  Serial.println("=== ROBOT BASE ESP32 READY ===");
  Serial.println("Commands:");
  Serial.println("M <left_pwm> <right_pwm>");
  Serial.println("X = brake stop");
  Serial.println("Z = reset encoder counts");
  Serial.println("S = status");
  Serial.println();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }

  checkCommandTimeout();
  checkFeedbackPrint();
}
