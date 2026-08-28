// robot_base_esp32.ino
// ESP32 firmware for the differential-drive robot base.
//
// This sketch accepts motion commands from the Raspberry Pi over USB serial,
// drives the left and right motor H-bridges, and reports encoder feedback for
// closed-loop control and odometry.
//
// Serial protocol, 115200 baud:
//   M <left_pwm> <right_pwm>   signed PWM command, range -255..255
//   X                          brake/stop immediately
//   Z                          reset encoder counts
//   S                          print status
//

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
const int PWM_FREQUENCY = 1000; // Motor PWM is switched at 1 kHz
const int PWM_RESOLUTION = 8;   // 8-bit PWM gives output values from 0 to 255
const int BRAKE_PWM = 255;      // PWM value used when the motors are actively braked
const int MAX_PWM = 255;        // Highest motor PWM value that can be commanded

// ---------- Timing ----------
const unsigned long COMMAND_TIMEOUT_MS = 300;  // Stop motors if no command is received after COMMAND_TIMEOUT_MS milliseconds
const unsigned long FEEDBACK_INTERVAL_MS = 50; // Send encoder feedback every FEEDBACK_INTERVAL_MS milliseconds
const size_t SERIAL_COMMAND_BUFFER_SIZE = 64;  // Maximum serial command length

// ---------- Encoder counters ----------

volatile int64_t leftTicks = 0;               // Accumulated encoder counts for the left side
volatile int64_t rightTicks = 0;              // Accumulated encoder counts for the right side
volatile int64_t leftInvalidTransitions = 0;  // Count encoder state changes that do not match a valid quadrature sequence
volatile int64_t rightInvalidTransitions = 0; // Same as above but for the right encoder
volatile uint8_t leftEncoderState = 0;        // Store the previous 2-bit encoder state for detecting direction and valid transitions
volatile uint8_t rightEncoderState = 0;       // Same as above but for the right encoder

int lastLeftPwm = 0;        // Most recent PWM command sent to left motor
int lastRightPwm = 0;       // most recent PWM command sent to right motor
bool commandActive = false; // Tells the program whether it currently has an active motion command

unsigned long lastCommandTime = 0;  // Store when the last command message occurred
unsigned long lastFeedbackTime = 0; // Store when the last feedback message occurred

char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE]; // Temporary storage for the text arriving over USB serial
size_t serialCommandLength = 0;                       // How many characters are currently stored
bool serialCommandOverflow = false;                   // Becomes true if an incoming command is too long for the buffer

struct EncoderSnapshot // Grouping these related values into one object
{
  int64_t leftTicks;
  int64_t rightTicks;
  int64_t leftInvalidTransitions;
  int64_t rightInvalidTransitions;
};

// ---------- Encoder interrupt handling ----------
// Read encoder channels A and B and combine them into a 2-bit state
uint8_t IRAM_ATTR readEncoderState(int pinA, int pinB)
{
  uint8_t a = digitalRead(pinA) ? 1 : 0;
  uint8_t b = digitalRead(pinB) ? 1 : 0;

  // A is the upper bit, and B is the lower bit
  return (a << 1) | b;
}

// Convert an encoder state change into a forward (+1), reverse (-1), or invalid/no-change (0) step
int8_t IRAM_ATTR decodeQuadratureTransition(uint8_t previousState, uint8_t currentState)
{
  // Combine the previous and current 2-bit states into one 4-bit value
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
// The motor functions map the signed PWM values from the host computer into the
// appropriate H-bridge outputs for forward, reverse, or braking behavior.
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
// These helpers make it easy to snapshot the encoder counters atomically and to
// reset them when the host requests a fresh baseline.
EncoderSnapshot getEncoderSnapshot()
{
  EncoderSnapshot snapshot;
  noInterrupts();
  snapshot.leftTicks = leftTicks;
  snapshot.rightTicks = rightTicks;
  snapshot.leftInvalidTransitions = leftInvalidTransitions;
  snapshot.rightInvalidTransitions = rightInvalidTransitions;
  interrupts();
  return snapshot;
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
  const EncoderSnapshot snapshot = getEncoderSnapshot();
  char feedbackLine[160];

  snprintf(
    feedbackLine,
    sizeof(feedbackLine),
    "FEEDBACK | LEFT ticks = %lld | RIGHT ticks = %lld | LEFT invalid = %lld | RIGHT invalid = %lld",
    static_cast<long long>(snapshot.leftTicks),
    static_cast<long long>(snapshot.rightTicks),
    static_cast<long long>(snapshot.leftInvalidTransitions),
    static_cast<long long>(snapshot.rightInvalidTransitions));

  Serial.println(feedbackLine);
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
// The firmware expects simple text commands from the serial link. Each newline
// terminates a command, which is then parsed and applied immediately.
char *trimWhitespace(char *text)
{
  while (*text != '\0' && isspace(static_cast<unsigned char>(*text)))
  {
    text++;
  }

  if (*text == '\0')
  {
    return text;
  }

  char *end = text + strlen(text) - 1;
  while (end > text && isspace(static_cast<unsigned char>(*end)))
  {
    *end = '\0';
    end--;
  }

  return text;
}

bool parseMotorCommand(const char *command, int &leftPwm, int &rightPwm)
{
  if (command[0] != 'M' && command[0] != 'm')
  {
    return false;
  }

  char *parseEnd = nullptr;
  long parsedLeft = strtol(command + 1, &parseEnd, 10);
  if (parseEnd == command + 1)
  {
    return false;
  }

  char *rightStart = parseEnd;
  long parsedRight = strtol(rightStart, &parseEnd, 10);
  if (parseEnd == rightStart)
  {
    return false;
  }

  while (*parseEnd != '\0' && isspace(static_cast<unsigned char>(*parseEnd)))
  {
    parseEnd++;
  }

  if (*parseEnd != '\0')
  {
    return false;
  }

  leftPwm = static_cast<int>(parsedLeft);
  rightPwm = static_cast<int>(parsedRight);
  return true;
}

void handleCommand(const char *rawCommand)
{
  char commandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
  strncpy(commandBuffer, rawCommand, sizeof(commandBuffer) - 1);
  commandBuffer[sizeof(commandBuffer) - 1] = '\0';

  char *command = trimWhitespace(commandBuffer);

  if (*command == '\0')
  {
    return;
  }

  char commandType = static_cast<char>(toupper(static_cast<unsigned char>(command[0])));
  if (commandType != 'M' && commandType != 'X' && commandType != 'Z' && commandType != 'S')
  {
    Serial.print("Ignored serial noise: ");
    Serial.println(command);
    return;
  }

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

void processSerialInput()
{
  while (Serial.available() > 0)
  {
    char incomingByte = static_cast<char>(Serial.read());

    if (incomingByte == '\r')
    {
      continue;
    }

    if (incomingByte == '\n')
    {
      if (serialCommandOverflow)
      {
        Serial.println("Serial command too long. Buffer cleared.");
      }
      else if (serialCommandLength > 0)
      {
        serialCommandBuffer[serialCommandLength] = '\0';
        handleCommand(serialCommandBuffer);
      }

      serialCommandLength = 0;
      serialCommandOverflow = false;
      continue;
    }

    if (serialCommandOverflow)
    {
      continue;
    }

    if (serialCommandLength >= SERIAL_COMMAND_BUFFER_SIZE - 1)
    {
      serialCommandLength = 0;
      serialCommandOverflow = true;
      continue;
    }

    serialCommandBuffer[serialCommandLength] = incomingByte;
    serialCommandLength++;
  }
}

// ---------- Periodic checks ----------
// The main loop periodically checks for stale commands and emits encoder feedback.
// If a command stops arriving, the robot is forced to brake for safety.
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
// Setup initializes the serial port, motor pins, encoder inputs, PWM outputs,
// and the initial motor/encoder state before normal operation begins.
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
  processSerialInput();
  checkCommandTimeout();
  checkFeedbackPrint();
}
