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

// Handle a change in the left encoder signals
void IRAM_ATTR onLeftEncoder()
{
  uint8_t currentState = readEncoderState(LEFT_A, LEFT_B);
  int8_t step = decodeQuadratureTransition(leftEncoderState, currentState);

  if (step != 0) // Add or subtract one tick for a valid transition
  {
    leftTicks += step;
  }
  else if (currentState != leftEncoderState) // Count transitions that changed state but were not valid
  {
    leftInvalidTransitions++;
  }

  leftEncoderState = currentState; // Store the current state for the next transition
}

// Handle a change in the right encoder signals
void IRAM_ATTR onRightEncoder()
{
  uint8_t currentState = readEncoderState(RIGHT_A, RIGHT_B);
  int8_t step = decodeQuadratureTransition(rightEncoderState, currentState);

  if (step != 0) // Add or subtract one tick for a valid transition
  {
    rightTicks += step;
  }
  else if (currentState != rightEncoderState) // Count transitions that changed state but were not valid
  {
    rightInvalidTransitions++;
  }

  rightEncoderState = currentState; // Store the current state for the next transition
}

// ---------- Motor control ----------

int clampPwm(int pwm) // Limit the PWM command to the allowed range
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

void setLeftMotor(int speedValue) // Set the left motor direction and PWM output
{
  speedValue = clampPwm(speedValue);

  if (speedValue > 0) // Forward direction
  {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    ledcWrite(LEFT_EN, speedValue);
  }
  else if (speedValue < 0) // Reverse direction
  {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    ledcWrite(LEFT_EN, -speedValue);
  }
  else // Stop motor (coast to a stop)
  {
    ledcWrite(LEFT_EN, 0);
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
}

void setRightMotor(int speedValue) // Set the right motor direction and PWM output
{
  speedValue = clampPwm(speedValue);

  if (speedValue > 0) // Forward direction
  {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    ledcWrite(RIGHT_EN, speedValue);
  }
  else if (speedValue < 0) // Reverse direction
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    ledcWrite(RIGHT_EN, -speedValue);
  }
  else // Stop motor (coast to a stop)
  {
    ledcWrite(RIGHT_EN, 0);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
}

void brakeLeftMotor() // Apply active braking to the left motor
{
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, HIGH);
  ledcWrite(LEFT_EN, BRAKE_PWM);
}

void brakeRightMotor() // Apply active braking to the right motor
{
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, HIGH);
  ledcWrite(RIGHT_EN, BRAKE_PWM);
}

void releaseMotors() // Stop driving both motors without active braking
{
  setLeftMotor(0);
  setRightMotor(0);

  // Clear the stored motor command
  lastLeftPwm = 0;
  lastRightPwm = 0;
  commandActive = false;
}

void brakeMotors() // Actively brake both motors and clear the current command
{
  brakeLeftMotor();
  brakeRightMotor();
  lastLeftPwm = 0;
  lastRightPwm = 0;
  commandActive = false;

  Serial.println("Motors stopped with active brake.");
}

void applyMotorCommand(int leftPwm, int rightPwm) // Apply a new PWM command to both motors
{
  // Keep both commands within the allowed PWM range
  leftPwm = clampPwm(leftPwm);
  rightPwm = clampPwm(rightPwm);

  // Check whether the new command differs from the previous one
  bool commandChanged = (leftPwm != lastLeftPwm || rightPwm != lastRightPwm);

  setLeftMotor(leftPwm);
  setRightMotor(rightPwm);

  // Store the current command and update its timestamp
  lastLeftPwm = leftPwm;
  lastRightPwm = rightPwm;
  commandActive = (leftPwm != 0 || rightPwm != 0);
  lastCommandTime = millis();

  if (commandChanged) // Print only when the commanded PWM values change
  {
    Serial.print("CONTROL | LEFT pwm = ");
    Serial.print(leftPwm);
    Serial.print(" | RIGHT pwm = ");
    Serial.println(rightPwm);
  }
}

// ---------- Encoder helper functions ----------

EncoderSnapshot getEncoderSnapshot() // Copy all encoder counters without allowing interrupts to change them
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

void resetEncoderCounts() // Reset encoder counters and store the current encoder states
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

void printEncoderFeedback() // Print the current encoder counts and invalid-transition counts
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

void printStatus() // Print the current motor command, timing settings, and encoder feedback
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

char *trimWhitespace(char *text) // Remove whitespace from the beginning and end of a command
{
  while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) // Skip whitespace at the beginning
  {
    text++;
  }

  if (*text == '\0') // Return immediately if the string is empty
  {
    return text;
  }

  char *end = text + strlen(text) - 1; // Find the last character in the string
  
  while (end > text && isspace(static_cast<unsigned char>(*end))) // Remove whitespace from the end
  {
    *end = '\0';
    end--;
  }

  return text;
}

bool parseMotorCommand(const char *command, int &leftPwm, int &rightPwm) // Read the left and right PWM values from an M command
{
  if (command[0] != 'M' && command[0] != 'm') // Motor commands must begin with M
  {
    return false;
  }

  // Read the first number as the left motor PWM
  char *parseEnd = nullptr;
  long parsedLeft = strtol(command + 1, &parseEnd, 10);
  
  if (parseEnd == command + 1) // Fail if no number was found
  {
    return false;
  }

  // Read the second number as the right motor PWM
  char *rightStart = parseEnd;
  long parsedRight = strtol(rightStart, &parseEnd, 10);
  
  if (parseEnd == rightStart) // Fail if no second number was found
  {
    return false;
  }

  while (*parseEnd != '\0' && isspace(static_cast<unsigned char>(*parseEnd))) // Skip any whitespace after the second number
  {
    parseEnd++;
  }

  if (*parseEnd != '\0') // Reject the command if any other characters remain
  {
    return false;
  }

  leftPwm = static_cast<int>(parsedLeft);
  rightPwm = static_cast<int>(parsedRight);
  return true;
}

void handleCommand(const char *rawCommand) // Process one complete serial command
{
  // Copy the received command into a local buffer
  char commandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
  strncpy(commandBuffer, rawCommand, sizeof(commandBuffer) - 1);
  commandBuffer[sizeof(commandBuffer) - 1] = '\0';

  char *command = trimWhitespace(commandBuffer); // Remove whitespace from the beginning and end

  if (*command == '\0') // Ignore empty commands
  {
    return;
  }

  char commandType = static_cast<char>(toupper(static_cast<unsigned char>(command[0]))); // Read the first character and convert it to uppercase
  
  if (commandType != 'M' && commandType != 'X' && commandType != 'Z' && commandType != 'S') // Ignore data that does not begin with a valid command letter
  {
    Serial.print("Ignored serial noise: ");
    Serial.println(command);
    return;
  }

  if (commandType == 'M' || commandType == 'm') // M: apply left and right motor PWM commands
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
  else if (commandType == 'X' || commandType == 'x') // X: actively brake both motors
  {
    brakeMotors();
    printEncoderFeedback();
  }
  else if (commandType == 'Z' || commandType == 'z') // Z: reset encoder counts
  {
    resetEncoderCounts();
  }
  else if (commandType == 'S' || commandType == 's') // S: print the current robot status
  {
    printStatus();
  }
  else
  {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

void processSerialInput() // Read serial input and build complete commands line by line
{
  while (Serial.available() > 0)
  {
    char incomingByte = static_cast<char>(Serial.read());

    if (incomingByte == '\r') // Ignore carriage-return characters
    {
      continue;
    }

    if (incomingByte == '\n') // A newline marks the end of a command
    {
      if (serialCommandOverflow)
      {
        Serial.println("Serial command too long. Buffer cleared.");
      }
      else if (serialCommandLength > 0)
      {
        // End the string and process the completed command
        serialCommandBuffer[serialCommandLength] = '\0';
        handleCommand(serialCommandBuffer);
      }

      // Prepare for the next command
      serialCommandLength = 0;
      serialCommandOverflow = false;
      continue;
    }

    if (serialCommandOverflow) // Ignore remaining characters if the command already overflowed
    {
      continue;
    }

    if (serialCommandLength >= SERIAL_COMMAND_BUFFER_SIZE - 1) // Reject commands that are too long for the buffer
    {
      serialCommandLength = 0;
      serialCommandOverflow = true;
      continue;
    }

    // Add the received character to the command buffer
    serialCommandBuffer[serialCommandLength] = incomingByte;
    serialCommandLength++;
  }
}

// ---------- Periodic checks ----------

void checkCommandTimeout() // Brake the motors if an active command has not been updated within the timeout
{
  if (commandActive && millis() - lastCommandTime >= COMMAND_TIMEOUT_MS)
  {
    Serial.println("Command timeout reached.");
    brakeMotors();
    printEncoderFeedback();
  }
}

void checkFeedbackPrint() // Send encoder feedback at the configured interval
{
  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL_MS)
  {
    printEncoderFeedback();
    lastFeedbackTime = millis();
  }
}

// ---------- Setup ----------

void setup() // Initialize communication, motor control, encoders, and PWM
{
  // Start USB serial communication at 115200 baud
  Serial.begin(115200);
  delay(2000);

  // Configure H-bridge direction pins as outputs
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Configure encoder channels as inputs
  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);

  // Store the encoder states at startup
  leftEncoderState = readEncoderState(LEFT_A, LEFT_B);
  rightEncoderState = readEncoderState(RIGHT_A, RIGHT_B);

  // Call the encoder interrupt functions whenever an encoder signal changes
  attachInterrupt(digitalPinToInterrupt(LEFT_A), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), onRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), onRightEncoder, CHANGE);

  // Configure the PWM outputs used to control motor speed
  ledcAttach(LEFT_EN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(RIGHT_EN, PWM_FREQUENCY, PWM_RESOLUTION);

  // Start with the motors stopped and encoder counts at zero
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

void loop() // Continuously process commands, check the safety timeout, and send feedback
{
  processSerialInput();
  checkCommandTimeout();
  checkFeedbackPrint();
}
