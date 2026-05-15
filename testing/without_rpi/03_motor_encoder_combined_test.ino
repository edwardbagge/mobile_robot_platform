// 03_motor_encoder_combined_test.ino
// Adafruit ESP32 Feather HUZZAH + L298N + encoders
// Serial Monitor: 115200 baud
//
// Purpose:
// Run motors slowly and print encoder pulse counts at the same time.
//
// Robot wiring used in this file:
// Left motor  = L298N OUT3 / OUT4, controlled by IN3 / IN4 / ENB
// Right motor = L298N OUT1 / OUT2, controlled by IN1 / IN2 / ENA
//
// Serial commands:
// lf = left motor forward
// lb = left motor backward
// rf = right motor forward
// rb = right motor backward
// f  = both motors forward
// b  = both motors backward
// s  = stop motors
// z  = reset encoder counts
// p  = print encoder status once

// ---------- Encoder pins ----------
const int LEFT_A  = 27;
const int LEFT_B  = 33;
const int RIGHT_A = 25;
const int RIGHT_B = 26;

// ---------- L298N pins ----------
const int IN1 = 13;
const int IN2 = 21;
const int ENA = 14;

const int IN3 = 22;
const int IN4 = 23;
const int ENB = 32;

// ---------- PWM settings ----------
const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;
const int TEST_SPEED = 120;

// ---------- Encoder counters ----------
volatile unsigned long leftAPulseCount  = 0;
volatile unsigned long leftBPulseCount  = 0;
volatile unsigned long rightAPulseCount = 0;
volatile unsigned long rightBPulseCount = 0;

void IRAM_ATTR onLeftA()
{
  leftAPulseCount++;
}

void IRAM_ATTR onLeftB()
{
  leftBPulseCount++;
}

void IRAM_ATTR onRightA()
{
  rightAPulseCount++;
}

void IRAM_ATTR onRightB()
{
  rightBPulseCount++;
}

void setLeftMotor(int speed)
{
  // Left motor is connected to OUT3 / OUT4, so it uses IN3 / IN4 / ENB.

  if (speed > 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(ENB, -speed);
  }
  else
  {
    ledcWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

void setRightMotor(int speed)
{
  // Right motor is connected to OUT1 / OUT2, so it uses IN1 / IN2 / ENA.
  // Direction is reversed in software so that "forward" matches the robot direction.

  if (speed > 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(ENA, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, -speed);
  }
  else
  {
    ledcWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void stopMotors()
{
  setLeftMotor(0);
  setRightMotor(0);
  Serial.println("Motors stopped.");
}

void resetEncoderCounts()
{
  noInterrupts();
  leftAPulseCount = 0;
  leftBPulseCount = 0;
  rightAPulseCount = 0;
  rightBPulseCount = 0;
  interrupts();

  Serial.println("Encoder counts reset.");
}

void printEncoderStatus()
{
  noInterrupts();
  unsigned long left_A = leftAPulseCount;
  unsigned long left_B = leftBPulseCount;
  unsigned long right_A = rightAPulseCount;
  unsigned long right_B = rightBPulseCount;
  interrupts();

  Serial.println();
  Serial.println("----- Encoder status -----");

  Serial.print("Left A state: ");
  Serial.println(digitalRead(LEFT_A));
  Serial.print("Left A pulse count: ");
  Serial.println(left_A);

  Serial.print("Left B state: ");
  Serial.println(digitalRead(LEFT_B));
  Serial.print("Left B pulse count: ");
  Serial.println(left_B);

  Serial.print("Right A state: ");
  Serial.println(digitalRead(RIGHT_A));
  Serial.print("Right A pulse count: ");
  Serial.println(right_A);

  Serial.print("Right B state: ");
  Serial.println(digitalRead(RIGHT_B));
  Serial.print("Right B pulse count: ");
  Serial.println(right_B);

  Serial.println("--------------------------");
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQUENCY, PWM_RESOLUTION);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B), onLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), onRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), onRightB, CHANGE);

  stopMotors();

  Serial.println();
  Serial.println("=== MOTOR + ENCODER COMBINED TEST ===");
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "lf")
    {
      setLeftMotor(TEST_SPEED);
      setRightMotor(0);
      Serial.println("Left motor forward.");
    }
    else if (command == "lb")
    {
      setLeftMotor(-TEST_SPEED);
      setRightMotor(0);
      Serial.println("Left motor backward.");
    }
    else if (command == "rf")
    {
      setLeftMotor(0);
      setRightMotor(TEST_SPEED);
      Serial.println("Right motor forward.");
    }
    else if (command == "rb")
    {
      setLeftMotor(0);
      setRightMotor(-TEST_SPEED);
      Serial.println("Right motor backward.");
    }
    else if (command == "f")
    {
      setLeftMotor(TEST_SPEED);
      setRightMotor(TEST_SPEED);
      Serial.println("Both motors forward.");
    }
    else if (command == "b")
    {
      setLeftMotor(-TEST_SPEED);
      setRightMotor(-TEST_SPEED);
      Serial.println("Both motors backward.");
    }
    else if (command == "s")
    {
      stopMotors();
    }
    else if (command == "z")
    {
      resetEncoderCounts();
    }
    else if (command == "p")
    {
      printEncoderStatus();
    }
    else
    {
      Serial.println("Unknown command.");
    }
  }
}