// 07_rpi_motor_command_test.ino

const int RIGHT_IN1 = 13;
const int RIGHT_IN2 = 21;
const int RIGHT_EN  = 14;

const int LEFT_IN1  = 22;
const int LEFT_IN2  = 23;
const int LEFT_EN   = 32;

const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;

const int TEST_SPEED = 90;
const unsigned long MOTOR_TIMEOUT = 1000;

String lastCommand = "none";
unsigned long lastMotorCommandTime = 0;
bool motorsRunning = false;

void setLeftMotor(int speedValue)
{
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
  // Right motor direction reversed to match working Test 3

  if (speedValue > 0)
  {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    ledcWrite(RIGHT_EN, speedValue);
  }
  else if (speedValue < 0)
  {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    ledcWrite(RIGHT_EN, -speedValue);
  }
  else
  {
    ledcWrite(RIGHT_EN, 0);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
}

void stopMotors()
{
  setLeftMotor(0);
  setRightMotor(0);

  motorsRunning = false;
  lastCommand = "stop";

  Serial.println("Motors stopped.");
}

void driveForward()
{
  setLeftMotor(TEST_SPEED);
  setRightMotor(TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastCommand = "forward";

  Serial.println("Motor command: forward");
}

void driveBackward()
{
  setLeftMotor(-TEST_SPEED);
  setRightMotor(-TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastCommand = "backward";

  Serial.println("Motor command: backward");
}

void turnLeft()
{
  setLeftMotor(-TEST_SPEED);
  setRightMotor(TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastCommand = "turn left";

  Serial.println("Motor command: turn left");
}

void turnRight()
{
  setLeftMotor(TEST_SPEED);
  setRightMotor(-TEST_SPEED);

  motorsRunning = true;
  lastMotorCommandTime = millis();
  lastCommand = "turn right";

  Serial.println("Motor command: turn right");
}

void printStatus()
{
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
  Serial.println();
}

void handleCommand(String command)
{
  command.trim();

  if (command.length() == 0) return;

  Serial.print("Received command: ");
  Serial.println(command);

  if (command == "f") driveForward();
  else if (command == "b") driveBackward();
  else if (command == "l") turnLeft();
  else if (command == "r") turnRight();
  else if (command == "x") stopMotors();
  else if (command == "s") printStatus();
  else Serial.println("Unknown command.");
}

void checkMotorTimeout()
{
  if (motorsRunning && millis() - lastMotorCommandTime >= MOTOR_TIMEOUT)
  {
    Serial.println("Motor timeout reached.");
    stopMotors();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  ledcAttach(LEFT_EN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(RIGHT_EN, PWM_FREQUENCY, PWM_RESOLUTION);

  stopMotors();

  Serial.println();
  Serial.println("=== RPI MOTOR COMMAND TEST ===");
  Serial.println("ESP32 ready.");
  Serial.println("Commands: f b l r x s");
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }

  checkMotorTimeout();
}