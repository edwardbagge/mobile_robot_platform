// 05_speed_estimation_test.ino
// Adafruit ESP32 Feather HUZZAH + quadrature encoders
// Serial Monitor: 115200 baud
//
// Purpose:
// Estimate wheel speed from encoder tick counts.
// This test does not run the motors.
//
// Serial commands:
// z = reset encoder counts
//
// Important:
// Set TICKS_PER_REVOLUTION to match the encoder count used by this code.

const int LEFT_A  = 27;
const int LEFT_B  = 33;
const int RIGHT_A = 25;
const int RIGHT_B = 26;

// Change this value to match the encoder.
// This code counts only changes on channel A, so use the count that matches that method.
const float TICKS_PER_REVOLUTION = 700.0;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

long previousLeftTicks = 0;
long previousRightTicks = 0;

unsigned long previousTime = 0;
const unsigned long SAMPLE_INTERVAL = 500;  // ms

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

void resetEncoderCounts()
{
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  previousLeftTicks = 0;
  previousRightTicks = 0;
  previousTime = millis();

  Serial.println("Encoder counts reset.");
}

void printSpeedEstimate()
{
  unsigned long currentTime = millis();
  float deltaTimeSeconds = (currentTime - previousTime) / 1000.0;

  long currentLeftTicks;
  long currentRightTicks;

  noInterrupts();
  currentLeftTicks = leftTicks;
  currentRightTicks = rightTicks;
  interrupts();

  long deltaLeftTicks = currentLeftTicks - previousLeftTicks;
  long deltaRightTicks = currentRightTicks - previousRightTicks;

  float leftRevolutions = deltaLeftTicks / TICKS_PER_REVOLUTION;
  float rightRevolutions = deltaRightTicks / TICKS_PER_REVOLUTION;

  float leftRPM = (leftRevolutions / deltaTimeSeconds) * 60.0;
  float rightRPM = (rightRevolutions / deltaTimeSeconds) * 60.0;

  Serial.print("LEFT | ticks = ");
  Serial.print(currentLeftTicks);
  Serial.print(" | delta ticks = ");
  Serial.print(deltaLeftTicks);
  Serial.print(" | RPM = ");
  Serial.print(leftRPM);

  Serial.print(" || RIGHT | ticks = ");
  Serial.print(currentRightTicks);
  Serial.print(" | delta ticks = ");
  Serial.print(deltaRightTicks);
  Serial.print(" | RPM = ");
  Serial.println(rightRPM);

  previousLeftTicks = currentLeftTicks;
  previousRightTicks = currentRightTicks;
  previousTime = currentTime;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), onRightA, CHANGE);

  previousTime = millis();

  Serial.println();
  Serial.println("=== SPEED ESTIMATION TEST ===");
  Serial.println("Rotate wheels by hand.");
  Serial.println("Expected: RPM changes when wheel speed changes.");
  Serial.println("Command: z = reset encoder counts");
  Serial.println();
}

void loop()
{
  if (millis() - previousTime >= SAMPLE_INTERVAL)
  {
    printSpeedEstimate();
  }

  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "z")
    {
      resetEncoderCounts();
    }
    else
    {
      Serial.println("Unknown command.");
    }
  }
}