// 04_quadrature_direction_test.ino
// Adafruit ESP32 Feather HUZZAH + quadrature encoders
// Serial Monitor: 115200 baud
//
// Purpose:
// Test quadrature encoder direction detection.
// When a wheel is rotated in one direction, the signed tick count should increase.
// When rotated in the opposite direction, the signed tick count should decrease.
//
// This test does not run the motors.
//
// Serial commands:
// z = reset signed tick counts

// ---------- Encoder pins ----------
const int LEFT_A  = 27;
const int LEFT_B  = 33;
const int RIGHT_A = 25;
const int RIGHT_B = 26;

// ---------- Signed encoder counters ----------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500;

// ---------- Interrupt service routines ----------
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

  Serial.println("Signed tick counts reset.");
}

void printEncoderStatus()
{
  noInterrupts();
  long left = leftTicks;
  long right = rightTicks;
  interrupts();

  Serial.print("LEFT | A state = ");
  Serial.print(digitalRead(LEFT_A));
  Serial.print(" | B state = ");
  Serial.print(digitalRead(LEFT_B));
  Serial.print(" | signed tick count = ");
  Serial.print(left);

  Serial.print(" || RIGHT | A state = ");
  Serial.print(digitalRead(RIGHT_A));
  Serial.print(" | B state = ");
  Serial.print(digitalRead(RIGHT_B));
  Serial.print(" | signed tick count = ");
  Serial.println(right);
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

  Serial.println();
  Serial.println("=== QUADRATURE DIRECTION TEST ===");
  Serial.println("Rotate each wheel by hand.");
  Serial.println("Expected: signed tick count increases in one direction and decreases in the opposite direction.");
  Serial.println("Command: z = reset signed tick counts");
  Serial.println();
}

void loop()
{
  if (millis() - lastPrintTime >= PRINT_INTERVAL)
  {
    lastPrintTime = millis();
    printEncoderStatus();
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