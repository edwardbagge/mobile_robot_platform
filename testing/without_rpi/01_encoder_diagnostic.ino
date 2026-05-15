// 01_encoder_diagnostic.ino
// Adafruit ESP32 Feather HUZZAH
// Serial Monitor: 115200 baud
//
// Purpose:
// Check whether encoder signal pins change when wheels are rotated by hand.
// This test does not run the motors.
//
// Serial commands:
// z = reset pulse counts

const int LEFT_A  = 27;
const int LEFT_B  = 33;
const int RIGHT_A = 25;
const int RIGHT_B = 26;

volatile unsigned long leftAChanges  = 0;
volatile unsigned long leftBChanges  = 0;
volatile unsigned long rightAChanges = 0;
volatile unsigned long rightBChanges = 0;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 3000;

void IRAM_ATTR onLeftA()
{
  leftAChanges++;
}

void IRAM_ATTR onLeftB()
{
  leftBChanges++;
}

void IRAM_ATTR onRightA()
{
  rightAChanges++;
}

void IRAM_ATTR onRightB()
{
  rightBChanges++;
}

void resetPulseCounts()
{
  noInterrupts();
  leftAChanges = 0;
  leftBChanges = 0;
  rightAChanges = 0;
  rightBChanges = 0;
  interrupts();

  Serial.println();
  Serial.println("Pulse counts reset.");
  Serial.println();
}

void printEncoderStatus()
{
  noInterrupts();
  unsigned long left_A = leftAChanges;
  unsigned long left_B = leftBChanges;
  unsigned long right_A = rightAChanges;
  unsigned long right_B = rightBChanges;
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

  attachInterrupt(digitalPinToInterrupt(LEFT_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B), onLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), onRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), onRightB, CHANGE);

  Serial.println();
  Serial.println("=== ENCODER DIAGNOSTIC TEST ===");
  Serial.println("Rotate wheels by hand.");
  Serial.println("Command: z = reset pulse counts");
  Serial.println();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "z")
    {
      resetPulseCounts();
    }
    else
    {
      Serial.println("Unknown command.");
    }
  }

  if (millis() - lastPrintTime >= PRINT_INTERVAL)
  {
    lastPrintTime = millis();
    printEncoderStatus();
  }
}