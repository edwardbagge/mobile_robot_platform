// 06_rpi_serial_command_test.ino
// Adafruit ESP32 Feather HUZZAH
// Raspberry Pi 5 sends commands to ESP32 over USB serial
// Serial baud: 115200
//
// Purpose:
// Test that Raspberry Pi can send serial commands to ESP32 over USB
// and receive replies from the ESP32.
//
// This test does not run the motors.
//
// Raspberry Pi sends:
// p = ping test
// s = status test
// l = toggle onboard LED
// z = reset command counter

const int LED_PIN = 13;  // Adafruit ESP32 Feather HUZZAH onboard LED

bool ledState = false;

unsigned long commandCounter = 0;
unsigned long lastCommandTime = 0;

void resetCommandCounter()
{
  commandCounter = 0;
  lastCommandTime = millis();

  Serial.println("Command counter reset.");
}

void handleCommand(String command)
{
  command.trim();

  if (command.length() == 0)
  {
    return;
  }

  commandCounter++;
  lastCommandTime = millis();

  Serial.print("Received command: ");
  Serial.println(command);

  if (command == "p")
  {
    Serial.println("PONG");
  }
  else if (command == "s")
  {
    Serial.println();
    Serial.println("=== STATUS ===");

    Serial.print("Serial baud = ");
    Serial.println(115200);

    Serial.print("Command counter = ");
    Serial.println(commandCounter);

    Serial.print("Last command time ms = ");
    Serial.println(lastCommandTime);

    Serial.print("LED state = ");
    if (ledState)
    {
      Serial.println("ON");
    }
    else
    {
      Serial.println("OFF");
    }

    Serial.println("USB serial link = OK");
    Serial.println();
  }
  else if (command == "l")
  {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);

    Serial.print("LED toggled. New state = ");
    if (ledState)
    {
      Serial.println("ON");
    }
    else
    {
      Serial.println("OFF");
    }
  }
  else if (command == "z")
  {
    resetCommandCounter();
  }
  else
  {
    Serial.println("Unknown command.");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lastCommandTime = millis();

  Serial.println();
  Serial.println("=== RPI SERIAL COMMAND TEST ===");
  Serial.println("ESP32 ready.");
  Serial.println("Waiting for commands from Raspberry Pi.");
  Serial.println();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }
}