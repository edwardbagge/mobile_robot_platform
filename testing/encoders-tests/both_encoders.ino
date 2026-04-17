// Dual motor + dual encoder test for ESP32 + L298N
// Uses the PWM method that already worked on your setup

volatile long left_ticks = 0;
volatile long right_ticks = 0;

volatile int left_direction = 0;
volatile int right_direction = 0;

// Encoder pins
const int LEFT_ENC_YELLOW  = 34;
const int LEFT_ENC_WHITE   = 39;

const int RIGHT_ENC_YELLOW = 25;
const int RIGHT_ENC_WHITE  = 26;

// L298N pins
const int IN1 = 13;
const int IN2 = 12;
const int ENA = 14;

const int IN3 = 27;
const int IN4 = 33;
const int ENB = 32;

// PWM
const int pwmFreq = 1000;
const int pwmResolution = 8;
const int leftSpeed = 120;
const int rightSpeed = 119;

unsigned long last_report_ms = 0;
const unsigned long report_interval_ms = 250;

void IRAM_ATTR leftEncoderISR() {
  int y = digitalRead(LEFT_ENC_YELLOW);
  int w = digitalRead(LEFT_ENC_WHITE);

  if (y == w) {
    left_ticks++;
    left_direction = 1;
  } else {
    left_ticks--;
    left_direction = -1;
  }
}

void IRAM_ATTR rightEncoderISR() {
  int y = digitalRead(RIGHT_ENC_YELLOW);
  int w = digitalRead(RIGHT_ENC_WHITE);

  if (y == w) {
    right_ticks++;
    right_direction = 1;
  } else {
    right_ticks--;
    right_direction = -1;
  }
}

void printEncoderState() {
  long left_snapshot;
  long right_snapshot;
  int left_dir_snapshot;
  int right_dir_snapshot;

  noInterrupts();
  left_snapshot = left_ticks;
  right_snapshot = right_ticks;
  left_dir_snapshot = left_direction;
  right_dir_snapshot = right_direction;
  interrupts();

  Serial.print("LEFT ticks: ");
  Serial.print(left_snapshot);
  Serial.print(" dir: ");
  Serial.print(left_dir_snapshot);
  Serial.print(" | RIGHT ticks: ");
  Serial.print(right_snapshot);
  Serial.print(" dir: ");
  Serial.println(right_dir_snapshot);
}

void resetCounts() {
  noInterrupts();
  left_ticks = 0;
  right_ticks = 0;
  left_direction = 0;
  right_direction = 0;
  interrupts();
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_ENC_YELLOW, INPUT_PULLUP);
  pinMode(LEFT_ENC_WHITE, INPUT_PULLUP);
  pinMode(RIGHT_ENC_YELLOW, INPUT_PULLUP);
  pinMode(RIGHT_ENC_WHITE, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);

  // Same motor directions as your working test
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, leftSpeed);
  ledcWrite(ENB, rightSpeed);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_YELLOW), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_YELLOW), rightEncoderISR, CHANGE);

  Serial.println("Dual motor + encoder test ready");
}

void loop() {
  unsigned long now = millis();

  if (now - last_report_ms >= report_interval_ms) {
    last_report_ms = now;
    printEncoderState();
  }

  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '\n' || cmd == '\r') {
      return;
    }

    switch (cmd) {
      case 'p':
        printEncoderState();
        break;

      case 'z':
        resetCounts();
        Serial.println("Counts reset");
        break;

      case 's':
        ledcWrite(ENA, 0);
        ledcWrite(ENB, 0);
        Serial.println("Motors stopped");
        break;

      default:
        Serial.println("Unknown command");
        break;
    }
  }
}
