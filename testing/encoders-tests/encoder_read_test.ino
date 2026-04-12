// Basic ESP32 quadrature encoder test scaffold.
// Fill in the correct GPIO pins for your left and right encoders
// before uploading.

volatile long left_ticks = 0;
volatile long right_ticks = 0;

volatile int left_direction = 0;
volatile int right_direction = 0;

// TODO: Replace these placeholder pins with your actual wiring.
const int LEFT_ENC_A = 34;
const int LEFT_ENC_B = 35;
const int RIGHT_ENC_A = 36;
const int RIGHT_ENC_B = 39;

unsigned long last_report_ms = 0;
const unsigned long report_interval_ms = 250;

void IRAM_ATTR leftEncoderISR() {
  int a = digitalRead(LEFT_ENC_A);
  int b = digitalRead(LEFT_ENC_B);

  if (a == b) {
    left_ticks++;
    left_direction = 1;
  } else {
    left_ticks--;
    left_direction = -1;
  }
}

void IRAM_ATTR rightEncoderISR() {
  int a = digitalRead(RIGHT_ENC_A);
  int b = digitalRead(RIGHT_ENC_B);

  if (a == b) {
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

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  Serial.println("ESP32 encoder test ready");
  Serial.println("Commands:");
  Serial.println("  p = print current counts");
  Serial.println("  z = zero counts");
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

      default:
        Serial.println("Unknown command");
        break;
    }
  }
}
