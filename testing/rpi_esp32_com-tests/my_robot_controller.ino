const int IN1 = 13;
const int IN2 = 12;
const int ENA = 14;

const int IN3 = 27;
const int IN4 = 33;
const int ENB = 32;

const int pwmFreq = 1000;
const int pwmResolution = 8;

// Tune these later if motors are mismatched
int speedA = 120;
int speedB = 120;

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, speedA);
  ledcWrite(ENB, speedB);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENA, speedA);
  ledcWrite(ENB, speedB);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, speedA);
  ledcWrite(ENB, speedB);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENA, speedA);
  ledcWrite(ENB, speedB);
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);

  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);

  stopMotors();

  Serial.println("ESP32 ready");
  Serial.println("Commands: f b l r s");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '\n' || cmd == '\r') {
      return;
    }

    Serial.print("Received: ");
    Serial.println(cmd);

    switch (cmd) {
      case 'f':
        forward();
        Serial.println("Action: forward");
        break;

      case 'b':
        backward();
        Serial.println("Action: backward");
        break;

      case 'l':
        turnLeft();
        Serial.println("Action: left");
        break;

      case 'r':
        turnRight();
        Serial.println("Action: right");
        break;

      case 's':
        stopMotors();
        Serial.println("Action: stop");
        break;

      default:
        Serial.println("Unknown command");
        break;
    }
  }
}
