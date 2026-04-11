const int IN1 = 13;
const int IN2 = 12;
const int ENA = 14;

const int IN3 = 27;
const int IN4 = 33;
const int ENB = 32;

const int pwmFreq = 1000;
const int pwmResolution = 8;

// 0-255
const int speedA = 120;
const int speedB = 120;

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
  Serial.println("Commands: f=forward, b=backward, l=left, r=right, s=stop");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Ignore newline / carriage return
    if (cmd == '\n' || cmd == '\r') {
      return;
    }

    Serial.print("Received: ");
    Serial.println(cmd);

    if (cmd == 'f') {
      forward();
      Serial.println("Action: forward");
    } else if (cmd == 'b') {
      backward();
      Serial.println("Action: backward");
    } else if (cmd == 'l') {
      turnLeft();
      Serial.println("Action: turn left");
    } else if (cmd == 'r') {
      turnRight();
      Serial.println("Action: turn right");
    } else if (cmd == 's') {
      stopMotors();
      Serial.println("Action: stop");
    } else {
      Serial.println("Unknown command");
    }
  }
}