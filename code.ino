// Encoder Pins
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 5

// Motor Control Pins
#define EN_L 6
#define IN1_L 7
#define IN2_L 8

#define EN_R 11
#define IN1_R 9
#define IN2_R 10

// Wheel Parameters
const double wheel_rad = 0.035;   // meters
const double wheel_sep = 0.232;   // meters

// Encoder Tick Counts
volatile int left_wheel_tick_count = 0;
volatile int right_wheel_tick_count = 0;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Motor Speed Variables
double linear_vel = 0.0;
double angular_vel = 0.0;

double w_l = 0.0;
double w_r = 0.0;

// Publish timing
const unsigned long interval = 100;  // milliseconds
unsigned long previousMillis = 0;

// Serial input buffer
String inputString = "";

void right_wheel_tick() {
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == HIGH) {
    right_wheel_tick_count++;
  } else {
    right_wheel_tick_count--;
  }

  if (right_wheel_tick_count > encoder_maximum) {
    right_wheel_tick_count = encoder_minimum;
  }

  if (right_wheel_tick_count < encoder_minimum) {
    right_wheel_tick_count = encoder_maximum;
  }
}

void left_wheel_tick() {
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == HIGH) {
    left_wheel_tick_count++;
  } else {
    left_wheel_tick_count--;
  }

  if (left_wheel_tick_count > encoder_maximum) {
    left_wheel_tick_count = encoder_minimum;
  }

  if (left_wheel_tick_count < encoder_minimum) {
    left_wheel_tick_count = encoder_maximum;
  }
}

void MotorL(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(EN_L, pwm);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  } else if (pwm < 0) {
    pwm = abs(pwm);
    analogWrite(EN_L, pwm);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  } else {
    analogWrite(EN_L, 0);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
}

void MotorR(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(EN_R, pwm);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  } else if (pwm < 0) {
    pwm = abs(pwm);
    analogWrite(EN_R, pwm);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  } else {
    analogWrite(EN_R, 0);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }
}

void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      inputString.trim();

      if (inputString.startsWith("CMD,")) {
        int firstComma = inputString.indexOf(',');
        int secondComma = inputString.indexOf(',', firstComma + 1);

        if (secondComma > 0) {
          String linStr = inputString.substring(firstComma + 1, secondComma);
          String angStr = inputString.substring(secondComma + 1);

          linear_vel = linStr.toFloat();
          angular_vel = angStr.toFloat();

          w_l = (linear_vel * 2.0) - ((angular_vel * wheel_sep) / (2.0 * wheel_rad));
          w_r = (linear_vel * 2.0) + ((angular_vel * wheel_sep) / (2.0 * wheel_rad));
        }
      }

      inputString = "";
    } else {
      inputString += c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Encoder setup
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // Motor setup
  pinMode(EN_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);

  pinMode(EN_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  MotorL(0);
  MotorR(0);
}

void loop() {
  readSerialCommand();

  MotorL(w_l * 180);
  MotorR(w_r * 180);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    Serial.print("TICKS,");
    Serial.print(left_wheel_tick_count);
    Serial.print(",");
    Serial.println(right_wheel_tick_count);
  }
}
