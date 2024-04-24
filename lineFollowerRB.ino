#define setpoint 0

//sensor pins
const int IR1 = 34;
const int IR2 = 35;
const int IR3 = 32;
const int IR4 = 33;

//driver pins
const int IN1 = 4;
const int IN2 = 2;
const int IN3 = 5;
const int IN4 = 18;
const int ENA = 19;
const int ENB = 15;

//motor speed
int rSpeed = 100;
int lSpeed = 100;

//PTDs
float Kp = 25;
float Ki = 0.5;
float Kd = 2.5;

//Other vars
int error;
int sensor;
int delta;
int integral;
int PID;
int lastErr;
int lTotal;
int rTotal;

void forward(int s1, int s2);

void setup() {
  Serial.begin (9600);

  //setting up the pins
  pinMode (IR1, INPUT);
  pinMode (IR2, INPUT);
  pinMode (IR3, INPUT);
  pinMode (IR4, INPUT);

  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

}

void loop() {
  int sensorState4 = digitalRead(IR4);
  int sensorState3 = digitalRead(IR3);
  int sensorState2 = digitalRead(IR2);
  int sensorState1 = digitalRead(IR1);

  Serial.print("Sensor 4: ");
  Serial.print(sensorState4);
  Serial.print(" // ");

  Serial.print("Sensor 3: ");
  Serial.print(sensorState3);
  Serial.print(" // ");

  Serial.print("Sensor 2: ");
  Serial.print(sensorState2);
  Serial.print(" // ");

  Serial.print("Sensor 1: ");
  Serial.print(sensorState1);
  Serial.print(" // ");

  Serial.print("PID: ");
  Serial.print(PID);
  Serial.print(" // ");

  Serial.print("left total: ");
  Serial.print(lTotal);
  Serial.print(" // ");

  Serial.print("Right total: ");
  Serial.print(rTotal);
  Serial.print(" // ");

  // kiri 1
  if (sensorState1 == HIGH && sensorState2 == HIGH && sensorState3 == HIGH && sensorState4 == LOW) {
    sensor = 2;
  }

  // kiri 2
  if (sensorState1 == HIGH && sensorState2 == HIGH && sensorState3 == LOW && sensorState4 == LOW) {
    sensor = 1;
  }

  // depan
  if (sensorState1 == HIGH && sensorState2 == LOW && sensorState3 == LOW && sensorState4 == HIGH) {
    sensor = 0;
  }

  // kanan 1
  if (sensorState1 == LOW && sensorState2 == LOW && sensorState3 == HIGH && sensorState4 == HIGH) {
    sensor = -1;
  }

  // kanan 2
  if (sensorState1 == LOW && sensorState2 == HIGH && sensorState3 == HIGH && sensorState4 == HIGH) {
    sensor = -2;
  }

  error = setpoint - error;
  integral = error + integral;

  if (integral >= 5) {
    integral = 5;
  }

  if (integral <= -5) {
    integral = -5;
  }

  PID = (Kp * error + Kd * delta + Ki * integral);

  lastErr = error;

  if (PID >= 100) {
    PID = 100;
  }

  if (PID <= -100) {
    PID = -100;
  }

  lTotal = lSpeed - PID;
  rTotal = rSpeed + PID;
  forward(lSpeed, rSpeed);
}

void forward(int s1, int s2) {
    if (s1 < 0) {
      s1 = 0;
    }

    if (s1 > 200) {
      s1 = 200;
    }

    if (s2 < 0) {
      s2 = 0;
    }

    if (s2 > 200) {
    s2 = 200;
    }

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, s1);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, s2);
  }
