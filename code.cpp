#include <SoftwareSerial.h>

#define ENA 10
#define IN1 8
#define IN2 9
#define ENB 5
#define IN3 11
#define IN4 12

#define TRIG 6
#define ECHO 7

#define BT_RX 2
#define BT_TX 3

SoftwareSerial bluetooth(BT_RX, BT_TX);

long duration;
int distance;

const int THRESHOLD = 20;

const int speedTurn = 150;
const int speedForward = 180;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  Serial.println("Robot ready. Send commands via Bluetooth:");
  Serial.println("F = forward, B = backward, L = left, R = right, S = stop, A = automatic mode");
}

void loop() {
  if (bluetooth.available()) {
    char command = bluetooth.read();
    Serial.print("Command received: ");
    Serial.println(command);
    
    switch (command) {
      case 'F':
      case 'f':
        maju();
        break;
      case 'B':
      case 'b':
        mundur();
        break;
      case 'L':
      case 'l':
        kiri();
        break;
      case 'R':
      case 'r':
        kanan();
        break;
      case 'S':
      case 's':
        stop();
        break;
      case 'A':
      case 'a':
        modeOtomatis();
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }
  
  delay(50);
}

int bacaJarak() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

void maju() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speedForward);
  analogWrite(ENB, speedForward);
  Serial.println("Forward");
}

void mundur() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedForward);
  analogWrite(ENB, speedForward);
  Serial.println("Backward");
}

void kiri() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, speedTurn);
  Serial.println("Left");
}

void kanan() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speedTurn);
  analogWrite(ENB, 0);
  Serial.println("Right");
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Stop");
}

void modeOtomatis() {
  Serial.println("Automatic mode active. Send 'S' to stop.");
  while (true) {
    int jarak = bacaJarak();
    Serial.print("Distance: ");
    Serial.print(jarak);
    Serial.println(" cm");
    
    if (jarak < THRESHOLD && jarak > 0) {
      stop();
      delay(500);
      mundur();
      delay(500);
      stop();
      if (random(0, 2) == 0) {
        kiri();
      } else {
        kanan();
      }
      delay(800);
      stop();
    } else {
      maju();
    }
    
    if (bluetooth.available()) {
      char cmd = bluetooth.read();
      if (cmd == 'S' || cmd == 's') {
        stop();
        Serial.println("Automatic mode stopped.");
        break;
      }
    }
    
    delay(100);
  }
}
