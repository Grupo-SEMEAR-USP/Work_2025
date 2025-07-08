#include <Wire.h>
#include <AccelStepper.h>
#include <Servo.h>

// Pinos dos steppers
#define STEP_PIN_ELEVATOR       A0
#define DIR_PIN_ELEVATOR        A1
#define STEP_PIN_BASEROTATION   A3
#define DIR_PIN_BASEROTATION    A2

// Pinos dos servos
#define SERVO_PIN_GRIPPER       9
#define SERVO_PIN_WRIST         10

// Número de sensores ultrassônicos
#define NUM_ULTRASONIC 5

const uint8_t trigPins[NUM_ULTRASONIC] = { 4,  6,  8, 10, 12 };
const uint8_t echoPins[NUM_ULTRASONIC] = { 3,  5,  7,  9, 11 };

// Instâncias
AccelStepper stepperElevator(1, STEP_PIN_ELEVATOR, DIR_PIN_ELEVATOR);
AccelStepper stepperBaseRotation(1, STEP_PIN_BASEROTATION, DIR_PIN_BASEROTATION);
Servo servoGripper;
Servo servoWrist;

// I2C
#define ADR 0x0a
#define N   4
#define SZ  (N * sizeof(float))
union Packet {
  byte b[SZ];
  float f[N];
} pkt;
float last[N] = {0};

// union para empacotar os floats dos 5 sensores
#define US_SZ (NUM_ULTRASONIC * sizeof(float))
union USPacket {
  byte b[US_SZ];
  float f[NUM_ULTRASONIC];
} uspkt;

// Protótipos
void onRx(int n);
void onRequest();
float readUltrasonic(int idx);
void handleElevator(float steps);
void handleBaseRotation(float steps);
void handleGripper(float angle);
void handleWrist(float angle);

// Funções de tratamento I2C
void handleElevator(float steps) {
  Serial.print("Elevator: mover ");
  Serial.print(steps);
  Serial.println(" passos");
  stepperElevator.move(steps);
}

void handleBaseRotation(float steps) {
  Serial.print("BaseRot: mover ");
  Serial.print(steps);
  Serial.println(" passos");
  stepperBaseRotation.move(steps);
}

void handleGripper(float angle) {
  angle = constrain(angle, 0, 180);
  Serial.print("Gripper: ângulo ");
  Serial.print(angle);
  Serial.println("°");
  servoGripper.write(angle);
}

void handleWrist(float angle) {
  angle = constrain(angle, 0, 180);
  Serial.print("Wrist: ângulo ");
  Serial.print(angle);
  Serial.println("°");
  servoWrist.write(angle);
}

void (*h[N])(float) = {
  handleElevator,
  handleBaseRotation,
  handleGripper,
  handleWrist
};

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando...");

  // Configuração dos steppers
  stepperElevator.setMaxSpeed(1000);
  stepperElevator.setAcceleration(500);

  stepperBaseRotation.setMaxSpeed(1000);
  stepperBaseRotation.setAcceleration(500);

  // Configura servos
  servoGripper.attach(SERVO_PIN_GRIPPER);
  servoWrist.attach(  SERVO_PIN_WRIST);

  // Configura cada ultrassônico
  for (int i = 0; i < NUM_ULTRASONIC; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }

  // Inicializa I2C como escravo
  Wire.begin(ADR);
  Wire.onReceive(onRx);
  Wire.onRequest(onRequest);
}

void loop() {
  stepperElevator.run();
  stepperBaseRotation.run();
}

// Callback de recepção I2C
void onRx(int n) {
  if (n != SZ) {
    while (Wire.available()) Wire.read();
    return;
  }
  for (int i = 0; i < SZ && Wire.available(); i++)
    pkt.b[i] = Wire.read();

  for (int i = 0; i < N; i++) {
    if (fabs(pkt.f[i] - last[i]) > 1e-5) {
      last[i] = pkt.f[i];
      h[i](pkt.f[i]);
    }
  }
}

void onRequest() {
  for (int i = 0; i < NUM_ULTRASONIC; i++) {
    uspkt.f[i] = readUltrasonic(i);
  }
  Wire.write(uspkt.b, US_SZ);
}

float readUltrasonic(int idx) {
  uint8_t t = trigPins[idx], e = echoPins[idx];
  digitalWrite(t, LOW);
  delayMicroseconds(2);
  digitalWrite(t, HIGH);
  delayMicroseconds(10);
  digitalWrite(t, LOW);

  unsigned long dur = pulseIn(e, HIGH, 30000UL);
  if (dur == 0) return -1.0f; 
  return (dur * 0.0343f) / 2.0f;
}
