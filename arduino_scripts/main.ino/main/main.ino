#include <Wire.h>
#include <AccelStepper.h>

#define STEP_PIN_ELEVATOR A0
#define DIR_PIN_ELEVATOR A1

#define STEP_PIN_BASEROTATION A3
#define DIR_PIN_BASEROTATION A2

AccelStepper stepperElevator(1, STEP_PIN_ELEVATOR, DIR_PIN_ELEVATOR);
AccelStepper stepperBaseRotation(1, STEP_PIN_BASEROTATION, DIR_PIN_BASEROTATION);

// I2C
#define ADR 0x0a
#define N   4
#define SZ  (N * sizeof(float))

union Packet {
  byte b[SZ];
  float f[N];
} pkt;

float last[N] = {0};

// Funções de tratamento
void handleElevator(float v) {
  Serial.print("Elevator comando recebido: ");
  Serial.println(v, 4);

  // Define velocidade do motor Elevator
  stepperElevator.setSpeed(v);
}

void handleBaseRotation(float v) {
  Serial.print("BaseRotation comando recebido: ");
  Serial.println(v, 4);
  
  // Define velocidade do motor BaseRotation
  stepperBaseRotation.setSpeed(v);
}

void handleGripper(float v) {
  Serial.print("Gripper comando recebido: ");
  Serial.println(v, 4);
}

void handleWrist(float v) {
  Serial.print("Wrist comando recebido: ");
  Serial.println(v, 4);
}

// Array de ponteiros de função
void (*h[N])(float) = {
  handleElevator,
  handleBaseRotation,
  handleGripper,
  handleWrist
};

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando...");

  // Configuração dos motores
  stepperElevator.setMaxSpeed(1000);
  stepperElevator.setAcceleration(500);
  stepperElevator.setSpeed(0);

  stepperBaseRotation.setMaxSpeed(1000);
  stepperBaseRotation.setAcceleration(500);
  stepperBaseRotation.setSpeed(0);

  // Inicializa I2C
  Wire.begin(ADR);
  Wire.onReceive(onRx);
}

void loop() {
  // Mantém movimento constante de ambos motores
  stepperElevator.runSpeed();
  stepperBaseRotation.runSpeed();
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
