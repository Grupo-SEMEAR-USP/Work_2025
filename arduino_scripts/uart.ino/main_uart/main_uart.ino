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

// Sensores ultrassônicos
#define NUM_ULTRASONIC 5
const uint8_t trigPins[NUM_ULTRASONIC] = { 4,  6,  8, 10, 12 };
const uint8_t echoPins[NUM_ULTRASONIC] = { 3,  5,  7,  9, 11 };

// Bytes inicializadores e finalizadores dos pacotes transmitidos por UART
#define BYTE_INIT 0x7E
#define BYTE_END 0x7F

// Instâncias
AccelStepper stepperElevator(1, STEP_PIN_ELEVATOR, DIR_PIN_ELEVATOR);
AccelStepper stepperBaseRotation(1, STEP_PIN_BASEROTATION, DIR_PIN_BASEROTATION);
Servo servoGripper;
Servo servoWrist;

// Protocolo
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
float readUltrasonic(int idx);
void handleElevator(float steps);
void handleBaseRotation(float steps);
void handleGripper(float angle);
void handleWrist(float angle);

// Funções de tratamento comunicação
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

// Handlers
void (*h[N])(float) = {
  handleElevator,
  handleBaseRotation,
  handleGripper,
  handleWrist
};

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando...");

  stepperElevator.setMaxSpeed(1000);
  stepperElevator.setAcceleration(500);
  stepperBaseRotation.setMaxSpeed(1000);
  stepperBaseRotation.setAcceleration(500);

  servoGripper.attach(SERVO_PIN_GRIPPER);
  servoWrist.attach(SERVO_PIN_WRIST);

  for (int i = 0; i < NUM_ULTRASONIC; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {
  stepperElevator.run();
  stepperBaseRotation.run();

  read_uart();
  write_uart();
}

// UART
void read_uart() {
  static bool receiving = false;
  static byte index = 0;

  while (Serial.available()) {
    byte b = Serial.read();

    if (!receiving) {
      if (b == BYTE_INIT) {
        receiving = true;
        index = 0;
      }
    } else {
      if (b == BYTE_END) {
        if (index == SZ) {
          // pacote completo em pkt.b
          for (int i = 0; i < N; i++) {
            if (fabs(pkt.f[i] - last[i]) > 1e-5) {
              last[i] = pkt.f[i];
              h[i](pkt.f[i]);
            }
          }
          Serial.print("Recebido: ");
          for (int i = 0; i < N; i++) {
            Serial.print(pkt.f[i], 4);
            Serial.print(i < N-1 ? ", " : "\n");
          }
          Serial.println("");
        } else {
          Serial.println("❌ Tamanho inválido");
        }
        receiving = false;  // fim do pacote
      } else {
        if (index < SZ) {
          pkt.b[index++] = b;
        } else {
          Serial.println("❌ Overflow de pacote");
          receiving = false;  // descarta
        }
      }
    }
  }
}


void write_uart(){
  if (Serial.available() == 0) {
    for (int i = 0; i < NUM_ULTRASONIC; i++) {
      uspkt.f[i] = readUltrasonic(i);
      //uspkt.f[i] = i*10;
    }

    char send_pkg[US_SZ + 2];
    memcpy(send_pkg+1, uspkt.b, US_SZ);
    send_pkg[0] = BYTE_INIT;
    send_pkg[US_SZ+1] = BYTE_END;

    //Enviando
    Serial.write(send_pkg, US_SZ+2);
    delay(100); // ajuste de taxa
  }
}

float readUltrasonic(int idx) {
  uint8_t t = trigPins[idx], e = echoPins[idx];
  digitalWrite(t, LOW); delayMicroseconds(2);
  digitalWrite(t, HIGH); delayMicroseconds(10);
  digitalWrite(t, LOW);
  unsigned long dur = pulseIn(e, HIGH, 30000UL);
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) / 2.0f;
}
