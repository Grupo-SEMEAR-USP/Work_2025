#include <Wire.h>

#define ADR 0x0a
#define N   4
#define SZ  (N * sizeof(float))

union Packet { byte b[SZ]; float f[N]; } pkt;
float last[N] = {0};

void handleElevator(float v) { Serial.print("Elevator ");     Serial.println(v, 4); }
void handleBaseRotation(float v) { Serial.print("BaseRotation "); Serial.println(v, 4); }
void handleGripper(float v) { Serial.print("Gripper ");      Serial.println(v, 4); }
void handleWrist(float v) { Serial.print("Wrist ");        Serial.println(v, 4); }

void (*h[N])(float) = { handleElevator, handleBaseRotation, handleGripper, handleWrist };

void setup(){
  Serial.begin(115200);
  Wire.begin(ADR);
  Wire.onReceive(onRx);
}

void loop(){}

void onRx(int n){
  if(n != SZ){ while(Wire.available()) Wire.read(); return; }
  for(int i=0;i<SZ && Wire.available();i++) pkt.b[i] = Wire.read();
  for(int i=0;i<N;i++){
    if(fabs(pkt.f[i] - last[i]) > 1e-5){
      last[i] = pkt.f[i];
      h[i](pkt.f[i]);
    }
  }
}
