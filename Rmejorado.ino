#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

#define IN1 22
#define IN2 19
#define IN3 18
#define IN4 21

///////////////////// PARAMETROS //////////////////////
const int R = 5880;  // Resolución real

double posL = 0.0, posR = 0.0;
double rpmL = 0.0, rpmR = 0.0;

volatile long nL = 0;
volatile long nR = 0;

volatile int antL = 0, actL = 0;
volatile int antR = 0, actR = 0;

long last_nL = 0;
long last_nR = 0;

///////////////////// PINES //////////////////////
// IZQUIERDO
const int C1L = 35;
const int C2L = 34;

// DERECHO
const int C1R = 32;
const int C2R = 33;

///////////////////// TIEMPO //////////////////////
unsigned long lastTime = 0;
unsigned long sampleTime = 100;

///////////////////// INTERRUPCIONES //////////////////////

void IRAM_ATTR encoderL() {
  antL = actL;

  if (digitalRead(C2L)) bitSet(actL, 0);
  else bitClear(actL, 0);

  if (digitalRead(C1L)) bitSet(actL, 1);
  else bitClear(actL, 1);

  if (antL == 2 && actL == 0) nL++;
  if (antL == 0 && actL == 1) nL++;
  if (antL == 3 && actL == 2) nL++;
  if (antL == 1 && actL == 3) nL++;

  if (antL == 1 && actL == 0) nL--;
  if (antL == 3 && actL == 1) nL--;
  if (antL == 0 && actL == 2) nL--;
  if (antL == 2 && actL == 3) nL--;
}

void IRAM_ATTR encoderR() {
  antR = actR;

  if (digitalRead(C2R)) bitSet(actR, 0);
  else bitClear(actR, 0);

  if (digitalRead(C1R)) bitSet(actR, 1);
  else bitClear(actR, 1);

  if (antR == 2 && actR == 0) nR++;
  if (antR == 0 && actR == 1) nR++;
  if (antR == 3 && actR == 2) nR++;
  if (antR == 1 && actR == 3) nR++;

  if (antR == 1 && actR == 0) nR--;
  if (antR == 3 && actR == 1) nR--;
  if (antR == 0 && actR == 2) nR--;
  if (antR == 2 && actR == 3) nR--;
}

///////////////////// FUNCIONES //////////////////////

void calcularPosicion() {
  long cL, cR;

  noInterrupts();
  cL = nL;
  cR = nR;
  interrupts();

  posL = (cL * 360.0) / R;
  posR = (cR * 360.0) / R;
}

void calcularVelocidad() {
  long aL, aR;

  noInterrupts();
  aL = nL;
  aR = nR;
  interrupts();

  long dL = aL - last_nL;
  long dR = aR - last_nR;

  last_nL = aL;
  last_nR = aR;

  rpmL = (dL * 60.0) / R;
  rpmR = (dR * 60.0) / R;
}

void resetEncoders() {
  noInterrupts();
  nL = 0;
  nR = 0;
  interrupts();
}

void alto() {  //0000
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void adelante() {  //1010
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void atras() {  //0101
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void giroIzq() {  //0110
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void giroDer() {  //1001
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


void setup() {
  SerialBT.begin("MrRootBot");

  pinMode(C1L, INPUT_PULLUP);
  pinMode(C2L, INPUT_PULLUP);

  pinMode(C1R, INPUT_PULLUP);
  pinMode(C2R, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  attachInterrupt(C1L, encoderL, CHANGE);
  attachInterrupt(C2L, encoderL, CHANGE);

  attachInterrupt(C1R, encoderR, CHANGE);
  attachInterrupt(C2R, encoderR, CHANGE);

  SerialBT.println("Sistema 2 motores listo");
}

///////////////////// LOOP //////////////////////
void loop() {
  if (millis() - lastTime >= sampleTime) {
    lastTime = millis();

    calcularPosicion();
    calcularVelocidad();

    SerialBT.print("L: ");
    SerialBT.print(posL);
    SerialBT.print(" deg | ");

    SerialBT.print("R: ");
    SerialBT.print(posR);
    SerialBT.print(" deg | ");

    SerialBT.print("RPM L: ");
    SerialBT.print(rpmL);
    SerialBT.print(" | RPM R: ");
    SerialBT.println(rpmR);
  }

  recibirComandos();
}

///////////////////// COMANDOS //////////////////////
void recibirComandos() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "reset") {
      resetEncoders();
      SerialBT.println("Encoders reseteados");
    } else if (cmd == "ADELANTE") {
      adelante();
    } else if (cmd == "ATRAS") {
      atras();
    } else if (cmd == "STOP") {
      alto();
    } else if (cmd == "IZQ") {
      giroIzq();
    } else if (cmd == "DER") {
      giroDer();
    }
  }
}