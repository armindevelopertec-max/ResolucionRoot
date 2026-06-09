#include "BluetoothSerial.h"
#include <math.h>
#include <stdlib.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

// ===================== MOTORES =====================
#define IN1 22
#define IN2 19
#define IN3 18
#define IN4 21

// ===================== ENCODERS =====================
const int C1L = 35;
const int C2L = 34;
const int C1R = 32;
const int C2R = 33;

// ===================== VARIABLES =====================
volatile long nL = 0;
volatile long nR = 0;

volatile int antL = 0, actL = 0;
volatile int antR = 0, actR = 0;

// RPM
volatile double velocidadL = 0.0;
volatile double velocidadR = 0.0;

// Velocidad lineal
double velocidadLinealL = 0.0;
double velocidadLinealR = 0.0;

// Constantes encoder
const int R = 5600;

// Posición
double posL = 0.0, posR = 0.0;

// Tiempo
unsigned long lastTime = 0;
unsigned long sampleTime = 100; // ms

// Aux RPM
volatile long prev_pulses_rpmL = 0;
volatile long prev_pulses_rpmR = 0;

// Robot
double diametro = 4.5; // cm
volatile long movementCountL = 0;
volatile long movementCountR = 0;
volatile bool movementActive = false;
volatile bool distanceMoveActive = false;
volatile long distanceTargetPulses = 0;
volatile long distanceStartL = 0;
volatile long distanceStartR = 0;
volatile bool rotationActive = false;
volatile long rotationTargetPulses = 0;
volatile long rotationStartL = 0;
volatile long rotationStartR = 0;
const double trackWidth = 13.5; 
// ===================== CONTROL =====================
void setupControl();
void ejecutarControl();
extern int pwmL;
extern int pwmR;
void startDistanceMove(double cm);
void checkDistanceMove();
void startRotation(double degrees);
void checkRotation();
// ===================== INTERRUPCIONES =====================
inline void recordPulseL() {
  if (movementActive) movementCountL++;
}

inline void recordPulseR() {
  if (movementActive) movementCountR++;
}

void IRAM_ATTR encoderL() {
  antL = actL;

  if (digitalRead(C2L)) bitSet(actL, 0);
  else bitClear(actL, 0);

  if (digitalRead(C1L)) bitSet(actL, 1);
  else bitClear(actL, 1);

  if (antL == 2 && actL == 0) {    nL++;    recordPulseL();  }
  if (antL == 0 && actL == 1) {    nL++;    recordPulseL();  }
  if (antL == 3 && actL == 2) {    nL++;    recordPulseL();  }
  if (antL == 1 && actL == 3) {    nL++;    recordPulseL();  }

  if (antL == 1 && actL == 0) {    nL--;    recordPulseL();  }
  if (antL == 3 && actL == 1) {    nL--;    recordPulseL();  }
  if (antL == 0 && actL == 2) {    nL--;    recordPulseL();  }
  if (antL == 2 && actL == 3) {    nL--;    recordPulseL();  }
}

void IRAM_ATTR encoderR() {
  antR = actR;

  if (digitalRead(C2R)) bitSet(actR, 0);
  else bitClear(actR, 0);

  if (digitalRead(C1R)) bitSet(actR, 1);
  else bitClear(actR, 1);

  if (antR == 2 && actR == 0) {    nR++;    recordPulseR();  }
  if (antR == 0 && actR == 1) {    nR++;    recordPulseR();  }
  if (antR == 3 && actR == 2) {    nR++;    recordPulseR();  }
  if (antR == 1 && actR == 3) {    nR++;    recordPulseR();  }

  if (antR == 1 && actR == 0) {    nR--;    recordPulseR();  }
  if (antR == 3 && actR == 1) {    nR--;    recordPulseR();  }
  if (antR == 0 && actR == 2) {    nR--;    recordPulseR();  }
  if (antR == 2 && actR == 3) {    nR--;    recordPulseR();  }
}

// ===================== FUNCIONES =====================

void calcularPosicion() {
  long cL, cR;

  noInterrupts();
  cL = nL;
  cR = nR;
  interrupts();

  posL = (cL * 360.0) / R;
  posR = (cR * 360.0) / R;
}

void resetEncoders() {
  noInterrupts();
  nL = 0;
  nR = 0;
  interrupts();

  prev_pulses_rpmL = 0;
  prev_pulses_rpmR = 0;

  velocidadL = 0.0;
  velocidadR = 0.0;

  movementCountL = 0;
  movementCountR = 0;
}

long cmToPulses(double cm) {
  double circumference = PI * diametro;
  return (long)round((cm * R) / circumference);
}


void startDistanceMove(double cm) {
  distanceTargetPulses = cmToPulses(cm);
  noInterrupts();
  distanceStartL = nL;
  distanceStartR = nR;
  interrupts();
  adelante();
  distanceMoveActive = true;
}
void startDistanceMoveRETRO(double cm) {
  distanceTargetPulses = cmToPulses(cm);
  noInterrupts();
  distanceStartL = nL;
  distanceStartR = nR;
  interrupts();
  atras();
  distanceMoveActive = true;
}

void checkDistanceMove() {
  if (!distanceMoveActive) return;

  long currentL, currentR;
  noInterrupts();
  currentL = nL;
  currentR = nR;
  interrupts();

  long traveledL = labs(currentL - distanceStartL);
  long traveledR = labs(currentR - distanceStartR);

  if (traveledL >= distanceTargetPulses && traveledR >= distanceTargetPulses) {
    alto();
    distanceMoveActive = false;
  }
}

long degreesToPulses(double degrees) {
  double distance = PI * trackWidth * (degrees / 360.0);
  double rotations = distance / (PI * diametro);
  return (long)round(rotations * R);
}

void startRotation(double degrees) {
  if (degrees == 0.0) return;

  rotationTargetPulses = abs(degreesToPulses(degrees));
  noInterrupts();
  rotationStartL = nL;
  rotationStartR = nR;
  interrupts();

  rotationActive = true;
  movementActive = false;
  if (degrees > 0) giroDer();
  else giroIzq();
}

void checkRotation() {
  if (!rotationActive) return;

  long currentL, currentR;
  noInterrupts();
  currentL = nL;
  currentR = nR;
  interrupts();

  long traveledL = labs(currentL - rotationStartL);
  long traveledR = labs(currentR - rotationStartR);

  if (max(traveledL, traveledR) >= rotationTargetPulses) {
    alto();
    rotationActive = false;
  }
}

// ===================== MOVIMIENTO =====================
void alto() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  movementActive = false;
  rotationActive = false;
}

void adelante() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  movementActive = true;
  movementCountL = 0;
  movementCountR = 0;
  rotationActive = false;
}

void atras() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  movementActive = true;
  rotationActive = false;
}

void giroIzq() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  movementActive = true;
}

void giroDer() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  movementActive = true;
}

// ===================== RPM =====================
void V_Rpm(void)
{
  long pulsesL, pulsesR;

  noInterrupts();
  pulsesL = nL;
  pulsesR = nR;
  interrupts();

  long deltaL = pulsesL - prev_pulses_rpmL;
  long deltaR = pulsesR - prev_pulses_rpmR;

  velocidadL = (deltaL * 60000.0) / (sampleTime * R);
  velocidadR = (deltaR * 60000.0) / (sampleTime * R);

  double longitud = PI * diametro;

  velocidadLinealL = (velocidadL / 60.0) * longitud;
  velocidadLinealR = (velocidadR / 60.0) * longitud;

  prev_pulses_rpmL = pulsesL;
  prev_pulses_rpmR = pulsesR;
}

// ===================== SETUP =====================
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

  setupControl();

  SerialBT.println("Sistema listo");
}

// ===================== LOOP =====================
void loop() {

  if (millis() - lastTime >= sampleTime) {
    lastTime = millis();

    V_Rpm();
    ejecutarControl(); // 👈 sincronizado con RPM

    calcularPosicion();
    checkDistanceMove();
    checkRotation();

    SerialBT.printf("RPM L: %.2f | RPM R: %.2f\n", velocidadL, velocidadR);
    SerialBT.printf("CM/s L: %.2f | CM/s R: %.2f\n", velocidadLinealL, velocidadLinealR);
    SerialBT.printf("Pos L: %.2f | Pos R: %.2f\n", posL, posR);
    SerialBT.printf("PWM L: %d | PWM R: %d\n", pwmL, pwmR);
    SerialBT.printf("Movement L: %ld | Movement R: %ld\n\n", movementCountL, movementCountR);
  }

  recibirComandos();
}

// ===================== COMANDOS =====================
void recibirComandos() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "reset") resetEncoders();
    else if (cmd == "ADELANTE") adelante();
    else if (cmd == "ATRAS") atras();
    else if (cmd == "STOP") alto();
    else if (cmd == "IZQ") giroIzq();
    else if (cmd == "DER") giroDer();
    else if (cmd.startsWith("AVANZA")) {
      double cm = 100;
      if (cmd.length() > 6) {
        String param = cmd.substring(6);
        param.trim();
        if (param.length()) cm = param.toDouble();
      }
      if (cm >= 0) startDistanceMove(cm);
      else startDistanceMoveRETRO(-cm);
    }else if (cmd.startsWith("RETROCEDER")) {
      double cm = 0;
      if (cmd.length() > 6) {
        String param = cmd.substring(6);
        param.trim();
        if (param.length()) cm = param.toDouble();
      }
      startDistanceMoveRETRO(cm);
    }
    else if (cmd.startsWith("GIRO")) {
      double degrees = 0;
      if (cmd.length() > 4) {
        String param = cmd.substring(4);
        param.trim();
        if (param.length()) degrees = param.toDouble();
      }
      startRotation(degrees);
    }
  }
}
