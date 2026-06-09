#include "BluetoothSerial.h"
#include <math.h>
#include <stdlib.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no esta habilitado!
#endif

BluetoothSerial SerialBT;

#define PIN_IN1 22
#define PIN_IN2 19
#define PIN_IN3 18
#define PIN_IN4 21
#define PIN_ENA 25
#define PIN_ENB 26

#define CANAL_IZQ 0
#define CANAL_DER 1

const int PIN_C1_IZQ = 35;
const int PIN_C2_IZQ = 34;
const int PIN_C1_DER = 32;
const int PIN_C2_DER = 33;

const int PULSOS_POR_REV = 5600;
const double DIAMETRO_RUEDA = 4.5;
const double ANCHO_VIA = 13.5;
const unsigned long TIEMPO_MUESTREO = 100;

const int FRECUENCIA_PWM = 1000;
const int RESOLUCION_PWM = 8;
const int PWM_MAXIMO = 255;
const int PWM_MINIMO = 235;
const int PWM_BASE_IZQ = 246;
const int PWM_BASE_DER = 255;

volatile long pulsosAcumuladosIzq = 0;
volatile long pulsosAcumuladosDer = 0;

volatile int estadoAntIzq = 0, estadoActIzq = 0;
volatile int estadoAntDer = 0, estadoActDer = 0;

volatile double rpmIzq = 0.0;
volatile double rpmDer = 0.0;
double vLinealIzq = 0.0;
double vLinealDer = 0.0;
volatile long pulsosPreviosRPMIzq = 0;
volatile long pulsosPreviosRPMDer = 0;

double posicionGradosIzq = 0.0, posicionGradosDer = 0.0;
unsigned long tiempoAnterior = 0;
volatile long conteoMovimientoIzq = 0;
volatile long conteoMovimientoDer = 0;
volatile bool movimientoActivo = false;
volatile bool rotacionActiva = false;
bool modoGrafica = false;

int pwmIzquierda = PWM_BASE_IZQ;
int pwmDerecha = PWM_BASE_DER;

volatile bool movDistanciaActivo = false;
volatile long pulsosObjetivoDist = 0;
volatile long pulsosInicioDistIzq = 0, pulsosInicioDistDer = 0;
volatile long pulsosObjetivoRot = 0;
volatile long pulsosInicioRotIzq = 0, pulsosInicioRotDer = 0;

inline void registrarPulsoIzq() { 
  if (movimientoActivo) conteoMovimientoIzq++; 
}

inline void registrarPulsoDer() { 
  if (movimientoActivo) conteoMovimientoDer++; 
}

void IRAM_ATTR manejadorEncoderIzq() {
  int s1 = digitalRead(PIN_C1_IZQ);
  int s2 = digitalRead(PIN_C2_IZQ);
  estadoAntIzq = estadoActIzq;
  estadoActIzq = (s1 << 1) | s2;
  if (estadoAntIzq != estadoActIzq) {
    if ((estadoAntIzq == 0 && estadoActIzq == 1) || (estadoAntIzq == 1 && estadoActIzq == 3) || (estadoAntIzq == 3 && estadoActIzq == 2) || (estadoAntIzq == 2 && estadoActIzq == 0)) pulsosAcumuladosIzq++; 
    else pulsosAcumuladosIzq--;
    registrarPulsoIzq();
  }
}

void IRAM_ATTR manejadorEncoderDer() {
  int s1 = digitalRead(PIN_C1_DER);
  int s2 = digitalRead(PIN_C2_DER);
  estadoAntDer = estadoActDer;
  estadoActDer = (s1 << 1) | s2;
  if (estadoAntDer != estadoActDer) {
    if ((estadoAntDer == 0 && estadoActDer == 1) || (estadoAntDer == 1 && estadoActDer == 3) || (estadoAntDer == 3 && estadoActDer == 2) || (estadoAntDer == 2 && estadoActDer == 0)) pulsosAcumuladosDer++; 
    else pulsosAcumuladosDer--;
    registrarPulsoDer();
  }
}

void detener() {
  digitalWrite(PIN_IN1, LOW); 
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); 
  digitalWrite(PIN_IN4, LOW);
  movimientoActivo = false; 
  rotacionActiva = false;
}

void avanzar() {
  digitalWrite(PIN_IN1, HIGH); 
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH); 
  digitalWrite(PIN_IN4, LOW);
  movimientoActivo = true; 
  conteoMovimientoIzq = 0; 
  conteoMovimientoDer = 0; 
  rotacionActiva = false;
}

void retroceder() {
  digitalWrite(PIN_IN1, LOW); 
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW); 
  digitalWrite(PIN_IN4, HIGH);
  movimientoActivo = true; 
  conteoMovimientoIzq = 0; 
  conteoMovimientoDer = 0; 
  rotacionActiva = false;
}

void girarIzquierda() {
  digitalWrite(PIN_IN1, LOW); 
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); 
  digitalWrite(PIN_IN4, LOW);
  movimientoActivo = true; 
  rotacionActiva = true;
}

void girarDerecha() {
  digitalWrite(PIN_IN1, HIGH); 
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW); 
  digitalWrite(PIN_IN4, HIGH);
  movimientoActivo = true; 
  rotacionActiva = true;
}

void aplicarPWM() {
#ifdef SOC_LEDC_SUPPORT_HS_MODE
  ledcWrite(CANAL_IZQ, pwmIzquierda);
  ledcWrite(CANAL_DER, pwmDerecha);
#else
  ledcWrite(PIN_ENA, pwmIzquierda);
  ledcWrite(PIN_ENB, pwmDerecha);
#endif
}

void balancearAvance() {
  static long uConteoIzq = 0, uConteoDer = 0;
  static float integral = 0;
  static bool estabaActivo = false;
  const float Kp = 0.8;  
  const float Ki = 0.02; 
  if (!movimientoActivo || rotacionActiva) {
    uConteoIzq = conteoMovimientoIzq; 
    uConteoDer = conteoMovimientoDer;
    integral = 0; 
    estabaActivo = false;
    pwmIzquierda = PWM_BASE_IZQ; 
    pwmDerecha = PWM_BASE_DER;
    return;
  }
  if (!estabaActivo) {
    uConteoIzq = conteoMovimientoIzq; 
    uConteoDer = conteoMovimientoDer;
    integral = 0; 
    estabaActivo = true;
    return;
  }
  long dIzq = conteoMovimientoIzq - uConteoIzq;
  long dDer = conteoMovimientoDer - uConteoDer;
  uConteoIzq = conteoMovimientoIzq; 
  uConteoDer = conteoMovimientoDer;
  float error = (float)(dIzq - dDer);
  integral = constrain(integral + error, -300, 300);
  float correccion = Kp * error + Ki * integral;
  int ajuste = (int)roundf(correccion);
  pwmIzquierda = constrain(PWM_BASE_IZQ - ajuste, PWM_MINIMO, PWM_MAXIMO);
  pwmDerecha = constrain(PWM_BASE_DER + ajuste, PWM_MINIMO, PWM_MAXIMO);
}

void configurarMotores() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(PIN_ENA, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcAttach(PIN_ENB, FRECUENCIA_PWM, RESOLUCION_PWM);
#else
  ledcSetup(CANAL_IZQ, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcSetup(CANAL_DER, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcAttachPin(PIN_ENA, CANAL_IZQ);
  ledcAttachPin(PIN_ENB, CANAL_DER);
#endif
  aplicarPWM();
}

void ejecutarControl() {
  balancearAvance();
  aplicarPWM();
}

void calcularPosicion() {
  long cL, cR;
  noInterrupts(); 
  cL = pulsosAcumuladosIzq; 
  cR = pulsosAcumuladosDer; 
  interrupts();
  posicionGradosIzq = (cL * 360.0) / PULSOS_POR_REV;
  posicionGradosDer = (cR * 360.0) / PULSOS_POR_REV;
}

void reiniciarEncoders() {
  noInterrupts(); 
  pulsosAcumuladosIzq = 0; 
  pulsosAcumuladosDer = 0; 
  interrupts();
  pulsosPreviosRPMIzq = 0; 
  pulsosPreviosRPMDer = 0;
  rpmIzq = 0.0; 
  rpmDer = 0.0;
  conteoMovimientoIzq = 0; 
  conteoMovimientoDer = 0;
}

long cmAPulsos(double cm) {
  return (long)round((cm * PULSOS_POR_REV) / (PI * DIAMETRO_RUEDA));
}

long gradosAPulsos(double grados) {
  double distanciaArco = PI * ANCHO_VIA * (grados / 360.0);
  return (long)round((distanciaArco / (PI * DIAMETRO_RUEDA)) * PULSOS_POR_REV);
}

void calcularVelocidad() {
  long pIzq, pDer;
  noInterrupts(); 
  pIzq = pulsosAcumuladosIzq; 
  pDer = pulsosAcumuladosDer; 
  interrupts();
  long deltaIzq = pIzq - pulsosPreviosRPMIzq;
  long deltaDer = pDer - pulsosPreviosRPMDer;
  rpmIzq = (deltaIzq * 60000.0) / (TIEMPO_MUESTREO * PULSOS_POR_REV);
  rpmDer = (deltaDer * 60000.0) / (TIEMPO_MUESTREO * PULSOS_POR_REV);
  double circ = PI * DIAMETRO_RUEDA;
  vLinealIzq = (rpmIzq / 60.0) * circ;
  vLinealDer = (rpmDer / 60.0) * circ;
  pulsosPreviosRPMIzq = pIzq;
  pulsosPreviosRPMDer = pDer;
}

void iniciarMovimientoDist(double cm) {
  pulsosObjetivoDist = cmAPulsos(cm);
  noInterrupts(); 
  pulsosInicioDistIzq = pulsosAcumuladosIzq; 
  pulsosInicioDistDer = pulsosAcumuladosDer; 
  interrupts();
  avanzar();
  movDistanciaActivo = true;
}

void iniciarRetrocesoDist(double cm) {
  pulsosObjetivoDist = cmAPulsos(cm);
  noInterrupts(); 
  pulsosInicioDistIzq = pulsosAcumuladosIzq; 
  pulsosInicioDistDer = pulsosAcumuladosDer; 
  interrupts();
  retroceder();
  movDistanciaActivo = true;
}

void verificarMovimientoDist() {
  if (!movDistanciaActivo) return;
  long cL, cR;
  noInterrupts(); 
  cL = pulsosAcumuladosIzq; 
  cR = pulsosAcumuladosDer; 
  interrupts();
  if (labs(cL - pulsosInicioDistIzq) >= pulsosObjetivoDist || labs(cR - pulsosInicioDistDer) >= pulsosObjetivoDist) {
    detener(); 
    movDistanciaActivo = false;
  }
}

void iniciarRotacion(double grados) {
  if (grados == 0.0) return;
  pulsosObjetivoRot = abs(gradosAPulsos(grados));
  noInterrupts(); 
  pulsosInicioRotIzq = pulsosAcumuladosIzq; 
  pulsosInicioRotDer = pulsosAcumuladosDer; 
  interrupts();
  rotacionActiva = true;
  if (grados > 0) girarDerecha(); 
  else girarIzquierda();
}

void verificarRotacion() {
  if (!rotacionActiva) return;
  long cL, cR;
  noInterrupts(); 
  cL = pulsosAcumuladosIzq; 
  cR = pulsosAcumuladosDer; 
  interrupts();
  if (max(labs(cL - pulsosInicioRotIzq), labs(cR - pulsosInicioRotDer)) >= pulsosObjetivoRot) {
    detener();
  }
}

void recibirComandos() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd == "reset") reiniciarEncoders();
    else if (cmd == "PLOT ON") modoGrafica = true;
    else if (cmd == "PLOT OFF") modoGrafica = false;
    else if (cmd == "ADELANTE") avanzar();
    else if (cmd == "ATRAS") retroceder();
    else if (cmd == "STOP") detener();
    else if (cmd == "IZQ") girarIzquierda();
    else if (cmd == "DER") girarDerecha();
    else if (cmd.startsWith("AVANZA")) {
      double cm = 100;
      if (cmd.length() > 6) {
        String p = cmd.substring(6); 
        p.trim();
        if (p.length()) cm = p.toDouble();
      }
      if (cm >= 0) iniciarMovimientoDist(cm); 
      else iniciarRetrocesoDist(-cm);
    } else if (cmd.startsWith("RETROCEDER")) {
      double cm = 100;
      if (cmd.length() > 10) {
        String p = cmd.substring(10); 
        p.trim();
        if (p.length()) cm = p.toDouble();
      }
      iniciarRetrocesoDist(cm);
    } else if (cmd.startsWith("GIRO")) {
      double d = 0;
      if (cmd.length() > 4) {
        String p = cmd.substring(4); 
        p.trim();
        if (p.length()) d = p.toDouble();
      }
      iniciarRotacion(d);
    }
  }
}

void setup() {
  SerialBT.begin("MrRootBot");
  pinMode(PIN_C1_IZQ, INPUT_PULLUP); 
  pinMode(PIN_C2_IZQ, INPUT_PULLUP);
  pinMode(PIN_C1_DER, INPUT_PULLUP); 
  pinMode(PIN_C2_DER, INPUT_PULLUP);
  pinMode(PIN_IN1, OUTPUT); 
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); 
  pinMode(PIN_IN4, OUTPUT);
  attachInterrupt(PIN_C1_IZQ, manejadorEncoderIzq, CHANGE); 
  attachInterrupt(PIN_C2_IZQ, manejadorEncoderIzq, CHANGE);
  attachInterrupt(PIN_C1_DER, manejadorEncoderDer, CHANGE); 
  attachInterrupt(PIN_C2_DER, manejadorEncoderDer, CHANGE);
  configurarMotores();
}

void loop() {
  if (millis() - tiempoAnterior >= TIEMPO_MUESTREO) {
    tiempoAnterior = millis();
    calcularVelocidad();            
    ejecutarControl();  
    calcularPosicion(); 
    verificarMovimientoDist();
    verificarRotacion();    
    if (modoGrafica) SerialBT.printf("%.2f,%.2f,%d,%d\n", rpmIzq, rpmDer, pwmIzquierda, pwmDerecha);
    else {
      SerialBT.printf("RPM Izq: %.2f | Der: %.2f  |  PWM Izq: %d | Der: %d\n", rpmIzq, rpmDer, pwmIzquierda, pwmDerecha);
      SerialBT.printf("Pos Izq: %.2f | Der: %.2f  |  Dist Izq: %ld | Der: %ld\n\n", posicionGradosIzq, posicionGradosDer, pulsosAcumuladosIzq, pulsosAcumuladosDer);
    }
  }
  recibirComandos(); 
}
