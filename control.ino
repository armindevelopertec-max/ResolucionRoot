// ===================== PWM =====================
#define ENA 25
#define ENB 26

const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;

const int PWM_MAX = 255;
const int PWM_MIN = 235;
const int FORWARD_PWM_L_BASE = 246;
const int FORWARD_PWM_R_BASE = 255;

int pwmL = FORWARD_PWM_L_BASE;
int pwmR = FORWARD_PWM_R_BASE;

extern volatile bool movementActive;
extern volatile long movementCountL;
extern volatile long movementCountR;
extern volatile bool rotationActive;

void applyPWM() {
  ledcWrite(ENA, pwmL);
  ledcWrite(ENB, pwmR);
}

void balanceForwardPWM() {
  static long lastCountL = 0;
  static long lastCountR = 0;
  static float integral = 0;
  static bool wasActive = false;
  const float Kp = 0.8;
  const float Ki = 0.02;

  if (!movementActive) {
    lastCountL = movementCountL;
    lastCountR = movementCountR;
    integral = 0;
    wasActive = false;
    pwmL = FORWARD_PWM_L_BASE;
    pwmR = FORWARD_PWM_R_BASE;
    return;
  }

  if (rotationActive) {
    pwmL = FORWARD_PWM_L_BASE;
    pwmR = FORWARD_PWM_R_BASE;
    return;
  }

  if (!wasActive) {
    lastCountL = movementCountL;
    lastCountR = movementCountR;
    integral = 0;
    wasActive = true;
    return;
  }

  long deltaL = movementCountL - lastCountL;
  long deltaR = movementCountR - lastCountR;
  lastCountL = movementCountL;
  lastCountR = movementCountR;

  float error = (float)(deltaL - deltaR);
  integral = constrain(integral + error, -300, 300);
  float correction = Kp * error + Ki * integral;

  int adjust = (int)roundf(correction);
  pwmL = constrain(FORWARD_PWM_L_BASE - adjust, PWM_MIN, PWM_MAX);
  pwmR = constrain(FORWARD_PWM_R_BASE + adjust, PWM_MIN, PWM_MAX);
}

// ===================== CONTROL =====================
void setupControl() {
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  applyPWM();
}

void ejecutarControl() {
  balanceForwardPWM();
  applyPWM();
}
