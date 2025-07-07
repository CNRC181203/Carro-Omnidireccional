/*********************************************************************/
/***************************   LIBRARIES   ***************************/
/*********************************************************************/

#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <EEPROM.h>
#include "esquinas.h"
/*********************************************************************/
/****************************   CLASSES   ****************************/
/*********************************************************************/

class SimplePID{
  /* 
  This class computes the control signal for velocity control of a 
  DC motor with a PID controller base on controller gains, a setpoint and 
  the actual velocity value.
  Params:
    - kp: Proportional gain.
    - kd: Derivative gain.
    - ki: Integral gain.
    - umax: Maximun control value.
    - eprec: Previous error in the control loop.
    - umax: Integral cumulative error.
    - vmin: Minimun velocity in rpm.
  */
  private:
    float kp, kd, ki, umax, vmin; // Parameters
    float eprev, eintegral; // Cumulative variables

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), vmin(15.0){}
  // A function to set the parameters   
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn, float vminIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn; vmin = vminIn;
  }
  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr){
    // Error
    float e = (target - value)*((float) fabs(target) > vmin);
    // Derivative
    float dedt = (e - eprev)/(deltaT)*((float) fabs(target) > vmin);
    // Integral
    eintegral = (eintegral + e * deltaT)*((float) fabs(target) > vmin);
    if (umax/ki<eintegral){
      eintegral = umax/ki;
    }
    if (-umax/ki>eintegral){
      eintegral = -umax/ki;
    }
    // Control signal
    float u = kp * e + kd * dedt + ki * eintegral;
    pwr = (int) fabs(u);
    // Truncate signal
    if (pwr > umax) {
      pwr = umax;
    }
    if (pwr < 0) {
      pwr = 0;
    }
    eprev = e;
  }
};

/*********************************************************************/
/***************************   VARIABLES   ***************************/
/*********************************************************************/

// Number of motors
#define NMOTORS 4
// Pins
const int enc[] = {4, 5, 8, 13};
const int DIR[] = {2, 7, 9, 12};
const int pwm[] = {3, 6, 10, 11};

// Globals
int posPrev[] = {0, 0, 0, 0};
float vel[]   = {0.0, 0.0, 0.0, 0.0};
float vt[]    = {0.0, 0.0, 0.0, 0.0};
float vfil[]  = {0.0, 0.0, 0.0, 0.0};
int dir[]     = {0, 0, 0, 0};
long prevT    = 0;
// PPR of each motor
const double ppr[] = {1390, 1390, 1390, 1390}; 
// const double ppr[] = {780, 780, 780, 780}; 
// Robot dimentions
const double a_b = 0.2025, R = 0.04;
const double l_a_b = 1/0.2025;

// Parámetros del filtro
const float Qkalman = 0.0001;    
const float Rkalman = 0.01;    

// Pose variables
double theta = 0;
double xPrev = 0, yPrev = 0, thetaPrev = 0; 


// PID class instances
SimplePID pid[NMOTORS];

// Dynamic variables
float velAng[]     = {0, 0, 0, 0};
int velEnc[]       = {0, 0, 0, 0};
int velEncSlack[]  = {0, 0, 0, 0};
float sampleT      = 0.1;
// Target variables for control
double vx = 0, vy = 0, vw = 0;



// Case variable
int secuencia = 1;
// Time periods of sequences
double T1 = 1.0, T2 = 1.0, T3 = 1.0;
// Initial and elapsed time of a sequence
double t0 = 0.0, t1 = 0.0;
// Velocity limits
double vminLim = 15.0;
// Velocities signs
int sgn[]     = {1, 1, 1, 1};
int sgnPrev[] = {1, 1, 1, 1};

//IMU
MPU6050 sensor;
int gx, gy, gz;
long tiempo_prev, dt;

int seq = 0;
float thetaGoal = PI/4;
double errorTheta = PI;

float directionGoal = 2*PI;
const int NUM_ESQUINAS = sizeof(esquinas) / sizeof(esquinas[0]);

float h_robot[NUM_ESQUINAS][2];

// Variables para control de movimiento
int esquina_actual = 1;  // Empezamos en la segunda esquina para el primer desplazamiento relativo
float xGoal = 0.0, yGoal = 0.0;
bool en_movimiento = true;

// Variables para posición actual estimada del robot (deberías actualizar estas según tu odometría)
float x = 0.0;
float y = 0.0;

// Variables por rueda
float x_est[NMOTORS];  // estimación de velocidad (rad/s)
float P[NMOTORS];      // incertidumbre (varianza) de la estimación

//Kalman filter imu
double state = 0, Pangulo = 0, Kangulo = 0;   // State, Proccess error variance, Kalman Gain
double Q_vAng  = 0.0025, R_angle = 0.00085; // Ruido de la medición

const int UF  = A1;  // Ultra Frontal
const int ULI = A2;  // Ultra Izquierdo
const int ULD = A0;  // Ultra Derecho

// Variables Kalman para sensores ultrasónicos
float US_state[3] = {0.0, 0.0, 0.0};      // Estimación actual
float US_P[3]     = {0.0, 0.0, 0.0};      // Incertidumbre
const float US_Q  = 0.005;                // Varianza del proceso (ajustable)
const float US_R  = 0.15;                 // Varianza de medición (ajustable)

bool corrigiendo = false;
double tiempo_corr_inicia = 0.0;

static int lastDySign = 0;
static float acumuladorExtra = 0.0;

/*********************************************************************/
/*****************************   SETUP   *****************************/
/*********************************************************************/

void setup() {
  // Begin communication
  Serial.begin(9600);
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
  sensor.setXAccelOffset(-3046);
  sensor.setYAccelOffset(1349);
  sensor.setZAccelOffset(921);
  sensor.setXGyroOffset(80);
  sensor.setYGyroOffset(93);
  sensor.setZGyroOffset(-25);
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  // Setup I/O pins
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enc[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(DIR[k], OUTPUT);
  }
  // PID gains for each motor
  pid[0].setParams(0.5, 0.02, 3.0, 255, vminLim);
  pid[1].setParams(0.5, 0.02, 3.0, 255, vminLim);
  pid[2].setParams(0.5, 0.02, 3.0, 255, vminLim);
  pid[3].setParams(0.5, 0.02, 3.0, 255, vminLim);


  // Activate interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[0]), readEncoder<0>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[1]), readEncoder<1>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[2]), readEncoder<2>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[3]), readEncoder<3>, CHANGE);
  delay(2000);
  prevT = micros();
  tiempo_prev = millis();

  for (int i = 0; i < NUM_ESQUINAS; i++) {
    float x_esq = esquinas[i][0];
    float y_esq = esquinas[i][1];

    h_robot[i][0] = -y_esq;  // x_robot = -y_esq
    h_robot[i][1] = -x_esq;  // y_robot = -x_esq
  }
  // Definir primer objetivo relativo si esquina_actual > 0
  xGoal = h_robot[1][0] - h_robot[0][0];
  yGoal = h_robot[1][1] - h_robot[0][1];
}

/*********************************************************************/
/*****************************   LOOP   ******************************/
/*********************************************************************/
void loop() {
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  t1 = ((float)(currT)) / 1.0e6;
  //IMU
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  sensor.getRotation(&gx,&gy,&gz);
  dt = millis() - tiempo_prev;
  tiempo_prev = millis();
  theta = theta + (float)(gz/131)*dt/1000*PI/180;
  //theta=kalmanFilterAngulo(deltaT,vw,theta);
  if (theta >= PI){
    theta -= 2*PI;
  }
  else if (theta <= -PI){
    theta += 2*PI;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (sampleT <= deltaT) {
    prevT = currT;

    noInterrupts();
    for (int k = 0; k < NMOTORS; k++) {
      velEncSlack[k] = velEnc[k];
      velEnc[k] = 0;
    }
    interrupts();

    
    for (int k = 0; k < NMOTORS; k++) {
      // Calcular velocidad en RPM
      vel[k] = velEncSlack[k] / deltaT / ppr[k] * 60.0;
      velAng[k]= sgn[k] * vel[k] * PI / 30.0;
      // Convertir a rad/s
      //float vel_rad = sgn[k] * vel[k] * PI / 30.0;

      // Filtrar con Kalman
      //KalmanUpdate(k, vel_rad);

      // Guardar velocidad angular filtrada
      //velAng[k] = x_est[k];
    }


    EulerEstimation(deltaT);
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Verificar si se llegó al objetivo actual
  if (en_movimiento && fabs(xGoal - x) < 0.05 && fabs(yGoal - y) < 0.05) {
      // Si el carrito ha llegado a la esquina, rotamos para poner la orientación en 0
       // Inicia el error de orientación
      CalculateOrientationError(0, vx, vy, vw, errorTheta);  // Calculamos el giro necesario para alinear a 0
      // Ejecutamos el movimiento de rotación
      CalculateVelAng(0, 0, vw);

      // Si la orientación es suficientemente precisa, pasamos a la siguiente esquina
      if (fabs(errorTheta) < PI/54) {
        // Hemos alcanzado la orientación deseada, entonces nos movemos a la siguiente esquina
        if (esquina_actual < NUM_ESQUINAS - 1) {
          esquina_actual++;
          float dx = h_robot[esquina_actual][0] - h_robot[esquina_actual - 1][0];
          float dy = h_robot[esquina_actual][1] - h_robot[esquina_actual - 1][1];
          xGoal += dx;
          yGoal+=dy*1.12;
        }
        else {
          en_movimiento = false;  // No hay más esquinas por visitar, terminamos el movimiento
        }
      }
    } else if(en_movimiento) {
      // Si no estamos en la posición correcta, calculamos las velocidades para ir a la siguiente esquina
      CalculatePositionError(xGoal, yGoal, vx, vy, vw);
      CalculateVelAng(vx, vy, vw);  
    }

    // Lectura de sensores ultrasónicos
    //float dist_izq = leerDistancia(ULI);
    //float dist_der = leerDistancia(ULD);
    //float dist_front = leerDistancia(UF);
    float dist_izq   = leerDistanciaFiltrada(A2, 0);  // índice 0
    float dist_front = leerDistanciaFiltrada(A1, 1);  // índice 1
    float dist_der   = leerDistanciaFiltrada(A0, 2);  // índice 2
    // Verificar si se requiere corrección
    if (dist_izq < 7.5 || dist_der < 7.5 || dist_front < 8.5) {
      if (!corrigiendo) {
        tiempo_corr_inicia = millis() / 1000.0;
        corrigiendo = true;
        StopMotors();
      }
      corregirPosicion(dist_izq, dist_der, dist_front);
      return;  // Saltamos todo lo demás en el loop mientras corregimos
    } 
    else if (corrigiendo) {
      // Salida de corrección: ajustar tiempo base si fuera necesario
      double tiempo_corr = (millis() / 1000.0) - tiempo_corr_inicia;
      t0 += tiempo_corr;
      corrigiendo = false;
    }

    if (!en_movimiento) {
    StopMotors();  // Detenemos todos los motores
    }

    for (int k = 0; k < NMOTORS; k++) {
      int pwr;
      pid[k].evalu(vel[k], vt[k], deltaT, pwr);
      setMotor(dir[k], pwr, pwm[k], DIR[k]);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
}

/******************************************************************************************************************************/
/************************************************   FUNCTIONS   ***************************************************************/
/******************************************************************************************************************************/

void CalculatePositionError(double xGoal_l, double yGoal_l, double &vx, double &vy, double &vw){
    // Error en posición
    double errorX = xGoal_l-x;
    double errorY = yGoal_l-y;
    
    //Probar valores de K y usar el que dé un buen resultado 
    //kp_pos es la constante proporcional al error de la posición
    double kp_pos=1.65;

    // Calcular la velocidad global en función del error y la constante k en cada eje
    double vx_global = kp_pos*errorX;
    double vy_global = kp_pos*errorY;

    double ct = cos(0); 
    double st = sin(0);

    // No hay rotación
    vw = 0;

    // Calcular la velocidad relativa para el robot: vx y vy
    vx = cos(0)*vx_global+sin(0)*vy_global;
    vy = cos(0)*vy_global-sin(0)*vx_global;

}

void CalculateOrientationError(double thetaGoal_l, double &vx, double &vy, double &vw, double &errorTheta){
    // Error de orientación
    errorTheta = thetaGoal_l-theta;

    //No modificar el condicional
    if (errorTheta >= PI){
      errorTheta -= 2*PI;
    }
    else if (errorTheta <= -PI){
      errorTheta += 2*PI;
    }

    //Colocar el valor a vx y vy, de manera que solo haya rotación
    /* COMPLETAR  */
    vx = 0;
    vy = 0;

    // Calcular la velocidad angular en función del error y la constante kp_ang
    //kp_ang es la constante proporcional al error del ángulo

    double kp_ang = 1.85;
    vw = kp_ang*errorTheta;

    // Lógica en caso vw sea mayor a PI/5
    if (fabs(vw) > PI/5){
        vw = (vw/fabs(vw))*PI/5;
    }
    // Lógica en caso errorTheta sea menor a 0.4, que genera velocidades muy pequeñas, asignar un valor alto de velocidad angular con el símbolo correspondiente
    if (fabs(errorTheta)<0.4){
        vw = (errorTheta/fabs(errorTheta))*PI/5.3;
    }

}


/******************************************************************************************************************************/
/************************************************  NO MODIFICAR ***************************************************************/
/******************************************************************************************************************************/

void EulerEstimation(double deltaT){
  /* 
  Function that integrates numericaly online robot position.
  Inputs:
    - deltaT: Time step.
  */
  // Pre-compute cosine and sine
  double ct, st;
  
  ct = cos(0); st = sin(0);

  // Euler numerical estimation (use pose and processed variables)

  x = xPrev + deltaT*(R/4 * ((ct+st)*velAng[0] + (ct-st)*velAng[1] + (ct-st)*velAng[2] + (ct+st)*velAng[3]));
  y = yPrev + deltaT*(R/4 * ((st-ct)*velAng[0] + (st+ct)*velAng[1] + (st+ct)*velAng[2] + (st-ct)*velAng[3]));
  

  // Update previous values
  xPrev = x;
  yPrev = y;
  thetaPrev = theta;

  // Print estimated pose
  Serial.print("X = ");
  Serial.print(x*1000);
  Serial.print(" (mm), ");
  Serial.print("Y = ");
  Serial.print(y*1000);
  Serial.print(" (mm), ");
  Serial.print("theta = ");
  Serial.print(theta*180/PI);
  Serial.println(" (deg)");
  Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%");

}


void CalculateVelAng(double vx, double vy, double vw) { 
  /* 
  Function that computes the velocity in rpm and the direction 
  of each wheel from the absolute velocity.

  Inputs:
    - vx: Linear velocity in X axis, in m/s.
    - vy: Linear velocity in Y axis, in m/s.
    - vw: Angular velocity in Z axis, in rad/s.
  */
  double w[] = {0, 0, 0, 0};
  // Angular velocity of each motor in rad/s
  w[0] = (vx - vy - vw * a_b) / R;
  w[1] = (vx + vy + vw * a_b) / R;
  w[2] = (vx + vy - vw * a_b) / R;
  w[3] = (vx - vy + vw * a_b) / R;
  for (int i = 0; i < NMOTORS; i++) {
    sgnPrev[i] = sgn[i];
    sgn[i] = w[i] / fabs(w[i]); 
    // Update motor direction
    dir[i] = (1 + sgn[i]) / 2;
    // Calculate desired angular velocity in rpm
    vt[i] = fabs(w[i]*30/PI);
  }
}

void setMotor(int dir, int pwmVal, int pwmch, int dirch) {
  /* 
  Function to setup pins to control motors.

  Inputs:
    - dir: Motor direction (1 or 0).
    - pwmVal: PWM control to pin.
    - pwmch: PWM pin channel.
    - dirch: Direction pin channel.
  */
  analogWrite(pwmch, pwmVal);
  if(dirch==12 || dirch==7){
    if (dir == 1) {
      digitalWrite(dirch, LOW);
    } else if (dir == 0) {
      digitalWrite(dirch, HIGH);
    } else {
      digitalWrite(dirch, LOW);
    }
  }
  else{
    if (dir == 1) {
      digitalWrite(dirch, HIGH);
    } else if (dir == 0) {
      digitalWrite(dirch, LOW);
    } else {
      digitalWrite(dirch, HIGH);
    }
  }
}


template <int j>
void readEncoder() {
  /* 
  Function that counts each rising edge of a encoder
  */
  velEnc[j]++;
}


void StopMotors(){
  /* 
  Function that stops each DC motor. 
  */
  CalculateVelAng(0,0,0);
  for(int k = 0; k < NMOTORS; k++){
    setMotor(dir[k], 0.0, pwm[k], DIR[k]);
  }
  CalculateVelAng(0,0,0);
  delay(100);
}

// Función que actualiza el estimado de Kalman para la rueda k
void KalmanUpdate(int k, float z) {
  // Paso 1: predicción
  P[k] += Qkalman;

  // Paso 2: ganancia de Kalman
  float K = P[k] / (P[k] + Rkalman);

  // Paso 3: corrección con la medición z (velocidad actual del encoder)
  x_est[k] += K * (z - x_est[k]);

  // Paso 4: actualización de incertidumbre
  P[k] *= (1 - K);
}

double kalmanFilterAngulo(float dt, float u_n, float z_n){

  /*******************Define prediction equations************************/
  state = state + u_n * dt;
  Pangulo     = Pangulo + Q_vAng*pow(dt,2);
  /**********************************************************************/

  /*******************Define update equations************************/
  Kangulo     = Pangulo / (Pangulo + R_angle*pow(dt,2));

  state = state + Kangulo * (z_n - state);
  Pangulo     = (1 - Kangulo) * Pangulo;
  /**********************************************************************/

  return state;
}

float leerDistancia(int sensorPin) {
  int lecturaADC = analogRead(sensorPin);
  float distancia = lecturaADC * 0.3 + 0.04; // Aproximación lineal
  return distancia;
}

float leerDistanciaFiltrada(int sensorPin, int idx) {
  int lecturaADC = analogRead(sensorPin);
  float distancia = lecturaADC * 0.3 + 0.04;
  return kalmanUS(idx, distancia);  // Aplica Kalman
}

void corregirPosicion(double dist_izq, double dist_der, double dist_front) {
  double vx_c = 0.0, vy_c = 0.0, vw_c = 0.0;

  if (dist_izq < 7.5) {
    vy_c = -0.1;  // Corrección hacia la derecha
  } 
  else if (dist_der < 7.5) {
    vy_c = 0.1;   // Corrección hacia la izquierda
  } 
  else if (dist_front < 8.5) {
    vx_c = -0.1;  // Retroceder
  }

  CalculateVelAng(vx_c, vy_c, vw_c);
  for (int k = 0; k < NMOTORS; k++) {
    int pwr;
    pid[k].evalu(vel[k], vt[k], 0.1, pwr);  // deltaT fijo para corrección
    setMotor(dir[k], pwr, pwm[k], DIR[k]);
  }
}

float kalmanUS(int idx, float z) {
  // Paso 1: predicción
  US_P[idx] += US_Q;

  // Paso 2: ganancia de Kalman
  float K = US_P[idx] / (US_P[idx] + US_R);

  // Paso 3: actualización
  US_state[idx] += K * (z - US_state[idx]);

  // Paso 4: actualizar incertidumbre
  US_P[idx] *= (1 - K);

  return US_state[idx];
}

