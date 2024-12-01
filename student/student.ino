#include <Servo.h>
#include <SignalSource.h>
#include <math.h>


// Global Variables
//TODO
static int16_t last_error = 0;
static int16_t integral_sum = 0;
static unsigned long last_time = 0;
static int16_t total_steps = 1024;
static int16_t offset = total_steps / 2;

// Globale Konstanten für den PID-Regler
//const float Kp = 0.1;    // Proportionalanteil
//const float Ki = 0.0;    // Integralanteil
//const float Kd = 0.0;   // Differentialanteil
//const int16_t tau_offset = 5;  // Offset-Moment aus Aufgabe 2

// Systemparameter
const float T_krit = 0.2;  // Kritische Periodendauer (aus k-Signal in Plotly)
const float K_R_krit = 2.0;  // Kritische Verstärkung

// PID Parameter nach Tabelle
const float Kp = 0.6 * K_R_krit;  // Proportionalanteil
const float Ti = 0.5 * T_krit;    // Nachstellzeit
const float Td = 0.125 * T_krit;  // Vorhaltezeit

// Umrechnung für diskrete Implementation
const float Ki = Kp / Ti;         // Integralanteil
const float Kd = Kp * Td;        // Differentialanteil
const int16_t tau_offset = 5;     // Offset-Moment beibehalten

//Resets the controller state when an experiment is started.
void reset(){
  //TODO
  last_error = 0;
  integral_sum = 0;
  last_time = millis();
}

/**
 * Rescales the position value from the motor.
 *
 * @param position The original position value to be rescaled.
 * @return The rescaled position value, adjusted to remain within a range of 0 to 360.
 */
float rescalepos(int16_t position) {  //rescaling function
    // Umrechnungsfaktor (Skalierung)
    const float scale_factor = 360.0 / total_steps;  // 1024 Schritte auf 360 Grad
    
    // Nullpunktverschiebung
    const int16_t offset_null = offset;  // Mittelpunkt der Messung (1024/2)
    
    // Berechnung der skalierten Position
    float scaled_position = (position - offset_null) * scale_factor;
    
    // Verschiebung um 180°, damit untere Gleichgewichtslage bei 180° liegt
    scaled_position += 180.0;
    
    // Normalisierung auf 0-360°
    if (scaled_position < 0) scaled_position += 360.0;
    if (scaled_position >= 360.0) scaled_position -= 360.0;
    
    return scaled_position;
}

/**
 * Calculates the error between the desired setpoint and the current position.
 *
 * @param setpoint The desired target value.
 * @param currentpos The current position value.
 * @return The error value, normalized to account for smallest angle to target value from current value.
 */
int16_t calculate_error(int16_t setpoint, int16_t currentpos){
  //TODO
  int16_t error = setpoint - currentpos;
  
  if (error > offset) {
      error -= total_steps;
  } else if (error < -offset) {
      error += total_steps;
  }
  
  return error;
}

/**
 * Calculates the derivative of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The change in error (delta) since the last call.
 */
int16_t error_derivative(int16_t error){
  //TODO
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  
  int16_t derivative = (error - last_error) / dt;
  
  last_error = error;
  last_time = current_time;
  
  return derivative;
}

/**
 * Calculates the integral of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The accumulated error over time (integral).
 */
int16_t error_integral(int16_t error){
  //TODO
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  
  const int16_t MAX_INTEGRAL = 1000;
  
  integral_sum += error * dt;
  
  if (integral_sum > MAX_INTEGRAL) {
      integral_sum = MAX_INTEGRAL;
  } else if (integral_sum < -MAX_INTEGRAL) {
      integral_sum = -MAX_INTEGRAL;
  }
  
  return integral_sum;
}

/**
 * Implements a basic controller using proportional, integral, and derivative (PID) control.
 *
 * @param error The current error value.
 * @param error_i The integral of the error.
 * @param error_d The derivative of the error.
 * @param measured_disturbance The measured disturbance value.
 * @return The calculated control torque.
 */
int16_t controller(int16_t error, int16_t error_i, int16_t error_d, int16_t measured_disturbance) {
  //TODO
    // Berechne PID-Regelung
    float control = Kp * error +           // P-Anteil
                   Ki * error_i +          // I-Anteil
                   Kd * error_d +          // D-Anteil
                   tau_offset;             // Offset
    
    // Begrenzung
    const int16_t MAX_TORQUE = 15;
    if (control > MAX_TORQUE) {
        control = MAX_TORQUE;
    } else if (control < -MAX_TORQUE) {
        control = -MAX_TORQUE;
    }
    
    return (int16_t)control;
}

/**
 * Placeholder for the main program loop. Do not change this.
 */
void loop(){
  start_loop();
}

/**
 * Sets up the controller by registering student-defined functions and initializing signals.
 * Do not change this.
 */
void setup(){
  register_student_fcns((studentFcns){
    .calculate_position=rescalepos,
    .calculate_error=calculate_error,
    .calculate_error_integral=error_integral,
    .calculate_error_derivative=error_derivative,
    .calculate_control=controller,
    .reset=reset});
  setup_signal();
}
