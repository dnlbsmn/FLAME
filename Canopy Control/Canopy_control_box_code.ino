// USER INSTRUCTIONS
// This code is used to control the frequency and phase of each disturbance generator
// This program is intended to communicate with Canopy_control_GUI.py but can be run independantly

// To run this code on the control box ensure the board is set to 'M-Duino family' (industrial shields) and the model is set to 'M-Duino 58+'

// To control the canopy using only this program the following commands can be sent over serial
// Stop motors                      'x'
// Set motor 1 frequency N (Hz)     'm1 N'
// Set motor 2 frequency N (Hz)     'm2 N'
// Set both motors frequency N (Hz) 'm3 N'
// Set Phase Offset N (Degrees)     'po N'
// Toggle print                     'tp'
// Zero cranks                      'zc'
// Simulate communication from gui  'gui,[MOTOR 1 FREQUENCY],[MOTOR 2 FREQUENCY],[PHASE OFFSET]'

#include <string.h>
#include <stdlib.h>

//Connections
#define POT_1 A0_5
#define B_1 I2_6
#define A_1 I0_6
#define X_1 I0_5

#define POT_2 A0_6
#define B_2 I1_5
#define A_2 I1_6
#define X_2 I2_5

#define LED Q1_2

#define PHASE_CONTROL_METHOD 1

#define P_CORRECTION 0.015

#define RELAY Q1_0

//Tuning parameters
#define P1 33 //65
#define I1 25 //50
#define D1 0

#define P2 33 //65
#define I2 25 //50
#define D2 0

#define TICK_ARRAY_LENGTH 48
#define MOTOR_INPUT_CAP 127

#define ENCODER_ABSOLUTE_DIFFERENCE 36

//UI configuration
#define PRINT_TIME 50
#define SERIAL_READ_TIME 5000

bool print_flag = 0;
bool print_toggle = 0;
bool print_for_gui = 0;

bool zero_crank1 = 0;
bool zero_crank2 = 0;

bool new_input_flag1 = 0;
bool new_input_flag2 = 0;
long revolutions1, revolutions2;
int initial_phase_offset = 0;
long counter1 = 0;
long counter2 = 0;

long tick_time1, tick_time2; 
double frequency1, frequency2; 
int B1_tick, B2_tick; 
int B1_prev, B2_prev; 
int X1_tick, X2_tick; 
int X1_prev, X2_prev; 
int A1_tick, A2_tick;
int input_PWM1, input_PWM2; 
double cumulative_error1, cumulative_error2; 

int tick_delays1[TICK_ARRAY_LENGTH];
int tick_delays2[TICK_ARRAY_LENGTH];

unsigned long read_time;
double val1, val2;
String val_str;
String val_sub;
long prev_millis;
double target_frequency1, target_frequency2;

long revolution_time_M1, revolution_time_M2;
long phase_time_shift;
double phase_unit_shift;
double phase_error, phase_error_degrees;
double phase_correction;
double follower_frequency;

double target_phase_offset;

void setup() {
  Serial.begin(9600);
  delay(200);
  
  // Set encoder pins as inputs
	pinMode(B_1,INPUT);
	pinMode(A_1,INPUT);

  pinMode(B_2,INPUT);
	pinMode(A_2,INPUT);

  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, 1);

  pinMode(LED,OUTPUT);
  digitalWrite(LED, 1);

  // Read the initial state of CLK
	B1_prev = digitalRead(B_1);
  B2_prev = digitalRead(B_2);
  tick_time1 = millis();
  tick_time2 = millis();
  read_time = millis();
  val_str = "0";

  analogWrite(POT_1, 127);
  analogWrite(POT_2, 127);
  input_PWM1 = 0; 
  input_PWM2 = 0; 
  cumulative_error1 = 0;
  cumulative_error2 = 0;

  revolution_time_M1 = 0;
  revolution_time_M2 = 0;

  phase_correction = 1;
  target_phase_offset = 0;
  Serial.print("BEGIN");
}

// Stop all canopy motion
void stop() {
  val1 = 0;
  val2 = 0;
  for (int h=0; h<TICK_ARRAY_LENGTH; h++) {
    tick_delays1[h] = 0;
    tick_delays2[h] = 0;
  }
  cumulative_error1 = 0;
  cumulative_error2 = 0;
  zero_crank1 = 0;
  zero_crank2 = 0;
}

// Update encoder readings
void read_encoders() {
  B1_tick = digitalRead(B_1);
  X1_tick = digitalRead(X_1);
  B2_tick = digitalRead(B_2);
  X2_tick = digitalRead(X_2);
  A1_tick = digitalRead(A_1);
  A2_tick = digitalRead(A_2);
}

void loop() {

  double average1, average2;
  float motor_speed1, motor_speed2;
  long val_count1, val_count2;
  double error1, error2;
  double error_prev1, error_prev2;
  double error_gradient1, error_gradient2;
  double integral1, integral2;
  double proportional1, proportional2;
  double derrivative1, derrivative2;
  double dt;
  
  // Check for serial inputs every few seconds
  if (Serial.available()){
    read_time = millis();
    val_str = Serial.readString();

    new_input_flag1 = 1;
    new_input_flag2 = 1;

    // if input is "x", stop 
    if ((val_str == "x\n") | (val_str == "stop\n")) {
      stop();
    } 
    // if input is "toggle_print" print until entered again
    else if ((val_str == "toggle print\n") | (val_str == "tp\n")){
      print_toggle = !print_toggle;
    } 
    // if input is "p" print one line
    else if (val_str == "p\n") {
      print_flag = 1;
    }
    // if input is in the form "m1 N", set motor 1 to frequency N
    else if (val_str.substring(0, 3) == "m1 ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        val1 = val_sub.toFloat();
      }        
    }
    // if input is in the form "m2 N", set motor 2 to frequency N
    else if (val_str.substring(0, 3) == "m2 ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        val2 = val_sub.toFloat();
      }
    }
    // if input is in the form "m3 N", set both motors to frequency N
    else if (val_str.substring(0, 3) == "m3 ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        val1 = val_sub.toFloat();
        val2 = val_sub.toFloat();
      }
    }
    // if input is in the form "po N", set target phase offset to N (Degrees)
    else if (val_str.substring(0, 3) == "po ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        target_phase_offset = (val_sub.toFloat())*(48.0/360);
        if (target_phase_offset > 0.5) target_phase_offset -= 1;
      }
    }
    else if (val_str.substring(0, 2) == "zc") {
      zero_crank1 = 1;
      zero_crank2 = 1;
    }
    // if input is in the form "po N", set target phase offset to N (Degrees)
    else if (val_str.substring(0, 3) == "gui") {
      char *token = strtok(val_str.c_str(), ",");
      int c = 0;
      while (token != NULL) {
        c++;
        if (c == 2) val1 = strtod(token,NULL);
        else if (c == 3) val2 = strtod(token,NULL);
        else if (c == 4) target_phase_offset = strtod(token,NULL)*(48.0/360.0);

        token = strtok(NULL, ",");
      } 
      print_for_gui = 1;
      print_toggle = 0;
      print_flag = 0;
    }
    // If input is none of the above, print error message
    else if (val_str != "Arbitrary string"){
      Serial.print("Unrecognised command");
      val_str = "Arbitrary string";
      new_input_flag1 = 0;
      new_input_flag2 = 0;
    }

    // Reset encoder tick timers
    tick_time1 = millis();
    tick_time2 = millis();

  } else {
    if (zero_crank1 == 1) {
      val1 = 0.5;
    }

    if (zero_crank2 == 1) {
      val2 = 0.5;
    }

    // Get encoder readings
    read_encoders();

    // On rising edge of X channel of Encoder 1
    if ((X1_tick != X1_prev) & (X1_tick == 1)){

      revolutions1++;

      if (zero_crank1 == 1) {
        zero_crank1 = 0;
        val1 = 0;
        counter1 = 0;
      } else if (new_input_flag1 == 1) {
        revolutions1 = 0;
        counter1 = 0;
        new_input_flag1 = 0;
      }
    }

    // On rising edge of Encoder 2's X channel, record time
    if (X2_tick != X2_prev & X2_tick == 1){
      revolutions2++;

      if (zero_crank2 == 1) {
        zero_crank2 = 0;
        val2 = 0;
        counter2 = 0;
      } else if (new_input_flag2 == 1) {
        revolutions2 = 0;
        counter2 = 0;
        new_input_flag2 = 0;
      }
    }

    X1_prev = X1_tick;
    X2_prev = X2_tick;

    // On rising edge of B channel Encoder 1
    if (B1_tick != B1_prev & B1_tick == 1){

      // If A channel == 0
      if (A1_tick == B1_tick) {
        counter1++;
        // Record time of tick
        tick_delays1[0] = millis() - tick_time1;

        // Roll Shift Register
        for (int i=(TICK_ARRAY_LENGTH-1);i>0;i--) {
          tick_delays1[i] = tick_delays1[i-1];
        }
        tick_time1 = millis();
      } else counter1--;
    }
    B1_prev = B1_tick;
    
    // Encoder 2 poll
    if (B2_tick != B2_prev & B2_tick == 1){

      if (A2_tick == B2_tick) {
        tick_delays2[0] = millis() - tick_time2;
        for (int i=(TICK_ARRAY_LENGTH-1);i>0;i--) {
          tick_delays2[i] = tick_delays2[i-1];
        }
        tick_time2 = millis();
        counter2++;
      } else counter2--;
    }
    B2_prev = B2_tick;
    
    average1 = 0;
    average2 = 0;
    motor_speed1 = 0;
    motor_speed2 = 0;
    val_count1 = 0;
    val_count2 = 0;

    // Find average tick times    
    for (int k=0;k<TICK_ARRAY_LENGTH;k++) {
      average1 += tick_delays1[k];
      if (tick_delays1[k] != 0) {
        val_count1 += 1;
      }

      average2 += tick_delays2[k];
      if (tick_delays2[k] != 0) {
        val_count2 += 1;
      }
    }
    
    // Targets from user input
    target_frequency1 = val1;
    initial_phase_offset = target_phase_offset + ENCODER_ABSOLUTE_DIFFERENCE; // Number of ticks

    if (val2 == 0) target_frequency2 = val2;
    else if (new_input_flag1 | new_input_flag2) target_frequency2 = val2;
    else {
      float correction_factor = (1 + (P_CORRECTION * (counter1-((counter2 - initial_phase_offset)*(val1/val2)))));
      if (correction_factor > 1.5) correction_factor = 1.5;
      else if (correction_factor < 0.5) correction_factor = 0.5;
      target_frequency2 = val2 * correction_factor;
    }

    // If no ticks detected for over a second, empty tick arrays
    if ((millis() - tick_time1 > 1000) & (millis() - tick_time1 < 1050)) {
      for (int h=0; h<TICK_ARRAY_LENGTH; h++) {
        tick_delays1[h] = 0;
      }
    }
    if ((millis() - tick_time2 > 1000) & (millis() - tick_time2 < 1050)) {
      for (int h=0; h<TICK_ARRAY_LENGTH; h++) {
        tick_delays2[h] = 0;
      }
    } 

    // Calculate frequency
    if (average1 == 0) {
      frequency1 = 0;
    } else {
      frequency1 = (val_count1*1000)/(48*average1);
    }
    if (average2 == 0) {
      frequency2 = 0;
    } else {
      frequency2 = (val_count2*1000)/(48*average2); 
    }

    // Get change in time (dt)
    dt = millis() - prev_millis;
    if (dt==0) dt = 1;
    dt = dt/1000; //convert to seconds    
    prev_millis = millis();

    // Get Errors for PID
    error1 = target_frequency1 - frequency1;
    error2 = target_frequency2 - frequency2;
    cumulative_error1 += error1*dt;
    cumulative_error2 += error2*dt;
    error_gradient1 = (error1 - error_prev1)/dt;
    error_gradient2 = (error2 - error_prev2)/dt;

    // Calculate PID components
    proportional1 = error1 * P1;
    proportional2 = error2 * P2;
    integral1 = cumulative_error1 * I1;
    integral2 = cumulative_error2 * I2;
    derrivative1 = 1-(error_gradient1 * D1);
    derrivative2 = 1-(error_gradient2 * D2);

    // Apply constraints
    if (proportional1 < 0) proportional1 = 0;
    if (proportional2 < 0) proportional2 = 0;
    if (val1 == 0) cumulative_error1 = 0;
    if (val2 == 0) cumulative_error2 = 0;
    
    // Calculate motor inputs
    motor_speed1 = (proportional1 + integral1) * derrivative1;
    motor_speed2 = (proportional2 + integral2) * derrivative2; 
    error_prev1 = error1;
    error_prev2 = error2;

    // Apply constraints
    if (motor_speed1 <= 0) {
      motor_speed1 = 0;
    } else if (motor_speed1 > MOTOR_INPUT_CAP) {
      motor_speed1 = MOTOR_INPUT_CAP;
    }
    if (motor_speed2 <= 0) {
      motor_speed2 = 0;
    } else if (motor_speed2 > MOTOR_INPUT_CAP) {
      motor_speed2 = MOTOR_INPUT_CAP;
    }

    // Update motor power
    input_PWM1 = 127 - int(motor_speed1);
    analogWrite(POT_1, input_PWM1);
    input_PWM2 = 127 - int(motor_speed2);
    analogWrite(POT_2, input_PWM2);

    // Print outputs 
    if (print_flag | print_toggle) {
      if (millis() % PRINT_TIME == 30) {
        Serial.print(val1);
        Serial.print(", ");
        Serial.print(frequency1);
        Serial.print(", ");
        Serial.print(input_PWM1);
        Serial.print(", ");
        Serial.print(target_frequency2);
        Serial.print(", ");
        Serial.print(frequency2);
        Serial.print(", ");
        Serial.print(input_PWM2);
        Serial.print(", ");
        Serial.print(target_phase_offset);
        Serial.print(", ");
        Serial.print(counter1);
        Serial.print(", ");
        Serial.print(counter2);
        Serial.print(", ");
        Serial.print(counter1-(counter2*(val1/val2)));
        Serial.print(", ");
        Serial.print(counter1*1.0/(counter2+target_phase_offset));
        Serial.print('\n');
        print_flag = 0;
      }
    } else if (print_for_gui) {
      if (millis() % PRINT_TIME == 30) {
        Serial.print(frequency1);
        Serial.print(", ");
        Serial.print(frequency2);
        Serial.print(", ");
        if (val2 > 0)
          Serial.print((counter1-((counter2 - (target_phase_offset + ENCODER_ABSOLUTE_DIFFERENCE))*(val1/val2)))*(360/48));
        else
          Serial.print('-');
        Serial.print(", ");
        Serial.print(counter1%48);
        Serial.print(", ");
        Serial.println(counter2%48);
      }
    }
  }
}