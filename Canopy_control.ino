// USER INSTRUCTIONS
// This code is used to control the frequency and phase of each disturbance generator
// User inputs are sent over Serial 

// INPUTS
// Stop motors                      'x'
// Set motor 1 frequency N (Hz)     'm1 N'
// Set motor 2 frequency N (Hz)     'm2 N'
// Set both motors frequency N (Hz) 'm3 N'
// Set Phase Offset N (Degrees)     'po N'

//Connections
#define POT_1 9
#define B_1 2
#define A_1 3
#define X_1 4

#define POT_2 11
#define B_2 12
#define A_2 5
#define X_2 6

#define PHASE_CONTROL_METHOD 1

#define P_CORRECTION 0.3

#define RELAY 8

//Tuning parameters
#define P1 60
#define I1 50
#define D1 0

#define P2 60
#define I2 50
#define D2 0

#define TICK_ARRAY_LENGTH 48

#define MOTOR_INPUT_CAP 255

//UI configuration
#define PRINT_TIME 250
#define SERIAL_READ_TIME 5000

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
int phase_align;
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
  Serial.println("BEGIN");
  delay(500);
  
  // Set encoder pins as inputs
	pinMode(B_1,INPUT);
	pinMode(A_1,INPUT);

  pinMode(B_2,INPUT);
	pinMode(A_2,INPUT);

  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, 1);

  // Read the initial state of CLK
	B1_prev = digitalRead(B_1);
  B2_prev = digitalRead(B_2);
  tick_time1 = millis();
  tick_time2 = millis();
  read_time = millis();
  val_str = "0";

  analogWrite(POT_1, 255);
  analogWrite(POT_2, 255);
  input_PWM1 = 0; 
  input_PWM2 = 0; 
  cumulative_error1 = 0;
  cumulative_error2 = 0;

  revolution_time_M1 = 0;
  revolution_time_M2 = 0;

  phase_correction = 1;
  target_phase_offset = 0;
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
  if (millis() - read_time > SERIAL_READ_TIME) {
    read_time = millis();
    val_str = Serial.readString();

    // if input is "x", stop 
    if (val_str == "x\n") {
      stop();
    }

    // if input is in the form "m1 N", set motor 1 to frequency N
    if (val_str.substring(0, 3) == "m1 ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        val1 = val_sub.toFloat();
        phase_align = 0;
      }        
    // if input is in the form "m2 N", set motor 2 to frequency N
    } else if (val_str.substring(0, 3) == "m2 ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        val2 = val_sub.toFloat();
        phase_align = 0;
      }
    // if input is in the form "m3 N", set both motors to frequency N
    } else if (val_str.substring(0, 3) == "m3 ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        val1 = val_sub.toFloat();
        val2 = val_sub.toFloat();
        phase_align = 0;
      }
    // if input is in the form "po N", set target phase offset to N (Degrees)
    } else if (val_str.substring(0, 3) == "po ") {
      val_sub = val_str.substring(3, '\n');
      if (val_sub.toFloat() > 0) {
        target_phase_offset = (val_sub.toFloat())/360;
        if (target_phase_offset > 0.5) target_phase_offset -= 1;
      }
    }

    // Reset encoder tick timers
    tick_time1 = millis();
    tick_time2 = millis();

  } else {
    // Get encoder readings
    read_encoders();

    // On rising edge of X channel of Encoder 1
    if (X1_tick != X1_prev & X1_tick == 1){
      // Record revolution time
      revolution_time_M1 = millis();

      // Get phase shift between the two disturbance generators
      double period = (1000/target_frequency1);
      phase_time_shift = revolution_time_M1 - revolution_time_M2;
      phase_unit_shift = (phase_time_shift/period);

      // If phase shift is reasonable
      if ((phase_unit_shift >= 0) & (phase_unit_shift <= 1)) {

        // Centre target phase offset 
        if (phase_unit_shift > target_phase_offset + 0.5) phase_unit_shift -= 1;    

        // Set phase correction value
        phase_error = phase_unit_shift - target_phase_offset;
        phase_correction = 1 - (phase_error * P_CORRECTION);

        phase_error_degrees = phase_error * 360;
      }
    }

    // On rising edge of Encoder 2's X channel, record time
    if (X2_tick != X2_prev & X2_tick == 1){
      revolution_time_M2 = millis();
    }

    X1_prev = X1_tick;
    X2_prev = X2_tick;

    // On rising edge of B channel Encoder 1
    if (B1_tick != B1_prev & B1_tick == 1){

      // If A channel == 0
      if (A1_tick == B1_tick) {
        // Record time of tick
        tick_delays1[0] = millis() - tick_time1;

        // Roll Shift Register
        for (int i=(TICK_ARRAY_LENGTH-1);i>0;i--) {
          tick_delays1[i] = tick_delays1[i-1];
        }
        tick_time1 = millis();
      }
    }
    B1_prev = B1_tick;
    
    // Encoder 2 poll
    if (B2_tick != B2_prev & B2_tick == 1){

      if (A2_tick != B2_tick) {
        tick_delays2[0] = millis() - tick_time2;
        for (int i=(TICK_ARRAY_LENGTH-1);i>0;i--) {
          tick_delays2[i] = tick_delays2[i-1];
        }
        tick_time2 = millis();
      }
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
    target_frequency2 = val2 * phase_correction;
    
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
    input_PWM1 = 255 - int(motor_speed1);
    analogWrite(POT_1, input_PWM1);
    input_PWM2 = 255 - int(motor_speed2);
    analogWrite(POT_2, input_PWM2);

    // Print outputs 
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
      Serial.println(phase_error_degrees);
    }
  }
}
