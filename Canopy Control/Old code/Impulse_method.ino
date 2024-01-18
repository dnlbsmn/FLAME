// USER INSTRUCTIONS
// This code is used to generate a pseudi-random simulation of kiwifruit motion
// User inputs are sent over Serial 

// INPUTS
// Stop motors                      'x'
// Set Intensity to value N (1-100) 'I N'

// Simulation characteristic parameters can be tuned and evaluated using the Impulse Mode Tuning spreadsheet
#define MOTOR1_INPUT_CAP 200
#define MOTOR2_INPUT_CAP 200
#define DIMINISHING_STEP 50
#define DIMINISHING_FACTOR_MIN 0.94
#define DIMINISHING_FACTOR_MAX 0.98

float IMPULSE_MAX = (intensity+150)/2;
float IMPULSE_MIN = intensity/3;
float TIME_DELAY_MIN = 500-(intensity*2);
float TIME_DELAY_MAX = 1000-(intensity*5);

#define SEED 1234

#define POT_1 9
#define POT_2 11
#define RELAY 8

int intensity = 0;

//UI configuration
#define PRINT_TIME 250
#define SERIAL_READ_TIME 5000

float motor_speed1, motor_speed2;
int input_PWM1, input_PWM2; 

unsigned long read_time;
double val1;

String val_str;
String val_sub;

long delay_time1;
long delay_time2;
long diminish_delay;

void setup() {
  Serial.begin(9600);

  analogWrite(POT_1, 255);
  analogWrite(POT_2, 255);
  Serial.println("BEGIN");
  delay(500);
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, 1);

  read_time = millis();
  val_str = "0";
  
  randomSeed(SEED);
  delay_time1 = 0;
  delay_time2 = 0;
}

// Stop motors
void stop() {
  val1 = 0;
  intensity = 0;
  motor_speed1 = 0;
  motor_speed2 = 0;
}

void generate_impulse(int speed1, int speed2) {
  // Diminish both signals
  if (millis() > diminish_delay) {
    motor_speed1 = speed1 * (random(DIMINISHING_FACTOR_MIN*1000, DIMINISHING_FACTOR_MAX*1000)/1000.00);
    motor_speed2 = speed2 * (random(DIMINISHING_FACTOR_MIN*1000, DIMINISHING_FACTOR_MAX*1000)/1000.00);
    diminish_delay = millis() + DIMINISHING_STEP;
  }

  // Apply impulses
  if (delay_time1 < millis()) {
    motor_speed1 += random(IMPULSE_MIN, IMPULSE_MAX);
    delay_time1 = millis() + random(TIME_DELAY_MIN, TIME_DELAY_MAX);
  }
  if (delay_time2 < millis()) {
    motor_speed2 += random(IMPULSE_MIN, IMPULSE_MAX);
    delay_time2 = millis() + random(TIME_DELAY_MIN, TIME_DELAY_MAX);
  }
}

void loop() {
  // Read user input every few seconds
  if (millis() - read_time > SERIAL_READ_TIME) {
    read_time = millis();
    val_str = Serial.readString();

    // if input is "x", stop
    if (val_str == "x\n") {
      stop();
    }

    // if input is in form "I N", set intensity to N
    if (val_str.substring(0, 2) == "I ") {
      val_sub = val_str.substring(2, '\n');
      if (val_sub.toFloat() > 0) {
        val1 = val_sub.toFloat();
      }        
      if (val1 >= 1) {
        randomSeed(SEED);
        intensity = val1;
        IMPULSE_MAX = (intensity+150)/2;
        IMPULSE_MIN = intensity/3;
        TIME_DELAY_MIN = 500-(intensity*2);
        TIME_DELAY_MAX = 1000-(intensity*5);
      }
    }
  } else {
    // Calculate new motor power
    if (val1 >= 1) generate_impulse(motor_speed1, motor_speed2);

    // Apply constraints
    if (motor_speed1 <= 0) {
      motor_speed1 = 0;
    } else if (motor_speed1 > MOTOR1_INPUT_CAP) {
      motor_speed1 = MOTOR1_INPUT_CAP;
    }
    if (motor_speed2 <= 0) {
      motor_speed2 = 0;
    } else if (motor_speed2 > MOTOR2_INPUT_CAP) {
      motor_speed2 = MOTOR2_INPUT_CAP;
    }

    // Write motor power
    input_PWM1 = 255 - motor_speed1;
    analogWrite(POT_1, input_PWM1);
    input_PWM2 = 255 - motor_speed2;
    analogWrite(POT_2, input_PWM2);

    // Print a couple time per second
    if (millis() % PRINT_TIME == 30) {
      Serial.print(intensity);
      Serial.print(", ");
      Serial.print(input_PWM1);
      Serial.print(", ");
      Serial.println(input_PWM2);
    }
  }
}
