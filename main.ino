// Constants
const int servoPin = 15;  // GPIO pin connected to the servo signal wire
const int potPin = 26;    // GPIO pin connected to the potentiometer
const float Kp = 3;     // Proportional gain
const float Ki = 0.0001;    // Integral gain
const float Kd = 0.88;    // Derivative gain
const int setpoint = 512; // Desired potentiometer value (midpoint of 0-1023)

// Variables for PID control
float integral = 0;
float previous_error = 0;
unsigned long previous_time = 0;

void setup() {
  // Initialize the servo pin
  pinMode(servoPin, OUTPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);

  // Get the current time
  unsigned long current_time = millis();
  float delta_time = (current_time - previous_time) / 1000.0; // Convert to seconds

  // Calculate the error
  float error = setpoint - potValue;

  // Calculate the integral
  integral += error * delta_time;

  // Calculate the derivative
  float derivative = (error - previous_error) / delta_time;

  // Calculate the PID output
  float pid_output = Kp * error + Ki * integral + Kd * derivative;

  // Update the previous error and time
  previous_error = error;
  previous_time = current_time;

  // Map the PID output to the servo range (0 to 180 degrees)
  int servo_position = constrain(map(pid_output, -1023, 1023, 0, 180), 0, 180);

  // Set the servo position
  setServoPosition(servo_position);

  // Send the position to serial for debugging
  Serial.print("Pot: ");
  Serial.print(potValue);
  Serial.print(", Servo: ");
  Serial.println(servo_position);

  // Small delay for control loop timing
  delay(100);
}

void setServoPosition(int angle) {
  int pulseWidth = map(angle, 0, 180, 544, 2400); // Convert angle to pulse width
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);
  delay(20); // Ensure a consistent period of 20ms
}
