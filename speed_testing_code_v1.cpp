int motorControlPin = 9; // Output PWM to control vehicle speed
int throttleSignalPin = 2; // Input from throttle signal wire
int pwmArray[] = {1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000};
int speedArray[] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20};

unsigned long debounceDelay = 50;
unsigned long lastThrottleSignalTime = 0;
int lastSpeed = 0; // Store the last speed for fail-safe recovery

// PID Controller Variables
float kp = 1.0, ki = 0.1, kd = 0.05;
float integral = 0, previousError = 0;
float integralMax = 100.0; // Max value to prevent integral windup

void setup() {
  Serial.begin(9600);
  pinMode(motorControlPin, OUTPUT);
  pinMode(throttleSignalPin, INPUT);

  Serial.println("Enter target speed in km/h (0 to 20) or type 'stop' to halt:");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n').trim(); // Combined input read and trim

    if (input.equalsIgnoreCase("stop")) {
      stopVehicle();
    } else {
      int targetSpeed = input.toInt();

      if (targetSpeed >= 0 && targetSpeed <= 20) {
        int currentSpeed = lastSpeed;

        // Gradually increase speed to the target speed
        while (currentSpeed < targetSpeed) {
          if (driverOverride()) return;

          currentSpeed = adaptiveCruiseControl(currentSpeed, targetSpeed);
          delayWithSafetyCheck(2000);
        }

        // Gradually decrease speed back to 0
        while (currentSpeed > 0) {
          if (driverOverride()) return;

          currentSpeed--;
          int pwmValue = getPWMValue(currentSpeed);
          analogWrite(motorControlPin, pwmValue);

          Serial.print("Decreasing Speed: ");
          Serial.print(currentSpeed);
          Serial.print(" km/h, PWM Value: ");
          Serial.println(pwmValue);

          delayWithSafetyCheck(2000);
        }

        analogWrite(motorControlPin, pwmArray[0]);
        Serial.println("Vehicle has stopped at 0 km/h.");
        lastSpeed = 0; // Reset lastSpeed after stopping
      } else {
        Serial.println("Invalid speed. Enter a value between 0 and 20 km/h.");
      }
    }
  }
}

// Function to get the PWM value corresponding to the current speed
int getPWMValue(int speed) {
  for (int i = 0; i < sizeof(speedArray) / sizeof(speedArray[0]); i++) {
    if (speed == speedArray[i]) {
      return pwmArray[i];
    }
  }
  return 1000;
}

// Adaptive Cruise Control using PID
int adaptiveCruiseControl(int currentSpeed, int targetSpeed) {
  float error = targetSpeed - currentSpeed;
  integral += error;

  // Prevent integral windup
  if (integral > integralMax) integral = integralMax;
  if (integral < -integralMax) integral = -integralMax;

  float derivative = error - previousError;

  int adjustment = kp * error + ki * integral + kd * derivative;

  previousError = error;

  currentSpeed += adjustment;

  // Limit speed to target
  if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
  if (currentSpeed < 0) currentSpeed = 0;

  int pwmValue = getPWMValue(currentSpeed);
  analogWrite(motorControlPin, pwmValue);

  Serial.print("Adaptive Speed: ");
  Serial.print(currentSpeed);
  Serial.print(" km/h, PWM Value: ");
  Serial.println(pwmValue);

  return currentSpeed;
}

// Function to check if the driver has pressed the accelerator (override) with debounce
bool driverOverride() {
  int throttleSignal = pulseIn(throttleSignalPin, HIGH);

  if (throttleSignal > 0 && (millis() - lastThrottleSignalTime > debounceDelay)) {
    lastThrottleSignalTime = millis();

    Serial.println("Driver override detected. Control bypassed to driver.");
    analogWrite(motorControlPin, throttleSignal); // Directly control the vehicle based on throttle input
    return true;
  }

  return false;
}

// Function to introduce a delay with safety checks
void delayWithSafetyCheck(unsigned long delayTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime) {
    if (driverOverride()) return;
    delay(50); // Short delay for safety checks
  }
}

// Function to gradually stop the vehicle
void stopVehicle() {
  int currentSpeed = lastSpeed;

  while (currentSpeed > 0) {
    if (driverOverride()) return;

    currentSpeed--;
    int pwmValue = getPWMValue(currentSpeed);
    analogWrite(motorControlPin, pwmValue);

    Serial.print("Stopping Vehicle: ");
    Serial.print(currentSpeed);
    Serial.print(" km/h, PWM Value: ");
    Serial.println(pwmValue);

    delayWithSafetyCheck(2000);
  }

  analogWrite(motorControlPin, pwmArray[0]);
  Serial.println("Vehicle has stopped at 0 km/h.");
  lastSpeed = 0; // Reset lastSpeed after stopping
}
