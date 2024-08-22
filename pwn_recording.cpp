int throttleSignalPin = 2; // Input from throttle signal wire

void setup() {
  Serial.begin(9600);
  pinMode(throttleSignalPin, INPUT);
  Serial.println("Throttle to PWM Recorder");
  Serial.println("Move the throttle and observe the PWM values:");
}

void loop() {
  int throttleSignal = pulseIn(throttleSignalPin, HIGH);
  
  if (throttleSignal > 0) {
    Serial.print("Throttle PWM Value: ");
    Serial.println(throttleSignal);
  }

  delay(500); // Adjust the delay as needed for your testing
}
