int soundSensorPin = A0;  // Define the sound sensor pin (analog pin A0)
float soundValue = 0;     // Variable to store the current sound value
float filteredValue = 0;  // Variable to store the high-pass filtered sound value
float prevFilteredValue = 0;  // Store the previous filtered value
float prevSoundValue = 0; // Variable to store the previous sound value

float cutoffFrequency = 0.1;  // Cutoff frequency for the filter (tuned for high-pass)
float RC = 1.0 / (2 * 3.14 * cutoffFrequency);  // Time constant
float dt = 0.05;  // Time step, corresponding to the 50ms delay
float alpha = dt / (RC + dt);  // Filter coefficient
int threshold = 7;  // Threshold to detect sound spikes
int debounceDelay = 100;  // Debounce delay to avoid false spikes
unsigned long lastSpikeTime = 0;  // Track the last significant sound spike

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
}

void loop() {
  soundValue = analogRead(soundSensorPin);  // Read the analog value from the sound sensor

  // Apply a high-pass filter (remove low-frequency components)
  filteredValue = alpha * (prevFilteredValue + soundValue - prevSoundValue);
 
  // Update previous values
  prevSoundValue = soundValue;
  prevFilteredValue = filteredValue;

  // Check if the filtered value exceeds the threshold and debounce time has passed
  if (abs(filteredValue) > threshold && (millis() - lastSpikeTime) > debounceDelay) {
    Serial.println(filteredValue);  // Send the significant high-frequency sound value over serial
    lastSpikeTime = millis();  // Update the time of the last detected spike
  } else {
    Serial.println(0);  // Print zero when no significant spike is detected
  }

  delay(50);  // Delay for 50 milliseconds before the next reading
}
