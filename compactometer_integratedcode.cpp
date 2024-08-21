// Define the sound sensor and Y-axis sensor pins
int soundSensorPin = A0;
int yPin = A2;

// Variables for sound sensor processing
float soundValue = 0;
float filteredSoundValue = 0;
float prevFilteredSoundValue = 0;
float prevSoundValue = 0;
float cutoffFrequency = 0.1;
float RC = 1.0 / (2 * 3.14 * cutoffFrequency);
float dt = 0.05;
float soundAlpha = dt / (RC + dt);
int soundThreshold = 10;  // Slightly increased threshold for better stability
int debounceDelay = 100;
unsigned long lastSpikeTime = 0;

// Variables for Y-axis sensor processing
float yValue = 0;
float yLastValue = 0;
float filteredYValue = 0;
float yAlpha = 0.95;
int yThreshold = 50;
int maxPlotValue = 700;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
}

void loop() {
  // Read and process sound sensor data
  soundValue = analogRead(soundSensorPin);
  filteredSoundValue = soundAlpha * (prevFilteredSoundValue + soundValue - prevSoundValue);
  prevSoundValue = soundValue;
  prevFilteredSoundValue = filteredSoundValue;

  // Check for significant sound spikes
  if (abs(filteredSoundValue) > soundThreshold && (millis() - lastSpikeTime) > debounceDelay) {
    Serial.print("FilteredSoundValue:");
    Serial.println(filteredSoundValue);
    lastSpikeTime = millis();
  } else {
    Serial.print("FilteredSoundValue:0\n");
  }

  // Read and process Y-axis sensor data
  yValue = analogRead(yPin);
  filteredYValue = yAlpha * (filteredYValue + yValue - yLastValue);
  yLastValue = yValue;

  // Apply threshold and clamp Y-axis output
  float outputYValue = abs(filteredYValue) > yThreshold ? filteredYValue : 0;
  outputYValue = constrain(outputYValue, -maxPlotValue, maxPlotValue);
  Serial.print("FilteredYValue:");
  Serial.println(outputYValue);

  // Delay for both sensors
  delay(50);  // Adjusted for balance between sensors
}
