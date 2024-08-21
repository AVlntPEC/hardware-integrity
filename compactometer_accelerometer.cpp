const int yPin = A2;

// High-pass filter variables
float yLastValue = 0;
float filteredYValue = 0;
float alpha = 0.95;  // Filter coefficient to focus on high frequencies
int threshold = 50;  // Threshold for significant frequency detection
int maxPlotValue = 700; // Maximum Y-axis value in the plot

void setup() {
  // Initialize the Serial Monitor and Serial Plotter
  Serial.begin(9600);
}

void loop() {
  // Read the analog value from the Y axis
  float yValue = analogRead(yPin);

  // Apply a high-pass filter to focus on high frequencies
  filteredYValue = alpha * (filteredYValue + yValue - yLastValue);

  // Update the last value for the next loop
  yLastValue = yValue;

  // Apply the threshold to only show significant values
  float outputValue = abs(filteredYValue) > threshold ? filteredYValue : 0;

  // Clamp the filtered value to the maximum plot value
  outputValue = constrain(outputValue, -maxPlotValue, maxPlotValue);

  // Output the clamped value to the Serial Plotter with a label
  Serial.print("FilteredYValue:");
  Serial.println(outputValue);

  delay(100);  // Increase delay for better visualization
}
