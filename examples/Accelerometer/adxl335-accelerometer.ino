// Define pins for ADXL335
const int xPin = A0;
const int yPin = A1;
const int zPin = A2;

// ADXL335 constants for converting analog readings to G-force
const float zeroG = 512.0;    // Assumes a 10-bit ADC with a range from 0 to 1023
const float sensitivity = 133; // Sensitivity of ADXL335 is approximately 330 mV/g

void setup() {
  // Start serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Read the analog values for X, Y, Z axes
  int xReading = analogRead(xPin);
  int yReading = analogRead(yPin);
  int zReading = analogRead(zPin);

  // Convert the readings to G-force (approximate conversion)
  float xG = (xReading - zeroG) / sensitivity;
  float yG = (yReading - zeroG) / sensitivity;
  float zG = (zReading - zeroG) / sensitivity;

  // Print the G-force values to the Serial Monitor
  Serial.print("X: ");
  Serial.print(xG);
  Serial.print("g\tY: ");
  Serial.print(yG);
  Serial.print("g\tZ: ");
  Serial.print(zG);
  Serial.println("g");

  // Small delay before next reading
  delay(1000);
}
