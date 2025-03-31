#include <dht11.h>
#include <SoftwareSerial.h> // Include the SoftwareSerial library

// Pin definitions
#define DHT11PIN A0   // Define pin A0 for DHT11 sensor
#define RELAY1PIN 8   // Define pin 8 for Fan 1 relay
#define RELAY2PIN 7   // Define pin 7 for Fan 2 relay
#define HC05_TX 10    // Define pin 10 as HC-05 TX
#define HC05_RX 11    // Define pin 11 as HC-05 RX
#define PaperSensorPin A3 // Define pin A3 for the paper sensor

// Sensor and communication objects
dht11 DHT11;
SoftwareSerial Bluetooth(HC05_TX, HC05_RX); // Create a Bluetooth serial object

// Global variables
bool fan2Locked = false; // Keeps track if Fan 2 is in the "locked-on" state
int PaperSensorValue = 0; // Variable to store paper sensor reading

void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);         // Start serial communication for monitoring
  Bluetooth.begin(9600);      // Start communication with HC-05 module

  // Initialize relay pins as outputs
  pinMode(RELAY1PIN, OUTPUT);
  pinMode(RELAY2PIN, OUTPUT);

  // Ensure both fans are off at the start
  digitalWrite(RELAY1PIN, LOW);
  digitalWrite(RELAY2PIN, LOW);

  Serial.println("System initialized.");
  Bluetooth.println("Bluetooth ready. Receiving data...");
}

void loop() {
  // Read DHT11 sensor
  int chk = DHT11.read(DHT11PIN);
  float humidity = (float)DHT11.humidity;
  float temperature = (float)DHT11.temperature;

  // Fan control logic
  if (!fan2Locked) {
    if (humidity < 90) {
      digitalWrite(RELAY1PIN, HIGH);  // Turn on Fan 1
      digitalWrite(RELAY2PIN, LOW);   // Turn off Fan 2
    } else if (humidity >= 90) {
      digitalWrite(RELAY1PIN, LOW);   // Turn off Fan 1
      digitalWrite(RELAY2PIN, HIGH);  // Turn on Fan 2
      fan2Locked = true;              // Lock Fan 2 on
    }
  } else {
    if (humidity < 50) {
      digitalWrite(RELAY2PIN, LOW);   // Turn off Fan 2
      fan2Locked = false;             // Unlock Fan 2
    }
  }

  // Read Paper Sensor
  PaperSensorValue = analogRead(PaperSensorPin);

  // Send temperature, humidity, and paper sensor value in CSV format
  Serial.print(temperature, 2);
  Serial.print(",");
  Serial.print(humidity, 2);
  Serial.print(",");
  Serial.println(PaperSensorValue);

  // Send humidity value to phone via Bluetooth
  Bluetooth.print("Humidity: ");
  Bluetooth.print(humidity, 2);
  Bluetooth.println("%");

  // Send Paper Sensor Value via Bluetooth
  Bluetooth.print("Paper Sensor Value: ");
  Bluetooth.println(PaperSensorValue);

  // Delay for sensor stability
  delay(1000);
}
