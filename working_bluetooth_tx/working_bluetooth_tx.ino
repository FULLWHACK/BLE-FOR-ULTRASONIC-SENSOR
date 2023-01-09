#include "BluetoothSerial.h"

// Define Trig and Echo pin:
#define trigPin 16
#define echoPin 17

// Define variables:
long duration;
int distance;

/* Check if Bluetooth configurations are enabled in the SDK */
/* If not, then you have to recompile the SDK */
//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("esp32test4");
  Serial.println("Bluetooth Started! Ready to pair...");
  
  // Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  if (Serial.available())
  {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available())
  {
    Serial.write(SerialBT.read());
  }

  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);
  
  delayMicroseconds(5);

 // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance:
  distance = duration*0.034/2;
  
  // Print the distance on the Serial Monitor (Ctrl+Shift+M):
  SerialBT.print(distance);
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(1500);
  }
