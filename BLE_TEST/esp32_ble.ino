#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE SERVER SETUP
BLEServer* pServer = NULL;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool lastDeviceConnected = false;

// UUID FOR BLE SERVER
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// SERVER CALLBACKS TO MONITOR CONNECTION STATUS
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// SERVER CALLBACK TO READ DATA
class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
      // WHATEVER YOU WANT TO QUERY
  }
// SERVER CALLBACK TO WRITE / TRIGGER FUNCTIONS
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        if (rxValue.find("A") != -1) { 
          //DO SOMETHING
        }
        else if (rxValue.find("B") != -1) {
          //DO SOMETHING
        }
      }
    }
};
// END OF CALLBACK

// Define Trig and Echo pin:
#define trigPin 16
#define echoPin 17

// Define variables:
long duration;
int distance;
int distanceValue;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Create the BLE Device
  BLEDevice::init("esp32test4"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("\nBluetooth Started! Ready to pair...");
}

void loop() {
  // put your main code here, to run repeatedly:
    if (deviceConnected) {
      
  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(5);

 // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds:
  //duration = analogRead(echoPin);
  duration = pulseIn(echoPin, HIGH);
  
  
  
  // Calculate the distance:
  distanceValue = duration*0.034/2;

    // Conver to char array
  char distance[8];
  // <SRC(FLOAT)> <DIGITS> <DECIMALS> <DESTINATION(STRING)>
  //dtostrf(distanceValue, 1, 2, distance);
    dtostrf(distanceValue, 1, 0, distance);
  
    pCharacteristic->setValue(distance);
    
    pCharacteristic->notify(); // Send Value to app
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(1500);

  }
      // HANDLE CLIENT DISCONNECTION
    if (!deviceConnected && lastDeviceConnected) {
        Serial.print("Device Disconnected!");
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Restarting Service...");
        lastDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !lastDeviceConnected) {
        // do stuff here on connecting
        lastDeviceConnected = deviceConnected;
    }
}
