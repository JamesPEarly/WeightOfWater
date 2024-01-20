/* This sketch is used configure an Arduino Nano 33 IoT
  with network information and broadcast IMU data
  in OSC format over a WiFi UDP connection
  */

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h> // Required for IMU
#include <Smoothed.h> // Smoothing
#include <OSCMessage.h> // Output message format
#include <FlashStorage.h> // Store network parameters across restarts

// Create a structure to store network information
typedef struct {
  char ssid[25];
  char password[50];
  int dstPort;
} NetworkInfo;

// Reserve a portion of flash memory to store a credentials and
// call it "my_flash_store".
FlashStorage(my_flash_store, NetworkInfo);

// Global Variables
// Setup WiFi connection (open)
int status = WL_IDLE_STATUS;
NetworkInfo netInfo;

WiFiUDP Udp; // A UDP instance to let us send packets using UDP
unsigned int srcPort = 7400;      // src port for UDP packets
unsigned int dstPort = 4444;      // dst port for UDP packets

IPAddress broadcastIp(192, 168, 1, 255);; // Broadcast IP

// Structures to hold data values for smoothing
// Accelerometer
Smoothed <float> accXvals;
Smoothed <float> accYvals;
Smoothed <float> accZvals;
// Gyroscope
Smoothed <float> gyrXvals;
Smoothed <float> gyrYvals;
Smoothed <float> gyrZvals;

float accX, accY, accZ, gyrX, gyrY, gyrZ; // latest gyroscope readings

void setup() {
  // Read existing netowrk info
  netInfo = my_flash_store.read();

  Serial.begin(9600);
  while(!Serial && millis()<5000){
    // Wait up to 5 seconds for the serial connection
  }

  // If serial is connected, ask for new parameters
  if (Serial) {
    Serial.setTimeout(20000); //Set timeout to 20 seconds
    Serial.println("Current network information");
    Serial.println("\tSSID: " + String(netInfo.ssid));
    Serial.println("\tPassword: " + String(netInfo.password));
    Serial.println("\tDst Port: " + String(netInfo.dstPort));
    Serial.println();

    // Read new network info
    Serial.println("Enter the new SSID: ");
    String ssid = Serial.readStringUntil('\n');

    // Read the passaord
    Serial.println("Enter the new network password or NONE:");
    String password = Serial.readStringUntil('\n');

    // Read the port number
    Serial.println("Enter the new destination port:");
    String dstPort = Serial.readStringUntil('\n');

    // Fill the credentials with the data entered by the user...
    ssid.toCharArray(netInfo.ssid, 25);
    password.toCharArray(netInfo.password, 50);
    netInfo.dstPort = dstPort.toInt();
  
    // Save everything into "my_flash_store"
    my_flash_store.write(netInfo);

    // Print a confirmation of the data inserted.
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(netInfo.ssid);
    Serial.print("Password: ");
    Serial.println(netInfo.password);
    Serial.print("Dst Port: ");
    Serial.println(netInfo.dstPort);
    Serial.println("Network inforrmation has been saved. Thank you!");
  }

  // initialize digital pin LED_BUILTIN as an output.
  // Used to signal WiFi connection
  pinMode(LED_BUILTIN, OUTPUT);

  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // Confirm board firmware is compatible with WiFiNINA
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // Activate UDP
  Udp.begin(srcPort);
  
  // Activate IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Initialize smoothing arrays
  accXvals.begin(SMOOTHED_AVERAGE, 20);
  accYvals.begin(SMOOTHED_AVERAGE, 20);
  accZvals.begin(SMOOTHED_AVERAGE, 20);
  gyrXvals.begin(SMOOTHED_AVERAGE, 20);
  gyrYvals.begin(SMOOTHED_AVERAGE, 20);
  gyrZvals.begin(SMOOTHED_AVERAGE, 20);
}

void loop() {
       // Turn off the LED
    digitalWrite(LED_BUILTIN, LOW);

  // Re/connect to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    // Reconnect
    wiFiConnect();
    // Set the LED high
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // Wait for new IMU values
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyrX, gyrY, gyrZ);

    // Store the new values
    accXvals.add(accX);
    accYvals.add(accY);
    accZvals.add(accZ);
    gyrXvals.add(gyrX);
    gyrYvals.add(gyrY);
    gyrZvals.add(gyrZ);

    // Broadcast the values
    sendOscPacket(broadcastIp, accXvals.get(), accYvals.get(), accZvals.get(),
      gyrXvals.get(), gyrYvals.get(), gyrZvals.get());

    // Sampling rate is 104Hz, so Wait 9ms for a new reading
    delay(9);
  }
}

void wiFiConnect() {
   while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to ");
    Serial.print(netInfo.ssid);
    Serial.print(" - ");
    Serial.println(netInfo.password);
    // Use WiFi password if set
    if (String(netInfo.password) == "NONE") {
      status = WiFi.begin(netInfo.ssid); // No enscryption on this network
    } else {
      status = WiFi.begin(netInfo.ssid, netInfo.password);
    }
    // wait 4 seconds for connection:
     delay(4000);
   }
   Serial.println("Connected");
}

// send an OSC message packet to the given address
unsigned long sendOscPacket(IPAddress& address, float accX, float accY, float accZ,
  float gyrX, float gyrY, float gyrZ) {
  // Send a packet containging an OSC message
  // The message wants an OSC address as first argument
   
  OSCMessage msg("/aXaYaZgXgYgZ");
  msg.add(accX);
  msg.add(accY);
  msg.add(accZ);
  msg.add(gyrX);
  msg.add(gyrY);
  msg.add(gyrZ);

  // Prepare the UDP packet
  if (netInfo.dstPort != 0) {
    dstPort = netInfo.dstPort;
  }
  // Craft UDP packet and broadcast
  Udp.beginPacket(address, dstPort);
  msg.send(Udp); // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  msg.empty(); // free space occupied by message
}