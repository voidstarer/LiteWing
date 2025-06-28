#include <esp_now.h>
#include <WiFi.h>

#define joystick_1_x_pin 36  // Joystick left
#define joystick_1_y_pin 39
#define joystick_2_x_pin 34  // Joystick right
#define joystick_2_y_pin 35
#define LED_PIN 2 // Define the onboard LED pin for ESP32

// Transmitter MAC Address - REPLACE WITH YOUR DRONE'S MAC ADDRESS
uint8_t transmitterMacAddress[] = {0xa0, 0x85, 0xe3, 0xed, 0xad, 0x08};

// Data structure for joystick control (what we send)
typedef struct joystick_data {
  int x1;
  int x2;
  int y2;
  float rollTrim;
  float pitchTrim;
  float AngleP;
  float AngleI;
  float AngleD;
  float rollPitchP;
  float rollPitchI;
  float rollPitchD;
  float yawP;
  float yawI;
  float yawD;
} joystick_data;

// Declaring the variable with initialization
joystick_data joystickReadings = {
  0,     // x1 (default throttle)
  0,     // x2 (default roll)
  0,     // y2 (default pitch)
  0.0,   // rollTrim
  0.0,   // pitchTrim
  2.0,   // rollPitchP Angle
  0.01,  // rollPitchI Angle
  0.1,   // rollPitchD Angle
  0.2,   // rollPitchP Rate
  0.01,  // rollPitchI Rate
  0.1,   // rollPitchD Rate
  5,   // yawP
  0.01,  // yawI
  0.1    // yawD
};

// Data structure for telemetry (what we receive)
typedef struct telemetry_data {
  float desired_roll;
  float desired_pitch;
  float roll_kalman;
  float pitch_kalman;
  float yaw_kalman;
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
  float desired_roll_rate;
  float desired_pitch_rate;
  float desired_yaw_rate;
  float roll_out;
  float pitch_out;
  float yaw_out;
  int throttle;
  int m1;
  int m2;
  int m3;
  int m4;
} telemetry_data;

// Variables to store data
telemetry_data receivedTelemetry;


unsigned long lastReceivedTime = 0;
bool telemetryReceived = false;
float prevX1 = 1500;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  //Serial.print("Received data from: ");
  //Serial.println(macStr);
  //Serial.print("Data length: ");
  //Serial.println(data_len);

  // Verify data size matches our structure
  if (data_len == sizeof(telemetry_data)) {
    // Copy the received data into our telemetry structure
    memcpy(&receivedTelemetry, data, sizeof(telemetry_data));
    telemetryReceived = true;
    lastReceivedTime = millis();
    
    // Print telemetry in the same format as the original function
    Serial.print("TELEMETRY,");
    
    // Desired angles
    Serial.print(receivedTelemetry.desired_roll);
    Serial.print(",");
    Serial.print(receivedTelemetry.desired_pitch);
    Serial.print(",");
    
    // Actual filtered angles
    Serial.print(receivedTelemetry.roll_kalman);
    Serial.print(",");
    Serial.print(receivedTelemetry.pitch_kalman);
    Serial.print(",");
    Serial.print(receivedTelemetry.yaw_kalman);
    Serial.print(",");
    
    // Actual rotation rates
    Serial.print(receivedTelemetry.roll_rate);
    Serial.print(",");
    Serial.print(receivedTelemetry.pitch_rate);
    Serial.print(",");
    Serial.print(receivedTelemetry.yaw_rate);
    Serial.print(",");
    
    // Desired yaw rate
    Serial.print(receivedTelemetry.desired_roll_rate);
    Serial.print(",");
    Serial.print(receivedTelemetry.desired_pitch_rate);
    Serial.print(",");
    Serial.print(receivedTelemetry.desired_yaw_rate);
    Serial.print(",");
    
    // PID outputs
    Serial.print(receivedTelemetry.roll_out);
    Serial.print(",");
    Serial.print(receivedTelemetry.pitch_out);
    Serial.print(",");
    Serial.print(receivedTelemetry.yaw_out);
    Serial.print(",");
    
    // Throttle
    Serial.print(receivedTelemetry.throttle);
    Serial.print(",");
    
    // Motor outputs
    Serial.print(receivedTelemetry.m1);
    Serial.print(",");
    Serial.print(receivedTelemetry.m2);
    Serial.print(",");
    Serial.print(receivedTelemetry.m3);
    Serial.print(",");
    Serial.println(receivedTelemetry.m4);
  } else {
    Serial.println("Error: Received data size doesn't match expected structure size");
    Serial.print("Expected: ");
    Serial.print(sizeof(telemetry_data));
    Serial.print(" bytes, Received: ");
    Serial.print(data_len);
    Serial.println(" bytes");
  }
}

void readSerialValues() {
    if (Serial.available()) {
        digitalWrite(LED_PIN, HIGH); // Turn on LED when data is received
        String data = Serial.readStringUntil('\n'); // Read the entire line
        data.trim(); // Remove any leading/trailing spaces or newline characters

        if (data.startsWith("CONFIG")) { // Ensure it's a valid data line
            data = data.substring(7); // Remove "CONFIG," prefix
            
            float rollTrim, pitchTrim, AngleP, AngleI, AngleD, rollPitchP, rollPitchI, rollPitchD, yawP, yawI, yawD;
            
            // Parse the values
            sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &rollTrim, &pitchTrim, &AngleP, &AngleI, &AngleD, &rollPitchP, &rollPitchI, &rollPitchD, &yawP, &yawI, &yawD);
            
            // assing these values to send data
            joystickReadings.rollTrim = rollTrim;
            joystickReadings.pitchTrim = pitchTrim;
            joystickReadings.AngleP = AngleP;
            joystickReadings.AngleI = AngleI;
            joystickReadings.AngleD = AngleD;
            joystickReadings.rollPitchP = rollPitchP;
            joystickReadings.rollPitchI = rollPitchI;
            joystickReadings.rollPitchD = rollPitchD;
            joystickReadings.yawP = yawP;
            joystickReadings.yawI = yawI;
            joystickReadings.yawD = yawD;
            
            
        }
        digitalWrite(LED_PIN, LOW); // Turn off LED after processing
    }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); // Set LED pin as output
  
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.println("\n\nESP-NOW Receiver Starting...");
  
  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Disconnect from any previously connected networks
  delay(100);  // Brief delay to let WiFi settle
  
  // Print the MAC address for verification
  Serial.print("Receiver MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW Initialized");
  
  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Callbacks registered");
  
  // Register peer (the transmitter)
  esp_now_peer_info_t peerInfo = {};  // Initialize all fields to 0
  memcpy(peerInfo.peer_addr, transmitterMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Delete any existing peer first
  esp_now_del_peer(transmitterMacAddress);
  
  // Add peer with error checking
  esp_err_t addStatus = esp_now_add_peer(&peerInfo);
  if (addStatus != ESP_OK) {
    Serial.print("Failed to add peer. Error code: ");
    Serial.println(addStatus);
    if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
      Serial.println("ESP-NOW not initialized");
    } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid argument");
    } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
      Serial.println("Peer list full");
    } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("Out of memory");
    } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
      Serial.println("Peer exists");
    } else {
      Serial.println("Unknown error");
    }
    return;
  }
  
  Serial.println("Peer added successfully");
  Serial.println("Setup completed - waiting for telemetry data...");
  Serial.println("---------------------------------------------");
}

void loop() {
  // Read raw joystick values
  int raw_x1 = analogRead(joystick_1_x_pin);
  int raw_x2 = analogRead(joystick_2_x_pin);
  int raw_y2 = analogRead(joystick_2_y_pin);
  
  // Map values to desired ranges
  joystickReadings.x1 = map(raw_x1, 2047, 4095, 1000, 2000);  // Map x1 to 1000-2000
  joystickReadings.x2 = map(raw_x2, 0, 4095, -40, 40);       // Map x2 to 20 to -20
  joystickReadings.y2 = map(raw_y2, 0, 4095, 40, -40);       // Map y2 to 20 to -20

  // Simple smoothing (adjust 0.2 to control smoothness - smaller = smoother)
  //prevX1 = prevX1 * 0.8 + mapped_x1 * 0.8;
  
  // Assign the smoothed value
  //joystickReadings.x1 = (int)prevX1;

  if (joystickReadings.x2 > -6 && joystickReadings.x2 < 6)  
  joystickReadings.x2 = 0;
  if (joystickReadings.y2 > -6 && joystickReadings.y2 < 6)  
  joystickReadings.y2 = 0;

  // Print values in a single line
  //Serial.print("X1: "); Serial.print(joystickReadings.x1);
  //Serial.print(" X2: "); Serial.print(joystickReadings.x2);
  //Serial.print(" Y2: "); Serial.println(joystickReadings.y2); // Ends with newline
  
  // Send joystick data via ESP-NOW
  esp_err_t result = esp_now_send(transmitterMacAddress, (uint8_t *) &joystickReadings, sizeof(joystickReadings));
  
  if (result != ESP_OK) {
    Serial.print("Error sending the data. Error code: ");
    Serial.println(result);
  }
  
  // Check if we haven't received telemetry for a while
  if (telemetryReceived && (millis() - lastReceivedTime > 5000)) {
    Serial.println("WARNING: No telemetry received in the last 5 seconds!");
    telemetryReceived = false;
  }

  readSerialValues(); // Continuously check for incoming serial data
  
  delay(50); // 50ms delay
}
