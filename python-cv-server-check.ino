/*
  This file is a good way to check if the remote python computer vision server is working. It sends bits from the camera over TCP to the Flask server. The Flask server should reply.
  You need to have remote-cv.py running for this to work. Also make sure the server IP and the port number in this code matches the IP of the flask server.
*/
#include <WiFi.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <ArduCAM.h>
#include <SPI.h>

// WiFi Credentials
const char* ssid = "BELL390";
const char* password = "4C253D7ED313";

// Camera
#define CS_PIN 7
ArduCAM myCAM(OV2640, CS_PIN);

// Server URL
IPAddress server(192,168,2,26);
const int serverPort = 5000;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected!");

  Wire.begin();
  SPI.begin();
  myCAM.write_reg(ARDUCHIP_MODE, 0x00);  // Reset the camera

  uint8_t vid, pid;
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);

  if ((vid != 0x26) || (pid != 0x42)) {
    Serial.println(F("Cannot find OV2640 module!"));
    while (1);
  } else {
    Serial.println(F("OV2640 detected."));
  }

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_640x480); // Set resolution to 160x120

  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());

  // Capture and send image
  while(!captureAndSendImage()){
    Serial.println("Attempt complete.");
    delay(5000);
  }
}

void resetArduCAM() {
  Serial.println("Resetting ArduCAM...");
  myCAM.write_reg(ARDUCHIP_MODE, 0x00);  // Reset the camera
  myCAM.InitCAM();                      // Reinitialize the camera
  myCAM.OV2640_set_JPEG_size(OV2640_640x480); // Set the desired resolution again
  Serial.println("ArduCAM reset complete.");
}

bool captureAndSendImage() {
  Serial.println("Capturing Image...");
  myCAM.flush_fifo();          // Clear the FIFO buffer
  myCAM.clear_fifo_flag();     // Clear the FIFO flags
  myCAM.start_capture();

  int length = myCAM.read_fifo_length();
  Serial.print("Image Length: ");
  Serial.println(length);

  if (length == 0 || length >= MAX_FIFO_SIZE) {
    Serial.println("FIFO size violation.");
    return false;
  }

  if (client.connect("192.168.2.66", 5000)) {
    Serial.println("Connected to server");

    // Send HTTP POST headers
    client.println("POST /upload HTTP/1.1");
    client.print("Host: ");
    client.println("192.168.2.26");
    client.println("Content-Type: application/octet-stream");
    client.print("Content-Length: ");
    client.println(length + 3); // Image size + EOF flag
    client.println("Connection: close");
    client.println();

    // Use a buffer to send the image in chunks
    const int bufferSize = 1024; // Define buffer size
    uint8_t buffer[bufferSize];
    int remainingBytes = length;

    myCAM.CS_LOW();
    myCAM.set_fifo_burst(); // Set FIFO burst mode

    while (remainingBytes > 0) {
      int bytesToRead = min(bufferSize, remainingBytes);

      for (int i = 0; i < bytesToRead; i++) {
        buffer[i] = SPI.transfer(0x00); // Read bytes into buffer
      }

      client.write(buffer, bytesToRead); // Send buffer to server
      remainingBytes -= bytesToRead;

      // For monitoring progress
      Serial.print("Sent: ");
      Serial.print(length - remainingBytes);
      Serial.print(" / ");
      Serial.println(length);
    }

    myCAM.CS_HIGH();

    // Send EOF flag
    client.write('E');
    client.write('O');
    client.write('F');

    bool status = true;

    // Wait for server response
    Serial.println("Image sent. Awaiting response...");
    while (client.connected() || client.available()) {
      if (client.available()) {
        String response = client.readStringUntil('\n');
        
        // Check if the line starts with "Output:"
        if (response.startsWith("Output: ")) {
          // Extract the text after "Output: "
          String extractedText = response.substring(8); // Start at character 8 (length of "Output: ")
          
          // Print the extracted text
          Serial.println("Captured Output:");
          Serial.println(extractedText);

          if(extractedText == "FAIL"){
            status = false;
          }

          
        }
      }
    }

    client.stop();

    if (!status) {
      resetArduCAM(); // Reset ArduCAM on failure
    }

    return status;

  } else {
    Serial.println("Connection to server failed!");
    return false;
  }
  return true;
}


void loop() {
  // Nothing to do here for this example.
}
