// = IMPORTS ====================
#include <Arduino.h>
#include <Servo.h>
#include <ezButton.h>
#include <ArduCAM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "HX711.h"

// = CONSTANTS =================
#define STEPS 200
#define IMAGE_MAX_SIZE 2048

// = PIN DEFINITIONS ===========
#define HOPPER_IR_PIN A0
#define LOADCELL_SCK_PIN  4
#define LOADCELL_DOUT_PIN  5
#define TRIGGER 9
#define ECHO 8
#define STEPPER_THRUST 3
#define STEPPER_DIR 2
#define CS_PIN 7

// = STATE MACHINE SETUP ======================
// Keeps track of system state (will be used for mock-parallelization)
enum SystemState
{
  IDLE,
  SENSED_MOVEMENT,
  FILLING_BOWL,
  BOWL_EXTENDED,
  OVERRIDE
};

SystemState system_state = IDLE;

// What type of pet are we looking for?
enum PetType{
  CAT,
  DOG,
  ANY
};

PetType pet_type = ANY; // By default, we don't care what type of pet is in front of the camera.
PetType recognized_pet = ANY; // This is for checking that we see the right pet from the computer vision module running remotely

// = SETUP HARDWARE GLOBALS =====================

HX711 scale; // This is for the loadcell; it wasn't super precise so it isn't actually used in the code
float calibration_factor = -7050 * 453.5; // Convert resistance to load (Deprecated)

Servo bowl_extension_servo;
int servo_angle = 0;
ezButton retracted_bowl_ls(1);
ezButton extended_bowl_ls(0);
ArduCAM myCAM(OV2640, CS_PIN);

// = SYSTEM CONFIGURATION =====================
// Settings
int desired_portion_g = 500; // Default 500g serving (DEPRECATED)
int desired_servings = 5; // How many servings of food to dispense
int serving_count = 0; // Count of how many servings we have already dispensed
int kibble_zero_depth_cm = -1; // Needs to be calibrated
int bowl_zero_weight_g = -1; // Needs to be calibrated
float duration_mins = 0; // Time between meals 
float meal_duration_mins = 1; // Time to leave the bowl out for
const unsigned long sensorUpdateInterval = 60000; // 1 minute interval in milliseconds

// Global tracking variables
float time_start = duration_mins; // Time of last meal; by default, meal time 
int sensed_cycle_count = 0; // How many cycles the ultrasonic has consecutively seen something close
unsigned long lastSensorUpdate = -1; // Tracks the last time sensor states were updated

// Hold Sensor Values
int kibble_depth_cm = 0; // The depth of the hopper when there is no kibbl ein it
int bowl_weight_g = 0; // Deprecated
bool extended = false; // Limit switch (bowl) state

// Stepper management
int directionFlag = 1;
double minDelay = 400;
double maxDelay = 900;

// Web configuration
char ssid[] = "BELL390"; 
char pass[] = "4C253D7ED313";

int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
String HTTP_req;            // stores the HTTP request (For the web client)
WiFiServer server(80);

// Remote computer vision server credentials; this is different from the web client hosted by the Arduino
WiFiClient cv_client;
String ip_address ="192.168.2.26";
int port = 5000;
bool img_sent = false; // Tells us if we successfully sent an image to the server or not, if not we need to resend next time step

// ===================== READ/WRITE CALIBRATION DATA TO EEPROM =====================
void saveCalibrationData(){ // Save relevant variables to EEPROM (User or environmental settings)
  EEPROM.write(0, 1); // Valid bit 
  EEPROM.write(4, desired_servings);
  EEPROM.write(8, meal_duration_mins);
  EEPROM.write(12, kibble_zero_depth_cm);
  EEPROM.write(16, duration_mins);
  EEPROM.write(20, pet_type);

  Serial.println("Calibration saved to EEPROM.");
}
bool loadCalibrationData(){ // On setup, load relevant data from EEPROM
  if(EEPROM.read(0) == 1){
    desired_servings = EEPROM.read(4);
    meal_duration_mins = EEPROM.read(8);
    kibble_zero_depth_cm = EEPROM.read(12);
    duration_mins = EEPROM.read(16);
    pet_type = (PetType)EEPROM.read(20);
    Serial.println("Calibration data loaded from EEPROM.");
    return true;
  } else{
    Serial.println("No valid bit in EEPROM. Not loading calibration data.");
    return false;
  } 
}

// ===================================================================================

// ===================== MAIN SETUP LOOP ============================================

void setup()
{
  // Pin and sensor setup
  pinMode(HOPPER_IR_PIN, INPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(STEPPER_THRUST, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);

  // Initialize servo motor
  bowl_extension_servo.attach(6);
  
  // Reset the food scale 
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0

  // System state initialization
  system_state = IDLE;

  // Begin the Serial Monitor 
  Serial.begin(9600);
  Serial.println("System initialized with IDLE system state.");

  // Setup the wifi
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);       // don't continue
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  // Attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status

  // Camera setup
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

  // Load calibration from EEPROM
  if(!loadCalibrationData()){
    calibrate();
  }

  // Ensure bowl is retracted.
  Serial.println("Retracting bowl");
  bowl_extension_servo.write(0);
  extended = false;
  Serial.println("Complete setup complete.");
}

// ======================================================================================================

// ============================= MAIN STATE MACHINE ===================================================== 

void loop()
{
  // Put limit switches in loop mode
  extended_bowl_ls.loop();
  retracted_bowl_ls.loop();

  // Hold ultrasonic distance
  int us_distance = 0;

  // First attend to web server
  handleServer(); 

  // Check the hopper status
  checkIfHopperEmpty();

  // Do current system activity
  switch (system_state)
  {
  case OVERRIDE: // This mode totally stops system operation to allow user to move the system manually
  break;
  case IDLE:
    serving_count = 0;
    us_distance = getUltrasonicDistance();
    Serial.println("Ultrasonic distance: ");
    Serial.println(us_distance);

    if (us_distance < 50) 
    {
      sensed_cycle_count++; // We need to see something close for 5 consecutive cycles or else it could be a fluke
    } else {
      sensed_cycle_count = 0;
    }

    if(sensed_cycle_count >= 5){
      sensed_cycle_count = 0; // Reset sensed cycle count
      system_state = SENSED_MOVEMENT;
      Serial.println("System sensed an animal, switching to SENSED_MOVEMENT state.");
    }

    break;
  case SENSED_MOVEMENT:
    if ( duration_mins > (millis()/60000 - time_start))
    {
      // If it is not meal time, do nothing and reset to idle.
      system_state = IDLE;
      Serial.println("It is not meal time.");
    } 
    else
    {
      if(pet_type == ANY){
        // We don't need to check dog vs cat if we don't care!
        // Go directly to next state.
        system_state = FILLING_BOWL; // Sanity check has passed and we can move states
        Serial.println("Moving to filling bowl state.");
        break;
      }

      // Send an image to a remote python server which will use a CNN model to check if this is a cat, dog or neither
      img_sent = captureAndSendImage();
      
      if(!img_sent){
        delay(5000); // Let the arducam breathe if we failed. Idk the reasoning behind this but it seems to help.
      } else {

        // Check that we saw the right animal
        if (pet_type == recognized_pet){
          system_state = FILLING_BOWL; // Sanity check has passed and we can move states
          Serial.println("Your pet is not an imposter! Moving to FILLING_BOWL state.");
        } else {
          system_state = IDLE;
          Serial.println("Seems that we spotted an imposter. Returning to IDLE state.")
        }
        
        img_sent = false; // Reset the image sent flag
      }
    }
    break;
  case FILLING_BOWL:
    if (desired_servings != serving_count)
    {
      runAtPercentMaxSpeed(0.95,3000); // Dispense one serving.
      serving_count++;
    }

    else
    {
      if (extended_bowl_ls.isPressed()) // We hit the extension limit switch
      {
        Serial.println("Bowl extended.");
        extended = true;
      }

      if(!extended){
        // Push bowl out to the pet
        extendBowl();
      }
      else
      {
        // Reset the food timer
        time_start = millis() / 60000;
        system_state = BOWL_EXTENDED;
        Serial.println("Bowl filled and extended to pet, switching to BOWL_EXTENDED state.");
      }
    }
    break;
  case BOWL_EXTENDED:
    if ( (millis() / 60000 - time_start) >= meal_duration_mins ) // If meal time is over
    {
      // Retract the bowl
      if (retracted_bowl_ls.isPressed()){
        Serial.println("Bowl retracted.");
        extended = false;
      }

      if(extended){
        retractBowl();
      }
      else
      {
        system_state = IDLE;
        Serial.println("Cycle complete, returning to IDLE state.");
      }
    }
    break;
  }
}

// ======================== ULTRASONIC SENSOR =====================================

int getUltrasonicDistance(){
  int totalDistance = 0; // Variable to store the sum of distances

  for (int i = 0; i < 10; i++) {
    // Clears the trigPin
    digitalWrite(TRIGGER, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 microseconds
    digitalWrite(TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    float duration = pulseIn(ECHO, HIGH);

    // Calculate the distance in centimeters
    int us_distance = duration / 58;

    // Add the reading to the total distance
    totalDistance += us_distance;

    // Small delay between readings
    delay(10);
  }

  return totalDistance / 10;
}

// =========================== HOPPER DEPTH MANAGEMENT (IR SENSOR) ======================

//interpolation of distance at 250mV intervals
const int TABLE_ENTRIES = 12;
const int INTERVAL  = 250;
static int ir_distance[TABLE_ENTRIES] = {150,140,130,100,60,50,40,35,30,25,20,15};

int getHopperDepth() {
  int mV = analogRead(HOPPER_IR_PIN) * (5000/1023);

  if (mV > INTERVAL * TABLE_ENTRIES - 1)      return ir_distance[TABLE_ENTRIES - 1];
  else {
    int index = mV / INTERVAL;
    float frac = (mV % 250) / (float)INTERVAL;
    return ir_distance[index] - ((ir_distance[index] - ir_distance[index + 1]) * frac);
  }
}

bool checkIfHopperEmpty()
{
  // Check if the hopper is empty
  if (kibble_depth_cm >= kibble_zero_depth_cm)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// ==================== SYSTEM CALIBRATION ==========================================

void calibrate() // Saves the current hopper depth as zero depth, puts everything on EEPROM
{
  Serial.println("Calibrating system");

  kibble_zero_depth_cm = getHopperDepth();
  Serial.print("Kibble zero depth is ");
  Serial.println(kibble_zero_depth_cm);

  saveCalibrationData();
}

// =========================== SERVO FUNCTIONS ======================================

// Extend the bowl to the pet using a servo,
void extendBowl()
{
  bowl_extension_servo.write(servo_angle);
  servo_angle += 10;
}

// Retract the bowl back to the feeder using a servo
void retractBowl()
{
  bowl_extension_servo.write(servo_angle);
  servo_angle -= 10;
}

// ================================ STEPPER FUNCTIONS =======================================
//This is a helper function that sends one pulse to the motor
void stepperPulse(double delayTime){
    digitalWrite(STEPPER_THRUST,HIGH); 
    delayMicroseconds(delayTime);    
    digitalWrite(STEPPER_THRUST,LOW); 
    delayMicroseconds(delayTime); 
}

//toggles direction
void toggleStepperDirection(){
  if(directionFlag){
    digitalWrite(STEPPER_DIR,HIGH);
    directionFlag = 0;
  }
  else{
    digitalWrite(STEPPER_DIR,LOW);
    directionFlag = 1;
  }
}

//inputs: percentage of max speed (between 0 and 1), and duration that you want the motor to spin at that speed for in milliseconds
void runAtPercentMaxSpeed(double percentage, double duration){
  unsigned long startTime = millis();
  double effectiveDelay = percentage * (minDelay-maxDelay) + maxDelay;
  
  while(1){
  unsigned long currentTime = millis();
  stepperPulse(effectiveDelay);
  if(currentTime-startTime>duration){
    return;
    }
  }
}

// =========================ARDUCAM-PYTHON REMOTE COMPUTER VISION===================================

// Reset the arducam after a failure (clears all bits)
// This can prevent sending old bits over wifi the next time we try to send something.
void resetArduCAM() {
  Serial.println("Resetting ArduCAM...");
  myCAM.write_reg(ARDUCHIP_MODE, 0x00);  // Reset the camera
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_640x480); // Set resolution to 160x120
  Serial.println("ArduCAM reset complete.");
}

// Captures a FIFO image with the ArduCam, and sends bytes in batches to a remote Flask server. The flask server hosts a small CNN that can recognize cats or dogs.
// The image is run through this CNN, and then it returns a string saying CAT, DOG, or NONE, indicating what is in front of the pet feeder.
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

  if (cv_client.connect("192.168.2.66", port)) {
    Serial.println("Connected to server");

    // Send HTTP POST headers
    cv_client.println("POST /upload HTTP/1.1");
    cv_client.print("Host: ");
    cv_client.println(ip_address);
    cv_client.println("Content-Type: application/octet-stream");
    cv_client.print("Content-Length: ");
    cv_client.println(length + 3); // Image size + EOF flag
    cv_client.println("Connection: close");
    cv_client.println();

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

      cv_client.write(buffer, bytesToRead); // Send buffer to server
      remainingBytes -= bytesToRead;

      // For monitoring progress
      Serial.print("Sent: ");
      Serial.print(length - remainingBytes);
      Serial.print(" / ");
      Serial.println(length);
    }

    myCAM.CS_HIGH();

    // Send EOF flag
    cv_client.write('E');
    cv_client.write('O');
    cv_client.write('F');

    bool status = true;

    // Wait for server response
    Serial.println("Image sent. Awaiting response...");
    while (cv_client.connected() || cv_client.available()) {
      if (cv_client.available()) {
        String response = cv_client.readStringUntil('\n');
        
        // Check if the line starts with "Output:"
        if (response.startsWith("Output: ")) {
          // Extract the text after "Output: "
          String extractedText = response.substring(8); // Start at character 8 (length of "Output: ")
          
          // Print the extracted text
          Serial.println("Captured Output:");
          Serial.println(extractedText);

          if(extractedText == "FAIL"){
            status = false;
          } else if (extractedText == "CAT"){
            recognized_pet = CAT;
          } else if (extractedText == "DOG"){
            recognized_pet = DOG;
          } else {
            recognized_pet = ANY;
          }
          
        }
      }
    }

    cv_client.stop();

    if (!status) {
      resetArduCAM(); // Reset ArduCAM on failure
    }

    return status;

  } else {
    resetArduCAM(); // Reset ArduCAM on failure
    Serial.println("Connection to server failed!");
    return false;
  }
  return true;
}


// ============================= WEB SERVER ==================================================

// Web UI Handling, runs once a time step and handles all user interfacing with the web client.
void handleServer(){
      // Establish WiFi client connection to server, waiting for incoming clients
    WiFiClient client = server.available();

    if (client) {  // Check if a client is connected
        boolean currentLineIsBlank = true;

        // While the client is connected, process its request
        while (client.connected()) {
            if (client.available()) {  // Client data available to read
                char c = client.read(); // Read 1 byte (character) from client
                HTTP_req += c;  // Append character to HTTP request

                // Process request once a blank line (end of request) is received
                if (c == '\n' && currentLineIsBlank) {
                    // Send standard HTTP response header
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: keep-alive");
                    client.println();

                    if (HTTP_req.indexOf("ajax_all_switches") > -1) {
                        Serial.println("Getting all switch states.");
                        GetAllSwitchStates(client);
                    } else if (HTTP_req.indexOf("update_setting") > -1) {
                      // Parse the HTTP request for the parameter and value
                      int paramStart = HTTP_req.indexOf('?') + 1; // Start of the parameter string
                      String paramStr = HTTP_req.substring(paramStart); // Get the substring with the parameters

                      // Split the parameter and value
                      int equalSignIndex = paramStr.indexOf('=');
                      String parameter = paramStr.substring(0, equalSignIndex); // e.g., "portion_size"
                      String value = paramStr.substring(equalSignIndex + 1);    // e.g., "3"

                      if(parameter == "pet_type"){ // Edits the type of pet we are looking for. This is anything by default, so that this can run offline
                        int httpIndex = value.indexOf("HTTP");
                        String result = value.substring(0, httpIndex);
                        
                        Serial.print("Request to set desired pet to ");
                        Serial.println(result); 

                        // Set global variable for pet type based on result
                        if(result == "any"){
                          pet_type = ANY;
                        } else if(result == "dog"){
                          pet_type = DOG;
                        } else{
                          pet_type = CAT;
                        }

                      } else{
                        // Convert the value to an integer if needed
                      int intValue = value.toInt();

                      // Handle specific parameters
                      if (parameter == "portion_size") {
                          Serial.print("Updating portion size to: ");
                          Serial.println(intValue);
                          desired_servings = intValue; // Update the global variable for portion size
                      } else if (parameter == "meal_duration") {
                          Serial.print("Updating meal duration to: ");
                          Serial.println(intValue);
                          meal_duration_mins = intValue; // Update the global variable for meal duration
                      } else if (parameter == "time_between_meals") {
                          Serial.print("Updating time between meals to: ");
                          Serial.println(intValue);
                          duration_mins = intValue; // Update the global variable for time between meals
                      }
                      }

                      
                  
                    } else if (HTTP_req.indexOf("override") > -1) {
                        Serial.println("Override toggled.");
                        
                        // Toggle override mode
                        if(system_state == OVERRIDE){
                          system_state = IDLE; 
                          pushIn();
                        } else{
                          system_state = OVERRIDE;
                        }

                        // client.println("ACK"); // Send acknowledgment back to the browser
                    } else if (HTTP_req.indexOf("push_in") > -1) {
                        Serial.println("Force push in.");
                        pushIn();
                        // client.println("ACK"); // Send acknowledgment back to the browser
                    } else if (HTTP_req.indexOf("push_out") > -1) {
                        Serial.println("Force push out.");
                        pushOut();
                        // client.println("ACK"); // Send acknowledgment back to the browser
                    } else if (HTTP_req.indexOf("force_feed") > -1) {
                        Serial.println("Force feeding.");
                        forceDispense();
                        // client.println("ACK"); // Send acknowledgment back to the browser
                    } else if (HTTP_req.indexOf("calibrate") > -1) {
                        Serial.println("Calibration requested.");
                        calibrate();
                        client.println("ACK"); // Send acknowledgment back to the browser
                    } else {  // Standard HTTP request for the web page
                        // Send web page with JavaScript and AJAX calls
                        client.println("<!DOCTYPE html>");
                        client.println("<html>");
                        client.println("<head>");
                        client.println("<title>Pet Feeder UI</title>");

                        // PAGE STYLING SECTION
                        client.println("<style type=\"text/css\">");
                        client.println("h1 { font-family: Arial, Helvetica, sans-serif; font-weight: bold; color: #7ed957; }");
                        client.println(" h3 { font-family: Arial, Helvetica, sans-serif; font-weight: bold; color: #7ed957; }");
                        client.println("body { font-family: Arial, Helvetica, sans-serif; color: #628d4f;  }");
                        client.println("p { font-family: Arial, Helvetica, sans-serif; color: #628d4f; }");
                        client.println("span { display: inline; }");
                        client.println("strong { font-family: Arial, Helvetica, sans-serif; color: #7ed957; }");
                        /* Add container box styling */
                        client.println(".content-box {");
                        client.println("    border: 2px solid #7ed957;"); // Green outline
                        client.println("    border-radius: 10px;"); // Rounded corners
                        client.println("    box-shadow: 2px 2px 8px rgba(0, 0, 0, 0.2);"); // Drop shadow
                        client.println("    padding: 20px;"); // Internal spacing
                        client.println("    margin-top: 20px;"); // Space above the box
                        client.println("    margin-left: 10px;"); 
                        client.println("    margin-right: 10px;"); 
                        client.println("    width: 80%;"); // Width of the box
                        client.println("    height: 80%;"); // Height of the box
                        client.println("    background-color: #f9f9f9;"); // Optional background color
                        client.println("    text-align: left;"); // Align content text to the left
                        client.println("    float: left;"); // Align the box itself to the left
                        client.println("}");
                        /* Style for the calibrate button */
                        client.println("button {");
                        client.println("    font-family: Arial, Helvetica, sans-serif;"); // Match other sections
                        client.println("    font-size: 16px;"); // Consistent text size
                        client.println("    background-color: #7ed957;"); // Green background
                        client.println("    color: white;"); // White text for contrast
                        client.println("    border: none;"); // Remove default border
                        client.println("    border-radius: 5px;"); // Slight rounding for modern look
                        client.println("    margin-top: 10px;");
                        client.println("    padding: 10px 15px;"); // Padding for better click area
                        client.println("    cursor: pointer;"); // Pointer cursor on hover
                        client.println("    box-shadow: 1px 1px 4px rgba(0, 0, 0, 0.2);"); // Subtle shadow
                        client.println("}");

                        // Hover effect for the button
                        client.println("button:hover {");
                        client.println("    background-color: #6bc84c;"); // Slightly darker green
                        client.println("}");
                        client.println("</style>");
                        // END PAGE STYLING

                        // SCRIPT SECTION: JavaScript for AJAX-based switch state retrieval
                        client.println("<script>");

                        // Add a new function to send a request to the server for calibration
                        client.println("function sendCalibrateRequest() {");
                        client.println("    document.getElementById('status').innerText = 'Waiting.';");
                        client.println("    var xhr = new XMLHttpRequest();");
                        client.println("    xhr.onreadystatechange = function() {");
                        client.println("        if (xhr.readyState == 4 && xhr.status == 200) {");
                        client.println("           document.getElementById('status').innerText = 'Calibrated.';");
                        client.println("        }");
                        client.println("    };");
                        client.println("    xhr.open('GET', 'calibrate', true);");
                        client.println("    xhr.send();");
                        client.println("}");
                        
                        // Change the portion size
                        client.println("function updatePortionSize(value) {");
                        client.println("    document.getElementById('portionValue').innerText = value + ' grams';");
                        client.println("    var xhr = new XMLHttpRequest();");
                        client.println("    xhr.open('GET', 'set_portion?value=' + value, true);");
                        client.println("    xhr.send();");
                        client.println("}");

                        client.println("function toggleOverrideMode() {");
                        client.println("    var button = document.getElementById('overrideButton');");
                        client.println("    var controls = document.getElementById('overrideControls');");
                        client.println("    if (button.innerText === 'ENTER OVERRIDE MODE') {");
                        client.println("        button.innerText = 'RELINQUISH OVERRIDE';");
                        client.println("        button.style.backgroundColor = 'green';");
                        client.println("        controls.style.display = 'block';");

                        client.println("    } else {");
                        client.println("        button.innerText = 'ENTER OVERRIDE MODE';");
                        client.println("        button.style.backgroundColor = 'red';");
                        client.println("        controls.style.display = 'none';");
                        client.println("    }");
                        client.println("    var xhr = new XMLHttpRequest();"); // Toggle override
                        client.println("    xhr.open('GET', 'override', true);");
                        client.println("    xhr.send();");
                        client.println("}");
                        client.println("function pushOut() { console.log('PUSH OUT triggered');");
                        client.println("    var xhr = new XMLHttpRequest();"); 
                        client.println("    xhr.open('GET', 'push_out', true);");
                        client.println("    xhr.send();");
                        client.println("}");
                        client.println("function pushIn() { console.log('PUSH IN triggered');");
                        client.println("    var xhr = new XMLHttpRequest();"); 
                        client.println("    xhr.open('GET', 'push_in', true);");
                        client.println("    xhr.send();");
                        client.println("}");
                        client.println("function forceDispense() { console.log('FORCE DISPENSE triggered');");
                        client.println("    var xhr = new XMLHttpRequest();"); 
                        client.println("    xhr.open('GET', 'force_feed', true);");
                        client.println("    xhr.send();");
                        client.println("}");

                        // Update code settings
                        client.println("function updateSetting(parameter) {");
                        client.println("    // Get the value from the corresponding input field");
                        client.println("    const input = document.getElementById(parameter);");
                        client.println("    const value = input.value;");
                        client.println("    if (!value) {");
                        client.println("        document.getElementById('status').innerText = `Please enter a value for ${parameter}.`;");
                        client.println("        return;");
                        client.println("    }");
                        client.println("    // Send the updated setting to the Arduino");
                        client.println("    var xhr = new XMLHttpRequest();");
                        client.println("    xhr.open('GET', `update_setting?${parameter}=${value}`, true);");
                        client.println("    xhr.send();");
                        client.println("}");

                        client.println("function GetAllSwitchStates() {");
                        client.println("    var xhr = new XMLHttpRequest();");
                        client.println("    xhr.onreadystatechange = function () {");
                        client.println("        if (this.readyState == 4 && this.status == 200) {");
                        client.println("            // Parse the response and update the respective HTML elements");
                        client.println("            const states = this.responseText.split('/');");
                        client.println("            console.log(states);");
                        client.println("            document.getElementById('switch_txt1').innerText = states[0];");
                        client.println("            document.getElementById('switch_txt2').innerText = states[1];");
                        client.println("            document.getElementById('switch_txt3').innerText = states[2];");
                        client.println("            document.getElementById('switch_txt4').innerText = states[3];");
                        client.println("            document.getElementById('switch_txt5').innerText = states[4];");
                        client.println("            document.getElementById('switch_txt6').innerText = states[5];");
                        client.println("            document.getElementById('switch_txt7').innerText = states[6];");
                        client.println("        }");
                        client.println("    };");
                        client.println("    xhr.open('GET', 'ajax_all_switches', true);");
                        client.println("    xhr.send();");
                        client.println("}");
                        client.println("");
                        client.println("// Automatically fetch states every 10 seconds");
                        client.println("setInterval(GetAllSwitchStates, 10000);");
                        client.println("</script>");
                        // END SCRIPT SECTION

                        // HTML BODY CONTENT
                        client.println("</head>");
                        client.println("<body onload=\"GetAllSwitchStates()\">");
                        client.println("<img src=\"https://i.imgur.com/aOcJEUp.png\" alt=\"Logo\"><br/>");

                        // Flexbox container
                        client.println("<div style='display: flex; justify-content: space-between; width: 70%; align-items: stretch;'>");

                        // Calibration Section
                        client.println("<div class=\"content-box\">");
                        client.println("<h3>CALIBRATION</h3>");
                        client.println("<div style='display: flex; align-items: baseline;'><strong>Bowl State:</strong> <p id=\"switch_txt1\"> Not requested...</p></div>");
                        client.println("<div style='display: flex; align-items: baseline;'><strong>Feed Countdown Remaining: </strong><p id=\"switch_txt2\"> Not requested...</p><span>&nbsp;minutes</span></div>");
                        client.println("<div style='display: flex; align-items: baseline;'><strong>Current Portion Setting: </strong><p id=\"switch_txt3\"> Not requested...</p><span>&nbsp;servings</span></div>");
                        client.println("<div style='display: flex; align-items: baseline;'><strong>Meal Duration: </strong><p id=\"switch_txt4\"> Not requested...</p><span>&nbsp;minutes</span></div>");
                        client.println("<div style='display: flex; align-items: baseline;'><strong>Hopper Fullness: </strong><p id=\"switch_txt5\"> Not requested...</p><span>&nbsp;%</span></div>");
                        client.println("<div style='display: flex; align-items: baseline;'><strong>Pet Feeder State: </strong><p id=\"switch_txt6\"> Not requested...</p></div>");
                        client.println("<div style='display: flex; align-items: baseline;'><strong>Animal Type: </strong><p id=\"switch_txt7\"> Not requested...</p></div>");
                        client.println("</br>");
                        client.println("<button onclick=\"sendCalibrateRequest()\">Calibrate</button>");
                        client.println("<span id=\"status\">No request.</span>");
                        client.println("</div>");

                        // New Settings Section
                        client.println("<div class=\"content-box\">");
                        client.println("<h3>SETTINGS</h3>");

                        // Dropdown to select pet type
                        client.println("<label for=\"pet_type\">Select Pet Type: </label>");
                        client.println("<select id=\"pet_type\">");
                        client.println("  <option value=\"cat\">Cat</option>");
                        client.println("  <option value=\"dog\">Dog</option>");
                        client.println("  <option value=\"any\">Any</option>");
                        client.println("</select>");
                        client.println("<button onclick=\"updateSetting('pet_type')\">Set</button>");
                        client.println("<br/>");

                        // Serving size
                        client.println("<label for=\"portion_size\">Serving Size (servings): </label>");
                        client.println("<input type=\"number\" id=\"portion_size\" min=\"1\" max=\"10\">");
                        client.println("<button onclick=\"updateSetting('portion_size')\">Set</button>");
                         client.println("</br");
                        // Meal Duration Setting
                   
                        client.println("<label for=\"meal_duration\">Meal Duration (minutes): </label>");
                        client.println("<input type=\"number\" id=\"meal_duration\" min=\"1\" max=\"60\">");
                        client.println("<button onclick=\"updateSetting('meal_duration')\">Set</button>");
                        client.println("</br");

                        // Time Between Meals Setting
                     
                        client.println("<label for=\"time_between_meals\">Time Between Meals (minutes): </label>");
                        client.println("<input type=\"number\" id=\"time_between_meals\" min=\"10\" max=\"1440\">");
                        client.println("<button onclick=\"updateSetting('time_between_meals')\">Set</button>");
                        client.println("</div>");

                        // Override Mode Section
                        client.println("<div class=\"content-box\">");
                        client.println("<h3>OVERRIDE MODE</h3>");
                        client.println("<button id='overrideButton' style='background-color: red; color: white; padding: 10px;' onclick='toggleOverrideMode()'>ENTER OVERRIDE MODE</button>");
                        client.println("<div id='overrideControls' style='display: none; margin-top: 10px;'>");
                        client.println("<button style='padding: 10px; margin-bottom: 5px;' onclick='pushOut()'>PUSH OUT</button><br>");
                        client.println("<button style='padding: 10px; margin-bottom: 5px;' onclick='pushIn()'>PUSH IN</button><br>");
                        client.println("<button style='padding: 10px;' onclick='forceDispense()'>FORCE DISPENSE</button>");
                        client.println("</div>");
                        client.println("</div>");

                        client.println("</div>"); // Close flexbox container

                        client.println("</body>");
                        client.println("</html>");
                    }

                    Serial.println("Http request recieved.");
                    HTTP_req = "";  // Reset HTTP request string for next client
                    break;
                }

                // Track blank lines for request end detection
                if (c == '\n') {
                    currentLineIsBlank = true;
                } 
                else if (c != '\r') {
                    currentLineIsBlank = false;
                }
            } // end if (client.available())
        } // end while (client.connected())

        // Delay to give the web browser time to receive data
        delay(1);
        // Close the connection after response is sent
        client.stop();
    } // end if (client)
}

// Print the Wifi Status
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

// ========================== OVERRIDE COMMANDS =======================================

// In override food, force move the auger
void forceDispense(){
  Serial.println("Force dispensing.");
  runAtPercentMaxSpeed(0.95,1000);
}

// In override mode, force the bowl to push in
void pushIn(){
  Serial.println("Pushing in.");
  bowl_extension_servo.write(0);
  extended = false;
  
}

// In override mode, force the bowl to push out
void pushOut(){
  Serial.println("Pushing out.");
  bowl_extension_servo.write(180);
  extended = true;
}

// ========================= WEB HELPER FUNCTIONS ======================================

// Quickly get all switch states in one response
void GetAllSwitchStates(WiFiClient cl) {
    String response = "";

    // Append state for each switch separated by '/'
    response += !extended ? "SECURED" : (extended_bowl_ls.isPressed() ? "EXTENDED" : "IN TRANSIT");
    response += "/";

    // Bottom out the duration remaining to zero minutes 
    float time = (duration_mins - (millis()/60000- time_start));
    response += (time < 0) ? 0 : duration_mins; // Minutes remaining until feed time
    response += "/";
    response += desired_servings;
    response += "/";
    response += meal_duration_mins; 
    response += "/";

    // Bottom out the hopper depth to 0%
    int depth = (kibble_zero_depth_cm - 5 - getHopperDepth()) * (100/(kibble_zero_depth_cm-5)); // Assuming this is a variable for Switch 5
    response += (kibble_zero_depth_cm < 0) ? 0 : kibble_zero_depth_cm;
    response += "/";
    switch(system_state){
      case IDLE:
        response+="IDLE";
        break;
      case OVERRIDE:
        response+="OVERRIDE";
        break;
      case SENSED_MOVEMENT:
        response+="CHECKING THAT YOUR PET IS NOT AN IMPOSTER.";
        break;
      case FILLING_BOWL:
        response+="FILLING BOWL";
        break;
      case BOWL_EXTENDED:
        response+="BOWL EXTENDED, WAITING FOR PET TO EAT";
        break;
    }
    response += "/";
      switch(pet_type){
      case ANY:
        response+="ANY";
        break;
      case DOG:
        response+="DOG";
        break;
      case CAT:
        response+="CAT";
        break;
    }
      
    // Send the compiled response
    cl.println(response);
    Serial.println("Sending the following response:");
    Serial.println(response);
}
