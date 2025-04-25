#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <Bounce2.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

// --- WiFi Credentials ---
// !!! IMPORTANT: Replace with your network details !!!
const char* ssid = ""; // --- CHANGE TO YOUR WIFI SSID ---
const char* password = ""; // --- CHANGE TO YOUR WIFI PASSWORD ---
// ------------------------

// --- Constants ---
constexpr int NUM_SERVOS = 6;
constexpr int MAX_RECORDINGS = 100;
constexpr int RECORD_SIZE = NUM_SERVOS * sizeof(uint16_t);
constexpr int EEPROM_SIZE = MAX_RECORDINGS * RECORD_SIZE + 1;
constexpr int EEPROM_NUM_RECORDED_ADDR = MAX_RECORDINGS * RECORD_SIZE;

constexpr int SERVO_DEADZONE = 8; // Increase if servos jitter when stopped
constexpr unsigned long SERVO_STEP_INTERVAL = 15; // Servo smoothing update rate (ms)
constexpr unsigned long PLAYBACK_STEP_DELAY = 1000; // Delay between playback steps (ms)
constexpr unsigned long BUTTON_DEBOUNCE_MS = 50; // Debounce time for buttons
constexpr unsigned long CLEAR_HOLD_DURATION_MS = 1000; // Hold time for clear button (ms)

// Servo pulse limits (adjust for your specific servos)
constexpr int MIN_SERVO_PULSE = 600;
constexpr int MAX_SERVO_PULSE = 2400;
constexpr int DEFAULT_SERVO_PULSE = 1500; // Center position

// --- Pin Definitions ---
#define LED_PIN 26 // Built-in LED or NeoPixel data pin
#define NUM_PIXELS 1 // Number of NeoPixels connected

// Analog input pins for potentiometers (Adjust if different)
const int potPins[NUM_SERVOS] = {39, 36, 35, 34, 33, 32};

// Digital input pins for buttons (Using internal pullups)
#define MEMORY_CLEAR_BUTTON_PIN 5
#define RECORD_PLAY_BUTTON_PIN 18  // SWAPPED: Pin for Recording
#define PROG_STOP_BUTTON_PIN 19    // Stop / Reset button
#define CYCLE_START_BUTTON_PIN 23  // SWAPPED: Pin for Playback (Manual/Semi/Full)

// --- Global Objects ---
Adafruit_PWMServoDriver pwm; // PCA9685 driver object
Adafruit_NeoPixel ledStrip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800); // NeoPixel object
WebServer server(80); // Web server object

// --- State Machine ---
enum OperatingMode {
    MODE_IDLE,             // Waiting for input
    MODE_MANUAL_POT,       // Controlled by potentiometers
    MODE_WEB_CONTROL,      // Controlled via web interface
    MODE_PLAYBACK_MANUAL,  // Stepping through sequence manually
    MODE_PLAYBACK_SEMI_AUTO, // Playing sequence, stops at end
    MODE_PLAYBACK_FULL_AUTO, // Playing sequence, loops at end
    MODE_CLEARING_MEM,     // Busy clearing EEPROM
    MODE_ERROR_WIFI,       // WiFi connection failed/lost
    MODE_ERROR_GENERIC     // Other system error (EEPROM, etc.)
};
OperatingMode currentMode = MODE_IDLE, previousMode = MODE_IDLE;

// --- Servo Positions ---
uint16_t targetServoPos[NUM_SERVOS]; // Where the servos should go
uint16_t currentServoPos[NUM_SERVOS]; // Where the servos currently are (smoothed)

// --- Playback & Recording ---
int playbackIndex = 0; // Index of the next step to play/load
int numRecorded = 0; // Number of steps currently stored in EEPROM

// --- Button Handling ---
Bounce clearMemoryButton = Bounce();
Bounce recordPlayButton = Bounce(); // Renamed, but handles Record action
Bounce progStopButton = Bounce();
Bounce cycleStartButton = Bounce(); // Renamed, but handles Playback actions

// Timers and state flags for button multi-press/hold logic
unsigned long lastServoStepTime = 0, lastPlaybackActionTime = 0;
constexpr unsigned long DOUBLE_PRESS_INTERVAL_MS = 500; // Max time between presses for double-press
constexpr unsigned long FULL_AUTO_HOLD_MS = 1000;      // Hold time for full-auto playback trigger
constexpr unsigned long STOP_DOUBLE_PRESS_INTERVAL_MS = 500; // Max time for stop double-press

// Run (Play) button state variables
static unsigned long runBtnFirstPressTime = 0;
static unsigned long runBtnFirstReleaseTime = 0;
static bool runBtnWaitingSecond = false;
static bool runBtnSecondPressActive = false;
static unsigned long runBtnSecondPressStart = 0;
static bool runBtnActionTaken = false;

// Stop button state variables
static unsigned long stopBtnFirstPressTime = 0;
static unsigned long stopBtnFirstReleaseTime = 0;
static bool stopBtnWaitingSecond = false;
static bool stopBtnActionTaken = false;

// --- Function Declarations ---
void updateLedStatus();
void stopAllMovement(bool returnToIdle = true);
void saveStepToEEPROM(int index, const uint16_t positions[]);
void loadStepFromEEPROM(int index, uint16_t positions[]);
void clearEEPROM();
void readPotentiometers();
void applyServoSmoothing();
void handleButtons();
void startPlayback(OperatingMode playbackMode);
void handlePlayback();
String generatePageHTML();
void handleRoot();
void handleStatus();
void handleServo();
void handleCommand();
void handleSteps();
void handleUpdateStep();
void handleNotFound();
void setupWebServer();
void connectWiFi();

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for serial connection (optional, remove for standalone)
    Serial.println("\n--- Robot Arm Booting Up ---");

    ledStrip.begin();
    ledStrip.setBrightness(50); // Adjust brightness (0-255)
    ledStrip.show(); // Initialize LED off

    Serial.print("Initializing EEPROM... ");
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed!");
        currentMode = MODE_ERROR_GENERIC;
        updateLedStatus(); // Show error color
        while(1) delay(1000); // Halt on critical error
    }
    Serial.println("OK.");
    numRecorded = EEPROM.read(EEPROM_NUM_RECORDED_ADDR);
    // Validate EEPROM data
    if (numRecorded > MAX_RECORDINGS || numRecorded < 0) {
        Serial.print("Invalid number of recordings found in EEPROM (");
        Serial.print(numRecorded);
        Serial.println("). Resetting to 0.");
        numRecorded = 0;
        EEPROM.write(EEPROM_NUM_RECORDED_ADDR, 0);
        if (!EEPROM.commit()) {
             Serial.println("EEPROM commit failed during validation reset!");
             // Non-critical error, maybe proceed?
        }
    }
    Serial.print("Number of recorded steps found: ");
    Serial.println(numRecorded);

    Serial.print("Initializing PWM Driver... ");
    pwm.begin();
    pwm.setPWMFreq(50); // Standard servo frequency
    // pwm.setOscillatorFrequency(27000000); // Uncomment/adjust if needed for your specific PCA9685 board
    Serial.println("OK.");

    Serial.print("Initializing Servos to default position... ");
    for (int i = 0; i < NUM_SERVOS; i++) {
        targetServoPos[i] = DEFAULT_SERVO_PULSE;
        currentServoPos[i] = DEFAULT_SERVO_PULSE;
        pwm.writeMicroseconds(i, DEFAULT_SERVO_PULSE);
    }
    Serial.println("OK.");

    Serial.print("Initializing Buttons... ");
    pinMode(MEMORY_CLEAR_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RECORD_PLAY_BUTTON_PIN, INPUT_PULLUP);
    pinMode(PROG_STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CYCLE_START_BUTTON_PIN, INPUT_PULLUP);

    // Attach buttons to Bounce2 library
    clearMemoryButton.attach(MEMORY_CLEAR_BUTTON_PIN);
    clearMemoryButton.interval(BUTTON_DEBOUNCE_MS);
    recordPlayButton.attach(RECORD_PLAY_BUTTON_PIN);
    recordPlayButton.interval(BUTTON_DEBOUNCE_MS);
    progStopButton.attach(PROG_STOP_BUTTON_PIN);
    progStopButton.interval(BUTTON_DEBOUNCE_MS);
    cycleStartButton.attach(CYCLE_START_BUTTON_PIN);
    cycleStartButton.interval(BUTTON_DEBOUNCE_MS);
    Serial.println("OK.");

    Serial.print("Connecting to WiFi (SSID: ");
    Serial.print(ssid);
    Serial.print(")... ");
    connectWiFi(); // Attempt connection

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Setting up Web Server... ");
        setupWebServer(); // Setup server endpoints
        Serial.println("OK.");
        currentMode = MODE_MANUAL_POT; // Start in Pot mode if WiFi connects
    } else {
        Serial.println("Failed to connect to WiFi.");
        currentMode = MODE_ERROR_WIFI; // Enter WiFi error mode
    }

    // Read initial pot values if starting in pot mode
    if(currentMode == MODE_MANUAL_POT) {
        Serial.println("Starting in Manual Potentiometer mode.");
        readPotentiometers();
    } else if (currentMode == MODE_ERROR_WIFI) {
        Serial.println("Starting in WiFi Error mode.");
    }

    playbackIndex = 0; // Start playback from the beginning
    previousMode = currentMode; // Sync previous mode
    updateLedStatus(); // Set initial LED color based on mode
    Serial.println("--- Setup Complete ---");
}

// --- Main Loop ---
void loop() {
    // Handle WiFi connection status and Web Server requests
    if (WiFi.status() == WL_CONNECTED) {
        server.handleClient(); // Process incoming client requests
    } else if (currentMode != MODE_ERROR_WIFI && currentMode != MODE_ERROR_GENERIC) {
        // WiFi connection lost after initially connecting
        if (previousMode != MODE_ERROR_WIFI) { // Prevent spamming
             Serial.println("WiFi connection lost!");
             currentMode = MODE_ERROR_WIFI;
             stopAllMovement(false); // Stop movement but don't change mode further
        }
    }

    // Update button states (call frequently)
    handleButtons();

    // Perform actions based on current operating mode
    switch (currentMode) {
        case MODE_MANUAL_POT:
            readPotentiometers(); // Read pots to update target positions
            applyServoSmoothing(); // Smoothly move servos towards target
            break;
        case MODE_WEB_CONTROL:
            // Target positions are set via web commands (handleServo)
            applyServoSmoothing(); // Smoothly move servos towards target
            break;
        case MODE_PLAYBACK_MANUAL:
        case MODE_PLAYBACK_SEMI_AUTO:
        case MODE_PLAYBACK_FULL_AUTO:
            handlePlayback(); // Manage playback sequence logic
            applyServoSmoothing(); // Smoothly move servos towards target step
            break;
        case MODE_IDLE:
            // Do nothing specific, wait for button/web command
            // Optional: small delay or yield() if needed
            // delay(1);
            break;
        case MODE_CLEARING_MEM:
            // EEPROM clearing happens in clearEEPROM function triggered by button/command
            // Mode is handled there. Maybe flash LED differently here?
             updateLedStatus(); // Keep flashing LED
             delay(5); // Small delay during clearing
            break;
        case MODE_ERROR_WIFI:
        case MODE_ERROR_GENERIC:
            // Error state, usually requires reset. Do nothing active.
            // LED is set by updateLedStatus
            break;
        default:
            // Should not happen, but handle unknown state
            Serial.print("Warning: Entered unknown operating mode: ");
            Serial.println(currentMode);
            currentMode = MODE_IDLE; // Recover to Idle state
            break;
    }

    // Update LED if mode has changed since last loop iteration
    if (currentMode != previousMode) {
        Serial.print("Mode changed from ");
        Serial.print(previousMode);
        Serial.print(" to ");
        Serial.println(currentMode);
        updateLedStatus(); // Update LED color to reflect new mode
        previousMode = currentMode; // Store new mode for next comparison
    }
}

// --- WiFi Connection ---
void connectWiFi() {
    WiFi.mode(WIFI_STA); // Set ESP32 to Station mode
    WiFi.begin(ssid, password); // Start connection attempt
    // Wait for connection with timeout
    for (int retries = 0; WiFi.status() != WL_CONNECTED && retries < 30; retries++) {
        delay(500);
        Serial.print(".");
        // Flash yellow during connection attempt
        ledStrip.setPixelColor(0, ledStrip.Color(50, 50, 0));
        ledStrip.show();
        delay(50);
        ledStrip.setPixelColor(0, ledStrip.Color(0, 0, 0)); // Brief off flash
        ledStrip.show();
    }
    // Final status update (will be refined by updateLedStatus later)
    ledStrip.setPixelColor(0, WiFi.status() == WL_CONNECTED ? ledStrip.Color(0, 0, 0) : ledStrip.Color(255,0,0));
    ledStrip.show();
}

// --- Web Server Setup ---
void setupWebServer() {
    server.on("/", HTTP_GET, handleRoot);             // Serve the main HTML page
    server.on("/status", HTTP_GET, handleStatus);     // Provide current status JSON
    server.on("/servo", HTTP_POST, handleServo);      // Receive servo position updates from web
    server.on("/command", HTTP_POST, handleCommand);  // Receive commands (play, record, stop, etc.)
    server.on("/steps", HTTP_GET, handleSteps);       // Provide list of recorded steps JSON
    server.on("/step", HTTP_POST, handleUpdateStep);  // Update a specific step's data
    server.onNotFound(handleNotFound);                // Handle 404 errors
    server.begin(); // Start the web server
}

// --- Web Server Handlers ---
void handleRoot() {
    server.send(200, "text/html", generatePageHTML());
}

void handleStatus() {
    StaticJsonDocument<512> doc; // JSON document for status
    doc["mode"] = currentMode; // Send the raw mode number
    String modeStr = "Unknown";
    // Determine descriptive mode string
    switch(currentMode) {
        case MODE_IDLE: modeStr = "Idle"; break;
        case MODE_MANUAL_POT: modeStr = "Manual (Pot)"; break;
        case MODE_WEB_CONTROL: modeStr = "Web Control"; break;
        case MODE_PLAYBACK_MANUAL: modeStr = "Playback Manual"; break;
        case MODE_PLAYBACK_SEMI_AUTO: modeStr = "Playback Semi-Auto"; break;
        case MODE_PLAYBACK_FULL_AUTO: modeStr = "Playback Full-Auto"; break;
        case MODE_CLEARING_MEM: modeStr = "Clearing Memory"; break;
        case MODE_ERROR_WIFI: modeStr = "WiFi Error"; break;
        case MODE_ERROR_GENERIC: modeStr = "System Error"; break;
    }
    doc["mode_str"] = modeStr; // Add descriptive string to JSON

    // Simplified control mode string (less useful now with JS changes, but keep for potential use)
    doc["control_mode_str"] = (currentMode == MODE_MANUAL_POT) ? "Potentiometer" : "Web";

    // Add other status info
    doc["steps_recorded"] = numRecorded;
    doc["next_playback_index"] = playbackIndex;
    doc["ip_address"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "N/A";

    // Add current servo positions
    JsonArray currentPos = doc.createNestedArray("current_pos");
    for (int i = 0; i < NUM_SERVOS; i++) {
        currentPos.add(currentServoPos[i]);
    }

    // Serialize JSON and send response
    String jsonResponse;
    serializeJson(doc, jsonResponse);
    server.send(200, "application/json", jsonResponse);
}

void handleServo() {
    // Check if in correct mode to accept web servo commands
    if (currentMode != MODE_WEB_CONTROL) {
        server.send(403, "text/plain", "Forbidden: Not in Web Control mode");
        return;
    }
    // Check if request body exists
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Bad Request: Missing JSON body");
        return;
    }
    // Parse JSON request body
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if (error) {
        Serial.print(F("deserializeJson() failed in handleServo: "));
        Serial.println(error.f_str());
        server.send(400, "text/plain", "Bad Request: Invalid JSON");
        return;
    }

    // Validate JSON structure
    if (!doc.containsKey("servos") || !doc["servos"].is<JsonArray>()) {
         server.send(400, "text/plain", "Bad Request: Missing or invalid 'servos' array");
         return;
    }
    JsonArray servoValues = doc["servos"];
    if (servoValues.size() != NUM_SERVOS) {
        server.send(400, "text/plain", "Bad Request: Incorrect number of servo values");
        return;
    }

    // Apply validated values to target positions
    bool valuesOk = true;
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (servoValues[i].is<int>()) {
             targetServoPos[i] = constrain(servoValues[i].as<int>(), MIN_SERVO_PULSE, MAX_SERVO_PULSE);
        } else {
            valuesOk = false; // Found non-integer value
            break;
        }
    }

    if (valuesOk) {
        server.send(200, "text/plain", "OK"); // Acknowledge successful update
    } else {
        server.send(400, "text/plain", "Bad Request: Non-integer value in servos array");
    }
}


void handleCommand() {
    Serial.print("handleCommand received. Current mode: ");
    Serial.print(currentMode);
    Serial.print(", Steps recorded: ");
    Serial.print(numRecorded);

    if (!server.hasArg("plain")) {
        Serial.println(" - Bad Request: Missing JSON body");
        server.send(400, "text/plain", "Bad Request: Missing JSON body"); return;
    }

    StaticJsonDocument<192> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if (error) {
        Serial.print(" - Bad JSON: ");
        Serial.println(error.f_str());
        server.send(400, "text/plain", "Bad Request: Invalid JSON"); return;
    }

    String command = doc["command"] | ""; // Get command string, default to empty
    Serial.print(", Command: '");
    Serial.print(command);
    Serial.println("'");

    bool success = true; // Assume success unless error occurs
    String message = "Command '" + command + "' executed."; // Default success message

    // --- Command Processing Logic ---
    if (command == "play_manual") {
        if ((currentMode == MODE_IDLE || currentMode == MODE_MANUAL_POT || currentMode == MODE_WEB_CONTROL) && numRecorded > 0) {
            startPlayback(MODE_PLAYBACK_MANUAL);
            message = "Manual step playback started.";
        } else {
            message = (numRecorded == 0) ? "No steps recorded to play." : "Cannot start manual playback in current mode.";
            success = false;
        }
    } else if (command == "play_semi_auto") {
        if ((currentMode == MODE_IDLE || currentMode == MODE_MANUAL_POT || currentMode == MODE_WEB_CONTROL) && numRecorded > 0) {
            startPlayback(MODE_PLAYBACK_SEMI_AUTO);
            message = "Semi-automatic playback started.";
        } else {
            message = (numRecorded == 0) ? "No steps recorded to play." : "Cannot start semi-auto playback in current mode.";
            success = false;
        }
    } else if (command == "play_full_auto") {
        if ((currentMode == MODE_IDLE || currentMode == MODE_MANUAL_POT || currentMode == MODE_WEB_CONTROL) && numRecorded > 0) {
            startPlayback(MODE_PLAYBACK_FULL_AUTO);
            message = "Full automatic playback started.";
        } else {
            message = (numRecorded == 0) ? "No steps recorded to play." : "Cannot start full auto playback in current mode.";
            success = false;
        }
    } else if (command == "record") {
        // Allow recording from Idle, Pot, or Web modes
        if (currentMode == MODE_MANUAL_POT || currentMode == MODE_WEB_CONTROL || currentMode == MODE_IDLE) {
            if (numRecorded < MAX_RECORDINGS) {
                saveStepToEEPROM(numRecorded, currentServoPos); // Record current smoothed position
                numRecorded++;
                EEPROM.write(EEPROM_NUM_RECORDED_ADDR, numRecorded);
                if (EEPROM.commit()) {
                     message = "Position recorded as step " + String(numRecorded - 1);
                     Serial.println(message);
                     playbackIndex = numRecorded % MAX_RECORDINGS; // Update next index
                } else {
                    message = "EEPROM commit failed during record."; success = false; Serial.println(message);
                    if(numRecorded > 0) numRecorded--; // Attempt rollback
                }
            } else { message = "Max recordings reached (" + String(MAX_RECORDINGS) + ")."; success = false; }
        } else { message = "Cannot record in current mode."; success = false; }
    } else if (command == "clear") {
        // Allow clearing unless playing back or already clearing
        if (currentMode != MODE_PLAYBACK_MANUAL && currentMode != MODE_PLAYBACK_SEMI_AUTO &&
            currentMode != MODE_PLAYBACK_FULL_AUTO && currentMode != MODE_CLEARING_MEM) {
            stopAllMovement(false); // Stop movement first
            clearEEPROM(); // Handles mode changes and commit
            message = "All steps cleared via web command.";
        } else { message = "Cannot clear memory during playback or while already clearing."; success = false; }
    } else if (command == "stop") {
        // Allow stop from most active modes
        if (currentMode >= MODE_MANUAL_POT && currentMode <= MODE_PLAYBACK_FULL_AUTO) {
            stopAllMovement(true); // Stop and return to Idle
            message = "Movement stopped, returned to Idle mode.";
        } else if (currentMode == MODE_IDLE) { message = "Already in Idle mode."; success = true; }
        else { message = "Nothing to stop in current mode."; success = true; } // Not an error if already stopped/error
    }
    // --- Inside handleCommand() ---
    else if (command == "toggle_mode") {
        if (currentMode == MODE_MANUAL_POT) {
            // Pot -> Web
            currentMode = MODE_WEB_CONTROL;
            message = "Web Control mode activated.";
            Serial.println(message);
        } else if (currentMode == MODE_WEB_CONTROL) {
            // Web -> Pot
            readPotentiometers(); // Read pots before switching back
            currentMode = MODE_MANUAL_POT;
            message = "Manual Potentiometer mode activated.";
            Serial.println(message);
        } else if (currentMode == MODE_IDLE) {
             // Idle -> Web (NEW BEHAVIOR)
             currentMode = MODE_WEB_CONTROL;
             message = "Switched from Idle to Web Control mode.";
             Serial.println(message);
             // Note: Do NOT readPotentiometers() when switching TO Web Control
        } else if (currentMode >= MODE_PLAYBACK_MANUAL && currentMode <= MODE_PLAYBACK_FULL_AUTO) {
            // Cannot toggle during playback
            message = "Cannot toggle mode during playback.";
            success = false;
        } else {
             // Cannot toggle from Error, Clearing, etc.
             message = "Cannot toggle mode from current state (" + String(currentMode) + ").";
             success = false;
        }
    }
    // --- End of toggle_mode case ---
    
    // --- END REVISED toggle_mode LOGIC ---
    else if (command == "jump_to_step") {
        int requestedIndex = doc["step_index"] | -1;
        // Allow jumping when Idle or in manual control modes
        if ((currentMode == MODE_IDLE || currentMode == MODE_MANUAL_POT || currentMode == MODE_WEB_CONTROL) && numRecorded > 0) {
            if (requestedIndex >= 0 && requestedIndex < numRecorded) {
                loadStepFromEEPROM(requestedIndex, targetServoPos); // Load target
                // Let smoothing handle the movement
                playbackIndex = (requestedIndex + 1) % numRecorded; // Set next playback index
                message = "Jumped target position to step " + String(requestedIndex) + ". Next playback step: " + String(playbackIndex);
                Serial.println(message);
            } else { message = "Invalid step index (" + String(requestedIndex) + "). Must be 0-" + String(numRecorded - 1) + "."; success = false; }
        } else { message = (numRecorded == 0) ? "No steps recorded to jump to." : "Cannot jump to step in current mode."; success = false; }
    } else if (command == "delete_step") {
        int indexToDelete = doc["step_index"] | -1;
        if (indexToDelete < 0 || indexToDelete >= numRecorded) {
            message = "Invalid step index (" + String(indexToDelete) + ") for deletion. Must be 0-" + String(numRecorded - 1) + "."; success = false;
        } else if (currentMode >= MODE_PLAYBACK_MANUAL && currentMode <= MODE_CLEARING_MEM) {
            message = "Cannot delete steps during playback or while clearing memory."; success = false;
        } else {
            Serial.print("Deleting step "); Serial.println(indexToDelete);
            uint16_t tempPositions[NUM_SERVOS];
            // Shift subsequent steps down
            for (int i = indexToDelete; i < numRecorded - 1; i++) {
                loadStepFromEEPROM(i + 1, tempPositions);
                saveStepToEEPROM(i, tempPositions);
            }
            numRecorded--;
            EEPROM.write(EEPROM_NUM_RECORDED_ADDR, numRecorded);
            if (EEPROM.commit()) {
                 message = "Step " + String(indexToDelete) + " deleted. Total steps: " + String(numRecorded); Serial.println(message);
                 // Adjust playback index if needed
                 if (playbackIndex > indexToDelete) playbackIndex--;
                 if (playbackIndex >= numRecorded) playbackIndex = (numRecorded > 0) ? 0 : 0;
                 Serial.print("Adjusted playback index to: "); Serial.println(playbackIndex);
            } else { message = "EEPROM commit failed during step deletion."; success = false; Serial.println(message); numRecorded++; /* Attempt rollback */ }
        }
    }
    else { message = "Unknown command: '" + command + "'"; success = false; }

    // Send JSON response back to the web client
    StaticJsonDocument<128> resp;
    resp["success"] = success;
    resp["message"] = message;
    String respJson;
    serializeJson(resp, respJson);
    server.send(success ? 200 : 400, "application/json", respJson);
    // Note: Mode changes are reflected in the next /status poll by the client
}


void handleSteps() {
    // Use DynamicJsonDocument for potentially large step lists
    DynamicJsonDocument doc(EEPROM_SIZE + 1024); // Allocate enough space

    JsonArray stepsArray = doc.createNestedArray("steps");
    if (numRecorded > 0) {
        uint16_t stepPositions[NUM_SERVOS];
        for (int i = 0; i < numRecorded; i++) {
            loadStepFromEEPROM(i, stepPositions);
            JsonArray step = stepsArray.createNestedArray();
            for (int j = 0; j < NUM_SERVOS; j++) {
                step.add(stepPositions[j]);
            }
        }
    }
    // Serialize and send the array of steps
    String jsonResponse;
    size_t jsonSize = serializeJson(doc, jsonResponse);
     if (jsonSize == 0) {
         Serial.println("Error: Failed to serialize steps JSON.");
         server.send(500, "text/plain", "Internal Server Error: Failed to serialize steps");
     } else {
         server.send(200, "application/json", jsonResponse);
     }
}

void handleUpdateStep() {
    if (!server.hasArg("plain")) { server.send(400, "text/plain", "Bad Request: Missing JSON body"); return; }

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
     if (error) { Serial.print(F("deserializeJson() failed in handleUpdateStep: ")); Serial.println(error.f_str()); server.send(400, "text/plain", "Bad Request: Invalid JSON"); return; }

    // Validate required fields
    if (!doc.containsKey("index") || !doc.containsKey("positions") || !doc["positions"].is<JsonArray>()) { server.send(400, "text/plain", "Bad Request: Missing 'index' or 'positions' array"); return; }

    int index = doc["index"];
    JsonArray positions = doc["positions"];

    if (index < 0 || index >= numRecorded || positions.size() != NUM_SERVOS) { server.send(400, "text/plain", "Bad Request: Invalid index or positions array size"); return; }

    // Validate and apply positions
    uint16_t newPositions[NUM_SERVOS];
    bool validPositions = true;
    for(int i=0; i<NUM_SERVOS; ++i) {
        if (positions[i].is<int>()) { newPositions[i] = constrain(positions[i].as<int>(), MIN_SERVO_PULSE, MAX_SERVO_PULSE); }
        else { validPositions = false; break; }
    }

    if (!validPositions) { server.send(400, "text/plain", "Bad Request: Non-integer value in positions array"); return; }

    // Save validated positions to EEPROM
    saveStepToEEPROM(index, newPositions);
    if(EEPROM.commit()) {
        Serial.print("Updated step "); Serial.println(index);
        StaticJsonDocument<64> resp; resp["success"] = true; resp["message"] = "Step " + String(index) + " updated.";
        String respJson; serializeJson(resp, respJson);
        server.send(200, "application/json", respJson);
    } else {
         Serial.print("EEPROM commit failed while updating step "); Serial.println(index);
         server.send(500, "text/plain", "Internal Server Error: Failed to save step to EEPROM");
    }
}

void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

// --- Core Logic Functions ---

void applyServoSmoothing() {
    unsigned long now = millis();
    // Check if enough time has passed since last servo update
    if (now - lastServoStepTime < SERVO_STEP_INTERVAL) {
        return;
    }
    lastServoStepTime = now; // Reset timer

    bool changed = false; // Track if any servo moved this interval
    for (int i = 0; i < NUM_SERVOS; ++i) {
        // Ensure target is within bounds (redundant check, good practice)
        targetServoPos[i] = constrain(targetServoPos[i], MIN_SERVO_PULSE, MAX_SERVO_PULSE);

        int diff = targetServoPos[i] - currentServoPos[i]; // Calculate difference

        // Only move if difference is outside the deadzone

        // Only move if difference is significant enough (outside deadzone)
        if (abs(diff) > SERVO_DEADZONE) {
            // Calculate step size (e.g., 1/10th of the difference, capped)
            // int step = constrain(diff / 10, -10, 10); // Simple linear step
            // Smoother step (smaller steps for smaller differences)
            int step = diff / 8; // Adjust divisor for speed
            step = constrain(step, -15, 15); // Max step size per interval

            // Ensure minimum step if diff is non-zero but step is calculated as 0
            if (step == 0) {
                step = (diff > 0) ? 1 : -1;
            }

            // Apply step
            if (abs(diff) <= abs(step)) {
                // If the remaining difference is smaller than the step, just go to target
                currentServoPos[i] = targetServoPos[i];
            } else {
                currentServoPos[i] += step;
            }
            // Ensure current position doesn't overshoot due to rounding/step logic
            currentServoPos[i] = constrain(currentServoPos[i], MIN_SERVO_PULSE, MAX_SERVO_PULSE);

            // Write the new position to the servo
            pwm.writeMicroseconds(i, currentServoPos[i]);
            changed = true; // Mark that a servo moved
        }
    }
    // Optional: If no servos moved, maybe increase sleep time slightly?
    // if (!changed) { delay(1); }
}


void stopAllMovement(bool returnToIdle) {
    Serial.print("Stopping all movement. ");
    // Set target positions to current positions to stop smoothing/movement requests
    for (int i = 0; i < NUM_SERVOS; i++) {
        targetServoPos[i] = currentServoPos[i];
    }
    // Optional: Re-assert current position to PWM driver? Usually not needed.
    // for (int i = 0; i < NUM_SERVOS; i++) {
    //     pwm.writeMicroseconds(i, currentServoPos[i]);
    // }

    if (returnToIdle && currentMode != MODE_ERROR_WIFI && currentMode != MODE_ERROR_GENERIC) {
        Serial.println("Returning to IDLE mode.");
        currentMode = MODE_IDLE;
    } else {
        Serial.println("Maintaining current mode.");
    }
    // Note: updateLedStatus() will be called in the main loop when it detects the mode change (if any)
}

void readPotentiometers() {
    if (currentMode != MODE_MANUAL_POT) return; // Only read if in pot mode

    for (int i = 0; i < NUM_SERVOS; i++) {
        int potValue = analogRead(potPins[i]);
        // Apply mapping - ensure MIN/MAX pulses are correct
        uint16_t newTarget = map(potValue, 0, 4095, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
        // Optional: Add a small deadzone for potentiometer jitter?
        // if (abs(newTarget - targetServoPos[i]) > POT_DEADZONE) {
             targetServoPos[i] = newTarget; // Update target position for smoothing
        // }
    }
}

void saveStepToEEPROM(int index, const uint16_t positions[]) {
    if (index < 0 || index >= MAX_RECORDINGS) {
        Serial.print("ERROR: Attempted to save step to invalid index: ");
        Serial.println(index);
        return;
    }
    int addr = index * RECORD_SIZE;
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Ensure positions are valid before writing
        EEPROM.writeUShort(addr + i * sizeof(uint16_t), constrain(positions[i], MIN_SERVO_PULSE, MAX_SERVO_PULSE));
    }
    // Note: EEPROM.commit() is called after updating numRecorded elsewhere
}

void loadStepFromEEPROM(int index, uint16_t positions[]) {
    if (index < 0 || index >= numRecorded) { // Check against numRecorded
        Serial.print("Warning: Attempted to load step from invalid or unrecorded index: ");
        Serial.print(index);
        Serial.print(" (NumRecorded: ");
        Serial.print(numRecorded);
        Serial.println("). Using default positions.");
        // Load default positions if index is invalid
        for(int i=0; i<NUM_SERVOS; ++i) {
            positions[i] = DEFAULT_SERVO_PULSE;
        }
        return;
    }
    int addr = index * RECORD_SIZE;
    for (int i = 0; i < NUM_SERVOS; i++) {
        positions[i] = EEPROM.readUShort(addr + i * sizeof(uint16_t));
        // Optional: Validate loaded values?
        positions[i] = constrain(positions[i], MIN_SERVO_PULSE, MAX_SERVO_PULSE);
    }
}

void clearEEPROM() {
    Serial.println("Clearing EEPROM...");
    // Enter clearing mode (blocks other actions, shows specific LED)
    OperatingMode previousClearingMode = currentMode; // Store mode before clearing
    currentMode = MODE_CLEARING_MEM;
    updateLedStatus(); // Show clearing color immediately

    for (int i = 0; i < EEPROM_NUM_RECORDED_ADDR; i++) { // Clear only recorded steps area
        if (EEPROM.read(i) != 0xFF) { // Optimization: only write if not already cleared
            EEPROM.write(i, 0xFF);
        }
        // Add a small delay or yield periodically if clearing takes too long
        // if (i % 100 == 0) { delay(1); yield(); }
    }
    EEPROM.write(EEPROM_NUM_RECORDED_ADDR, 0); // Write 0 recorded steps
    if(EEPROM.commit()) {
        numRecorded = 0;
        playbackIndex = 0;
        Serial.println("EEPROM cleared successfully.");
    } else {
        Serial.println("EEPROM commit failed during clear!");
        // Handle error? Maybe revert mode?
    }
    // Restore previous mode or go to Idle? Go to Idle is safer.
    currentMode = MODE_IDLE;
    // updateLedStatus will be called in loop when mode changes back from CLEARING_MEM
}


void startPlayback(OperatingMode playbackMode) {
    if (numRecorded == 0) {
        Serial.println("Cannot start playback: No steps recorded.");
        return;
    }
    // Allow starting playback from Idle, Pot, or Web modes
    if (currentMode != MODE_IDLE && currentMode != MODE_MANUAL_POT && currentMode != MODE_WEB_CONTROL) {
        Serial.print("Cannot start playback from current mode: ");
        Serial.println(currentMode);
        return;
    }

    // Ensure playback index is valid
    if (playbackIndex < 0 || playbackIndex >= numRecorded) {
        Serial.print("Playback index out of bounds (");
        Serial.print(playbackIndex);
        Serial.print("), resetting to 0.");
        playbackIndex = 0;
    }

    Serial.print("Starting playback mode ");
    Serial.print(playbackMode);
    Serial.print(" from step ");
    Serial.println(playbackIndex);

    // Load the first step's positions into the target
    loadStepFromEEPROM(playbackIndex, targetServoPos);
    currentMode = playbackMode;
    lastPlaybackActionTime = millis(); // Reset timer for step delay
    // updateLedStatus will be called in loop when mode changes
}

void handlePlayback() {
    // Check if the arm has reached the target position for the current step
    bool stepComplete = true;
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Use a slightly larger deadzone for completion check than for movement check
        if (abs(targetServoPos[i] - currentServoPos[i]) > SERVO_DEADZONE + 2) {
            stepComplete = false;
            break; // Exit check early if any servo is not in position
        }
    }

    if (stepComplete) {
        unsigned long now = millis();

        // Actions to take once the current step's movement is finished
        if (currentMode == MODE_PLAYBACK_MANUAL) {
            // Manual mode: Step complete, wait for next button press
            Serial.print("Manual playback step ");
            Serial.print(playbackIndex);
            Serial.println(" complete. Returning to Idle.");
            // Increment index for the *next* potential manual step
            playbackIndex = (playbackIndex + 1) % numRecorded;
            // Return to Idle mode to wait for the next command/button press
            currentMode = MODE_IDLE;
            // updateLedStatus will be called in loop
            return; // Exit playback handling for this loop iteration
        }
        else if (currentMode == MODE_PLAYBACK_SEMI_AUTO || currentMode == MODE_PLAYBACK_FULL_AUTO) {
            // Semi-Auto or Full-Auto: Check if delay has passed
            if (now - lastPlaybackActionTime >= PLAYBACK_STEP_DELAY) {
                Serial.print("Step ");
                Serial.print(playbackIndex);
                Serial.println(" complete. Moving to next step.");

                playbackIndex++; // Move to the next step index

                if (playbackIndex >= numRecorded) {
                    // Reached the end of the recorded sequence
                    if (currentMode == MODE_PLAYBACK_FULL_AUTO) {
                        Serial.println("Reached end of sequence, looping back to step 0 (Full Auto).");
                        playbackIndex = 0; // Loop back to the beginning
                    } else { // Semi-Auto mode
                        Serial.println("Reached end of sequence, stopping (Semi Auto).");
                        stopAllMovement(true); // Stop movement and return to Idle
                        return; // Exit playback handling
                    }
                }

                // Load the next step's positions
                Serial.print("Loading step ");
                Serial.println(playbackIndex);
                loadStepFromEEPROM(playbackIndex, targetServoPos);
                lastPlaybackActionTime = now; // Reset timer for the new step
            }
            // else: Delay hasn't passed yet, do nothing, wait for next loop iteration
        }
    }
    // else: Step not yet complete, applyServoSmoothing() will continue moving
}

// --- BUTTON LOGIC ---
void handleButtons() {
    // Update all button states using Bounce2 library
    clearMemoryButton.update();
    recordPlayButton.update();
    progStopButton.update();
    cycleStartButton.update();

    static unsigned long clearButtonPressTime = 0;
    unsigned long now = millis();

        // --- Clear memory button (Hold Logic) ---
    if (clearMemoryButton.fell()) { // Button just pressed
        clearButtonPressTime = now;
        Serial.println("Clear button pressed, hold for 1 sec...");
    }
    // Check if the button is currently held down (debounced state is LOW)
    if (clearMemoryButton.read() == LOW) { // <--- CORRECTED LINE
         // Check if the hold duration has been met since the initial press
         if (clearButtonPressTime > 0 && now - clearButtonPressTime >= CLEAR_HOLD_DURATION_MS) {
             // Check if already cleared or in incompatible mode
             if (currentMode < MODE_PLAYBACK_MANUAL && currentMode != MODE_CLEARING_MEM) {
                  Serial.println("Clear button held long enough. Clearing EEPROM.");
                  stopAllMovement(false); // Stop movement first
                  clearEEPROM(); // This function handles mode changes and commit
                  clearButtonPressTime = 0; // Prevent re-clearing if held longer by resetting timer
             } else if (clearButtonPressTime != 0) { // Only print warning once if hold started but mode is wrong
                 Serial.println("Cannot clear memory in current mode or while already clearing.");
                 clearButtonPressTime = 0; // Prevent spamming message by resetting timer
             }
         }
         // else: Button is down, but not long enough yet. Do nothing.
    }
    if (clearMemoryButton.rose()) { // Button just released
        if (clearButtonPressTime != 0 && now - clearButtonPressTime < CLEAR_HOLD_DURATION_MS) {
             // Optional feedback if released too soon
             Serial.println("Clear button released too soon.");
        }
        // Always reset the press timer on release, regardless of hold duration
        clearButtonPressTime = 0;
    }

    // --- RECORD BUTTON LOGIC (Simple Press) ---
    if (recordPlayButton.fell()) { // Record button just pressed
        if ((currentMode == MODE_MANUAL_POT || currentMode == MODE_WEB_CONTROL || currentMode == MODE_IDLE)) {
            if (numRecorded < MAX_RECORDINGS) {
                Serial.print("Record button pressed. Recording current position as step ");
                Serial.println(numRecorded);
                saveStepToEEPROM(numRecorded, currentServoPos);
                numRecorded++;
                EEPROM.write(EEPROM_NUM_RECORDED_ADDR, numRecorded);
                if(EEPROM.commit()) {
                     playbackIndex = numRecorded % MAX_RECORDINGS; // Update next playback index
                     // Flash LED briefly to confirm recording
                     uint32_t currentLedColor = ledStrip.getPixelColor(0);
                     ledStrip.setPixelColor(0, ledStrip.Color(255, 0, 255)); // Bright magenta flash
                     ledStrip.show();
                     delay(150); // Brief flash duration
                     ledStrip.setPixelColor(0, currentLedColor); // Restore previous color
                     ledStrip.show();
                     // updateLedStatus(); // Or call updateLedStatus to be sure
                } else {
                    Serial.println("EEPROM commit failed after recording!");
                    if (numRecorded > 0) numRecorded--; // Try rollback count
                }
            } else {
                Serial.println("Record button pressed, but Max recordings reached.");
                // Optional: Add a different LED flash for error/max reached?
            }
        } else {
             Serial.println("Record button pressed, but cannot record in current mode.");
        }
    }

    // --- PLAYBACK BUTTON LOGIC (Single/Double/Hold for Manual/Semi/Full Auto) ---
    if (cycleStartButton.fell()) { // Play button just pressed
        if (!runBtnWaitingSecond) { // This is the first press
            runBtnFirstPressTime = now;
            runBtnActionTaken = false; // Reset action flag for this sequence
            runBtnWaitingSecond = true; // Start the window for a potential second press
            Serial.println("Run button: First press detected.");
        } else { // This is the second press within the double-press interval
            if (now - runBtnFirstPressTime <= DOUBLE_PRESS_INTERVAL_MS) {
                runBtnSecondPressActive = true; // Mark that the second press is currently down
                runBtnSecondPressStart = now; // Record when the second press started (for hold detection)
                runBtnActionTaken = false; // Reset action flag (might have been set by timeout)
                Serial.println("Run button: Second press detected (within interval).");
            } else {
                 // Second press occurred, but *after* the double-press window. Ignore it for double-press logic.
                 // Reset state as if it was a new first press.
                 runBtnFirstPressTime = now;
                 runBtnActionTaken = false;
                 runBtnWaitingSecond = true;
                 runBtnSecondPressActive = false; // Not active second press
                 Serial.println("Run button: Second press detected (OUTSIDE interval - treating as new first press).");
            }
        }
    }

    if (cycleStartButton.rose()) { // Play button just released
        Serial.println("Run button: Released.");
        if (runBtnSecondPressActive) { // Was the second press of a double-press sequence active?
            unsigned long heldDuration = now - runBtnSecondPressStart;
             Serial.print("Run button: Second press released after holding for ");
             Serial.print(heldDuration);
             Serial.println(" ms.");
            if (!runBtnActionTaken) { // Only take action if none already taken for this sequence
                if (heldDuration >= FULL_AUTO_HOLD_MS) {
                    // Held long enough -> Full Auto Playback
                    Serial.println("Run button: Action -> Start Full-Auto Playback.");
                    startPlayback(MODE_PLAYBACK_FULL_AUTO); // Function handles checks
                    runBtnActionTaken = true;
                } else { // Released quickly after second press -> Semi Auto Playback
                    Serial.println("Run button: Action -> Start Semi-Auto Playback.");
                    startPlayback(MODE_PLAYBACK_SEMI_AUTO); // Function handles checks
                    runBtnActionTaken = true;
                }
            }
            // Reset double-press state machine
            runBtnWaitingSecond = false;
            runBtnSecondPressActive = false;
        } else if (runBtnWaitingSecond) { // Was this the release of the *first* press?
            runBtnFirstReleaseTime = now; // Record the time of the first release
            Serial.println("Run button: First press released.");
            // Don't take action yet, wait for timeout to confirm it's a single press
        }
    }

    // Timeout logic for single press detection (checked every loop)
    if (runBtnWaitingSecond && !runBtnActionTaken && !runBtnSecondPressActive) {
        // We are in the waiting window, no action taken yet, and second press is not active
        if (runBtnFirstReleaseTime > 0 && (now - runBtnFirstReleaseTime > DOUBLE_PRESS_INTERVAL_MS)) {
            // Enough time has passed since the first release without a second press -> Single Press Action
            Serial.println("Run button: Timeout expired. Action -> Start Manual Playback.");
            startPlayback(MODE_PLAYBACK_MANUAL); // Function handles checks
            runBtnActionTaken = true;
            // Reset state machine
            runBtnWaitingSecond = false;
            runBtnFirstReleaseTime = 0;
        }
    }


    // --- STOP BUTTON LOGIC (Single/Double Press for Stop/Reset) ---
    // Simplified: Any press (quick or double) stops and returns to Idle.
    if (progStopButton.fell()) { // Stop button just pressed
        if (!stopBtnWaitingSecond) { // First press
            stopBtnFirstPressTime = now;
            stopBtnActionTaken = false; // Reset flag for this sequence
            stopBtnWaitingSecond = true; // Start waiting window
            Serial.println("Stop button: First press detected.");
        } else { // Second press within interval
             if (now - stopBtnFirstPressTime <= STOP_DOUBLE_PRESS_INTERVAL_MS) {
                 // Double Press Action: Stop, Reset to Step 0, Go to Idle
                 Serial.println("Stop button: Second press detected (within interval). Action -> Stop & Reset.");
                 if (!stopBtnActionTaken) { // Check if action already taken by timeout
                     if (numRecorded > 0) { // Only reset if there are steps
                         loadStepFromEEPROM(0, targetServoPos);
                         for (int i = 0; i < NUM_SERVOS; i++) {
                             currentServoPos[i] = targetServoPos[i]; // Force position immediately
                             pwm.writeMicroseconds(i, currentServoPos[i]);
                         }
                         playbackIndex = 0;
                         Serial.println("Reset position and index to step 0.");
                     } else {
                         Serial.println("No steps recorded, just stopping.");
                     }
                     stopAllMovement(true); // Stop movement AND return to IDLE mode
                     stopBtnActionTaken = true; // Mark action as taken
                 }
                 stopBtnWaitingSecond = false; // End the waiting window
             } else {
                  // Second press occurred, but *after* the double-press window. Ignore it for double-press logic.
                  // Reset state as if it was a new first press.
                  stopBtnFirstPressTime = now;
                  stopBtnActionTaken = false;
                  stopBtnWaitingSecond = true;
                  Serial.println("Stop button: Second press detected (OUTSIDE interval - treating as new first press).");
             }
        }
    }
    if (progStopButton.rose()) { // Stop button just released
         Serial.println("Stop button: Released.");
        if (stopBtnWaitingSecond) { // Was waiting for potential second press?
            stopBtnFirstReleaseTime = now; // Record release time
        }
    }

    // Timeout logic for single press (checked every loop)
    if (stopBtnWaitingSecond && !stopBtnActionTaken) { // Still waiting AND no action taken yet?
        // Check if enough time has passed since the FIRST release
        if (stopBtnFirstReleaseTime > 0 && (now - stopBtnFirstReleaseTime > STOP_DOUBLE_PRESS_INTERVAL_MS)) {
            // Timeout expired -> Treat as Single Press Action: Stop, Go to Idle (No Reset)
            Serial.println("Stop button: Timeout expired. Action -> Stop.");
            stopAllMovement(true); // Stop movement AND return to IDLE mode (CHANGED TO TRUE FOR SIMPLICITY)
            stopBtnActionTaken = true; // Mark action as taken
            // Reset state machine
            stopBtnWaitingSecond = false;
            stopBtnFirstReleaseTime = 0;
        }
    }
}
// --- End of button logic ---


void updateLedStatus() {
    uint32_t color = ledStrip.Color(0, 0, 0); // Default off

    // --- DEBUG: Print current mode before switch ---
    Serial.printf("updateLedStatus called. Current Mode: %d - ", currentMode);

    switch (currentMode) {
        case MODE_IDLE:
            Serial.println("Setting LED: Idle (Blue)");
            color = ledStrip.Color(0, 0, 20); break;                // Blue
        case MODE_MANUAL_POT:
            Serial.println("Setting LED: Manual Pot (Cyan)");
            color = ledStrip.Color(0, 20, 20); break;         // Cyan
        case MODE_WEB_CONTROL:
             Serial.println("Setting LED: Web Control (Magenta)");
            color = ledStrip.Color(20, 0, 20); break;        // Magenta
        case MODE_PLAYBACK_MANUAL:
             Serial.println("Setting LED: Playback Manual (Orange)");
            color = ledStrip.Color(25, 10, 0); break;    // Orange
        case MODE_PLAYBACK_SEMI_AUTO:
             Serial.println("Setting LED: Playback Semi-Auto (Yellow)");
            color = ledStrip.Color(25, 25, 0); break; // Yellow
        case MODE_PLAYBACK_FULL_AUTO:
            // --- DEBUG: Confirming this case is hit ---
            Serial.println("Setting LED: Playback Full-Auto (Green)");
            color = ledStrip.Color(0, 25, 0); break;  // Green
        case MODE_CLEARING_MEM:
            Serial.println("Setting LED: Clearing Memory (Flashing Orange?)"); // Or solid bright color
            // Make it flash or use a distinct color like bright orange
            color = (millis() / 250) % 2 ? ledStrip.Color(255, 100, 0) : ledStrip.Color(50, 20, 0); // Flashing Orange
            break;
        case MODE_ERROR_WIFI:
        case MODE_ERROR_GENERIC:
            // --- DEBUG: Confirming this case is hit ---
            Serial.println("Setting LED: ERROR Mode (Red)");
            color = ledStrip.Color(255, 0, 0); break; // Red
        default:
             Serial.println("Setting LED: Unknown Mode (Off)");
             color = ledStrip.Color(0,0,0); // Off for unknown
             break;
    }
    ledStrip.setPixelColor(0, color);
    ledStrip.show();
}
// --- HTML Page Generation ---
String generatePageHTML() {
    // Use R"rawliteral(...)rawliteral" for multi-line strings without escaping
    return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Robot Arm Control</title>
    <style>
        body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;margin:0;padding:15px;background-color:#f0f2f5;color:#333}
        .container{max-width:900px;margin:auto;background-color:#fff;padding:20px;border-radius:8px;box-shadow:0 2px 10px rgba(0,0,0,.1)}
        h1,h2{text-align:center;color:#1a73e8}
        .section{margin-bottom:25px;padding:15px;border:1px solid #ddd;border-radius:5px;background-color:#fafafa}
        .section h2{margin-top:0;color:#333;font-size:1.2em;border-bottom:1px solid #eee;padding-bottom:5px;margin-bottom:15px}
        .status-bar{display:flex;flex-wrap:wrap;justify-content:space-around;background-color:#e8f0fe;padding:10px;border-radius:5px;margin-bottom:20px;font-weight:700;font-size:.9em}
        .status-bar div{margin:5px 10px}
        .servo-display{display:flex;align-items:center;margin-bottom:10px}
        .servo-display label{width:100px;font-size:.9em;flex-shrink:0;text-align:right;padding-right:10px}
        .servo-display input[type=range]{flex-grow:1;margin:0 10px;cursor:pointer}
        .servo-display input[type=range]:disabled{cursor:not-allowed}
        .servo-display .value{min-width:40px;text-align:right;font-family:monospace;font-weight:700}
        .servo-bar-container{flex-grow:1;height:18px;background-color:#e0e0e0;border-radius:3px;margin:0 10px;overflow:hidden;border:1px solid #ccc}
        .servo-bar{height:100%;background-color:#4285f4;width:50%;transition:width .1s linear}
        .buttons{text-align:center;margin-top:15px;display:flex;flex-wrap:wrap;justify-content:center;gap:10px}
        .button{background-color:#1a73e8;color:#fff;border:none;padding:10px 18px;border-radius:5px;cursor:pointer;font-size:.95em;transition:background-color .2s}
        .button:hover{background-color:#1558b0}
        .button.record{background-color:#34a853}.button.record:hover{background-color:#2a8c45}
        .button.clear{background-color:#ea4335}.button.clear:hover{background-color:#c73a2e}
        .button.playback{background-color:#fbbc05;color:#333}.button.playback:hover{background-color:#d6a104}
        .button.stop{background-color:#d93025}.button.stop:hover{background-color:#b0261e}
        .button.toggle{background-color:#9aa0a6}.button.toggle:hover{background-color:#7f858c}
        .button.jump{background-color:#ff6d00}.button.jump:hover{background-color:#e66000}
        .button:disabled{background-color:#ccc!important;color:#666!important;cursor:not-allowed!important}
        .steps-table{width:100%;border-collapse:collapse;margin-top:15px;font-size:.9em;table-layout:fixed}
        .steps-table th,.steps-table td{border:1px solid #ddd;padding:8px;text-align:center;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
        .steps-table th{background-color:#e8f0fe;font-weight:700}
        .steps-table th:first-child,.steps-table td:first-child{width:40px}
        .steps-table th:last-child,.steps-table td:last-child{width:180px}
        .steps-table td input[type=number]{width:90%;box-sizing:border-box;text-align:center;border:1px solid #ccc;border-radius:3px;padding:3px;font-size:1em}
        .steps-table .action-button{font-size:.8em;padding:3px 6px;margin:0 2px}
        .editable-row td{background-color:#fffacd}
        .highlight-row td{background-color:#d1eaff!important}
        #toast-notification{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background-color:rgba(0,0,0,.7);color:#fff;padding:10px 20px;border-radius:5px;font-size:.9em;z-index:1000;opacity:0;transition:opacity .5s ease-out}
        #toast-notification.show{opacity:1}
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Arm Control</h1>
        <div class="status-bar">
            <div>Status: <span id="status-mode">--</span></div>
            <div>Control: <span id="status-control">--</span></div>
            <div>Steps: <span id="status-steps">--</span></div>
            <div>Next: <span id="status-next-step">--</span></div>
            <div>IP: <span id="status-ip">--</span></div>
        </div>
        <div class="section servo-controls">
            <h2>Live Control & Visualization</h2>
            <div id="servo-display-area"></div>
            <div class="buttons">
                 <button id="toggle-button" class="button toggle" onclick="toggleWebMode()">Toggle Pot/Web</button>
                 <button id="stop-button" class="button stop" onclick="sendCommand('stop')">STOP Movement</button>
            </div>
        </div>
        <div class="section recording-playback">
            <h2>Recording & Playback</h2>
             <div class="buttons">
                <button id="record-button" class="button record" onclick="sendCommand('record')">Record Current Pos</button>
                <button id="clear-button" class="button clear" onclick="if(confirm('Clear All Steps?\\nHold button on ESP32 for 1 sec OR click OK to send clear command.')) sendCommand('clear');">Clear All (Web)</button>
                <button id="play-manual-button" class="button playback" onclick="sendCommand('play_manual')">Manual Step</button>
                <button id="play-semi-button" class="button playback" onclick="sendCommand('play_semi_auto')">Semi-Auto</button>
                <button id="play-full-button" class="button playback" onclick="sendCommand('play_full_auto')">Full Auto</button>
            </div>
        </div>
        <div class="section recorded-steps">
            <h2>Stored Steps (<span id="steps-count">0</span>)</h2>
            <table class="steps-table">
                <thead>
                    <tr>
                        <th>#</th><th>Base</th><th>Shoulder</th><th>Elbow</th><th>Wrist R</th><th>Wrist UD</th><th>Gripper</th><th>Actions</th>
                    </tr>
                </thead>
                <tbody id="steps-tbody"><tr><td colspan="8">Loading steps...</td></tr></tbody>
            </table>
        </div>
    </div>
    <div id="toast-notification"></div>

    <script>
        // --- JavaScript Constants ---
        const NUM_SERVOS = 6;
        const MIN_PULSE = 600;
        const MAX_PULSE = 2400;
        const MAX_RECORDINGS = 100; // <<<====== ADDED THIS LINE
        const servoNames = ['Base', 'Shoulder', 'Elbow', 'Wrist R', 'Wrist UD', 'Gripper'];
        // ---------------------------

        let currentServoValues = Array(NUM_SERVOS).fill(1500);
        let systemMode = 'Idle'; let editingStepIndex = -1; let recordedStepsCount = 0; let nextPlaybackIndex = 0;
        let statusIntervalId = null; let stepsIntervalId = null; let toastTimeoutId = null; let debounceTimeout = null;

        function showToast(message) {
            const toast = document.getElementById('toast-notification'); toast.textContent = message; toast.classList.add('show');
            if (toastTimeoutId) clearTimeout(toastTimeoutId);
            toastTimeoutId = setTimeout(() => { toast.classList.remove('show'); }, 3000);
        }

        function createServoDisplay() {
            const container = document.getElementById('servo-display-area'); container.innerHTML = '';
            for (let i = 0; i < NUM_SERVOS; i++) {
                const div = document.createElement('div'); div.className = 'servo-display';
                const label = document.createElement('label'); label.htmlFor = `servo${i}`; label.textContent = servoNames[i]; div.appendChild(label);
                const slider = document.createElement('input'); slider.type = 'range'; slider.id = `servo${i}`; slider.min = MIN_PULSE; slider.max = MAX_PULSE; slider.value = currentServoValues[i]; slider.step = 1;
                slider.oninput = () => updateServoValueDisplay(i, slider.value); slider.onchange = () => sendServoValue(i, slider.value); slider.disabled = true; div.appendChild(slider);
                const valueSpan = document.createElement('span'); valueSpan.className = 'value'; valueSpan.id = `servoValue${i}`; valueSpan.textContent = currentServoValues[i]; div.appendChild(valueSpan);
                const barContainer = document.createElement('div'); barContainer.className = 'servo-bar-container'; const bar = document.createElement('div'); bar.className = 'servo-bar'; bar.id = `servoBar${i}`; barContainer.appendChild(bar); div.appendChild(barContainer);
                container.appendChild(div); updateServoBar(i, currentServoValues[i]);
            }
        }

        function updateServoValueDisplay(index, value) { const valueSpan = document.getElementById(`servoValue${index}`); if (valueSpan) valueSpan.textContent = value; updateServoBar(index, value); }
        function updateServoBar(index, value) { const bar = document.getElementById(`servoBar${index}`); if (bar) { const percentage = ((value - MIN_PULSE) / (MAX_PULSE - MIN_PULSE)) * 100; bar.style.width = `${Math.max(0, Math.min(100, percentage))}%`; } }

        function sendServoValue(index, value) {
             if (systemMode !== 'Web Control') { console.warn("Not in Web Control mode."); return; }
             const intValue = parseInt(value); currentServoValues[index] = intValue; updateServoValueDisplay(index, intValue);
             clearTimeout(debounceTimeout);
             debounceTimeout = setTimeout(() => {
                 const dataToSend = { servos: [...currentServoValues] }; console.log("Sending servo values:", dataToSend);
                 fetch('/servo', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(dataToSend) })
                 .then(response => { if (!response.ok) { console.error('Error sending servo values:', response.statusText); showToast(`Error: ${response.statusText}`); } })
                 .catch(error => { console.error('Network error sending servo values:', error); showToast('Network Error updating servos.'); });
             }, 50);
        }

        function sendCommand(commandName, params = {}) {
            const body = { command: commandName, ...params }; console.log("Sending command:", body);
            fetch('/command', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
            .then(response => response.json().then(data => ({ status: response.status, body: data })))
            .then(({ status, body }) => {
                 console.log("Command response:", status, body); showToast(body.message || (status === 200 ? 'OK' : 'Error'));
                 updateStatus(); // Refresh status immediately
                 if (['record', 'clear', 'delete_step'].includes(commandName)) { fetchSteps(); } // Refresh steps if needed
            })
            .catch(error => { console.error('Error sending command:', error); showToast(`Network Error: ${commandName}.`); });
        }

        function toggleWebMode() { sendCommand('toggle_mode'); }
        function jumpToStep(index) { if (isNaN(index) || index < 0 || (recordedStepsCount > 0 && index >= recordedStepsCount)) { alert(`Invalid index (0-${Math.max(0, recordedStepsCount - 1)}).`); return; } sendCommand('jump_to_step', { step_index: index }); }

        function updateStatus() {
            fetch('/status')
                .then(response => { if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`); return response.json(); })
                .then(data => {
                    const previousSystemMode = systemMode; systemMode = data.mode_str || 'Unknown';
                    recordedStepsCount = data.steps_recorded || 0;
                    const previousNextPlaybackIndex = nextPlaybackIndex; nextPlaybackIndex = data.next_playback_index !== undefined ? data.next_playback_index : 0;

                    document.getElementById('status-mode').textContent = systemMode; document.getElementById('status-steps').textContent = recordedStepsCount;
                    document.getElementById('status-next-step').textContent = nextPlaybackIndex; document.getElementById('status-ip').textContent = data.ip_address || 'N/A';

                    // --- Determine Control State & Slider Enablement ---
                    let displayControlMode = "System"; let slidersShouldBeEnabled = false;
                    if (systemMode === 'Manual (Pot)') { displayControlMode = "Potentiometer"; slidersShouldBeEnabled = false; }
                    else if (systemMode === 'Web Control') { displayControlMode = "Web"; slidersShouldBeEnabled = true; } // Enable sliders ONLY for Web Control
                    else if (systemMode.startsWith('Playback')) { displayControlMode = "Playback"; slidersShouldBeEnabled = false; }
                    else if (systemMode === 'Idle') { displayControlMode = "Idle"; slidersShouldBeEnabled = false; } // Show Idle correctly
                    else if (systemMode.includes('Error')) { displayControlMode = "ERROR"; slidersShouldBeEnabled = false; }
                    else if (systemMode === 'Clearing Memory') { displayControlMode = "Clearing"; slidersShouldBeEnabled = false; }
                    document.getElementById('status-control').textContent = displayControlMode;

                    // --- Update Servo Sliders ---
                    document.querySelectorAll('.servo-controls input[type=range]').forEach((s, i) => {
                         s.disabled = !slidersShouldBeEnabled; // Set disabled state
                         if (!slidersShouldBeEnabled || previousSystemMode === 'Web Control') { // Update value if not in web mode OR just left web mode
                             const currentPos = data.current_pos[i];
                             if (s.value != currentPos) { s.value = currentPos; updateServoValueDisplay(i, currentPos); }
                         }
                         // Always sync local JS array with ESP32's reported state
                         currentServoValues[i] = data.current_pos[i];
                    });

                    // --- Update Button Enabled States ---
                    const canPlayback = (systemMode === 'Idle' || systemMode === 'Manual (Pot)' || systemMode === 'Web Control') && recordedStepsCount > 0;
                    document.getElementById('play-manual-button').disabled = !canPlayback; document.getElementById('play-semi-button').disabled = !canPlayback; document.getElementById('play-full-button').disabled = !canPlayback;
                    // Use the JavaScript MAX_RECORDINGS constant here
                    const canRecord = (systemMode === 'Manual (Pot)' || systemMode === 'Web Control' || systemMode === 'Idle') && recordedStepsCount < MAX_RECORDINGS; // <<< THIS LINE NOW WORKS
                    document.getElementById('record-button').disabled = !canRecord;
                    const canToggle = !(systemMode.startsWith('Playback') || editingStepIndex !== -1 || systemMode.includes('Error') || systemMode === 'Clearing Memory');
                    document.getElementById('toggle-button').disabled = !canToggle;
                    document.getElementById('stop-button').disabled = systemMode.includes('Error');
                    const canClear = !(systemMode.startsWith('Playback') || systemMode === 'Clearing Memory' || editingStepIndex !== -1 || systemMode.includes('Error'));
                    document.getElementById('clear-button').disabled = !canClear;

                    // --- Highlight Next Step Row ---
                    if (previousNextPlaybackIndex !== nextPlaybackIndex || systemMode !== previousSystemMode) {
                         highlightStepRow(nextPlaybackIndex, (systemMode === 'Idle' || systemMode === 'Manual (Pot)' || systemMode === 'Web Control') && recordedStepsCount > 0);
                    }
                })
                .catch(error => {
                    // Log the error that caused the catch block
                    console.error('Error fetching/processing status:', error); // <<< MODIFIED for better debugging
                    document.getElementById('status-mode').textContent = 'Error'; document.getElementById('status-control').textContent = 'N/A';
                    document.querySelectorAll('.button').forEach(b => b.disabled = true); document.querySelectorAll('input[type=range]').forEach(s => s.disabled = true);
                 });
        }

        function highlightStepRow(index, shouldHighlight) {
             document.querySelectorAll('#steps-tbody tr').forEach(r => r.classList.remove('highlight-row'));
             if (shouldHighlight) { const row = document.getElementById(`step-row-${index}`); if (row) row.classList.add('highlight-row'); }
        }

        function fetchSteps() {
            fetch('/steps')
                .then(response => { if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`); return response.json(); })
                .then(data => {
                    const tbody = document.getElementById('steps-tbody'); tbody.innerHTML = ''; const stepsData = data.steps || []; const currentStepsCount = stepsData.length;
                    document.getElementById('steps-count').textContent = currentStepsCount;
                    if (currentStepsCount === 0) { tbody.innerHTML = '<tr><td colspan="8">No steps recorded.</td></tr>'; return; }
                    stepsData.forEach((stepPositions, index) => {
                        const row = tbody.insertRow(); row.id = `step-row-${index}`;
                        row.insertCell().textContent = index;
                        stepPositions.forEach(pos => { row.insertCell().textContent = pos; });
                        // Shortened button text for better fit
                        row.insertCell().innerHTML = `<button class="button action-button" onclick="editStep(${index})" title="Edit">E</button> <button class="button action-button jump" onclick="jumpToStep(${index})" title="Jump">J</button> <button class="button action-button clear" onclick="deleteStep(${index})" title="Delete">D</button>`;
                    });
                    highlightStepRow(nextPlaybackIndex, (systemMode === 'Idle' || systemMode === 'Manual (Pot)' || systemMode === 'Web Control') && recordedStepsCount > 0); // Re-apply highlight
                })
                .catch(error => { console.error('Error fetching steps:', error); document.getElementById('steps-tbody').innerHTML = '<tr><td colspan="8">Error loading steps.</td></tr>'; });
        }

        function editStep(index) {
            if (editingStepIndex !== -1 && editingStepIndex !== index) { cancelEditStep(editingStepIndex); } if (editingStepIndex === index) return;
            editingStepIndex = index; const row = document.getElementById(`step-row-${index}`); if (!row) return; row.classList.add('editable-row');
            const cells = row.cells; let originalValues = [];
            for (let i = 1; i < cells.length - 1; i++) { const currentVal = cells[i].textContent; originalValues.push(currentVal); cells[i].innerHTML = `<input type="number" id="edit-servo-${index}-${i-1}" value="${currentVal}" min="${MIN_PULSE}" max="${MAX_PULSE}" style="width:60px;">`; }
            row.dataset.originalValues = JSON.stringify(originalValues);
            // Shortened button text
            cells[cells.length - 1].innerHTML = `<button class="button action-button record" onclick="saveEditedStep(${index})">S</button> <button class="button action-button clear" onclick="cancelEditStep(${index})">C</button>`;
            updateStatus(); // Disable incompatible buttons
        }

        function saveEditedStep(index) {
            const row = document.getElementById(`step-row-${index}`); if (!row) return; const inputs = row.querySelectorAll('input[type=number]'); let newPositions = []; let isValid = true;
            inputs.forEach(input => { const val = parseInt(input.value); if (isNaN(val) || val < MIN_PULSE || val > MAX_PULSE) { isValid = false; input.style.border = '1px solid red'; } else { input.style.border = ''; newPositions.push(val); } });
            if (!isValid) { showToast(`Invalid value ( ${MIN_PULSE}-${MAX_PULSE} ).`); return; }
            console.log("Saving edited step:", index, newPositions);
            fetch('/step', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ index: index, positions: newPositions }) })
            .then(response => response.json().then(data => ({ status: response.status, body: data })))
            .then(({ status, body }) => {
                showToast(body.message || (status === 200 ? 'Save OK' : 'Save Error'));
                if(status === 200) { restoreRowDisplay(index, newPositions); editingStepIndex = -1; fetchSteps(); } // Success: restore display, exit edit mode, refresh steps
                else { console.error('Error saving step:', body.message); cancelEditStep(index); } // Error: cancel edit
            })
            .catch(error => { console.error('Network error saving step:', error); showToast('Network Error saving.'); cancelEditStep(index); })
            .finally(() => { updateStatus(); }); // Ensure UI state is correct
        }

         function cancelEditStep(index) {
             const row = document.getElementById(`step-row-${index}`); if (!row || editingStepIndex !== index) return; console.log("Cancelling edit for step:", index);
             let originalPositions = []; try { if (row.dataset.originalValues) { originalPositions = JSON.parse(row.dataset.originalValues); } } catch (e) { console.error("Error parsing original values", e); }
             restoreRowDisplay(index, originalPositions); editingStepIndex = -1; updateStatus();
         }

         function restoreRowDisplay(index, positions) {
             const row = document.getElementById(`step-row-${index}`); if (!row) return; row.classList.remove('editable-row'); row.style.backgroundColor = ''; row.removeAttribute('data-original-values');
             const posArray = Array.isArray(positions) ? positions : []; let cellsHTML = `<td>${index}</td>`;
             for(let i=0; i<NUM_SERVOS; ++i) { cellsHTML += `<td>${posArray[i] !== undefined ? posArray[i] : '?'}</td>`; }
             // Shortened button text
             cellsHTML += `<td><button class="button action-button" onclick="editStep(${index})" title="Edit">E</button> <button class="button action-button jump" onclick="jumpToStep(${index})" title="Jump">J</button> <button class="button action-button clear" onclick="deleteStep(${index})" title="Delete">D</button></td>`;
             row.innerHTML = cellsHTML;
         }

         function deleteStep(index) { if (!confirm(`Delete step ${index}?`)) return; sendCommand('delete_step', { step_index: index }); }

        // --- Initialization ---
        window.onload = () => {
            console.log("Window loaded. Initializing UI."); createServoDisplay(); updateStatus(); fetchSteps();
            if (statusIntervalId) clearInterval(statusIntervalId); if (stepsIntervalId) clearInterval(stepsIntervalId); // Clear previous intervals if any
            statusIntervalId = setInterval(updateStatus, 2000); // Poll status every 2s
            stepsIntervalId = setInterval(() => { if (editingStepIndex === -1) fetchSteps(); }, 5000); // Poll steps every 5s (if not editing)
        };
    </script>
</body>
</html>
)rawliteral";
}


