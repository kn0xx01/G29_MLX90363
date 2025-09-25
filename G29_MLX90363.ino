// G29_MLX90363.ino
// Complete optimized version for G29 with mechanical stops
// Stable, fast, and reliable - no rotation counting needed

#include "MLX90363.h"
#include "Joystick.h"

// MLX90363 pins
const byte MLX90363_CS_PIN = 10;
MLX90363 mlx90363(SPI, MLX90363_CS_PIN);
const int STATUS_LED = 13;

// G29 SPECIFIC PARAMETERS
const float GEAR_RATIO = 8.17;
const float MAX_STEERING_ANGLE = 450.0; // G29 physical limit
const float MIN_STEERING_ANGLE = -450.0;
const float TOTAL_RANGE = 900.0; // 450° each way

// Relative tracking with G29 constraints
uint16_t lastRawLSB = 0;
float currentWheelAngle = 0.0; // In WHEEL degrees (-450 to +450)
bool needsCentering = true;

// Optimized filtering for gaming
const float DEADZONE = 1.5; // Smaller deadzone for responsive feel
const float SMOOTHING = 0.8; // Less smoothing for better response
const float MOVEMENT_THRESHOLD = 0.5; // Filter tiny movements (noise)

// Joystick parameters
const uint16_t CENTER_STEERING = 32767;
const uint16_t MAX_STEERING_VALUE = 65535;

// Create Joystick object
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_MULTI_AXIS, 1, 0, false, false, false, false, false, false, false, false, true, true);

// Data logging
bool loggingEnabled = false;
unsigned long logStartTime = 0;
const unsigned long LOG_INTERVAL = 100;

// ========== SENSOR FUNCTIONS ==========
float calculateAngleChange(uint16_t previous, uint16_t current) {
  int16_t diff = (int16_t)current - (int16_t)previous;
  
  // Handle 16384 wrap-around (360° point)
  if (diff > 8192) diff -= 16384;
  else if (diff < -8192) diff += 16384;
  
  // Convert sensor degrees to wheel degrees using gear ratio
  return (diff * (360.0 / 16384.0)) / GEAR_RATIO;
}

bool readSensor() {
  if (!mlx90363.sendGet1Alpha()) {
    return false;
  }
  
  uint16_t currentRawLSB = mlx90363.receivedGetAlphaRaw();
  
  if (needsCentering) {
    // First reading - set initial position
    lastRawLSB = currentRawLSB;
    currentWheelAngle = 0.0;
    needsCentering = false;
    Serial.println("SENSOR: Initialized and centered");
    return true;
  }
  
  float angleChange = calculateAngleChange(lastRawLSB, currentRawLSB);
  lastRawLSB = currentRawLSB;
  
  // Filter out tiny movements (noise)
  if (abs(angleChange) < MOVEMENT_THRESHOLD) {
    return true;
  }
  
  // Apply smoothing
  angleChange = angleChange * SMOOTHING;
  
  // Update wheel angle with G29 constraints
  currentWheelAngle += angleChange;
  currentWheelAngle = constrain(currentWheelAngle, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
  
  return true;
}

// ========== JOYSTICK FUNCTIONS ==========
void updateJoystickPosition() {
  // Apply small deadzone around center
  if (abs(currentWheelAngle) < DEADZONE) {
    Joystick.setSteering(CENTER_STEERING);
  } else {
    // Map -450° to +450° to 0-65535
    uint16_t pos = map(currentWheelAngle, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE, 0, 65535);
    Joystick.setSteering(pos);
  }
}

void sendJoystickData() {
  updateJoystickPosition();
  Joystick.setAccelerator(0);
  Joystick.setBrake(0);
  Joystick.sendState();
}

// ========== CALIBRATION FUNCTIONS ==========
void calibrateCenter() {
  currentWheelAngle = 0.0;
  needsCentering = false;
  
  // Re-initialize with current sensor reading
  if (mlx90363.sendGet1Alpha()) {
    lastRawLSB = mlx90363.receivedGetAlphaRaw();
  }
  
  Serial.print("CALIBRATION:CENTER:");
  Serial.println(currentWheelAngle, 4);
}

void resetSteering() {
  currentWheelAngle = 0.0;
  needsCentering = true;
  Serial.println("RESET: Steering reset to center");
}

// ========== AUTO-RECENTER DETECTION ==========
void checkAutoRecenter() {
  // If we're near the physical limits, we know we're at ±450°
  if (abs(currentWheelAngle) >= 445.0) { // 5° before actual stop for safety
    Serial.print("AUTO_RECENTER: Near mechanical stop at ");
    Serial.println(currentWheelAngle, 1);
    
    // Optional: Could add automatic re-centering logic here
    // calibrateCenter(); // Uncomment if you want auto-recenter at stops
  }
}

// ========== DATA LOGGING FUNCTIONS ==========
void sendSensorData() {
  uint16_t currentRawLSB = mlx90363.receivedGetAlphaRaw();
  
  Serial.print("DATA:TIME:");
  Serial.print(millis());
  Serial.print(":RAW_LSB:");
  Serial.print(currentRawLSB);
  Serial.print(":WHEEL_ANGLE:");
  Serial.print(currentWheelAngle, 4);
  Serial.print(":JOYSTICK_POS:");
  
  // Calculate what joystick position would be
  if (abs(currentWheelAngle) < DEADZONE) {
    Serial.println(CENTER_STEERING);
  } else {
    uint16_t pos = map(currentWheelAngle, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE, 0, 65535);
    Serial.println(pos);
  }
}

void sendInfo() {
  Serial.print("INFO:WHEEL_ANGLE:");
  Serial.print(currentWheelAngle, 4);
  Serial.print(":SENSOR_RAW:");
  Serial.print(lastRawLSB);
  Serial.print(":NEEDS_CENTERING:");
  Serial.println(needsCentering ? "YES" : "NO");
}

// ========== COMMAND PROCESSING ==========
void processCommand(String command) {
  command.trim();
  
  // Filter out game commands
  if (command.length() == 0 || command.length() > 20) {
    Serial.println("IGNORED: Empty or too long");
    return;
  }
  
  // Convert to uppercase for consistent comparison
  command.toUpperCase();
  
  Serial.print("CMD: ");
  Serial.println(command);
  
  if (command == "GET_DATA") {
    sendSensorData();
  }
  else if (command == "CALIBRATE_CENTER" || command == "CALIBRATE") {
    calibrateCenter();
  }
  else if (command == "GET_INFO") {
    sendInfo();
  }
  else if (command == "RESET_STEERING" || command == "RESET") {
    resetSteering();
  }
  else if (command == "START_LOGGING") {
    loggingEnabled = true;
    logStartTime = millis();
    Serial.println("LOGGING:START");
  }
  else if (command == "STOP_LOGGING") {
    loggingEnabled = false;
    Serial.println("LOGGING:STOP");
  }
  else if (command == "HELP") {
    Serial.println("=== G29 OPTIMIZED COMMANDS ===");
    Serial.println("GET_DATA - Current sensor readings");
    Serial.println("CALIBRATE_CENTER - Set center position");
    Serial.println("GET_INFO - System information");
    Serial.println("RESET_STEERING - Reset steering to center");
    Serial.println("START_LOGGING - Start data logging");
    Serial.println("STOP_LOGGING - Stop data logging");
    Serial.println("HELP - This message");
  }
  else {
    Serial.println("IGNORED: Unknown command");
  }
}

// ========== ERROR HANDLING ==========
void indicateError() {
  pinMode(STATUS_LED, OUTPUT);
  while(1) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
}

// ========== ARDUINO STANDARD FUNCTIONS ==========
void setup() {
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  // Initialize joystick FIRST
  Joystick.setSteeringRange(0, 65535);
  Joystick.setAcceleratorRange(0, 65535);
  Joystick.setBrakeRange(0, 65535);
  Joystick.begin();
  
  // Initialize sensor
  SPI.begin();
  if (!mlx90363.begin()) {
    Serial.println("ERROR: MLX90363 initialization failed!");
    indicateError();
  }
  
  // Initialize serial
  Serial.begin(115200);
  delay(2000); // Wait for serial stabilization
  
  // Get initial sensor reading
  if (mlx90363.sendGet1Alpha()) {
    lastRawLSB = mlx90363.receivedGetAlphaRaw();
    Serial.print("SENSOR: Initial reading: ");
    Serial.println(lastRawLSB);
  }
  
  // Initial calibration
  calibrateCenter();
  
  digitalWrite(STATUS_LED, HIGH); // Ready indicator
  
  Serial.println("=== G29 OPTIMIZED CONTROLLER ===");
  Serial.println("INFO:READY - Relative Movement Tracking Active");
  Serial.println("INFO:GEAR_RATIO: 8.17:1");
  Serial.println("INFO:MAX_ANGLE: ±450° (G29 Mechanical Limits)");
  Serial.println("INFO:No rotation counting - optimized for stability");
  Serial.println("INFO:Send HELP for commands");
  Serial.println("================================");
}

void loop() {
  // Read sensor and update position
  if (readSensor()) {
    checkAutoRecenter();
    sendJoystickData();
    
    // Log data if enabled
    if (loggingEnabled && (millis() - logStartTime >= LOG_INTERVAL)) {
      sendSensorData();
      logStartTime = millis();
    }
  } else {
    Serial.println("ERROR: Sensor read failed!");
    delay(100); // Longer delay on error
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  delay(5); // Optimized delay for fast response
}