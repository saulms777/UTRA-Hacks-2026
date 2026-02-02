/*
 * Autonomous Line Following Robot with Obstacle Avoidance
 * 
 * Features:
 * - Line following using color sensor
 * - Ultrasonic obstacle detection and avoidance
 * - Adaptive search pattern when line is lost
 * - Individual motor calibration for straight driving
 * 
 * Pin Configuration:
 * Motor Driver (L298N):
 *   ENA (PWM) - Pin 2  (Left motor speed)
 *   IN1       - Pin 4  (Left motor direction)
 *   IN2       - Pin 5  (Left motor direction)
 *   ENB (PWM) - Pin 3  (Right motor speed)
 *   IN3       - Pin 6  (Right motor direction)
 *   IN4       - Pin 7  (Right motor direction)
 * 
 * Color Sensor (TCS3200):
 *   S2        - Pin 10 (Filter selection)
 *   S3        - Pin 11 (Filter selection)
 *   OUT       - Pin 8  (Frequency output)
 * 
 * Ultrasonic Sensor (HC-SR04):
 *   TRIG      - Pin 12 (Trigger pulse)
 *   ECHO      - Pin 13 (Echo pulse)
 */

// ============================================
// PIN MAPPING
// ============================================

// Motor driver pins
const int ENA = 2;      // Left motor PWM speed control
const int IN1 = 4;      // Left motor direction control 1
const int IN2 = 5;      // Left motor direction control 2
const int ENB = 3;      // Right motor PWM speed control
const int IN3 = 6;      // Right motor direction control 1
const int IN4 = 7;      // Right motor direction control 2

// Color sensor pins
const int S2 = 10;      // Color filter selection pin 2
const int S3 = 11;      // Color filter selection pin 3
const int colorOut = 8; // Frequency output pin

// Ultrasonic sensor pins
const int trigPin = 12; // Trigger pulse output
const int echoPin = 13; // Echo pulse input

// ============================================
// ROBOT SETTINGS
// ============================================

// Movement speeds
int driveSpeed = 80;              // Normal line following speed (0-255)
int searchSpeed = 80;             // Speed during search maneuvers (0-255)

// Search algorithm parameters
int initialSweepTime = 150;       // Initial search angle duration (ms) ~15 degrees
int maxSweepTime = 1000;          // Maximum search angle duration (ms) ~100 degrees
int sweepIncrement = 100;         // Angle increase per failure (ms) ~10 degrees
int overrotatePercent = 80;       // Overrotation percentage when path is found

// Search failure thresholds
int backupThreshold = 3;          // Failed searches before backing up

// Timing parameters
int turn90Time = 1500;            // Time for 90-degree turn (ms)
int passTime = 3000;              // Time to drive past obstacles (ms)
int backUpTime = 600;             // Time for small backward movement (ms)

// Obstacle avoidance settings
int avoidBackupTime = 300;        // Backup duration when obstacle detected (ms)
int avoidTurnTime = 500;          // Time for 90-degree avoidance turns (ms)
int avoidForwardTime1 = 1600;     // Time to clear obstacle side (ms)
int avoidForwardTime2 = 2000;     // Time to pass obstacle (ms)
int avoidForwardSpeed = 80;       // Forward speed during avoidance (0-255)
int avoidBackupSpeed = 80;        // Backup speed during avoidance (0-255)
int avoidTurnSpeed = searchSpeed; // Turning speed during avoidance (0-255)
int avoidSearchTimeout = 50000;   // Time to search for path after avoidance (ms)

// ============================================
// MOTOR CALIBRATION
// ============================================

/*
 * Calibration factors compensate for motor variations and ensure straight driving.
 * 
 * leftMotorFactor:  Multiply left motor speed by this value (0.5-1.5)
 * rightMotorFactor: Multiply right motor speed by this value (0.5-1.5)
 * leftMotorOffset:  Add this value to left motor speed (-50 to 50)
 * rightMotorOffset: Add this value to right motor speed (-50 to 50)
 * 
 * Typical adjustments:
 * - If robot drifts left: Increase rightMotorFactor or rightMotorOffset
 * - If robot drifts right: Increase leftMotorFactor or leftMotorOffset
 * - If turning is uneven: Adjust the slower motor's factor
 */
float leftMotorFactor = 1.0;   // Left motor speed multiplier
float rightMotorFactor = 1.0;  // Right motor speed multiplier
int leftMotorOffset = 0;       // Left motor speed offset
int rightMotorOffset = 0;      // Right motor speed offset

// ============================================
// SEARCH STATE VARIABLES
// ============================================

int currentSweepTime = initialSweepTime;  // Current search angle duration
int searchDirection = 1;                  // 1 = right, -1 = left
bool searchPhase = false;                 // True when actively searching
int consecutiveFailures = 0;              // Count of failed searches

// ============================================
// SETUP FUNCTION
// ============================================

void setup() {
  // Configure ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Configure motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configure color sensor pins
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Check for obstacles using ultrasonic sensor
  long distance = getDistance();
  
  // If obstacle detected within 15cm, initiate avoidance maneuver
  if (distance > 0 && distance <= 20) {
    resetSearchState();
    avoidObstacle();
  } 
  else {
    // No obstacle detected, perform normal line following
    String color = getPathColor();
    
    if (color != "OFF-PATH") {
      // On path: drive forward and reset search state
      resetSearchState();
      driveForward(driveSpeed);
    } else {
      // Off path: initiate adaptive search
      if (!searchPhase) {
        // First time losing path - initialize search
        searchPhase = true;
        consecutiveFailures = 0;
        currentSweepTime = initialSweepTime;
      }
      performAdaptiveSearch();
    }
  }
}

// ============================================
// SEARCH STATE MANAGEMENT
// ============================================

/**
 * Resets search algorithm parameters to initial state
 * Called when path is found or after completing avoidance
 */
void resetSearchState() {
  if (searchPhase) {
    searchPhase = false;
  }
  consecutiveFailures = 0;
  currentSweepTime = initialSweepTime;
  searchDirection = 1;
}

// ============================================
// ADAPTIVE SEARCH ALGORITHM
// ============================================

/**
 * Main adaptive search algorithm
 * - Starts with small search angles
 * - Increases angle after each failure
 * - Alternates search direction
 * - Performs full circle scan at maximum search angle
 */
void performAdaptiveSearch() {
  // Attempt search with current parameters
  bool found = searchWithOverturn(currentSweepTime, searchDirection);
  
  if (found) {
    // Path found - reset for next time
    resetSearchState();
  } else {
    // Search failed - update parameters
    consecutiveFailures++;
    
    // Increase search angle, capped at maximum
    currentSweepTime = min(initialSweepTime + (consecutiveFailures * sweepIncrement), maxSweepTime);
    
    // Alternate search direction
    searchDirection = -searchDirection;
  }
}

// ============================================
// SEARCH WITH OVERTURN
// ============================================

/**
 * Performs a search turn with optional backup and overrotation
 * 
 * @param sweepDuration Time to turn while searching (ms)
 * @param direction Turn direction (1 = right, -1 = left)
 * @return True if path found, false otherwise
 */
bool searchWithOverturn(int sweepDuration, int direction) {
  // Backup before turning if enough failures have occurred
  if (consecutiveFailures >= backupThreshold) {
    driveBackward(driveSpeed);
    delay(backUpTime);
    stopMotors();
  }
  
  // Begin turning in specified direction
  unsigned long turnStartTime = millis();
  bool pathFound = false;
  unsigned long foundTime = 0;
  
  if (direction == 1) {
    turnRight(searchSpeed);
  } else {
    turnLeft(searchSpeed);
  }
  
  // Monitor for path detection during turn
  while (millis() - turnStartTime < sweepDuration) {
    if (getPathColor() != "OFF-PATH") {
      pathFound = true;
      foundTime = millis() - turnStartTime;
      break;
    }
  }
  
  if (pathFound) {
    // Apply overrotation to ensure crossing the line
    int remainingTime = sweepDuration - foundTime;
    int overrotateTime = remainingTime * overrotatePercent / 100;
    
    // Continue turning for overrotation period
    unsigned long overrotateStart = millis();
    while (millis() - overrotateStart < overrotateTime) {
      // Empty loop - motors continue turning
    }
    
    stopMotors();
    return true;
  }
  
  // Path not found during search
  stopMotors();
  return false;
}

// ============================================
// OBSTACLE AVOIDANCE MANEUVER
// ============================================

/**
 * Standard obstacle avoidance procedure
 * 
 * Procedure:
 * 1. Backup from obstacle
 * 2. Turn right 90°
 * 3. Move forward to clear obstacle side
 * 4. Turn left 90° (now parallel to path)
 * 5. Move forward to pass obstacle
 * 6. Turn left 90° (heading back toward path)
 * 7. Backup slightly
 * 8. Drive forward while searching for path
 */
void avoidObstacle() {
  stopMotors();
  
  // Step 1: Backup from obstacle
  driveBackward(avoidBackupSpeed);
  delay(avoidBackupTime);
  stopMotors();
  
  // Step 2: Turn right to move around obstacle
  turnRight(avoidTurnSpeed);
  delay(avoidTurnTime);
  stopMotors();
  
  // Step 3: Clear the side of the obstacle
  driveForward(avoidForwardSpeed);
  delay(avoidForwardTime1);
  stopMotors();
  
  // Step 4: Turn parallel to original path
  turnLeft(avoidTurnSpeed);
  delay(avoidTurnTime * 1.3);
  stopMotors();
  
  // Step 5: Pass the obstacle
  driveForward(avoidForwardSpeed);
  delay(avoidForwardTime2);
  stopMotors();
  
  // Step 6: Turn back toward original path
  turnLeft(avoidTurnSpeed);
  delay(avoidTurnTime * 1.5);
  stopMotors();
  
  // Step 7: Backup slightly before search
  driveBackward(avoidBackupSpeed);
  delay(avoidBackupTime / 2);
  stopMotors();
  
  // Step 8: Search for path
  driveForward(avoidForwardSpeed);
  
  unsigned long searchStart = millis();
  bool pathFound = false;
  
  while (millis() - searchStart < avoidSearchTimeout) {
    if (getPathColor() != "OFF-PATH") {
      pathFound = true;
      break;
    }
  }
  
  stopMotors();
}

// ============================================
// ULTRASONIC DISTANCE SENSING
// ============================================

/**
 * Measures distance using HC-SR04 ultrasonic sensor
 * 
 * @return Distance in centimeters, 0 if out of range
 */
long getDistance() {
  // Send 10μs trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure echo pulse duration
  long duration = pulseIn(echoPin, HIGH, 25000);
  
  // Convert duration to distance in centimeters
  // Speed of sound = 340 m/s = 0.034 cm/μs
  // Distance = (time * speed) / 2 (round trip)
  long cm = duration * 0.034 / 2;
  
  return cm;
}

// ============================================
// COLOR SENSING FUNCTIONS
// ============================================

/**
 * Determines current path color using color sensor
 * 
 * @return String indicating detected color:
 *         "GREEN", "RED", "BLACK", or "OFF-PATH"
 */
String getPathColor() {
  // Read red and green frequency values
  int red = getFreq(LOW, LOW);    // S2=LOW, S3=LOW for red filter
  int green = getFreq(HIGH, HIGH);// S2=HIGH, S3=HIGH for green filter
  
  // Determine color based on frequency values
  if (red <= 35 && green <= 35) return "OFF-PATH";  // No line detected
  if (green < 60 && green < red) return "GREEN";    // Green line
  if (red < 150 && red < green) return "RED";       // Red line
  if (red > 150 || green > 150) return "BLACK";     // Black line
  
  return "OFF-PATH";  // Default to off-path
}

/**
 * Reads frequency from color sensor with specified filter settings
 * 
 * @param s2 S2 pin state for filter selection
 * @param s3 S3 pin state for filter selection
 * @return Frequency measurement (lower = more color absorption)
 */
int getFreq(int s2, int s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  return pulseIn(colorOut, LOW);
}

// ============================================
// MOTOR CONTROL FUNCTIONS
// ============================================

/**
 * Drives both motors forward with calibrated speeds
 * 
 * @param speed Base speed (0-255)
 */
void driveForward(int speed) {
  // Apply calibration factors and constraints
  int leftSpeed = constrain((speed * leftMotorFactor) + leftMotorOffset, 0, 255);
  int rightSpeed = constrain((speed * rightMotorFactor) + rightMotorOffset, 0, 255);
  
  // Set left motor forward
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Set right motor forward
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

/**
 * Drives both motors backward with calibrated speeds
 * 
 * @param speed Base speed (0-255)
 */
void driveBackward(int speed) {
  // Apply calibration factors and constraints
  int leftSpeed = constrain((speed * leftMotorFactor) + leftMotorOffset, 0, 255);
  int rightSpeed = constrain((speed * rightMotorFactor) + rightMotorOffset, 0, 255);
  
  // Set left motor backward
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Set right motor backward
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

/**
 * Turns robot left (counter-clockwise) in place
 * 
 * @param speed Turning speed (0-255)
 */
void turnLeft(int speed) {
  // Apply calibration factors and constraints
  int leftSpeed = constrain((speed * leftMotorFactor) + leftMotorOffset, 0, 255);
  int rightSpeed = constrain((speed * rightMotorFactor) + rightMotorOffset, 0, 255);
  
  // Left motor backward, right motor forward
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

/**
 * Turns robot right (clockwise) in place
 * 
 * @param speed Turning speed (0-255)
 */
void turnRight(int speed) {
  // Apply calibration factors and constraints
  int leftSpeed = constrain((speed * leftMotorFactor) + leftMotorOffset, 0, 255);
  int rightSpeed = constrain((speed * rightMotorFactor) + rightMotorOffset, 0, 255);
  
  // Left motor forward, right motor backward
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

/**
 * Stops both motors immediately
 */
void stopMotors() {
  // Set all PWM outputs to 0
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Set all direction pins to LOW (brake mode)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}