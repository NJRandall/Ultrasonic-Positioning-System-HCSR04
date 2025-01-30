#include <math.h>

// Define boundary constants
const int X_MIN_BOUNDARY = 0;
const int X_MAX_BOUNDARY = 130;
const int Y_MIN_BOUNDARY = 0;
const int Y_MAX_BOUNDARY = 145;

const float VALID_RANGE_MIN = 1.8;
const float VALID_RANGE_MAX = 200;

// Define number of pins
const int NUM_OF_MAIN_TRIGGERS = 3;  // Number of main triggers
const int NUM_OF_ECHO_PINS = 3;      // Number of echo pins

// Define pins for the sensors
const int mainTriggerPins[NUM_OF_MAIN_TRIGGERS] = {22, 24, 26};  // Main trigger pins
const int beaconEchoPins[NUM_OF_ECHO_PINS] = {30, 32, 34};  // Echo pins for the 3 receiver sensors

const int errorLedPin = 13;          // Main LED pin for error indication
const int adjustmentLedPin = 12;     // LED pin for when X or Y is adjusted
const int successLedPin = 11;        // LED pin for successful circular intersection

// Position variables for 3 receivers
float receiverCoords[NUM_OF_ECHO_PINS][2] = {
  {120, 145},    // Receiver 1 coordinates
  {130, 145},    // Receiver 2 coordinates
  {130, 135}     // Receiver 3 coordinates
};

float distances[NUM_OF_MAIN_TRIGGERS][NUM_OF_ECHO_PINS] = {{0, 0, 0}, 
                                                            {0, 0, 0}, 
                                                            {0, 0, 0}};

float vehicleX = 0, vehicleY = 0;  // Estimated vehicle position

void setup() {
  Serial.begin(9600);

  // Initialize trigger and echo pins
  for (int i = 0; i < NUM_OF_MAIN_TRIGGERS; i++) {
    pinMode(mainTriggerPins[i], OUTPUT);
  }
  for (int i = 0; i < NUM_OF_ECHO_PINS; i++) {
    pinMode(beaconEchoPins[i], INPUT);
  }

  pinMode(errorLedPin, OUTPUT);      // Initialize the error LED pin
  pinMode(successLedPin, OUTPUT);    // Initialize the success LED pin
  pinMode(adjustmentLedPin, OUTPUT); // Initialize the adjustment LED pin
}

void loop() {
  vehicleX = 0, vehicleY = 0;
  // Loop through each trigger pin
  for (int triggerIdx = 0; triggerIdx < NUM_OF_MAIN_TRIGGERS; triggerIdx++) {
    // Measure distances for each receiver from the current trigger pin
    for (int i = 0; i < NUM_OF_ECHO_PINS; i++) {
      distances[triggerIdx][i] = measureDistance(beaconEchoPins[i], mainTriggerPins[triggerIdx]);
    }
  }

  // Print the distances for each receiver
  printDistances();

  // Call the trilateration function to estimate the vehicle's coordinates
  if (!trilateratePosition()) { //Never running
    Serial.println("Error: Unable to estimate valid vehicle position.");
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    digitalWrite(errorLedPin, HIGH);  // Turn on error LED if position estimation fails
    digitalWrite(successLedPin, LOW);  // Turn off success LED
    digitalWrite(adjustmentLedPin, LOW); // Turn off adjustment LED
  } else {
    if(vehicleX == 0 && vehicleY == 0) {
      Serial.println("Invalid vehicle coordinates");
      digitalWrite(successLedPin, LOW); // Turn on success LED
      digitalWrite(adjustmentLedPin, LOW); // Turn on success LED
      digitalWrite(errorLedPin, HIGH);     // Turn off error LED
    } else {
      // Print the estimated vehicle coordinates
      // TODO: Send signal communication
      digitalWrite(successLedPin, HIGH); // Turn on success LED
      digitalWrite(errorLedPin, LOW);     // Turn off error LED
  }
  }

  delay(1000);  // Wait for 1 second before the next estimation
}

// Function to measure distance from a given echo pin and trigger pin
float measureDistance(int echoPin, int triggerPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0343;  // Convert to one-way distance (cm)

  // Check if the distance is within the valid range (1.8 cm to 200 cm)
  if (distance < VALID_RANGE_MIN || distance > VALID_RANGE_MAX) {
    return -1;  // Mark invalid distances with -1
  }
  return distance;  // Return valid distance
}

// Function to perform trilateration using two smallest valid distances
bool trilateratePosition() {
  float minDist1 = 9999, minDist2 = 9999;  // Initialize to large values
  int minIdx1_trigger = -1, minIdx1_receiver = -1;
  int minIdx2_trigger = -1, minIdx2_receiver = -1;

  // Step 1: Find the two smallest distances from the distances array
  for (int triggerIdx = 0; triggerIdx < NUM_OF_MAIN_TRIGGERS; triggerIdx++) {
    for (int i = 0; i < NUM_OF_ECHO_PINS; i++) {
      if (distances[triggerIdx][i] > 0) {  // Only valid distances greater than 0 (invalid ones are -1)
        if (distances[triggerIdx][i] < minDist1) {
          // Move the first smallest to the second smallest
          minDist2 = minDist1;
          minIdx2_trigger = minIdx1_trigger;
          minIdx2_receiver = minIdx1_receiver;

          // Update the first smallest
          minDist1 = distances[triggerIdx][i];
          minIdx1_trigger = triggerIdx;
          minIdx1_receiver = i;
        } else if (distances[triggerIdx][i] < minDist2 && i != minIdx1_receiver) {
          // Only update the second smallest if it's from a different receiver
          minDist2 = distances[triggerIdx][i];
          minIdx2_trigger = triggerIdx;
          minIdx2_receiver = i;
        }
      }
    }
  }

  // Print the minimum distances and their indices for debugging
  Serial.print("Min Distances: ");
  Serial.print(minDist1);
  Serial.print(" (Trigger: ");
  Serial.print(minIdx1_trigger + 1);
  Serial.print(", Receiver: ");
  Serial.print(minIdx1_receiver + 1);
  Serial.print("), ");
  Serial.print(minDist2);
  Serial.print(" (Trigger: ");
  Serial.print(minIdx2_trigger + 1);
  Serial.print(", Receiver: ");
  Serial.print(minIdx2_receiver + 1);
  Serial.println(")");

  float x1 = receiverCoords[minIdx1_receiver][0];
  float y1 = receiverCoords[minIdx1_receiver][1];
  float d1 = minDist1;
  float x2 = receiverCoords[minIdx2_receiver][0];
  float y2 = receiverCoords[minIdx2_receiver][1];
  float d2 = minDist2;

  // Call circleIntersection with the found coordinates and distances
  if (circleIntersection(x1, y1, d1, x2, y2, d2, vehicleX, vehicleY)) {
    // Check if the calculated vehicle position is within boundaries
    if (vehicleX >= X_MIN_BOUNDARY && vehicleX <= X_MAX_BOUNDARY &&
        vehicleY >= Y_MIN_BOUNDARY && vehicleY <= Y_MAX_BOUNDARY) {
      // Valid position within boundaries, return success
      digitalWrite(errorLedPin, LOW);  // Turn off error LED
      digitalWrite(successLedPin, HIGH); // Turn on success LED
      return true;
    }
  } else {
    // No intersection, apply the direct adjustment for specific receivers
    Serial.println("### Attempting Adjustment ###");
    bool adjusted = false;  // Track if any adjustment was made

    if (minIdx1_receiver == 0 || minIdx1_receiver == 5) {  // Receiver 1 or 6
      vehicleX = receiverCoords[minIdx1_receiver][0] - minDist1;
      adjusted = true;
    } else if (minIdx1_receiver == 2 || minIdx1_receiver == 3) {  // Receiver 3 or 4
      vehicleY = receiverCoords[minIdx1_receiver][1] - minDist1;
      adjusted = true;
    }

    if (minIdx2_receiver == 0 || minIdx2_receiver == 5) {  // Receiver 1 or 6
      vehicleX = receiverCoords[minIdx2_receiver][0] - minDist2;
      adjusted = true;
    } else if (minIdx2_receiver == 2 || minIdx2_receiver == 3) {  // Receiver 3 or 4
      vehicleY = receiverCoords[minIdx2_receiver][1] - minDist2;
      adjusted = true;
    }

    // Check if adjusted position is within boundaries
    if (vehicleX >= X_MIN_BOUNDARY && vehicleX <= X_MAX_BOUNDARY &&
        vehicleY >= Y_MIN_BOUNDARY && vehicleY <= Y_MAX_BOUNDARY) {
      digitalWrite(errorLedPin, LOW);  // Turn off error LED
      digitalWrite(adjustmentLedPin, HIGH); // Turn on adjustment LED
      return true;
    }
  }
  
  Serial.print("!!! Adjustment Failed: ");
  digitalWrite(errorLedPin, HIGH);  // Turn off error LED
  digitalWrite(adjustmentLedPin, LOW); // Turn on adjustment LED
  return false;  // If all checks fail, return false
}

// Function to calculate intersection of two circles
bool circleIntersection(float x1, float y1, float r1, float x2, float y2, float r2, float &x, float &y) {
  float d = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); // Distance between centers

  if (d > r1 + r2 || d < abs(r1 - r2) || d == 0) {
    return false;  // No intersection
  }

  // Calculate intersection
  float a = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
  float h = sqrt(pow(r1, 2) - pow(a, 2));

  float x0 = x1 + (a * (x2 - x1)) / d;
  float y0 = y1 + (a * (y2 - y1)) / d;

  x = x0 + (h * (y2 - y1)) / d;  // First intersection point
  y = y0 - (h * (x2 - x1)) / d;  // First intersection point

  return true;  // Intersection exists
}

// Function to print distances for debugging
void printDistances() {
  for (int i = 0; i < NUM_OF_MAIN_TRIGGERS; i++) {
    Serial.print("Distances from Trigger ");
    Serial.print(i + 1);
    Serial.print(": ");
    for (int j = 0; j < NUM_OF_ECHO_PINS; j++) {
      Serial.print(distances[i][j]);
      Serial.print(" cm");
      if (j < NUM_OF_ECHO_PINS - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
  }
}
