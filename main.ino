/*
 * RPLidarC1 Teensy 4.1 Example with Simple Autonomy + Moving Average Filter
 *
 * Hardware connections (Teensy 4.1):
 * - RPLidar VCC -> 5V
 * - RPLidar GND -> GND
 * - RPLidar TX -> Pin 0 (RX1)
 * - RPLidar RX -> Pin 1 (TX1)
 */

// RX (pin 0) and TX (pin 1) serial1 for the Teensy
#define C1UART Serial1

//Pins for the Motor Drivers. #1 = Right wheel | #2 = Left wheels
#define PWM1 9
#define PWM2 6
#define INA1 7
#define INA2 4
#define INB1 8
#define INB2 5

#include "RPLidarC1.h"

RPLidarC1 lidar(&Serial1);
RPLidarHealth health;

//Used for holding and getting measurements. 
static RPLidarMeasurement frame[2000];
static int frameCount = 0;
static bool collecting = false;

static int rotationCount = 0;
bool done = false;

// Set-Up for Thresh-holds and filtering data
#define FILTER_SIZE 15
float frontDistances[FILTER_SIZE];
int filterIndex = 0;
bool filterFilled = false;
#define FRONT_THRESHOLD 304   // mm (~12 inches)

//Used for finding largest distance from the left and right
float maxRight = 0;
float maxLeft = 0;
String sidestate;

//Used for objects within thresh-holds
#define HYSTERESIS_MARGIN 50  // mm

bool obstacleDetected = false;

enum AGVSTATE { FORWARD, REVERSE, LEFT, RIGHT};
AGVSTATE state = FORWARD;

//Time designated for operations
unsigned long stateStartTime = 0;
unsigned long reverseTime = 3000;
unsigned long turnTime = 1000;

//Functions for each direction.
void reverse(int PWM){
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW);
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, HIGH);
  analogWrite(PWM1, PWM);
  analogWrite(PWM2, PWM);
}

void goForward(int PWM){
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, HIGH);
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  analogWrite(PWM1, PWM);
  analogWrite(PWM2, PWM);
}

void goLeft(int PWM){
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  analogWrite(PWM1, PWM);
  analogWrite(PWM2, PWM);
}

void goRight(int PWM){
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  analogWrite(PWM1, PWM);
  analogWrite(PWM2, PWM);
}


void addDistance(float d) {
    frontDistances[filterIndex] = d;
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
    if (filterIndex == 0) filterFilled = true;
}

float getSmoothedDistance() {
    float sum = 0;
    int count = filterFilled ? FILTER_SIZE : filterIndex;
    if (count == 0) return 1000; // default "no obstacle" value
    for (int i = 0; i < count; i++){
      sum += frontDistances[i];
    } 
    return sum / count;
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(INA1, OUTPUT);
    pinMode(INA2, OUTPUT);
    pinMode(INB1, OUTPUT);
    pinMode(INB2, OUTPUT);

    Serial.println("RPLidarC1 Teensy Example with Simple Autonomy");

    // Initialize the lidar
    Serial.println("Initializing RPLidar...");
    if (!lidar.begin(460800, 4000)) {
        Serial.println("Failed to initialize RPLidar!");
        while (1) delay(1000);
    }

    // Check health status
    if (lidar.get_health(&health)) {
        lidar.print_health(&health);
        if (health.status != RPLIDAR_STATUS_OK) {
            Serial.println("Warning: Lidar not healthy, attempting reset...");
            lidar.reset();
            delay(2000);
        }
    } 
    else {
        Serial.println("Failed to get health status");
    }

    // Start scanning
    if (!lidar.start_scan()) {
        Serial.println("Failed to start scan");
        while (1) delay(1000);
    }
    Serial.println("Scan started successfully");
}

void loop() {
  if (done) return;

  RPLidarMeasurement m;
  if (!lidar.get_measurement(&m)) return;

  // Store lidar measurement in frame
  if (collecting && frameCount < 2000) {
    frame[frameCount++] = m; 
  }

    // Detect rotation start
  if (m.start_flag) {
    if (collecting && frameCount > 10) {
      rotationCount++;

      // Sort frame by angle
      for (int i = 0; i < frameCount - 1; i++) {
        for (int j = i + 1; j < frameCount; j++) {
          if (frame[j].angle < frame[i].angle) {
            RPLidarMeasurement tmp = frame[i];
            frame[i] = frame[j];
            frame[j] = tmp;
          }
        }
      }

            // Reset left/right maxima
            maxLeft = 0;
            maxRight = 0;

            // Process frame measurements
            for (int i = 0; i < frameCount; i++) {
                float angle = frame[i].angle;
                float dist = frame[i].distance;

                // Front ±40°
                if (angle >= 320 || angle <= 40) addDistance(dist);

                // Left front (40-90°)
                if (angle > 40 && angle <= 90) maxLeft = max(maxLeft, dist);

                // Right front (270-320°)
                if (angle >= 270 && angle < 320) maxRight = max(maxRight, dist);
            }

            float smoothedFront = getSmoothedDistance();

            // Obstacle detection
            if (!obstacleDetected && smoothedFront < FRONT_THRESHOLD) {
                obstacleDetected = true;
                state = REVERSE;
                stateStartTime = millis();

                sidestate = (maxLeft > maxRight) ? "LEFT" : "RIGHT";
                Serial.print("OBSTACLE DETECTED - Will turn: ");
                Serial.println(sidestate);
            }

            // State machine
            switch (state) {
                case FORWARD:
                    goForward(50);
                    Serial.println("STATE: FORWARD");
                    break;

                case REVERSE:
                    reverse(35);
                    Serial.println("STATE: REVERSE");

                    if (millis() - stateStartTime >= reverseTime) {
                        // After reverse, decide direction
                        if (sidestate == "LEFT") state = LEFT;
                        else state = RIGHT;
                        stateStartTime = millis(); // reset timer for turn
                    }
                    break;

                case LEFT:
                    goLeft(60);
                    Serial.println("STATE: LEFT");

                    if (millis() - stateStartTime >= turnTime) {
                        state = FORWARD;
                        obstacleDetected = false; // obstacle cleared
                        filterIndex = 0;          // reset moving average
                    }
                    break;

                case RIGHT:
                    goRight(60);
                    Serial.println("STATE: RIGHT");

                    if (millis() - stateStartTime >= turnTime) {
                        state = FORWARD;
                        obstacleDetected = false;
                        filterIndex = 0;
                    }
                    break;
            }
        }

        // Reset frame for next rotation
        frameCount = 0;
        collecting = true;
    }
}
