#include <Arduino.h>
#include <zirconLib.h>

#define LINE_THRESHOLD 650
#define MOTOR_POWER 100

// correct ball vector
// compass angle readings
// react to lines
// react to line and compass
// react to line and compass and towards ball
// line, compass, orbit

// read sensors
// react to line
// react to compass + fuse ball and orbit

// utils
struct Vector2D {
    float x;
    float y;

    // Default constructor
    Vector2D() : x(0), y(0) {}

    // Parameterized constructor
    Vector2D(float x, float y) : x(x), y(y) {}

    // Method to convert the vector to a string
    String toString() const {
        return "(" + String(x) + ", " + String(y) + ")";
    }

    // Method to print the vector
    void print() const {
        Serial.println(toString());
    }
};

// Directions for sensors arranged clockwise from front
const float angles[8] = {
    M_PI / 2, M_PI / 4, 0, -M_PI / 4,
    -M_PI / 2, -3 * M_PI / 4, -M_PI, 3 * M_PI / 4
};

// Function to read ball sensors and compute the resulting direction vector
Vector2D readBallSensors() {
    float sensorValues[8];

    // Read sensor values
    for (int i = 0; i < 8; i++) {
        sensorValues[i] = readBall(i + 1); // Assuming readBall takes sensor ID 1 to 8
    }

    Vector2D result;

    // Sum the directions weighted by sensor values
    for (int i = 0; i < 8; i++) {
        result.x += sensorValues[i] * cos(angles[i]);
        result.y += sensorValues[i] * sin(angles[i]);
    }

    return result;
}

void moveMotors(const Vector2D& direction, float rotationFactor) {
    float angle = atan2(direction.y, direction.x);
    Serial.println("angle " + String(180*angle/M_PI));
    

    // Calculate motor powers (assuming 3 omni-directional wheels)
    float motor1Power = MOTOR_POWER*max(min(sin(angle - 5*M_PI/6) + rotationFactor, 1), -1);
    float motor2Power = MOTOR_POWER*max(min(sin(angle - 1*M_PI/6) + rotationFactor, 1), -1);
    float motor3Power = MOTOR_POWER*max(min(sin(angle - 9*M_PI/6) + rotationFactor, 1), -1);

    // Convert the powers to suitable PWM values and set motor directions
    motor1(abs(motor1Power), motor1Power < 0 ? 1 : 0);
    motor2(abs(motor2Power), motor2Power < 0 ? 1 : 0);
    motor3(abs(motor3Power), motor3Power < 0 ? 1 : 0);


}





int goalAngle = 0;


void setup(void)
{
  Serial.begin(115200);
  InitializeZircon();
  motor1(50, 0);
  delay(200);
  motor1(0,0);
  while (readButton(1) == 0) {
    goalAngle = readCompass();
  }
}


void loop(void)
{

  Vector2D ballVector = readBallSensors();

  int compassDiff = readCompass() - goalAngle;
  if (compassDiff > 180) {
    compassDiff -= 360;
  } else if (compassDiff < -180) {
    compassDiff += 360;
  }

  // Serial.println("line sensors: " + String(readLine(1)) + " " + String(readLine(2)) + " " + String(readLine(3)));
  // Serial.println("compass diff: " + String(compassDiff) + " ball vector: " + ballVector.toString());

  if (readLine(1) > LINE_THRESHOLD) {
    motor1(100, 0);
    motor2(100, 1);
    motor3(0, 0);
  } else if (readLine(2) > LINE_THRESHOLD) {
    motor1(0, 0);
    motor2(100, 0);
    motor3(100, 1);
  } else if (readLine(3) > LINE_THRESHOLD) {
    motor1(100, 1);
    motor2(0, 0);
    motor3(100, 0);


  // } else if (abs(compassDiff) > 10) {
  //   int speed = abs(compassDiff) + 40;
  //   int direction = (compassDiff < 0) ? 0 : 1;
  //   motor1(speed, direction);
  //   motor2(speed, direction);
  //   motor3(speed, direction);
    
  } else {
    Serial.println("ball vector: " + ballVector.toString() + " compass diff: " + String(compassDiff/180.0));
    moveMotors(ballVector, compassDiff/180.0);
  }


  
  // if (readBall(1) > 650) {

  //   if (abs(goalAngle - readCompass()) < 30) {
  //     motor1(100, 0);
  //     motor2(100, 1);
  //     motor3(0, 0);
  //   } else {
  //     motor1(70, 0);
  //     motor2(10, 1);
  //     motor3(70, 1);
  //   }
    
  // } else {
  //   motor1(40, 1);
  //   motor2(40, 1);
  //   motor3(40, 1);
  // }

  delay(100);


}