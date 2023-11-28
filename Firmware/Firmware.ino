// Include Libraries
#include "Arduino.h"
#include "BTHC05.h"
#include "DCMDriverL298.h"
#include "DCMotor.h"
#include "NewPing.h"
#include "Servo.h"
#include "Pump.h"
#include <AFMotor.h>


// Pin Definitions
#define BTHC05_PIN_TXD 3
#define BTHC05_PIN_RXD 10
#define INT1 2
#define ENB 6
#define INT2 4
#define ENA 5
#define INT3 7
#define INT4 8
#define DCMOTOR_PIN_COIL1 9
#define HCSR04_PIN_TRIG 13
#define HCSR04_PIN_ECHO 12
#define SERVO9G_PIN_SIG A4
#define WATERPUMP_PIN_COIL1 11
#define WATERLEVELSENSOR_5V_PIN_SIG A3
#define WATERPUMP 11

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

bool isAnalogMotorRunning = false;
unsigned long servoPreviousMillis = 0;
const unsigned long servoInterval = 1000;
unsigned long motorControlPreviousMillis = 0;
const unsigned long motorControlInterval = 100; // Adjust as needed
unsigned long waterLevelCheckPreviousMillis = 0;
const unsigned long waterLevelCheckInterval = 1000; // Adjust as needed

// Global variables and defines
BTHC05 bthc05(BTHC05_PIN_RXD, BTHC05_PIN_TXD);
DCMDriverL298 dcMotorDriverL298(ENA,INT1,INT2,ENB,INT3,INT4);
DCMotor dcMotor(DCMOTOR_PIN_COIL1);
NewPing hcsr04(HCSR04_PIN_TRIG, HCSR04_PIN_ECHO);
Servo servo9g;
Pump waterpump(WATERPUMP_PIN_COIL1);
char command;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup()
{
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  pinMode(WATERPUMP, OUTPUT);
  pinMode(DCMOTOR_PIN_COIL1, OUTPUT);

  // Setup Serial which is useful for debugging
  // Use the Serial Monitor to view printed messages
  Serial.begin(9600);
  bthc05.begin(9600);
  servo9g.attach(SERVO9G_PIN_SIG);
  servo9g.write(0);
  // Initialize the previous millis values
  servoPreviousMillis = millis();
  motorControlPreviousMillis = millis();
  waterLevelCheckPreviousMillis = millis();
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop()
{
  unsigned long currentMillis = millis();

  // Handle servo movement every servoInterval milliseconds
  if (currentMillis - servoPreviousMillis >= servoInterval)
  {
    servoPreviousMillis = currentMillis;
    servo9g.write(0);
    delay(500);
    servo9g.write(90);
    delay(500);
  }

  // Handle motor control every motorControlInterval milliseconds
  if (currentMillis - motorControlPreviousMillis >= motorControlInterval)
  {
    motorControlPreviousMillis = currentMillis;

    if (bthc05.available() > 0)
    {
      command = bthc05.read();
      if (!isAnalogMotorRunning)
      {
        // Execute motor control commands only if the analog motor is not running
        executeCommand(command);
      }
    }
  }

  // Handle water level check every waterLevelCheckInterval milliseconds
  if (currentMillis - waterLevelCheckPreviousMillis >= waterLevelCheckInterval)
  {
    waterLevelCheckPreviousMillis = currentMillis;
    int waterLevel = analogRead(A3);

    if (waterLevel < 200)
    {
      digitalWrite(WATERPUMP, LOW);
    }
    else
    {
      digitalWrite(WATERPUMP, HIGH);
    }
  }
  if (currentMillis - waterLevelCheckPreviousMillis >= waterLevelCheckInterval)
  {
   if (bthc05.available() > 0) {
    char receivedChar = bthc05.read(); // Read incoming character
    if (receivedChar == 'v') {
      int sensorValue = analogRead(WATERLEVELSENSOR_5V_PIN_SIG); // Read sensor value
      float waterLevelPercent = map(sensorValue, 0, 1023, 0, 100); // Map sensor value to percentage (assuming sensor range is 0-1023)

      // Send water level percentage over Bluetooth
      bthc05.write((byte*)&waterLevelPercent, sizeof(waterLevelPercent));
    }
  }
}
}

void executeCommand(char command)
{
  // Motor control logic
  if (command == 'F')
  {
    digitalWrite(INT1, HIGH);
    digitalWrite(INT2, LOW);
  }
  else if (command == 'B')
  {
    digitalWrite(INT1, LOW);
    digitalWrite(INT1, HIGH);
  }
  else if (command == 'L')
  {
    digitalWrite(INT1, LOW);
    digitalWrite(INT2, LOW);
    digitalWrite(INT3, HIGH);
    digitalWrite(INT4, LOW);
  }
  else if (command == 'R')
  {
    digitalWrite(INT1, HIGH);
    digitalWrite(INT2, LOW);
    digitalWrite(INT3, LOW);
    digitalWrite(INT4, LOW);
  }
  else if (command == 'S')
  {
    digitalWrite(INT1, LOW);
    digitalWrite(INT2, LOW);
    digitalWrite(INT3, LOW);
    digitalWrite(INT4, LOW);
  }
}
