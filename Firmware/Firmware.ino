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
#define DCMOTORDRIVERL298_PIN_INT1 2
#define DCMOTORDRIVERL298_PIN_ENB 6
#define DCMOTORDRIVERL298_PIN_INT2 4
#define DCMOTORDRIVERL298_PIN_ENA 5
#define DCMOTORDRIVERL298_PIN_INT3 7
#define DCMOTORDRIVERL298_PIN_INT4 8
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
DCMDriverL298 dcMotorDriverL298(DCMOTORDRIVERL298_PIN_ENA, DCMOTORDRIVERL298_PIN_INT1, DCMOTORDRIVERL298_PIN_INT2, DCMOTORDRIVERL298_PIN_ENB, DCMOTORDRIVERL298_PIN_INT3, DCMOTORDRIVERL298_PIN_INT4);
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
}

void executeCommand(char command)
{
  // Motor control logic
  if (command == 'F')
  {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  else if (command == 'B')
  {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  else if (command == 'L')
  {
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
  }
  else if (command == 'R')
  {
    motor1.run(FORWARD);
    motor2.run(BACKWARD);
  }
  else if (command == 'S')
  {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }
}
