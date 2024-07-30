#define SERIAL_DEBUG_ENABLE 1 /* 0 = disable, 1 = enable */
#define MPU6050_CALIBRATION 1 /* 0 = disable, 1 = enable */
#define CONTROLLER_MODE 0     /* 0 = Speed stabilization only, 1 = Speed and Attitude stabilization, 2 = same as 1, but change set point every N secondes */

// -------PINS------- arduino 2-3-4-----esp32  25- 33 - 32
#define PIN_DIR 25
#define PIN_STEP 33
#define PIN_SLEEP 4

// -----STEPPER------
#include "AccelStepper.h"   
                       // This is a hacked version of this library to allow
                                                   // speed control instead of position control.
AccelStepper Stepper_motor(1, PIN_STEP, PIN_DIR);  // 1 means a stepper driver (with Step and Direction pins)
double motorSpeed = 0;
#define MICROSTEPPING 4   /* 1 or 2 or 4 or 8 or 16 or 32 */
#define ACCELERATION 1750 /* Steps per s */
#define MAX_SPEED (600 * MICROSTEPPING)
// ------------------

#include "BluetoothSerial.h"

const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "ESP32-BT-Slave";
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
BluetoothSerial SerialBT;


// -------PID--------P 0.05 D 0.017------0.02-0-0.7
#include "PID.h"
const double P_TERM = 0.4;
const double I_TERM = 0;
const double D_TERM = 1.5;

PIDController pidSpeed(P_TERM, I_TERM, D_TERM);

// PIDAngleController pidAttitude(0.5, 0, 400);
// ------------------2.5, 0, 400------2-0-0

// ------MPU6050-----
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);


//-----GENERAL VARIABLES--------
long t_precedente, t_attuale, t_inizio;
const int n_lett = 8;                                          // numero letture
double letture_velocita[n_lett] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // vettore delle letture
int i_lett = 0;                                                // indice lettura
double total = 0, rollingAvg = 0;
double angolo_attuale = 0, angolo_precedente = 0;
double singola_velang = 0;
double targetSpeed = 0;

// --------- SETUP ---------

void setup() {
#if SERIAL_DEBUG_ENABLE == 1
  Serial.begin(115200);
  delay(500);
  Serial.println("Attitude Speed");
#endif

 SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Initial stepper parameters
  Stepper_motor.setEnablePin(PIN_SLEEP);
  Stepper_motor.setAcceleration(ACCELERATION);
  // funzione per settare il movimento e la max speed del motore
  setSpeedStepper(0);

  t_attuale = millis();  // number of milliseconds passed since the Arduino board began running the current program
  // timeStart = timeCur;
}

// FSM variables
byte controllerState = 0;
int counts = 0;

// Main loop
void loop() {
  // Every 10ms, read MPU and call controllers
  if (millis() - t_attuale > 10) {

     if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
    if (SerialBT.available()) {
    // Read the incoming data from Bluetooth
    String incomingData = SerialBT.readStringUntil('\n');
    
    // Parse the incoming command and extract the prefix and value
    char commandPrefix = incomingData.charAt(0);
    float inputValue = incomingData.substring(2).toFloat();

    // Perform the calculation based on the selected equation
    float result;
    if (commandPrefix == 'c') {
      // Equation 2: 45 * inputValue
      result = 45 * inputValue;
    } else if (commandPrefix == 'b') {
      // Equation 1: 30 * inputValue
      result = 30 * inputValue;
    } else {
      // Invalid command
      SerialBT.println("Invalid command. Use 'b' or 'c' prefix.");
      return;
    }

    // Print the result via Bluetooth
    SerialBT.print("Result: ");
    SerialBT.println(result);
  }
    mpu6050.update();
    t_precedente = t_attuale;
    t_attuale = millis();

    //----- MISURE ANGOLO E VELOCITA' -----------
    /*angolo_attuale = mpu6050.getAngleZ();
    singola_velang = 1000*(angolo_attuale - angolo_precedente)/(millis() - t_precedente);
    angolo_precedente = angolo_attuale;*/
    singola_velang = mpu6050.getGyroZ();

    //-----MEDIA MOBILE-------
    total = total - letture_velocita[i_lett];
    letture_velocita[i_lett] = singola_velang;
    total = total + singola_velang;
    // -----Incrementa l'indice e riavvolgi se necessario--------
    i_lett = (i_lett + 1) % n_lett;
    // -----Calcola la nuova media mobile------
    rollingAvg = total / n_lett;

// Compute controller output
#if CONTROLLER_MODE == 0
    targetSpeed += pidSpeed.compute(0, rollingAvg, t_attuale - t_precedente);
#endif

    // Constrain speed to valid interval (saturation)
    if (targetSpeed > MAX_SPEED) targetSpeed = MAX_SPEED;
    else if (targetSpeed < -MAX_SPEED) targetSpeed = -MAX_SPEED;


    setSpeedStepper(targetSpeed);
    Serial.print("RollingAvg: ");
    Serial.println(rollingAvg);
    Serial.print("Angolo: ");
    Serial.println(angolo_attuale);
    Serial.print("Out controllo: ");
    Serial.println(targetSpeed);
  }
  // Pulse stepper
  Stepper_motor.run();
}

// Set the current speed and direction of the motor
void setSpeedStepper(double targetSpeed) {
  if (targetSpeed < 0)
    Stepper_motor.moveTo(1000000);
  else
    Stepper_motor.moveTo(-1000000);

  Stepper_motor.setMaxSpeed(targetSpeed);
}