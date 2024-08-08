#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5600/MagneticSensorAS5600.h"
#include <Wire.h>
#include <math.h>
#include "bhy.h"
#include "firmware/Bosch_PCB_7183_di03_BMI160_BMM150-7183_di03.2.1.11696_170103.h"

// Define I2C instances
TwoWire I2C0 = TwoWire(4, 5); // I2C0 pins
TwoWire I2C1 = TwoWire(2, 3); // I2C1 pins

// BLDC motor & driver instances
BLDCMotor motor0 = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(15, 14, 13, 23);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(12, 11, 10, 23);

// Encoder instances
MagneticSensorAS5600 sensor;
MagneticSensorAS5600 sensor1;

// IMU sensor instance
BHYSensor bhi160;
volatile bool intrToggled = false;
bool newOrientationData = false;
float heading, roll, pitch;
float rollOffset = 0, pitchOffset = 0, headingOffset = 0;

uint8_t status;
float speed = 0.0;
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.scalar(&motor0.target, cmd); }
void onMotor(char* cmd) { command.scalar(&motor1.target, cmd); }
void setSpeed(char* cmd){ command.scalar(&speed, cmd);}

void bhyInterruptHandler(void)
{
    intrToggled = true;
}

bool checkSensorStatus(void)
{
    if (bhi160.status == BHY_OK)
        return true;

    if (bhi160.status < BHY_OK)
    {
        Serial.println("Error code: (" + String(bhi160.status) + "). " + bhi160.getErrorString(bhi160.status));
        return false;
    }
    else
    {
        Serial.println("Warning code: (" + String(bhi160.status) + ").");
        return true;
    }
}

// IMU data handler
void orientationHandler(bhyVector data, bhyVirtualSensor type)
{
    heading = data.x - headingOffset;
    roll = data.z - rollOffset;
    pitch = data.y - pitchOffset;
    status = data.status;
    newOrientationData = true;
}


const float maxPitch = 10.0; 
const float maxSpeed = 8;

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    Serial.println("Serial initialized");

    // Initialize I2C instances
    I2C0.begin();
    I2C1.begin();
    
    Serial.println("I2C initialized");

    sensor1.init();
    motor1.linkSensor(&sensor1);

    driver1.voltage_power_supply = 12;
    driver1.init();
    motor1.linkDriver(&driver1);

    motor1.controller = MotionControlType::torque;



    


    // Controller configuration
    motor1.PID_velocity.P = 0.125;
    motor1.PID_velocity.I = 7;
    motor1.PID_velocity.D = 0;
    motor1.voltage_limit = 12;
    motor0.LPF_velocity.Tf = 0.01;
    motor0.P_angle.P = 12;
    motor0.velocity_limit = 50;

    motor1.useMonitoring(Serial);

    motor1.init();
    motor1.initFOC();
    motor1.target = 0;

    command.add('A', onMotor, "motor");
    command.add('S', setSpeed, "Speed");

    Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

    


    

    // Initialize IMU
    Serial.println("Initializing IMU");
    attachInterrupt(digitalPinToInterrupt(22), bhyInterruptHandler, RISING);
    bhi160.begin(0x28); 
    
    if (!checkSensorStatus()) {
        Serial.println("IMU initialization failed");
        return;
    }

    Serial.println("Sensor found over I2C! Product ID: 0x" + String(bhi160.productId, HEX));
    Serial.println("Uploading Firmware.");
    bhi160.loadFirmware(bhy1_fw);

    // Add a progress bar for firmware upload
    unsigned long startMillis = millis();
    while (bhi160.status == BHY_OK && millis() - startMillis < 10000) { // Adjust time limit as needed
        Serial.print(".");
        delay(500); // Update every half second
    }

    if (!checkSensorStatus()) {
        Serial.println("Firmware upload failed");
        return;
    }

    Serial.println("\nFirmware booted");

    if (bhi160.installSensorCallback(BHY_VS_ORIENTATION, true, orientationHandler)) {
        checkSensorStatus();
        Serial.println("Orientation callback installation failed");
        return;
    } else {
        Serial.println("Orientation callback installed");
    }

    if (bhi160.configVirtualSensor(BHY_VS_ORIENTATION, true, BHY_FLUSH_ALL, 200, 0, 0, 0)) {
        Serial.println("Failed to enable virtual sensor (" + bhi160.getSensorName(BHY_VS_ORIENTATION) + ").");
    } else {
        Serial.println(bhi160.getSensorName(BHY_VS_ORIENTATION) + " virtual sensor enabled");
    }

    Serial.println("Calibrating IMU...");
    unsigned long startMillisCal = millis();
    while (millis() - startMillisCal < 5000) { // Wait for 5 seconds for calibration
        bhi160.run();
        if (newOrientationData) {
            rollOffset = roll;
            pitchOffset = pitch + 5.66;
            headingOffset = heading;
            Serial.println("Calibration complete");
            break;
        }
    }

    
    // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
    motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE ; 
    motor1.monitor_downsample = 100; 
    _delay(1000);
}

float x = 0;
void loop() {
    // Update IMU data
    bhi160.run();

    if (newOrientationData) {
        Serial.println("IMU Data: Heading=" + String(heading) + ", Pitch=" + String(pitch) + ", Roll=" + String(roll) + ", Status=" + String(status));
        newOrientationData = false;

        float pitchError = pitch; 
        if (abs(pitchError) > maxPitch) {
            pitchError = (pitchError > 0) ? maxPitch : -maxPitch;
        }

        float pitchGain = 0.5; 
        float pitchCorrection = pitchError * pitchGain;
        float target = speed - pitchCorrection;
        motor1.target = target;

        if (motor1.target > maxSpeed) { 
            motor1.target = maxSpeed;
        }
        if (motor1.target < -maxSpeed) { 
            motor1.target = -maxSpeed;
        }

        motor1.loopFOC();
        motor1.move();
    }

    command.run();
    x += 0.001;
}