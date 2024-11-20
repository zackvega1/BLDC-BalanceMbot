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

BLDCMotor motor = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);


BLDCDriver3PWM driver = BLDCDriver3PWM(15, 14, 13, 23);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(12, 11, 10, 23);

MagneticSensorAS5600 sensor;
MagneticSensorAS5600 sensor1;

BHYSensor bhi160;
volatile bool intrToggled = false;
bool newOrientationData = false;
float heading, roll, pitch;
float rollOffset = 0, pitchOffset = 0, headingOffset = 0;
uint8_t status;

float speed = 0.0;
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.scalar(&motor.target, cmd); }
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


void setup() {
    Serial.begin(115200);
    Serial.println("Serial initialized");

    I2C0.begin();
    I2C1.begin();



    // Initialize encoder sensor hardware
    Serial.println("Initializing sensor 0");
    sensor.init(&I2C0);
    Serial.println("Sensor 0 initialized");

    Serial.println("Initializing sensor 1");
    sensor1.init(&I2C1);
    Serial.println("Sensor 1 initialized");

    // Link the motor to the sensor
    Serial.println("Linking motor0 to sensor");
    motor.linkSensor(&sensor);
    Serial.println("Motor0 linked to sensor");

    Serial.println("Linking motor1 to sensor1");
    motor1.linkSensor(&sensor1);
    Serial.println("Motor1 linked to sensor1");


    driver1.voltage_power_supply = 12;
    driver1.init();
    motor1.linkDriver(&driver1);   

    driver.init();
    motor.linkDriver(&driver);

    motor1.PID_velocity.P = 0.125;
    motor1.PID_velocity.I = 7;
    motor1.PID_velocity.D = 0;
    motor1.voltage_limit = 12;
    motor1.LPF_velocity.Tf = 0.01;
    motor1.P_angle.P = 12;
    motor1.velocity_limit = 50;

    motor1.useMonitoring(Serial);

    
    motor.PID_velocity.P = 0.125;
    motor.PID_velocity.I = 7;
    motor.PID_velocity.D = 0;
    motor.voltage_limit = 12;
    motor.LPF_velocity.Tf = 0.01;
    motor.P_angle.P = 12;
    motor.velocity_limit = 50;

    motor.useMonitoring(Serial);



    motor1.controller = MotionControlType::Stability;
    motor.controller = MotionControlType::Stability;
    
    motor1.init();
    motor1.initFOC();
    motor1.target = 0;

    motor.init();
    motor.initFOC();
    motor.target = 0;

    Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

    motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE ; 
    motor1.monitor_downsample = 100; 


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
            pitchOffset = pitch;
            headingOffset = heading;
            Serial.println("Calibration complete");
            break;
        }
    }

    _delay(1000);
}

long ts;

void loop() {
bhi160.run();
    sensor.update();
    sensor1.update();
    if (newOrientationData) {
       // Serial.println("IMU Data: Heading=" + String(heading) + ", Pitch=" + String(pitch) + ", Roll=" + String(roll) + ", Status=" + String(status));
       
        newOrientationData = false;
        motor1.target = 0;
        motor1.pitch = -roll * .6;

        motor.target = 0;
        motor.pitch = -roll * .6;
        
        Serial.println(roll);
       // Serial.println(motor1.pitch);
        // float pitchError = pitch; 
        // if (abs(pitchError) > maxPitch) {
        //     pitchError = (pitchError > 0) ? maxPitch : -maxPitch;
        // }

        // float pitchGain = 0.5; 
        // float pitchCorrection = pitchError * pitchGain;
        // float target = speed - pitchCorrection;
        // motor1.target = target;

        // if (motor1.target > maxSpeed) { 
        //     motor1.target = maxSpeed;
        // }
        // if (motor1.target < -maxSpeed) { 
        //     motor1.target = -maxSpeed;
        // }

        motor1.loopFOC();
        motor1.move();

        motor.loopFOC();
        motor.move();
    }

    command.run();
}