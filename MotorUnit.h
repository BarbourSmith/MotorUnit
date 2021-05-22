#ifndef MotorUnit_h
#define MotorUnit_h
#include "memory"
// #include "grbl.h"

#include <Arduino.h>
#include <TLC59711.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "MiniPID.h"     //https://github.com/tekdemo/MiniPID
#include <DRV8873LED.h>
#include <AS5048A.h>

enum pid_mode {REVOLUTIONS, CURRENT, DISTANCE, SPEED, MAX = SPEED};
class MotorUnit{

public:
    MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS,
               double mmPerRev,
               double desiredAccuracy);
    std::unique_ptr<MiniPID> pid;
    std::unique_ptr<DRV8873LED> motor;
    std::unique_ptr<AS5048A> angleSensor;
    void zero();
    bool testEncoder();
    int test();
    int setTarget(double newTarget);
    int setPosition(double newPosition);
    double getError();
    bool retract(double targetLength);
    bool comply(unsigned long *timeLastMoved, double *lastPosition, double *amtToMove);
    double getTarget();
    void repeatabilityTest();
    double getCurrent();
    double getPosition();
    void stop();
    int recomputePID();
    void updateEncoderPosition();
    long angleTotal = 0;
    long angleCurrent  = 0;
    long anglePrevious = 0;

private:

    double _mmPerRevolution = 44.0;
    double lastInterval = 0.001;
    unsigned long lastUpdate = millis();

    double p = 10000;
    double i = 0;
    double d = 10000;

    bool disabled = false;
    bool inRegulation = false;

    double accuracy = 0.05; // Accuracy in mm to set in regulation flag

    int output = 0;
    double currentState = 0.0;
    double setpoint = 0.0;
    double errorDist = 0.0;

    double mampsCurrent  = 0.0;
    pid_mode controlMode = DISTANCE;

};

#endif
