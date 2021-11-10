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

class MotorUnit{

public:
    MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               byte angleCS,
               void (*webPrint) (double arg1));
    std::unique_ptr<MiniPID> positionPID;
    std::unique_ptr<MiniPID> velocityPID;
    std::unique_ptr<DRV8873LED> motor;
    std::unique_ptr<AS5048A> angleSensor;
    void zero();
    bool testEncoder();
    int test();
    int setTarget(double newTarget);
    int setPosition(double newPosition);
    double getError();
    bool retract(double targetLength);
    bool comply(unsigned long *timeLastMoved, double *lastPosition, double *amtToMove, double maxSpeed);
    double getTarget();
    void repeatabilityTest();
    double getCurrent();
    double getPosition();
    void stop();
    void recomputePID();
    int  recomputeVelocityPID();
    void updateEncoderPosition();
    void decompressBelt();
    void fullOut();
    void setVelocityTarget(double newVelocity);
    double getVelocity();

private:

    double _mmPerRevolution = 44.0;
    double lastInterval = 0.001;
    unsigned long lastUpdate = millis();

    double p = 10;  //10
    double i = 0;   //0
    double d = 0;  //10


    double pv = 1500; //3000
    double iv = 40;    //0
    double dv = 5000; //5000

    bool disabled = false;

    int output = 0;
    double currentState = 0.0;
    double setpoint = 0.0;
    double errorDist = 0.0;
    
    long angleTotal = 0;
    long angleCurrent  = 0;
    long anglePrevious = 0;

    unsigned long timeLastEncoderRead = 0; //In microseconds
    double velocity = 0;
    double velocitySetpoint = 0;
    
    int _stallThreshold = 20; //The number of times in a row needed to trigger a warning
    int _stallCurrent = 26;  //The current threshold needed to count
    int _stallCount = 0;
    void (*_webPrint) (double arg1);
    int removeDeadband(int commandPWM);

};

#endif
