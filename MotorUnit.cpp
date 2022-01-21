/***************************************************
 *  This is a library for control of a DC motor via a TLC Led driver with angle
 *  sensor feedback on an ESP32.
 *
 *  By Alexander Martin-Ginnold for Maslow CNC
 ****************************************************/
#include "MotorUnit.h"

/*!
 *  @brief  Instantiates a new MotorUnit class. Instantiates classes for
 *          all other necessary controls.
 *  @param  tlc Pointer to a TLC59711 object to output PWM signals
 *  @param  forwardPin Output pin number for the TLC59711. If this pin is at max
 *          output and the other pin is at 0 the motor turns forward
 *  @param  backwardPin Output pin number for the TLC59711. If this pin is at
 *          max output and the other pin is at 0 the motor turns backward
 *  @param  readbackPin ESP32 adc_channel_t pin number for current readback
 *  @param  angleCS ESP32 pin for the chip select of the angle sensor
 *
 */
MotorUnit::MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               byte angleCS,
               void (*webPrint) (double arg1)){
    _mmPerRevolution = 44;
    positionPID.reset(new MiniPID(p,i,d));
    positionPID->setOutputLimits(-6553,6553);

    motor.reset(new DRV8873LED(tlc, forwardPin, backwardPin, readbackPin));
    angleSensor.reset(new AS5048A(angleCS));
    angleSensor->init();
    _webPrint = webPrint;
    
    zero();
}

//---------------------Functions related to testing the motor unit--------------------------------------------------


/*!
 *  @brief  Tests the function of the encoder
 */
bool MotorUnit::testEncoder(){
    motor->stop();
    
    // Test reading from angle sensor
    if(angleSensor->getRotation() != 0){
        Serial.print(angleSensor->getRotation());
        Serial.println(" -- encoder pass");
        return true;
    }
    else{
        Serial.println("encoder fail");
        return false;
    }
}

/*!
 *  @brief  Tests all the functions of this axis
 */
int MotorUnit::test(){
    
    bool testPass = true;
    
    testPass = testEncoder();
    
     // Test writing to motor
    motor->fullOut();
    
    delay(1000);
    
    if(motor->readCurrent() > 0){
        Serial.print(".");
        // Serial.print("Direction one nominal at ");
        // Serial.print(motor->readCurrent());
        // Serial.println("% current draw");
    }
    else{
        Serial.print("~");
        testPass = false;
        // Serial.print("Direction one fail at ");
        // Serial.print(motor->readCurrent());
        // Serial.println("% current draw");
    }
    
    
    motor->stop();
    
    delay(1000);
    
    motor->fullIn();
    
    delay(1000);
    
    if(motor->readCurrent() > 0){
        Serial.print(".");
        // Serial.print("Direction two nominal at ");
        // Serial.print(motor->readCurrent());
        // Serial.println("% current draw");
    }
    else{
        Serial.print("~");
        testPass = false;
        // Serial.print("Direction two fail at ");
        // Serial.print(motor->readCurrent());
        // Serial.println("% current draw");
    }
    
    motor->stop();
    
     // Test moving to a commanded position
    zero();
    setTarget(25);
    unsigned long time = millis();
    unsigned long elapsedTime = millis()-time;
    while(elapsedTime < 3000){
        elapsedTime = millis()-time;
        recomputePID();
    }
    
    if(abs(getError()) > .1){
        Serial.print("~");
        testPass = false;
        // Serial.print("Failed to move to target position with error: ");
        // Serial.println(getError());
    }
    else{
        Serial.print(".");
        // Serial.print("Pass move to target position with error: ");
        // Serial.println(getError());
    }
    
    zero();
    setTarget(-25);
    time = millis();
    elapsedTime = millis()-time;
    while(elapsedTime < 3000){
        elapsedTime = millis()-time;
        recomputePID();
    }
    
    if(abs(getError()) > .1){
        Serial.print("~");
        testPass = false;
        // Serial.print("Failed to move to target position with error: ");
        // Serial.println(getError());
    }
    else{
        Serial.print(".");
        // Serial.print("Pass move to target position with error: ");
        // Serial.println(getError());
    }
    
    motor->stop();
    
    if(testPass){
        Serial.println("Pass");
    }
    else{
        Serial.println("Fail");
    }
    
}


//---------------------Functions related to the motor unit's position interface-------------------------------------


/*!
 *  @brief  Resets the axis position to zero
 */
void MotorUnit::zero(){
    angleTotal = 0;
    
    angleCurrent  = angleSensor->getRawRotation();
    anglePrevious = angleSensor->getRawRotation();
}

/*!
 *  @brief  Sets the target location
 */
int MotorUnit::setTarget(double newTarget){
    setpoint = newTarget;
}

/*!
 *  @brief  Gets the target location
 */
double MotorUnit::getTarget(){
    return setpoint;
}

/*!
 *  @brief  Sets the position of the cable
 */
int MotorUnit::setPosition(double newPosition){
    
    angleCurrent  = angleSensor->RotationRawToAngle(angleSensor->getRawRotation());
    anglePrevious = angleSensor->RotationRawToAngle(angleSensor->getRawRotation());
    
    angleTotal = newPosition*16384;
}

/*!
 *  @brief  Reads the current position of the axis
 */
double MotorUnit::getPosition(){
    return (angleTotal/16384.0)*_mmPerRevolution;
}

/*!
 *  @brief  Gets the current motor power draw
 */
double MotorUnit::getCurrent(){
    return motor->readCurrent();
}

/*!
 *  @brief  Computes and returns the error in the axis positioning
 */
double MotorUnit::getError(){
    
    double errorDist = setpoint - getPosition();
    
    return errorDist;
    
}

/*!
 *  @brief  Stops the motor
 */
void MotorUnit::stop(){
    motor->stop();
}

/*!
 *  @brief  Runs the motor out at full speed...where is this used? Why is it not symmetric with a fullIn() option?
 */
void MotorUnit::fullOut(){
    motor->fullOut();
}

//---------------------Functions related to maintaining the PID controllers-----------------------------------------



/*!
 *  @brief  Reads the encoder value and updates it's position and measures the velocity since the last call
 */
void MotorUnit::updateEncoderPosition(){
    long oldAngleTotal = angleTotal;

    angleCurrent = angleSensor->getRawRotation();
    angleSensor->AbsoluteAngleRotation(&angleTotal, &angleCurrent, &anglePrevious);
    
}

/*!
 *  @brief  Recomputes the PID and drives the output
 */
double MotorUnit::recomputePID(){
    
    updateEncoderPosition();
    

    //Read the motor current and check for stalls
    double currentNow = getCurrent();
    if(currentNow > _stallCurrent){
        _stallCount = _stallCount + 1;
    }
    else{
        _stallCount = 0;
    }
    if(_stallCount > _stallThreshold){
        _webPrint(currentNow);
        _stallCount = 0;
    }
    
    double commandPWM = 10*positionPID->getOutput(getPosition(),setpoint);

    if(commandPWM > 0){
        commandPWM = commandPWM + 7000;
    }

    if(commandPWM > 65530){
        commandPWM = 65530;
    }

    motor->runAtPWM(commandPWM);

    return commandPWM;
}

void MotorUnit::sendVoltage(double voltage){
    motor->runAtPWM(voltage);
}

/*!
 *  @brief  Runs the motor to extend for a little bit to put some slack into the coiled belt. Used to make it easier to extend.
 */
void MotorUnit::decompressBelt(){
    unsigned long time = millis();
    unsigned long elapsedTime = millis()-time;
    while(elapsedTime < 500){
        elapsedTime = millis()-time;
        motor->fullOut();
        updateEncoderPosition();
    }
}

/*!
 *  @brief  Sets the motor to comply with how it is being pulled
 */
bool MotorUnit::comply(unsigned long *timeLastMoved, double *lastPosition, double *amtToMove, double maxSpeed){
    
    //Update position and PID loop
    recomputePID();
    
    //If we've moved any, then drive the motor outwards to extend the belt
    float positionNow = getPosition();
    float distMoved = positionNow - *lastPosition;
    

    //If the belt is moving out, let's keep it moving out
    if( distMoved > .04){
        //Increment the target
        setTarget(positionNow + *amtToMove);
        
        *amtToMove = *amtToMove + 1;
        
        *amtToMove = min(*amtToMove, maxSpeed);
        
        //Reset the last moved counter
        *timeLastMoved = millis();
    
    //If the belt is moving in we need to stop it from moving in
    }else if(distMoved < -.04){
        *amtToMove = 0;
        setTarget(positionNow + .1);
        stop();
    }
    //Finally if the belt is not moving we want to spool things down
    else{
        *amtToMove = *amtToMove / 2;
        setTarget(positionNow);
        stop();
    }
    

    *lastPosition = positionNow;

    //Return indicates if we have moved within the timeout threshold
    if(millis()-*timeLastMoved > 5000){
        return false;
    }
    else{
        return true;
    }
}

/*!
 *  @brief  Fully retracts this axis and zeros it out or if it is already retracted extends it to the targetLength
 */
bool MotorUnit::retract(double targetLength){
    
    Serial.println("Retracting");
    int currentThreshold = 14;
    //Start pulling
    motor->fullIn();
    
    //Add a delay to wait for the inrush current to pass
    unsigned long time = millis();
    unsigned long elapsedTime = millis()-time;
    while(elapsedTime < 100){
        elapsedTime = millis()-time;
        updateEncoderPosition();
    }
    
    //Pull until taught
    while(true){
        
        updateEncoderPosition();
        //When taught
        if(motor->readCurrent() > currentThreshold){
            motor->stop();
            _webPrint(getPosition());
            zero();
            
            //If we hit the current limit immediately because there wasn't any slack we will extend
            elapsedTime = millis()-time;
            if(elapsedTime < 500){
                
                //Extend some belt to get things started
                decompressBelt();
                
                unsigned long timeLastMoved = millis();
                double lastPosition = getPosition();
                double amtToMove = 0.1;
                
                unsigned long lastTime = micros();
                double lastAngle = 150.0;
                int transitionTime = 0;
                while(getPosition() < targetLength){
                    //Check for timeout
                    if(!comply(&timeLastMoved, &lastPosition, &amtToMove, 100)){//Comply updates the encoder position and does the actual moving
                        
                        //Stop and return
                        setTarget(getPosition());
                        motor->stop();
                        
                        return false;
                    }
                    
                    // Delay without blocking
                    unsigned long time = millis();
                    unsigned long elapsedTime = millis()-time;
                    while(elapsedTime < 50){
                        elapsedTime = millis()-time;
                    }
                }
                
                //Position hold for 2 seconds to make sure we are in the right place
                setTarget(targetLength);
                time = millis();
                elapsedTime = millis()-time;
                while(elapsedTime < 500){
                    elapsedTime = millis()-time;
                    recomputePID();
                }
                
                motor->stop();
                return true;
            }
            else{
                return false;
            }
        }
    }
}




