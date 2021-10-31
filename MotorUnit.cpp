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
 *  @param  senseResistor Value in Ohms of the sense resistor for this channel
 *  @param  cal ESP32 adc calibration results for more accurate readings
 *  @param  angleCS ESP32 pin for the chip select of the angle sensor
 *
 */
MotorUnit::MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS,
               double mmPerRev,
               double desiredAccuracy,
               void (*webPrint) (double arg1)){
    _mmPerRevolution = mmPerRev;
    accuracy = desiredAccuracy;
    pid.reset(new MiniPID(p,i,d));
    pid->setOutputLimits(-65535,65535);
    motor.reset(new DRV8873LED(tlc, forwardPin, backwardPin, readbackPin, senseResistor, cal));
    angleSensor.reset(new AS5048A(angleCS));
    angleSensor->init();
    _webPrint = webPrint;
    
    zero();
}

/*!
 *  @brief  Resets the axis position to zero
 */
void MotorUnit::zero(){
    angleTotal = 0;
    
    angleCurrent  = angleSensor->getRawRotation();
    anglePrevious = angleSensor->getRawRotation();
}

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
 *  @brief  Sets the position of the cable
 */
int MotorUnit::setPosition(double newPosition){
    
    angleCurrent  = angleSensor->RotationRawToAngle(angleSensor->getRawRotation());
    anglePrevious = angleSensor->RotationRawToAngle(angleSensor->getRawRotation());
    
    angleTotal = newPosition*16384;
}

/*!
 *  @brief  Stops the motor
 */
void MotorUnit::stop(){
    motor->stop();
}

/*!
 *  @brief  Runs the motor out at full speed
 */
void MotorUnit::fullOut(){
    motor->fullOut();
}

/*!
 *  @brief  Reads the encoder value and updates it's position
 */
void MotorUnit::updateEncoderPosition(){
    angleCurrent = angleSensor->getRawRotation();
    angleSensor->AbsoluteAngleRotation(&angleTotal, &angleCurrent, &anglePrevious);
}

/*!
 *  @brief  Recomputes the PID and drives the output
 */
int MotorUnit::recomputePID(){
    
    updateEncoderPosition();
    
    //Read the motor current and check for stalls
    double currentNow = getCurrent();
    if(currentNow > _stallCurrent){
        _stallCount = _stallCount + 1;
    }
    else{
        _stallCount = 0;
    }
    // if(_stallCount > _stallThreshold){
        // _webPrint(currentNow);
    // }
    
    int commandSpeed = pid->getOutput(getPosition(),setpoint);
    
    motor->runAtPWM(commandSpeed);
}

/*!
 *  @brief  Computes and returns the error in the axis positioning
 */
double MotorUnit::getError(){
    
    double errorDist = setpoint - getPosition();
    
    return errorDist;
    
}

/*!
 *  @brief  Sets the motor to comply with how it is being pulled
 */
void MotorUnit::repeatabilityTest(){
    
    //Start off from a known location
    setPosition(0);
    setTarget(0);
    
    while(true){
        
        if(abs(getError()) <.05){
            if(getTarget() == 0){
                
                //Pause to rest and check alignment
                unsigned long time = millis();
                unsigned long elapsedTime = millis()-time;
                while(elapsedTime < 5000){
                    elapsedTime = millis()-time;
                    recomputePID();
                }
                setTarget(400);
            }
            else{
                setTarget(0);
            }
        }
        
        unsigned long time = millis();
        unsigned long elapsedTime = millis()-time;
        while(elapsedTime < 10){
            elapsedTime = millis()-time;
            recomputePID();
        }
        
        Serial.println(getError());
    }
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
    
    if( distMoved > .01){
        //Increment the target
        setTarget(positionNow + *amtToMove);
        
        *amtToMove = *amtToMove + 0.1;
        
        *amtToMove = min(*amtToMove, maxSpeed);
        
        //Reset the last moved counter
        *timeLastMoved = millis();
        
        *lastPosition = positionNow;
        
    }else{
        
        //Prevent creep if the position is slightly overshot and there is no pulling force
        if(getTarget() > getPosition()){
            setTarget(getPosition());
        }
        *amtToMove = *amtToMove/2;  //Spool down...this leaves some slack in the cable
        
        //Don't allow position to go too negative which makes it hard to start again
        if(distMoved < -0.05){
            *lastPosition = positionNow + 0.05;
        }
        
        //If we haven't moved in more time than the threshold then return
        if(millis()-*timeLastMoved > 5000){
            return false;
        }
    }
    
    return true;
}

/*!
 *  @brief  Fully retracts this axis and zeros it out or if it is already retracted extends it to the targetLength
 */
bool MotorUnit::retract(double targetLength){
    
    int currentThreshold = 18;
    
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
                        
                        //Do linearizion if we are at constant speed
                        // if(amtToMove > 10){
                            // linearize(&lastAngle, &lastTime, &transitionTime);
                        // }
                    }
                }
                
                //Position hold for 2 seconds to make sure we are in the right place
                setTarget(targetLength);
                time = millis();
                elapsedTime = millis()-time;
                while(elapsedTime < 2000){
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

/*!
 *  @brief  Linearizes the internal encoder
 */
void MotorUnit::linearize(double *lastAngle, unsigned long *lastTime, int *transitionTime){
    
    double currentAngle = angleSensor->RotationRawToAngle(angleSensor->getRawRotation());
    
    if(*lastAngle > 200.0 && currentAngle < 100.0){
        *transitionTime = micros() - *lastTime;
        *lastTime = micros();
    }
    
    if(*transitionTime > 200){
        double expectedAngle = 360.0*((double)(micros() - *lastTime) / (double) *transitionTime);
        
        Serial.print(" * ");
        Serial.print(currentAngle);
        Serial.print(", ");
        Serial.print(expectedAngle);
        Serial.print(", ");
        Serial.print(micros() - *lastTime);
        // Serial.print(", ");
        // Serial.print(currentAngle - expectedAngle);
        Serial.println(" ");
    }
    
    *lastAngle = currentAngle;
}



