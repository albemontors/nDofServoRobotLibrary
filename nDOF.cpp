#include "nDOF.h"

/* ===================================
            SERVO CLASS             
====================================*/

Servo::Servo(uint16_t servoIDlink, Adafruit_PWMServoDriver *driverlink){

    this->servoID = servoIDlink;
    this->driver = driverlink;
    this->servoMin = SERVOMIN;
    this->servoMax = SERVOMAX;
    this->usMin = USMIN;
    this->usMax = USMAX;
    this->offset = 0;
    this->k = 1.0f;

}

Servo::Servo(uint16_t servoIDlink, Adafruit_PWMServoDriver *driverlink, int offset, float k){

    this->servoID = servoIDlink;
    this->driver = driverlink;
    this->servoMin = SERVOMIN;
    this->servoMax = SERVOMAX;
    this->usMin = USMIN;
    this->usMax = USMAX;
    this->offset = offset;
    this->k = k;

}

uint16_t Servo::move(int angle){

    // scale to k factor
    angle *= k;

    // add angle offset
    angle += this->offset;

    // check if angle is in range
    if(angle < LOWERLIMIT || angle > UPPERLIMIT) return OUT_OF_RANGE;

    // evaluate pulselenght
    uint16_t pulse = (usMax*1.0f - usMin*1.0f) * ((angle*1.0f - servoMin*1.0f) / (servoMax*1.0f - servoMin*1.0f)) + usMin;

    // check pulse is in boundaries
    if(pulse < usMin || pulse > usMax) return EVALUATION_ERROR; // this should never happen, but safety first

    // write the value to the driver
    driver->setPWM(servoID, 0, pulse);
    return NO_ERROR;

}

/* ===================================
            link CLASS             
====================================*/

Link::Link(Servo* Joint, float alpha, float a, float d){
    this->Joint = Joint;
    this->alpha = alpha;
    this->d = d;
    this->a = a;
    
}

