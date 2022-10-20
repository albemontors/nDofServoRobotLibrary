/* =======================================================

Library for control of servo motors robotic systems

======================================================== */

#ifndef _NDOF_H
#define _NDOF_H
#include <cstdint>
#endif

#include "Adafruit_PWMServoDriver.h"
#include "mbed.h"

#define SERVOMIN 10
#define SERVOMAX 170
#define USMIN  150 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150 #600
#define USMAX  525 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600 #2400
#define LOWERLIMIT 10
#define UPPERLIMIT 170

class Mat4 {
    public:
        Mat4();
        Mat4(float** b);
        Mat4 write(Mat4 b);                                  
        Mat4 multiply(Mat4 b);
        Mat4 generateIdentity();                           
        Mat4 generateTrnX(float val);                                        //generate translation along X
        Mat4 generateTrnZ(float val);                                        //generate translation along Z         
        Mat4 generateRotX(float theta);                                        //generate rotation around X
        Mat4 generateRotZ(float theta);                                       //generate rotation around Z
        Mat4 multiply4DH(Mat4 TrZ, Mat4 RtZ, Mat4 RtX, Mat4 TrX);
        float readCell(uint8_t row, uint8_t column);
        Mat4 writeCell(uint8_t row, uint8_t column, float value);
        Mat4 verbose();

    private:
        float a[4][4];
};

class Servo {

    public:
        Servo(uint16_t servoIDlink, Adafruit_PWMServoDriver *driverlink);
        Servo(uint16_t servoIDlink, Adafruit_PWMServoDriver *driverlink, int offset, float k);
        uint16_t move(int angle);

        enum errorCode {NO_ERROR, OUT_OF_RANGE, EVALUATION_ERROR};

    private:
        Adafruit_PWMServoDriver *driver;
        uint16_t servoID;
        uint16_t servoMin;
        uint16_t servoMax;
        uint16_t usMin;
        uint16_t usMax;
        int offset;
        float k;

};

class Link{
    public:
        Link(Servo* Joint, float alpha, float a, float d);
        uint16_t move(float theta);

    private:
        Servo* Joint;
        float alpha;
        float a;
        float d;
        Mat4 dh;
        float theta;

};

