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
        /**
        * @brief Constructor, makes a 0s 4x4 matrix
        * @return this
        */
        Mat4();
        /**
        * @brief Constructor, makes a 4x4 matrix starting from a square array
        * @param b[4][4] has to be a square array
        * @return this
        */
        Mat4(float** b);
        /**
        * @brief writes the input into this
        * @param matrix to write into this
        * @return this
        */
        Mat4 write(Mat4 b);    
        /**
        * @brief row by column multiplication between this and param
        * @param b second member of multiplication
        * @return this
        */                              
        Mat4 multiply(Mat4 b);
        /**
        * @brief writes this with identity 4x4 matrix
        * @return this
        */
        Mat4 generateIdentity(); 
        /**
         * @brief Writes the translation matrix around X axis on this
         * @param val lenght of translation
         * @return this
         */                          
        Mat4 generateTrnX(float val);
        /**
         * @brief Writes the translation matrix around Z axis on this
         * @param val lenght of translation
         * @return this
         */                                            
        Mat4 generateTrnZ(float val); 
        /**
         * @brief Writes the rotational matrix around X axis on this
         * @param val angle in rads
         * @return this
         */                                                  
        Mat4 generateRotX(float theta);
        /**
         * @brief Writes the rotational matrix around Z axis on this
         * @param val angle in rads
         * @return this
         */                                           
        Mat4 generateRotZ(float theta);
        /**
         * @brief multiplyes the 4 matrices in row and writes on this
         * @param RtZ matrix of rotation around Z
         * @param TrZ matrix of translation along Z
         * @param TrX matrix of translation along X
         * @param RtX matrix of rotation around X
         * @return this
         */                                          
        Mat4 multiply4DH(Mat4 RtZ, Mat4 TrZ, Mat4 TrX, Mat4 RtX);
        /**
        * @brief reads a cell
        * @param row number [0-3]
        * @param column number [0-3]
        * @return this
        */
        float readCell(uint8_t row, uint8_t column);
        /**
        * @brief writes a cell
        * @param row number [0-3]
        * @param column number [0-3]
        * @param value to write
        * @return this
        */
        Mat4 writeCell(uint8_t row, uint8_t column, float value);
        /**
        * @brief prints the matrix to terminal in a fancy layout
        * @return this
        */
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

