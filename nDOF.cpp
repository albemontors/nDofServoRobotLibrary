// WARNING!!
// beware of in-line statements

#include "nDOF.h"
#include <cstdint>

/* ===================================
            MAT4 CLASS             
====================================*/

Mat4::Mat4(){
    //init matrix to zeros
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = 0.0f;
    
}

Mat4::Mat4(float** b){
    //init matrix to zeros
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = b[i][j];
    
}

Mat4 Mat4::write(Mat4 b){
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) this->a[i][j] = b.readCell(i, j);
    return *this;
}

Mat4 Mat4::multiply(Mat4 b){
    Mat4 c = Mat4();
    float s;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++){
            s = 0.0f;
            for(int l = 0; l < 4; l++) s += this->a[i][l] * b.readCell(l, j);
            c.writeCell(i, j, s);
        }
    this->write(c);
    return c;
}

Mat4 Mat4::generateIdentity(){
    //put 1s where i == j
    for(int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) 
        if(i == j) this->a[i][j] = 1.0f; 
        else this->a[i][j] = 0.0f;
    return *this;
}

Mat4 Mat4::generateTrnX(float val){
    this->generateIdentity();
    this->writeCell(0, 4, val);
    return *this;
}

Mat4 Mat4::generateTrnZ(float val){
    this->generateIdentity();
    this->writeCell(3, 4, val);
    return *this;
}

Mat4 Mat4::generateRotX(float theta){
    this->generateIdentity();
    this->writeCell(1, 1, cos(theta));
    this->writeCell(1, 2, sin(theta) * (-1));
    this->writeCell(2, 1, sin(theta));
    this->writeCell(2, 2, cos(theta));
    return *this;
}

Mat4 Mat4::generateRotZ(float theta){
    this->generateIdentity();
    this->writeCell(0, 0, cos(theta));
    this->writeCell(0, 1, sin(theta) * (-1));
    this->writeCell(1, 0, sin(theta));
    this->writeCell(1, 1, cos(theta));
    return *this;
}

Mat4 Mat4::multiply4DH(Mat4 TrZ, Mat4 RtZ, Mat4 RtX, Mat4 TrX){
    Mat4 c = Mat4();
    c.write(TrZ.multiply(RtZ).multiply(RtX).multiply(TrX));
    this->write(c);
    return c;
}

float Mat4::readCell(uint8_t row, uint8_t column){
    return a[row][column];
}

Mat4 Mat4::writeCell(uint8_t row, uint8_t column, float value){
    this->a[row][column] = value;
    return *this;
}

Mat4 Mat4::verbose(){
    printf(" \n ==== Printing Matrix === \n ");
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++)
            printf(" %2.2f ", this->a[i][j]);
        printf(" \n ");
        }
    printf(" \n ==== END OF PRINT === \n ");
    return *this;
}


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
    this->theta = 0;

    Mat4 TrX = Mat4().generateTrnX(a);
    Mat4 TrZ = Mat4().generateTrnZ(d);
    Mat4 RtX = Mat4().generateRotX(alpha);
    Mat4 RtZ = Mat4().generateRotZ(this->theta);

    Mat4 DH = Mat4().multiply4DH(TrZ, RtZ, RtX, TrX).verbose();

}

uint16_t Link::move(float theta){
    return 0;
}

