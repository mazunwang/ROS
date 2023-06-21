#ifndef _QUADRUPED_H_
#define _QUADRUPED_H_

#include<iostream>

class Quadruped
{
private:

public:
    double bodyLength,bodyWidth,bodyMass;
    double HipXLength,HipYLength,KneeLength;
    double HipXMass,HipYMass,KneeMass;
    double jointFriction,jointDamping;
    Quadruped(/* args */){

    }
    ~Quadruped(){
        
    }
};




#endif