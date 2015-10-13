#pragma once

#include <Vector3d.h>

class RBForceAccumulator
{
public:
    //this is the total net force acting on a body
    Vector3d netForce;
    //this is the total net torque (takes into account the torques due to forces as well)
    Vector3d netTorque;
public:
    RBForceAccumulator(void);
    ~RBForceAccumulator(void);
};
