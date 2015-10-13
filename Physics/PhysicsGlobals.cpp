#include "stdafx.h"

#include "PhysicsGlobals.h"

//initialize all the parameters to some sensible values.

//give this a very high value so that we can use the scripted values in the rb specs for the value to use
double PhysicsGlobals::gravity = -9.81;//*/0.0;
Vector3d PhysicsGlobals::up = Vector3d(0, 1, 0);
