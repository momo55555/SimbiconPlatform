#pragma once

#include <Vector3d.h>

/**
	This class is used as a container for all the constants that are pertinent for the physical simulations, the controllers, etc.
*/

class PhysicsGlobals {
public:
	//We will assume that the gravity is in the y-direction (this can easily be changed if need be), and this value gives its magnitude. 
	static double gravity;
	static Vector3d up;

};
