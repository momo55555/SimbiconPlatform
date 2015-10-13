#pragma once

#include <Point3d.h>
#include <Vector3d.h>
#include <Quaternion.h>

/*======================================================================================================================================================================*
 * This class acts as a container for the state information (position, orientation, velocity and angular velocity - all of them stored in world coordinates, about the  *
 * center of mass) of a rigid body.                                                                                                                                     *
 *======================================================================================================================================================================*/

class RBState
{
public:
	//NOTE: all the quantities here are in world coordinates

	// the position of the center of mass of the rigid body
	Point3d position;
	// its orientation
	Quaternion orientation;
	// the velocity of the center of mass
	Vector3d velocity;
	// and finally, the angular velocity about the center of mass
	Vector3d angularVelocity;
	
public:
	/**
		Default constructor - populate the data members using safe values..
	*/
	RBState(void);

	/**
		A copy constructor.
	*/
	RBState(const RBState& other);

	/**
		and a copy operator	
	*/
	RBState& operator = (const RBState& other);
	/**
		Default destructor.
	*/
	~RBState(void);
};
