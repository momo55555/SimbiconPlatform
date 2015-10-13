#pragma once

#include <Matrix.h>
#include <TransformationMatrix.h>

/*===================================================================================================================================================================*
 * This class represents a container for the physical properties of a rigid body. A rigid body is characterized by its mass, and its inertia matrix. For this class  *
 * we will be storing these properties, as well as their inverses, as they are the ones that we will actually use when we simulate the dynamics of the rigid body.   *
 *===================================================================================================================================================================*/

class RBProperties
{
public:
	//the mass
	double mass;
	//we'll store the moment of inertia in a vector that represents the diagonal of the local frame MOI
	Vector3d MOI_local;
	//we'll also compute the inverse MOI, in local coordinates
	Vector3d invMOI_local;
	//and in this matrix we will keep the world relative inverse moment of inertia
	double invI[9];
	//keep a copy of the world-coordinates moment of intertia I
	double I[9];
	//and keep a copy of the inverse mass as well, for easy access
	double invMass;

	//we will also store the coefficient of restitution
	double epsilon;
	//and the coefficient of friction
	double mu;
	//this variable indicates wether or not this rigid body is fixed. When setting this to true/false, the inverse mass and inverse moment of inertia
	//will be set to zero (for true), or to the proper inverses otherwise. 
	bool isLocked;

	//these are a few specific properties (that make sense with ODE - don't know how they would match up with other simulators)
	double groundSoftness;
	double groundPenalty;

	//if this method is set to true, then the body is constrained to be a planar object (move in the x-y plane and only rotate about the z-axis)
	bool isPlanar;

public:
	/**
		default constructor.
	*/
	RBProperties();

	/**
		default destructor.
	*/
	~RBProperties();

	/**
		sets the mass of the rigid body
	*/
	void setMass(double m);

	/**
		set the moment of inertia of the rigid body. The three variables represent to the x, y and z principal moments.
	*/
	void setMOI(double xM, double yM, double zM);

	const Vector3d& getMOI() const { return MOI_local; }

	/**
		This method sets the rigid body's state as fixed.
	*/
	void lockBody( bool isLocked = true );

	/**
		This method sets the rigid body's state as not fixed.
	*/
	void releaseBody();

	inline void getInverseLocalMOI(Vector3d* res){
		res->setValues(invMOI_local.x, invMOI_local.y, invMOI_local.z);
	}

	inline void getLocalMOI(Vector3d* res){
		res->setValues(MOI_local.x, MOI_local.y, MOI_local.z);
	}

};