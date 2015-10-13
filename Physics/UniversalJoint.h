#pragma once

#include <Physics/Joint.h>

/*======================================================================================================================================================================*
 * This class is used to implement a universal joint - angular impulses that allow only two degrees of freedom between the parent and the child must be computed.       *
 *======================================================================================================================================================================*/

class UniversalJoint : public Joint{
friend class ODEWorld;
friend class PhysXWorld;
friend class PhysX3World;
friend class BulletWorld;
friend class VortexWorld;
private:
	//This joint can only rotate about the vector a, that is stored in parent coordinates
	Vector3d a;
	//or about vector b that is stored in child coordinates
	Vector3d b;
	//and the min and max allowed angles (around a axis)
	double minAngleA, maxAngleA;
	//and around the b axis
	double minAngleB, maxAngleB;
	//keep track, at any point in the simulation, if we need to enforce the joint limits about either axis A or axis B
	bool needJointLimitConstraintA, needJointLimitConstraintB;

public:
	UniversalJoint() {}
	~UniversalJoint(void);

	/**
		This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
		been read from an input file.
	*/
	virtual void readAxes(char* axes);

	/**
		Sets the two axes
	*/
	void setAxes( const Vector3d& parentAxis, const Vector3d& childAxis ) {
		this->a = parentAxis;
		this->b = childAxis;
		this->a.toUnit();
		this->b.toUnit();
	}
	
	void setParentAxis( const Vector3d& parentAxis ) {
		this->a = parentAxis;
		this->a.toUnit();
	}

	const Vector3d& getParentAxis() const { return a; }

	void setChildAxis( const Vector3d& childAxis ) {
		this->b = childAxis;
		this->b.toUnit();
	}

	const Vector3d& getChildAxis() const { return b; }

	/**
		This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
		have been read from an input file.
	*/
	virtual void readJointLimits(char* limits);

	/**
		Set the joint limits
	*/
	void setJointLimits( double minAngleParent, double maxAngleParent, 
						 double minAngleChild, double maxAngleChild ) {

		this->minAngleA = minAngleParent;
		this->maxAngleA = maxAngleParent;
		this->minAngleB = minAngleChild;
		this->maxAngleB = maxAngleChild;

		useJointLimits = true;
	}

	/**
		This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child.
	*/
	virtual void fixAngularConstraint(const Quaternion& qRel);

	/**
		Return the A rotation axis
	*/
	inline Vector3d getRotAxisA(){return a;}

	/**
		Return the B rotation axis
	*/
	inline Vector3d getRotAxisB(){return b;}


	/**
		Returns the type of the current joint
	*/
	virtual int getJointType(){ return UNIVERSAL_JOINT;}

};