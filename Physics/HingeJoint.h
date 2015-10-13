#pragma once

#include "Joint.h"


/*======================================================================================================================================================================*
 * This class is used to implement a hinge joint - angular impulses that allow relative rotation between the parent and the child only around a given axis must be      *
 * computed.                                                                                                                                                            *
 *======================================================================================================================================================================*/
class Joint;
class  HingeJoint : public Joint{
friend class ODEWorld;
friend class PhysXWorld;
friend class PhysX3World;
friend class BulletWorld;
friend class VortexWorld;
private:
/**
	Quantities that do not change
*/
	//This joint only allows relative motion about axis a - stored in parent coordinates
	Vector3d a;
	//keep track of the joint limits as well - min and max allowed angles around the rotation axis
	double minAngle;
	double maxAngle;

	//this variable, updated at every sim step indicates wether there is a need for a joint limit constraint or not
	bool needJointLimitConstraint;

public:
	HingeJoint() {}
	virtual ~HingeJoint(void);

	/**
		This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
		been read from an input file.
	*/
	virtual void readAxes(char* axes);

	/**
		Sets the axis
	*/
	void setAxis( const Vector3d& axis ) {
		a = axis;
		a.toUnit();
	}

	const Vector3d& getAxis() const { return a; }

	/**
		This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
		have been read from an input file.
	*/
	virtual void readJointLimits(char* limits);

	/**
		Set the joint limits
	*/
	void setJointLimits( double minAngle, double maxAngle ) {
		this->minAngle = minAngle;
		this->maxAngle = maxAngle;
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
		Returns the type of the current joint
	*/
	virtual int getJointType(){return HINGE_JOINT;}

};
