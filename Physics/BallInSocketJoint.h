#pragma once

#include "Joint.h"

/*==============================================================================================================================================================*
 * This class implements a ball in socket joint type.                                                                                                           *
 *==============================================================================================================================================================*/
class BallInSocketJoint : public Joint{
friend class PhysX3World;
private:
/**
	Quantities that do not change
*/

	//BallInSocket joints are free to rotate about any axis. However, in order to be able to apply joint limits, we will 
	//identify some axis, about which we will place joint limits. In particular, we'll use a swing and twist decomposition
	//of the relative orientations between the two bodies of a hinge joint

	//these two axes define the plane of vectors along which the rotations represent a swing - stored in parent coordinates
	Vector3d swingAxis1, swingAxis2;
	//and this one is stored in child coordinates - this is the twist axis
	Vector3d twistAxis;
	//if there is no desired axis 2, it will be computed automatically from a cross product
	Vector3d desiredSwingAxis2;

	//and the min and max allowed angles along the two swing axes (define an ellipsoid that can be offset if the min/max angles are not equal in magnitude)
	double minSwingAngle1, maxSwingAngle1, minSwingAngle2, maxSwingAngle2;
	//and limits around the twist axis
	double minTwistAngle, maxTwistAngle;

	bool needJointLimitConstraintTwist, needJointLimitConstraintSwing;
	bool needSwingDamping;

	void updateAxis2IfNeeded() {
		// Sets the swingAxis2 to the cross product of swingAxis1 and twistAxis if they are specified
		if( desiredSwingAxis2.isZeroVector() &&
			!swingAxis1.isZeroVector() &&
			!twistAxis.isZeroVector() ) {

			swingAxis2 = swingAxis1.crossProductWith(twistAxis);
			swingAxis2.toUnit();
		}
	}

public:
	BallInSocketJoint() {}
	~BallInSocketJoint(void);

	/**
		This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child.
	*/
	virtual void fixAngularConstraint(const Quaternion& qRel);

	/**
		This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
		have been read from an input file.
	*/
	virtual void readJointLimits(char* limits);

	/**
		Set the joint limits
	*/
	void setJointLimits( double minSwingAngle1, double maxSwingAngle1, 
						 double minSwingAngle2, double maxSwingAngle2,
						 double minTwistAngle, double maxTwistAngle ) {

		this->minSwingAngle1 = minSwingAngle1;
		this->maxSwingAngle1 = maxSwingAngle1;
		this->minSwingAngle2 = minSwingAngle2;
		this->maxSwingAngle2 = maxSwingAngle2;
		this->minTwistAngle  = minTwistAngle;
		this->maxTwistAngle  = maxTwistAngle;

		useJointLimits = true;
	}

	/**
		This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
		been read from an input file.
	*/
	virtual void readAxes(char* axes);

	/**
		Sets the three axes
	*/
	void setAxes( const Vector3d& swingAxis1, const Vector3d& swingAxis2, const Vector3d& twistAxis ) {
		this->swingAxis1 = swingAxis1;
		this->swingAxis2 = swingAxis2;
		this->twistAxis = twistAxis;
		this->swingAxis1.toUnit();
		this->swingAxis2.toUnit();
		this->twistAxis.toUnit();
	}

	/**
		Sets a swing and a twist axis, the other swing axis is the cross product
	*/
	void setAxes( const Vector3d& swingAxis1, const Vector3d& twistAxis ) {
		setAxes( swingAxis1,
				 swingAxis1.crossProductWith(twistAxis),
				 twistAxis );
	}

	void setSwingAxis1( const Vector3d& swingAxis1 )
	{
		this->swingAxis1 = swingAxis1;
		this->swingAxis1.toUnit();
		updateAxis2IfNeeded();
	}

	const Vector3d& getSwingAxis1() const { return swingAxis1; }

	void setSwingAxis2( const Vector3d& swingAxis2 )
	{
		if( swingAxis2.length() < 0.001 ) {
			this->desiredSwingAxis2 = Vector3d(0,0,0);
			updateAxis2IfNeeded();
		}
		else {
			this->swingAxis2 = swingAxis2;
			this->swingAxis2.toUnit();
			this->desiredSwingAxis2 = desiredSwingAxis2;
		}
	}

	const Vector3d& getSwingAxis2() const { return swingAxis2; }

	void setTwistAxis( const Vector3d& twistAxis )
	{
		this->twistAxis = twistAxis;
		this->twistAxis.toUnit();
		updateAxis2IfNeeded();
	}

	const Vector3d& getTwistAxis() const { return twistAxis; }

	/**
	*/

	/**
		Returns the type of the current joint
	*/
	virtual int getJointType(){ return BALL_IN_SOCKET_JOINT;}

};
