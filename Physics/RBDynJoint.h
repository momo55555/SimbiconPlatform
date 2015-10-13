#pragma once

/*-----------------------------------------------------------------------------------------------------------------------------------------------------*
 * This class provides an interface, and some of the common and necessary class members + methods that the joints used in the RBDyn library will need. *
 *-----------------------------------------------------------------------------------------------------------------------------------------------------*/
class RBDynJoint {
private:
	//this matrix is used to project quantities into a different manifold (i.e. a plane or on a line). Typically, this is
	//the plane or line along which there should be 0 relative orientation. For instance, for a hinge joint, rotation is only
	//permitted along a certain axis, so it shouldn't be allowed in the plane that the axis is perpendicular on.
	Matrix P;
	//this list of vectors is used to easily set up the P matrix. The entries in this vector represent the axis along which rotation should be constrained
	PODDynamicArray<Vector3d*> cVecs;

	//this is the location, in world coordinates, of the joint. It is located at the mid-point between the world coordinates of the
	//child and parent's joint poisition - this should really be the same point in world coordinates, otherwise we have drift problems!!!
	Point3d globalP;

	/**
		This method is used to set up the P matrix using the vectors in the constrained vector above.
	*/
	void setUpProjectionMatrix();

	/**
		This method is used to determine if we need to enforce joint limits, and which limits (for universal joints for instance)
		we need to enforce. The details for the return value are implemented on a per joint basis.
	*/
	virtual int getJointLimitsToEnforce(const Quaternion& qRel, const Vector3d &relAngVel) = 0;

	/**
		This method is used to give a rough estimate of the new relative positition between the connected rigid bodies, given the
		current orientation and relative angular velocity. This way we can see if a joint limit needs to be enforced or not.
	*/
	Quaternion getPredictedRelativeOrientation(const Quaternion& qRel, const Vector3d &relAngVel);

	/**
		This method returns true if there is a need for an angular impulse, false otherwise. 
	*/
	virtual bool needAngularImpulse() = 0;

	/**
		This method is used to prepare the proper quantities (i.e. P, the joint angles, etc) for the current joint. 
		The relative orientation and relative angular velocity are passed in so that we can predict 
		if joint limits need to be enforced or not.
	*/
	virtual void prepareAngularConstraint(const Quaternion& qRel, const Vector3d &relAngVel) = 0;

public:
	RBDynJoint(void);
	~RBDynJoint(void);
};
