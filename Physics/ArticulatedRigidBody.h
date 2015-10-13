#pragma once


#include "RigidBody.h"
#include "Joint.h"

/*=======================================================================================================================================================================*
 * We will treat the articulated rigid bodies as normal rigid bodies that are connected by joints. The joints are needed to enforce constraints between the articulated  *
 * rigid bodies, but other than that, the dynamics are the same as for rigid bodies. We will assume that every articulated figure will be loop-free (tree hierarchies).  *
 *=======================================================================================================================================================================*/
class ArticulatedRigidBody : public RigidBody{
friend class Joint;
friend class ArticulatedFigure;
friend class SimBiController;
friend class VirtualModelController;
friend class IKVMCController;

friend class TestApp;
private:
	//this is the parent joint.
	Joint* pJoint;
	//and these are the child joints - it can have as many as it wants.
	DynamicArray<Joint*> cJoints;
	//this is the articulated figure that the rigid body belongs to
	ArticulatedFigure* AFParent;
public:
	/**
		Default constructor
	*/
	ArticulatedRigidBody(void);

	/**
		This method draws the current rigid body.
	*/

	/**
		Default destructor
	*/
	virtual ~ArticulatedRigidBody(void);

	/**
		returns the parent joint for the current articulated body
	*/
	inline Joint* getParentJoint(){return pJoint;};

	inline int getChildJointCount() const { return cJoints.size(); }
	inline Joint* getChildJoint(int i) { return cJoints[i]; }

	/**
		this method always returns true
	*/
	virtual bool isArticulated(){
		return true;
	}

	void setAFParent(ArticulatedFigure* parent){
		AFParent = parent;
	}

	ArticulatedFigure* getAFParent(){
		return AFParent;
	}


};
