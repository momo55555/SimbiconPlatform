#include "stdafx.h"

#include "Hingejoint.h"
#include <PUtils.h>

#include "ArticulatedRigidBody.h"

HingeJoint::~HingeJoint(void){

}

/**
	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	been read from an input file.
*/
void HingeJoint::readAxes(char* axes){
	if (sscanf(axes, "%lf %lf %lf", &a.x, &a.y, &a.z) != 3)
        return;

	a.toUnit();

}

/**
	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	have been read from an input file.
*/
void HingeJoint::readJointLimits(char* limits){
	if (sscanf(limits, "%lf %lf", &minAngle, &maxAngle) != 2)
        return;
	useJointLimits = true;
}


//FILE* fp = fopen("jointAng.txt", "w");


/**
	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child.
*/
void HingeJoint::fixAngularConstraint(const Quaternion& qRel){
	//make sure that the relative rotation between the child and the parent is around the a axis
	Vector3d axis = qRel.getV().toUnit();
	//this is the rotation angle around the axis above, which may not be the rotation axis
	double rotAngle = 2 * safeACOS(qRel.getS());
	//get the rotation angle around the correct axis now (we are not in the world frame now)
	double ang = axis.dotProductWith(a) * rotAngle;

/* DEBUG ONLY
	if (strcmp(parent->name, "lupperarm") == 0 || strcmp(parent->name, "rupperarm") == 0){
		fprintf(fp, "%lf\n", ang);
		fflush(fp);
	}
*/
	//and compute the correct child orientation
	child->state.orientation = parent->state.orientation * Quaternion::getRotationQuaternion(ang, a);
}
