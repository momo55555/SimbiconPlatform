#include "stdafx.h"

#include "UniversalJoint.h"

#include "ArticulatedRigidBody.h"


#define ANGLE_A_CONSTRAINT		1
#define ANGLE_B_CONSTRAINT		2


UniversalJoint::~UniversalJoint(void){

}



/**
	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	been read from an input file.
*/
void UniversalJoint::readAxes(char* axes){
	if (sscanf(axes, "%lf %lf %lf %lf %lf %lf", &a.x, &a.y, &a.z, &b.x, &b.y, &b.z) != 6)
        return;
	a.toUnit();
	b.toUnit();
}

/**
	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	have been read from an input file.
*/
void UniversalJoint::readJointLimits(char* limits){
    if (sscanf(limits, "%lf %lf %lf %lf", &minAngleA, &maxAngleA, &minAngleB, &maxAngleB)!=4)
        return;
    else
        useJointLimits = true;
}

//FILE* fp = fopen("jointAng.txt","w");

/**
	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child.
*/
void UniversalJoint::fixAngularConstraint(const Quaternion& qRel){
	//to go from the child's coord frame to its parent, first rotate around the axis b, then around the axis a.

	//compute two rotations, such that qRel = tmpQ1 * tmpQ2, and tmpQ2 is a rotation about the vector b (expressed in child coordinates)
	qRel.decomposeRotation(&tmpQ1, &tmpQ2, b);

	//now make sure that tmpQ1 represents a rotation about axis a (expressed in parent coordinates)
	double angA = tmpQ1.getRotationAngle(a);
	tmpV1 = tmpQ1.v;
	tmpV1.toUnit();
	double mod = tmpV1.dotProductWith(a);
	if (mod < 0) mod = -mod;
	angA *= mod;
	child->state.orientation = parent->state.orientation * Quaternion::getRotationQuaternion(angA, a) * tmpQ2;
}
