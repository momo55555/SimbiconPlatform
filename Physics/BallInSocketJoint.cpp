#include "stdafx.h"

#include "BallInSocketJoint.h"
#include <PUtils.h>

#include "ArticulatedRigidBody.h"

#define ANGLE_A_CONSTRAINT		1
#define ANGLE_B_CONSTRAINT		2


BallInSocketJoint::~BallInSocketJoint(void){

}

//FILE* fp = fopen("jointAng.txt", "w");

/**
	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
	the orientation of the child.
*/
void BallInSocketJoint::fixAngularConstraint(const Quaternion& qRel){


	//nothing to fix here (well maybe joint limits at some point)

/**  DEBUG only
	angleB = decomposeRotation(qRel, b, &angleA, &a);


	if (strcmp(this->joint->child->name, "torso") == 0){
		fprintf(fp, "%lf\t%lf\n", angleA, angleB);
		fflush(fp);
	}
*/
}

/**
	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	been read from an input file.
*/
void BallInSocketJoint::readAxes(char* axes){
	if (sscanf(axes, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",&swingAxis1.x, &swingAxis1.y, &swingAxis1.z, &swingAxis2.x, &swingAxis2.y, &swingAxis2.z, &twistAxis.x, &twistAxis.y, &twistAxis.z) != 9){
		if (sscanf(axes, "%lf %lf %lf %lf %lf %lf",&swingAxis1.x, &swingAxis1.y, &swingAxis1.z, &twistAxis.x, &twistAxis.y, &twistAxis.z) != 6){
            return;
		}
		else
			swingAxis2 = swingAxis1.crossProductWith(twistAxis);
	} else {
		desiredSwingAxis2 = swingAxis2;
		desiredSwingAxis2.toUnit();
	}
	swingAxis1.toUnit();
	swingAxis2.toUnit();
	twistAxis.toUnit();
}

/**
	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	have been read from an input file.
*/
void BallInSocketJoint::readJointLimits(char* limits){
	int n = sscanf(limits, "%lf %lf %lf %lf %lf %lf", &minSwingAngle1, &maxSwingAngle1, &minSwingAngle2, &maxSwingAngle2, &minTwistAngle, &maxTwistAngle);
	if (n!= 6)
    {
        return;
    }

	useJointLimits = true;
}
