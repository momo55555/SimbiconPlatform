#include "RBDynJoint.h"

RBDynJoint::RBDynJoint(void){

}

RBDynJoint::~RBDynJoint(void){

}



/**
	This method is used to determine if we need to enforce joint limits, and which limits (for universal joints for instance)
	we need to enforce. The details for the return value are implemented on a per joint basis.
*/
int Joint::getJointLimitsToEnforce(const Quaternion& qRel, const Vector3d &relAngVel){
	return 0;
}


/**
	This method is used to set up the P matrix using the vectors in the constrained vector above.
*/
void Joint::setUpProjectionMatrix(){
	if (cVecs.size()==0 || cVecs.size()>3)
		return;

	P.resizeTo(cVecs.size(), 3);
	for (int i=0;i<cVecs.size();i++)
		setMatrixRow(&P, *cVecs[i], i);
}