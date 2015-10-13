#include "stdafx.h"
#include "PreCollisionQuery.h"

PreCollisionQuery::PreCollisionQuery(void){

}


PreCollisionQuery::~PreCollisionQuery(void){

}

/**
	This method returns true if the pair of rigid bodies here should be checked for collisions, false otherwise.
	Note that the ground does not have a rigid body associated with it, so if a rigid body is NULL, it means it
	corresponds to the ground. The joined variable is set to true if the two bodies are connected by a joint,
	false otherwise.
*/
bool PreCollisionQuery::shouldCheckForCollisions(RigidBody* rb1, RigidBody* rb2, bool joined){
	//we do not want to check the objects for collisions if they are connected by a joint (joint limits should be used for that).
	if (joined == true)
		return false;

	//don't allow collisions between static things
	if (rb1->isLocked() && rb2->isLocked())
		return false;

	if (rb1->getAFParent() != NULL)
		if (rb1->getAFParent() == rb2->getAFParent())
			return false;
	return true;
}

