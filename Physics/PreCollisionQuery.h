#pragma once

#include <PUtils.h>

#include "RigidBody.h"


/**
 *	This class provides an interface for classes that need to interact with a physics simulator, in order to control its behaviour. The methods here 
 *  will be called by the simulator at certain points (for instance, when determining if collisions need to be performed between two arbitrary objects). 
 *  It is up to the class that implements this interface to handle each method accordingly. 
 */
class PreCollisionQuery{
protected:
	
public:
	PreCollisionQuery(void);
	virtual ~PreCollisionQuery(void);

	/**
		This method returns true if the pair of rigid bodies here should be checked for collisions, false otherwise.
		Note that the ground does not have a rigid body associated with it, so if a rigid body is NULL, it means it
		corresponds to the ground. The joined variable is set to true if the two bodies are connected by a joint,
		false otherwise.
	*/
	virtual bool shouldCheckForCollisions(RigidBody* rb1, RigidBody* rb2, bool joined);

};
