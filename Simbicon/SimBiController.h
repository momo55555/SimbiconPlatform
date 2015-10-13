/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "PoseController.h"
#include "BaseControlFramework.h"
#include <PUtils.h>
#include <RigidBody.h>
#include "SimBiConState.h"


/**
	This structure is used to store the state of a simbicon controller. It can be used to save/load a controller's
	states, where the state here does not refer to the states in the Finite State Machine. The state refers to the
	phase in the current FSM state, and also the stance.
*/
typedef struct {
	int stance;
	double phi;
	int FSMStateIndex;
	bool bodyGroundContact;
} SimBiControllerState;


/**
 * A simbicon controller is a fancy PoseController. The root (i.e. pelvis or torso), as well as the two hips are controlled
 * relative to a semi-global coordinate frame (it needs only have the y-axis pointing up), but all other joints are controlled
 * exactely like in a normal PoseController. The SimBiController operates on a biped, which means that we can make special
 * assumptions about it: it must have two joints, lHip and rHip, connecting the root to the left and right upper-legs,
 * and it must also have two feet (lFoot and rFoot) as rigid bodies in the articulated linkage.
 */

class SimBiController : public PoseController{
friend class ConCompositionFramework;
friend class SimbiconPlayer;
friend class ControllerEditor;
private:
/**
	These are quantities that are set only once
*/
	//we will keep a reference to the left and right feet to be able to determine when the stance switches
	RigidBody* lFoot;
	RigidBody* rFoot;
	//we will also keep a reference to the root of the figure, to be able to identify the semi-global coordinate frame quickly
	RigidBody* root;
	//we also need to keep track of the joint indices in the articulated figure for the two hips, since they are special
	int lHipIndex;
	int rHipIndex;
	//this is a collection of the states that are used in the controller
	DynamicArray<SimBiConState*> states;

	//the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
	//while the pose controller does not use these values, other types of controllers may need this information
	ControlParams rootControlParams;

	double stanceHipDamping;
	double stanceHipMaxVelocity;
	double rootPredictiveTorqueScale;


/**
	these are quantities that get updated throughout the simulation
*/
	//this value indicates which side is the stance side. 
	int stance;
	//a pointer to the swing and stance feet
	RigidBody* stanceFoot;
	RigidBody* swingFoot;
	//keep track of the swing and stance hip indices
	int stanceHipIndex;
	int swingHipIndex;
	//this is the index of the controller that is currently active
	int FSMStateIndex;

	//this is the vector from the cm of the stance foot to the cm of the character
	Vector3d d;
	//this is the velocity of the cm of the character
	Vector3d v;


	//the phase parameter, phi must have values between 0 and 1, and it indicates the progress through the current state.
	double phi;

	//this quaternion gives the current heading of the character. The complex conjugate of this orientation is used
	//to transform quantities from world coordinates into a rotation/heading-independent coordinate frame (called the character frame).
	//I will make the asumption that the character frame is a coordinate frame that is aligned with the vertical axis, but has 0 heading, and
	//the characterFrame quaternion is used to rotate vectors from the character frame to the real world frame
	Quaternion characterFrame;

	//this variable, updated everytime the controller state is advanced in time, is set to true if any body other than the feet are in contact
	//with the ground, false otherwise. A higer level process can determine if the controller failed or not, based on this information.
	bool bodyTouchedTheGround;

	/**
		This method is used to parse the information passed in the string. This class knows how to read lines
		that have the name of a joint, followed by a list of the pertinent parameters. If this assumption is not held,
		then classes extended this one are required to provide their own implementation of this simple parser
	*/
	virtual void parseGainLine(char* line);

	/**
		This method is used to set the current FSM state of the controller to that of the index that
		is passed in.
	*/
	void setFSMStateTo(int index);

	/**
		This method should be called when the controller transitions to this state.
	*/
	void transitionToState(int stateIndex);

	/**
		This method returns the net force on the body rb, acting from the ground
	*/
	Vector3d getForceOn(RigidBody* rb, DynamicArray<ContactPoint> *cfs);

	/**
		This method returns the net force on the body rb, acting from the ground
	*/
	Vector3d getForceOnFoot(RigidBody* foot, DynamicArray<ContactPoint> *cfs);

	/**
		This method is used to determine if the rigid body that is passed in as a parameter is a
		part of a foot
	*/
	bool isFoot(RigidBody* rb);

	/**
		This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
	*/
	bool isStanceFoot(RigidBody* rb);

	/**
		This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
	*/
	bool isSwingFoot(RigidBody* rb);

	/**
		This method is used to return the ratio of the weight that is supported by the stance foot.
	*/
	double getStanceFootWeightRatio(DynamicArray<ContactPoint> *cfs);

	/**
		This method is used to compute the torques that need to be applied to the stance and swing hips, given the
		desired orientation for the root and the swing hip. The coordinate frame that these orientations are expressed
		relative to is computed in this method. It is assumed that the stanceHipToSwingHipRatio variable is
		between 0 and 1, and it corresponds to the percentage of the total net vertical force that rests on the stance
		foot.
	*/
	void computeHipTorques(const Quaternion& qRootD, const Quaternion& qSwingHipD, double stanceHipToSwingHipRatio);

	/**
		This method is used to resolve the names (map them to their index) of the joints
	*/
	void resolveJoints(SimBiConState* state);

	/**
		This method is used to set the stance 
	*/
	void setStance(int newStance);


	/**
		This method is used to return a pointer to a rigid body, based on its name and the current stance of the character
	*/
	RigidBody* getRBBySymbolicName(char* sName);

public:
	//keep a copy of the initial character state, starting stance and file that contains the character's initial state
	int startingState;
	int startingStance;
	char initialBipState[100];

public:
	/**
		Default constructor
	*/
	SimBiController(Character* b);

	/**
		Destructor
	*/
	virtual ~SimBiController(void);

	/**
		This method is used to compute the torques that are to be applied at the next step.
	*/
	virtual void computeTorques(DynamicArray<ContactPoint> *cfs);

	/**
		This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
		used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
		or the index of the state that it transitions to otherwise.
	*/
	int advanceInTime(double dt, DynamicArray<ContactPoint> *cfs);

	/**
		This method is used to populate the structure that is passed in with the current state
		of the controller;
	*/
	void getControllerState(SimBiControllerState* cs);

	/**
		This method is used to populate the state of the controller, using the information passed in the
		structure
	*/
	void setControllerState(const SimBiControllerState &cs);
	
	/**
		This method loads all the pertinent information regarding the simbicon controller from a file.
	*/
	void loadFromFile(char* fName);

	/**
		This method is used to return the value of bodyGroundContact
	*/
	inline bool isBodyInContactWithTheGround()
    {
		return bodyTouchedTheGround;
	}

	/**
		This method is used to return the value of the phase (phi) in the current FSM state.
	*/
	inline double getPhase()
    {
		return phi;
	}

	/**
		This method returns the position of the CM of the stance foot, in world coordinates
	*/
	inline Point3d getStanceFootPos()
    {
		if (stanceFoot)
			return stanceFoot->getCMPosition();
		return Point3d(0,0,0);
	}

	/**
		This method returns the position of the CM of the swing foot, in world coordinates
	*/
	inline Point3d getSwingFootPos(){
		if (swingFoot)
			return swingFoot->getCMPosition();
		return Point3d(0,0,0);
	}

	/**
		This method is used to write the current controller to a file
	*/
	void writeControllers(const char fileName[]);

	/**
		This method is used to return the current state number
	*/
	inline int getFSMState(){
		return this->FSMStateIndex;
	}

	/**
		This method returns the character frame orientation
	*/
	Quaternion getCharacterFrame(){
		return characterFrame;
	}

	/**
		This method is used to update the d and v parameters, as well as recompute the character coordinate frame.
	*/
	void updateDAndV();

	/**
		This makes it possible to externally access the states of this controller
		Returns null if the index is out of range
	 */
	SimBiConState* getState( uint idx );

	/**
		This method makes it possible to evaluate the debug pose at any phase angle
		Negative phase angle = Use the current controller phase angle
	*/
	void updateTrackingPose(DynamicArray<double>& trackingPose, double phiToUse = -1);

	/**
		this method returns the stance of the character
	*/
	inline int getStance(){
		return stance;
	}

	// Evaluate the D trajectory
	inline void computeD0( double phi, Vector3d* d0 ) 
    {
		SimBiConState* currState = states[getFSMState()];
		computeDorV( phi, currState->dTrajX, currState->dTrajZ, stance, d0 );
	}


	// Evaluate the V trajectory 
	inline void computeV0( double phi, Vector3d* v0 ) 
    {
		SimBiConState* currState = states[getFSMState()];
		computeDorV( phi, currState->vTrajX, currState->vTrajZ, stance, v0 );
	}


	// Evaluate the V trajectory 
	inline static void computeDorV( double phi, Trajectory1D* trajX, Trajectory1D* trajZ, int stance, Vector3d* result ) {
		result->y = 0;
		double signReverse = (stance == RIGHT_STANCE)?-1:1;
		if( trajX == NULL )
			result->x = 0;
		else
			result->x = trajX->evaluate_catmull_rom( phi ) * signReverse;
		if( trajZ == NULL )
			result->z = 0;
		else
			result->z = trajZ->evaluate_catmull_rom( phi );
	}

};
