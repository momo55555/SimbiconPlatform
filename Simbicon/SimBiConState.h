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

#include "Trajectory.h"
#include "BalanceFeedback.h"
#include <PUtils.h>
#include <Vector3d.h>
#include <Quaternion.h>
#include "ConUtils.h"
#include "SimGlobals.h"

#include <fstream>



/**
 *  This helper class is used to hold information regarding one component of a state trajectory. This includes (mainly): the base trajectory, 
 *	a data member that specifies the feedback law to be used, and the axis about which it represents a rotation, 
 */
class TrajectoryComponent
{
public:
	//this is the array of basis functions that specify the trajectories for the sagittal plane.
	Trajectory1D baseTraj;
	//if this variable is set to true, then when the stance of the character is the left side, the 
	//static target provided by this trajectory should be negated
	bool reverseAngleOnLeftStance;
	//if this variable is set to true, then when the stance of the character is the right side, the 
	//static target provided by this trajectory should be negated
	bool reverseAngleOnRightStance;
	//this is the rotation axis that the angles obtained from the trajectory represent rotations about
	Vector3d rotationAxis;

	//this is the balance feedback that is to be used with this trajectory
	LinearBalanceFeedback* bFeedback;

	//this is the base value for the trajectory
	double offset;

	/**
		default constructor
	*/
	TrajectoryComponent(){
		rotationAxis = Vector3d();
		reverseAngleOnLeftStance = false;
		reverseAngleOnRightStance = false;
		bFeedback = NULL;
		offset = 0;
	}

	/**
		default destructor.
	*/
	~TrajectoryComponent(){
		delete bFeedback;
	}

	/**
		this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
		and the d and v values used for feedback.
	*/
	inline Quaternion evaluateTrajectoryComponent(SimBiController* con, Joint* j, int stance, double phi, const Vector3d& d, const Vector3d& v){
		double baseAngle = offset;
		if (baseTraj.getKnotCount() > 0)
			baseAngle += baseTraj.evaluate_catmull_rom(phi);

		if (stance == LEFT_STANCE && reverseAngleOnLeftStance)
			baseAngle = -baseAngle;
		if (stance == RIGHT_STANCE && reverseAngleOnRightStance)
			baseAngle = -baseAngle;

		double feedbackValue = computeFeedback(con, j, phi, d, v);

		return Quaternion::getRotationQuaternion(baseAngle + feedbackValue, rotationAxis);
	}

	/**
		this method is used to evaluate the feedback contribution, given the current phase, d and v.
	*/
	inline double computeFeedback(SimBiController* con, Joint* j, double phi, const Vector3d& d, const Vector3d& v){
		if (bFeedback == NULL)
			return 0;
		return bFeedback->getFeedbackContribution(con, j, phi, d, v);
	}

	/** 
		Update this component to recenter it around the new given D and V trajectories
	*/
	void updateComponent(SimBiController* con, Joint* j, Trajectory1D& newDTrajX, Trajectory1D& newDTrajZ, Trajectory1D& newVTrajX, Trajectory1D& newVTrajZ, 
					      Trajectory1D* oldDTrajX, Trajectory1D* oldDTrajZ, Trajectory1D* oldVTrajX, Trajectory1D* oldVTrajZ, int nbSamples );

	/**
		This method is used to read the knots of a base trajectory from the file, where they are specified one (knot) on a line
	*/
	void readBaseTrajectory(FILE* f);

	/**
		This method is used to read a trajectory from a file
	*/ 
	void readTrajectoryComponent(FILE* f);

	/**
		This method is used to read a trajectory from a file
	*/ 
	void writeBaseTrajectory(FILE* f);
    void writeBaseTrajectory(std::ofstream& f);
	/**
		This method is used to write a trajectory to a file
	*/
	void writeTrajectoryComponent(FILE* f);
    void writeTrajectoryComponent(std::ofstream& f);
};


/**
 *  This helper class is used to hold information regarding one state trajectory. This includes: a sequence of components, 
 *	the index of the joint that this trajectory applies to, the coordinate frame in which the final orientation is expressed, etc.
 */
class Trajectory
{
public:
	//these are the components that define the current trajectory
	DynamicArray<TrajectoryComponent*> components;
	
	//if the biped that is controlled is in a left-sideed stance, then this is the index of the joint that
	//the trajectory is used to control - it is assumed that if this is -1, then the trajectory applies
	//to the torso, and not to a joint
	int leftStanceIndex;
	//and this is the index of the joint that the trajectory is associated to if the biped is in a
	//right-side stance
	int rightStanceIndex;
	//we'll keep the joint name, for debugging purposes
	char jName[100];

	//if this variable is set to true, then the desired orientation here is expressed in character coordinates, otherwise it is relative
	//to the parent
	bool relToCharFrame;

	//this is the trajectory for the strength of the joing.
	Trajectory1D* strengthTraj;

	/**
		default constructor
	*/
	Trajectory(){
		leftStanceIndex = rightStanceIndex = -1;
		strcpy_s(jName, "NoNameJoint");
		strengthTraj = NULL;
		relToCharFrame = false;
	}

	/**
		default destructor.
	*/
	~Trajectory(){
		for (uint i=0;i<components.size();i++)
			delete components[i];
	}

	/**
		this method is used to evaluate the trajectory at a given point phi, knowing the stance of the character, 
		and the d and v values used for feedback.
	*/
	inline Quaternion evaluateTrajectory(SimBiController* con, Joint* j, int stance, double phi, const Vector3d& d, const Vector3d& v){
		Quaternion q = Quaternion(1, 0, 0, 0);

		for (uint i=0;i<components.size();i++)
			q = components[i]->evaluateTrajectoryComponent(con, j, stance, phi, d, v) * q;

		return q;
	}

	/**
		this method is used to evaluate the strength of the joint at a given value of phi.
	*/
	inline double evaluateStrength(double phiToUse) {
		if( strengthTraj == NULL ) return 1.0;
		return strengthTraj->evaluate_catmull_rom( phiToUse );
	}

	/**
		this method returns the joint index that this trajectory applies to, unless this applies to the root, in which case it returns -1.
	*/
	inline int getJointIndex(int stance){
		return (stance == LEFT_STANCE)?(leftStanceIndex):(rightStanceIndex);
	}

	/** 
		Update all the components to recenter them around the new given D and V trajectories
	*/
	void updateComponents(SimBiController* con, Joint* j, Trajectory1D& newDTrajX, Trajectory1D& newDTrajZ, Trajectory1D& newVTrajX, Trajectory1D& newVTrajZ,
						   Trajectory1D* oldDTrajX, Trajectory1D* oldDTrajZ, Trajectory1D* oldVTrajX, Trajectory1D* oldVTrajZ, int nbSamples ) {
		int nbComponents = components.size();
		for (int i=0;i<nbComponents;i++)
			components[i]->updateComponent(con, j, newDTrajX, newDTrajZ, newVTrajX, newVTrajZ,
										   oldDTrajX, oldDTrajZ, oldVTrajX, oldVTrajZ, nbSamples );
	}

	/**
		This method is used to read a trajectory from a file
	*/ 
	void readTrajectory(FILE* f);

	/**
		This method is used to write the knots of a strength trajectory to the file, where they are specified one (knot) on a line
	*/
	void writeStrengthTrajectory(FILE* f);
    void writeStrengthTrajectory(std::ofstream& f);

	/**
		This method is used to write a trajectory to a file
	*/
	void writeTrajectory(FILE* f);
    void writeTrajectory(std::ofstream& f);
};

/**
 *	A simbicon controller is made up of a set of a number of states. Transition between states happen on foot contact, time out, user interaction, etc.
 *  Each controller state holds the trajectories for all the joints that are controlled. 
 */
class SimBiConState
{
friend class ControllerEditor;
friend class SimBiController;
private:
	//this is the array of trajectories, one for each joint that is controlled
	
	//this is a description of this state, for debugging purposes
	char description[100];
	//this is the number of the state that we should transition to in the controller's finite state machine
	int nextStateIndex;
	//this is the ammount of time that it is expected the biped will spend in this state
	double stateTime;

	//upon a transition to a new FSM state, it is assumed that the stance of the character either will be given stance, it will be reverseed , or keept the same.
	//if a state is designed for a certain stance, it is given by this variable
	//for generic states, this variable is used to determine if the stance should be reversed (as opposed to set to left or right), or stay the same.
	bool reverseStance;
	//and if this is the same, then upon entering this FSM state, the stance will remain the same
	bool keepStance;
	//if both keepStance and reverseStance are set to false, then this is the state that the character is asumed to take
	int stateStance;

	//if this variable is set to true, it indicates that the transition to the new state should occur when the swing foot contacts the ground
	//if this variable is false, then it will happen when the time of the controller goes up
	bool transitionOnFootContact;
	//if we are to allow a transition on foot contact, we need to take care of the possibility that it
	//will occur early. In this case, we may still want to switch. If phi is at least this, then it is assumed
	//that we can transition;
	double minPhiBeforeTransitionOnFootContact;
	//also, in order to make sure that we don't transition tooooo early, we expect a minimum force applied on the swing foot before
	//it should register as a contact
	double minSwingFootForceForContact;

	//this is the trajectory for the zero value of  the feedback d
	Trajectory1D* dTrajX;
	Trajectory1D* dTrajZ;

	//this is the trajectory for the zero value of  the feedback v
	Trajectory1D* vTrajX;
	Trajectory1D* vTrajZ;

public:
	/**
		default constructor
	*/
	SimBiConState(void){
		strcpy_s(description, "Uninitialized state");
		nextStateIndex = -1;
		this->stateTime = 0;
		transitionOnFootContact = true;
		minPhiBeforeTransitionOnFootContact = 0.5;
		minSwingFootForceForContact = 20.0;
		reverseStance = false;
		keepStance = false;

		dTrajX = NULL;
		dTrajZ = NULL;
		vTrajX = NULL;
		vTrajZ = NULL;
	}

    DynamicArray<Trajectory*> sTraj;

	/**
		destructor
	*/
	~SimBiConState(void){
		for (uint i=0;i<sTraj.size();i++)
			delete sTraj[i];
	}

	/**
		this method is used to determine the new stance, based on the information in this state and the old stance
	*/
	inline int getStateStance(int oldStance){
		if (keepStance == true)
			return oldStance;
		if (reverseStance == false)
			return stateStance;
		if (oldStance == LEFT_STANCE)
			return RIGHT_STANCE;
		return LEFT_STANCE;
	}

	/**
		Returns the time we're expecting to spend in this state
	*/
	inline double getStateTime(){
		return stateTime;
	}

	/**
		this method is used to retrieve the index of the next state 
	*/
	inline int getNextStateIndex(){
		return nextStateIndex;
	}

	/**
		this method is used to return the number of trajectories for this state
	*/
	inline int getTrajectoryCount(){
		return sTraj.size();
	}

	/**
		Access a given trajectory
	*/
	inline Trajectory* getTrajectory( uint idx ) {
		if( idx >= sTraj.size() ) return NULL;
		return sTraj[idx];
	}

	/**
		Access a given trajectory by name
	*/
	inline Trajectory* getTrajectory( const char* name) {
		for (uint i=0;i<sTraj.size();i++)
			if (strcmp(sTraj[i]->jName, name) == 0)
				return sTraj[i];
		return NULL;
	}

	/**
		This method is used to determine if, based on the parameters passed in and the type of state this is,
		the current state in the controller FSM needs to be transitioned from.
	*/
	inline bool needTransition(double phi, double swingFootVerticalForce, double stanceFootVerticalForce){
		//if it is a foot contact based transition
		if (transitionOnFootContact == true){
			//transition if we have a meaningful foot contact, and if it does not happen too early on...
			if ((phi > minPhiBeforeTransitionOnFootContact && swingFootVerticalForce > minSwingFootForceForContact) || phi >= 1)
				return true;
			return false;
		}

		//otherwise it must be a time-based transition
		if (phi >= 1)
			return true;

		return false;
	}

	/**
		This method makes it possible to access the state description
	*/
	inline const char* getDescription() {
		return description;
	}

	/**
		This method is used to read the state parameters from a file
	*/
	void readState(FILE* f, int offset);

	/**
		This method is used to write the state parameters to a file
	*/
	void writeState(FILE* f, int index);

    void writeState(std::ofstream& f, int index);


	/** 
		Update all the trajectories to recenter them around the new given D and V trajectories
		Also save these new D and V trajectories.
	*/
	void updateDVTrajectories(SimBiController* con, Joint* j, Trajectory1D& newDTrajX, Trajectory1D& newDTrajZ, Trajectory1D& newVTrajX, Trajectory1D& newVTrajZ, int nbSamples = 100 );


	/**
		This method is used to read the knots of a 1D trajectory from the file, where they are specified one (knot) on a line
		The trajectory is considered complete when a line starting with endingLineType is encountered
	*/
	static void readTrajectory1D(FILE* f, Trajectory1D& result, int endingLineType );

	static void writeTrajectory1D(FILE* f, Trajectory1D& result, int startingLineType, int endingLineType );
    static void writeTrajectory1D(std::ofstream& f, Trajectory1D& result, int startingLineType, int endingLineType );

};


