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

#include <PUtils.h>
#include "Controller.h"
#include "Character.h"
#include <fstream>



/**
	This class is used as a container for the properties needed by a PD controller
*/
class ControlParams{
public:
	//this variable is set to true if the current joint is being actively controlled, false otherwise
	bool controlled;
	//these two variables are the proporitonal and derivative gains for the PD controller used to compute the torque
	double kp, kd;
	//this is the maximum absolute value torque allowed at this joint
	double maxAbsTorque;
	//the torques about the about the x, y and z axis will be scaled differently to account for the potentially different principal moments of inertia
	//of the child
	Vector3d scale;

	double strength;

	//this variable, if true, indicates that the desired orientation is specified in the character coordinate frame
	bool relToCharFrame;

	//and this is the coordinate frame that the desired orientation is specified in, if relToCharFrame is true
	Quaternion charFrame;

	/**
		This constructor initializes the variables to some safe, default values
	*/
	ControlParams(){
		controlled = false;
		kp = kd = 0;
		maxAbsTorque = 0;
		scale = Vector3d();
		strength = 1;
		relToCharFrame = false;
	}
};

/**
	A pose controller is used to have the character track a given pose.
	Each pose is given as a series of relative orientations, one for
	each parent-child pair (i.e. joint). Classes extending this one 
	have to worry about setting the desired relative orientation properly.
*/
class PoseController : public Controller
{
protected:
	//this is the pose that the character is aiming at achieving
	DynamicArray<double> desiredPose;
	//this is the array of joint properties used to specify the 
	DynamicArray<ControlParams> controlParams;

	/**
		This method is used to parse the information passed in the string. This class knows how to read lines
		that have the name of a joint, followed by a list of the pertinent parameters. If this assumption is not held,
		then classes extended this one are required to provide their own implementation of this simple parser
	*/
	virtual void parseGainLine(char* line);
public:
	/**
		Constructor - expects a character that it will work on
	*/
	PoseController(Character* ch);
	virtual ~PoseController(void);

	/**
		This method is used to compute the torques, based on the current and desired poses
	*/
	virtual void computeTorques(DynamicArray<ContactPoint> *cfs);

	/**
		This method is used to compute the PD torque, given the current relative orientation of two coordinate frames (child and parent),
		the relative angular velocity, the desired values for the relative orientation and ang. vel, as well as the virtual motor's
		PD gains. The torque returned is expressed in the coordinate frame of the 'parent'.
	*/
	static Vector3d computePDTorque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel, const Vector3d& wRelD, ControlParams* pdParams);

	/**
		This method is used to scale and apply joint limits to the torque that is passed in as a parameter. The orientation that transforms 
		the torque from the coordinate frame that it is currently stored in, to the coordinate frame of the 'child' to which the torque is 
		applied to (it wouldn't make sense to scale the torques in any other coordinate frame)  is also passed in as a parameter.
	*/
	static void scaleAndLimitTorque(Vector3d* torque, ControlParams* pdParams, const Quaternion& qToChild);

	/**
		This method is used to apply joint limits to the torque passed in as a parameter. It is assumed that
		the torque is already represented in the correct coordinate frame
	*/
	static void limitTorque(Vector3d* torque, ControlParams* cParams);

	/**
		This method is used to read the gain coefficients, as well as max torque allowed for each joint
		from the file that is passed in as a parameter.
	*/
	void readGains(char* fName);

	/**
		This method is used to read the gain coefficients, as well as max torque allowed for each joint
		from the file that is passed in as a parameter.
	*/
	void readGains(FILE* f);
	
	
	/**
		This method is used to write the gain coefficients, as well as max torque allowed for each joint
		from the file that is passed in as a parameter.
	*/
	void writeGains(FILE* f);

    void writeGains(std::ofstream& f);

	/**
		sets the targets to match the current state of the character
	*/
	void setTargetsFromState();
};


