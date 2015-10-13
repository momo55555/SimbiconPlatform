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
#include "Character.h"
#include <World.h>

/**
	This class is used to provide a generic interface to a controller. A controller acts on a character - it computes torques that are
	applied to the joints of the character. The details of how the torques are computed are left up to the classes that extend this one.
*/
class Controller
{
protected:
	//this is the character that the controller is acting on
	Character* character;
	//and this is the array of torques that will be computed in order for the character to match the desired pose - stored in world coordinates
	DynamicArray<Vector3d> torques;
	//this is the number of joints of the character - stored here for easy access
	int jointCount;
public:
	Controller(Character* ch);
	virtual ~Controller(void);

	/**
		This method is used to compute the torques, based on the current and desired poses
	*/
	virtual void computeTorques(DynamicArray<ContactPoint> *cfs) = 0;

	/**
		This method is used to apply the torques that are computed to the character that is controlled.
	*/
	void applyTorques();

	/**
		This method is used to reset the torques that are to be applied.
	*/
	void resetTorques();


};
