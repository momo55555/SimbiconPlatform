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

#include "stdafx.h"

#include "SimGlobals.h"

//initialize all the parameters to some sensible values.

//give this a very high value so that we can use the scripted values in the rb specs for the value to use
double SimGlobals::gravity = -9.8;
Vector3d SimGlobals::up = Vector3d(0, 1, 0);
int SimGlobals::forceHeadingControl = 1;
double SimGlobals::desiredHeading = 0;
double SimGlobals::dt = 1.0/(2000.0);
World* SimGlobals::activeRbEngine = NULL;

double SimGlobals::conInterpolationValue;
double SimGlobals::bipDesiredVelocity;


double SimGlobals::targetPos = 0;


double SimGlobals::targetPosX = 0;
double SimGlobals::targetPosZ = 0;

int SimGlobals::constraintSoftness = 1;
int SimGlobals::CGIterCount = 0;
int SimGlobals::linearizationCount = 1;


double SimGlobals::rootSagittal = 0;
double SimGlobals::rootLateral = 0;
double SimGlobals::swingHipSagittal = 0;
double SimGlobals::swingHipLateral = 0;
double SimGlobals::stanceAngleSagittal = 0;
double SimGlobals::stanceAngleLateral = 0;
double SimGlobals::stanceKnee = 0;

double SimGlobals::COMOffsetX = 0;
double SimGlobals::COMOffsetZ = 0;
