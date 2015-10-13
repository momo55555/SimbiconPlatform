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

#include "SimBiConFramework.h"
#include <PUtils.h>
#include "PoseController.h"
#include "SimBiController.h"
#include "SimGlobals.h"
#include <PhysX3World.h>

SimBiConFramework::SimBiConFramework(char* input, char* conFile){
    //create the physical world...
    pw = SimGlobals::getRBEngine();
    con = NULL;
    bip = NULL;
    bool conLoaded = false;


    //now we'll interpret the input file...
    FILE *f = fopen(input, "r");


    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    //this is where it happens.
    while (!feof(f)){
        //get a line from the file...

        fgets(buffer, 200, f);
        if (feof(f))
            break;
        char *line = lTrim(buffer);
        int lineType = getConLineType(line);
        switch (lineType) {
        case LOAD_RB_FILE:
            pw->loadRBsFromFile(trim(line));
            if (bip == NULL && pw->getAFCount()>0) {
                bip = new Character(pw->getAF(0));
                con = new SimBiController(bip);
            }
            break;
        case LOAD_CON_FILE:
            if( conFile != NULL ) break; // Controller file

            con->loadFromFile(trim(line));
            conLoaded = true;
            break;
        case CON_NOT_IMPORTANT:

            break;
        case CON_COMMENT:
            break;
        default:
           return;
        }
    }
    fclose(f);

    if( conFile != NULL ) {

        con->loadFromFile(conFile);
        conLoaded = true;
    }

    if (!conLoaded)
        return;

    //in case the state has changed while the controller was loaded, we will update the world again...
    //	pw->updateTransformations();

    //initialize the last foot position to the position of the stance foot - this may not always be the best decision, but whatever...
    lastFootPos = con->getStanceFootPos();
}


SimBiConFramework::~SimBiConFramework(void){
	delete con;
}

/**
	this method is used to advance the simulation. Typically, we will first compute the control, and then we will take one
	simulation step. If we are to apply control at this point in the simulation, we can either use a controller to recompute it,
	or we can use the values that were set before. This method returns true if the controller transitions to a new state, false
	otherwise.
*/
bool SimBiConFramework::advanceInTime(double dt, bool applyControl, bool recomputeTorques, bool advanceWorldInTime){
	if (applyControl == false) 
		con->resetTorques();
	else
		if (recomputeTorques == true)
			con->computeTorques(pw->getContactForces());

	//not applying control is the same as just resetting the torques
	con->applyTorques();
	if (advanceWorldInTime)
		pw->advanceInTime(dt);


	bool newFSMState = (con->advanceInTime(dt, pw->getContactForces()) != -1);
	con->updateDAndV();

	//here we are assuming that each FSM state represents a new step. 
	if (newFSMState){
		lastStepTaken = Vector3d(lastFootPos, con->getStanceFootPos());
		//now express this in the 'relative' character coordinate frame instead of the world frame
		lastStepTaken = con->getCharacterFrame().getComplexConjugate().rotate(lastStepTaken);
		lastFootPos = con->getStanceFootPos();
	}

	return newFSMState;
}

/**
	populates the structure that is passed in with the state of the framework
*/
void SimBiConFramework::getState(SimBiConFrameworkState* conFState){
	conFState->worldState.clear();
	//read in the state of the world (we'll assume that the rigid bodies and the ode world are synchronized), and the controller
	pw->getState(&(conFState->worldState));
	con->getControllerState(&(conFState->conState));
	conFState->lastFootPos = lastFootPos;
	//now copy over the contact force information
	conFState->cfi.clear();
	DynamicArray<ContactPoint> *cfs = pw->getContactForces();
	for (uint i=0;i<cfs->size();i++){
		conFState->cfi.push_back(ContactPoint((*cfs)[i]));
	}
}

/**
	populates the state of the framework with information passed in with the state of the framework
*/
void SimBiConFramework::setState(SimBiConFrameworkState& conFState){
	//set the state of the world, and that of the controller
	pw->setState(&(conFState.worldState));
	con->setControllerState(conFState.conState);
	lastFootPos = conFState.lastFootPos;
	DynamicArray<ContactPoint> *cfs = pw->getContactForces();
	cfs->clear();
	//now copy over the contact force information
	for (uint i = 0; i < conFState.cfi.size(); i++)
		cfs->push_back(ContactPoint(conFState.cfi[i]));
}
