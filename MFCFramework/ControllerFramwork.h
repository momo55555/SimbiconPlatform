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

#include <SimBiConFramework.h>
#include "Application.h"
#include <Trajectory.h>
#include <Vector3d.h>
#include "Globals.h"

#include <vector>

using namespace std;


/**
  * This class is used to build ControllerFramework and use it to control articulated figures.
  */
class ControllerEditor : public Application{
public:

    World* world;


    int stepNum;
    Vector3d FootSize;
//     ObjectiveFunctions obj1;
//     shark::MOCMA mocma;

	//SimBiConFramework* conF;

    void SimulatedAnnealing(Vector3d footSize, ReducedCharacterState rNew);

	//this array will be used to save/load the state of the dynamic world
	DynamicArray<double> worldState;

	//this is the name of the input file
	char inputFile[100];

	//this is the initial state of the framework...
	SimBiConFrameworkState conState;

	// This indicates the number of the future control shot and the maximal number of shots
	int nextControlShot;
	int maxControlShot;
    int nextControlShotToWrite;

	//keep track of the two variables below to report the speed of the character
	double avgSpeed;
	int timesVelSampled;


	// These trajectories are reset after every cycle
	// dTrajX and vTrajX are sign-reversed on right stance cycles
	Trajectory1D dTrajX;
	Trajectory1D dTrajZ;
	Trajectory1D vTrajX;
	Trajectory1D vTrajZ;

	// Contains the FSM state index of the last simulation step
	int lastFSMState;

	/**
		This method is called whenever the controller has just taken a new step
	*/
	virtual void stepTaken();

public:
	/**
	 * Constructor.
	 */
	ControllerEditor(void);

	/**
	 * Destructor.
	 */
	virtual ~ControllerEditor(void);

	/**
		This method draws the desired target that is to be tracked.
	*/
	void drawDesiredTarget();

    Vector3d TenStep(int& controlShotToWrite);

	// This is our associated curve editor
/*	DynamicArray<CurveEditor*> curveEditors;*/

	/**
	 * This method is used to create a physical world and load the objects in it.
	 */
	virtual void loadFramework();

	/**
	 * This method is used to create a physical world and load the objects in it.
	 */
	void loadFramework( int controlShot );

	/**
	 * This method is called whenever the window gets redrawn.


	/**
	 * This method is used to restart the application.
	 */
	virtual void restart();

	/**
	 * This method is used to reload the application.
	 */
	virtual void reload();

	/**
		This method is used to undo the last operation
	*/
	virtual void undo();

	/**
		This method is used to redo the last operation
	*/
	virtual void redo();

    virtual void setParameter(vector<Vector3d>());

    virtual void reloadParameters(vector<double> OptimizedParameters);

	/**
     *	This method is used when a mouse event gets generated. This method returns true if the message gets processed, false otherwise.
	 */

// 	/**
// 	 * Delete all curve editors
// 	 */
// 	inline void clearEditedCurves() {
// 		for( uint i = 0; i < curveEditors.size(); ++i )
// 			delete curveEditors[i];
// 		curveEditors.clear();
// 	}
// 
// 	/**
// 	 * Selects the trajectory to edit
// 	 */
// 	inline void addEditedCurve( Trajectory1D* editedCurve ) {
// 		curveEditors.push_back( new CurveEditor(300*curveEditors.size(), 0, 300, 200) );
// 		curveEditors.back()->setTrajectory( editedCurve );
// 	}

	/**
	 * Registers TCL functions specific to this application
	 */


	inline SimBiConFramework* getFramework(){
		return conF;
	}



	/**
	 * This method returns the target that the camera should be looking at
	 */
	//Point3d getCameraTarget();

	/**
	* This method will get called on idle. This method should be responsible with doing the work that the application needs to do 
	* (i.e. run simulations, and so on).
	*/
	virtual Vector3d processTask();


	/**
     * This method is to be implemented by classes extending this one. The output of this function is a point that
	 * represents the world-coordinate position of the dodge ball, when the position in the throw interface is (x, y).
	 */
/*	virtual void getDodgeBallPosAndVel(double x, double y, double strength, Point3d* pos, Vector3d* vel);*/


};




