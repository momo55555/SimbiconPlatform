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

//disable all the 'deprecated function' warnings
#pragma warning( disable : 4996)

#include "Application.h"

/**
	This class is used as a container for all the global variables that are needed in the system.
*/
class Globals{
public:
	//indicates whether or not the animation (i.e. simulation, play back, etc) is playing

    static Application* app;
	static int animationRunning;
	//gives the ratio of animation time to real time.
	static double animationTimeToRealTimeRatio;
	//this is the desired frame rate
	static double desiredFrameRate;
	//flag that controls the drawing of the FPS information
	static int drawFPS;
	//flag that controls the drawing of the cubeMap
	static int drawCubeMap;
	//flag that controls the drawing of the golbal axes
	static int drawGlobalAxes;
	//flag that controls the drawing of the shadows
	static int drawShadows;
	//flag that controls the drawing of the collision primitives
	static int drawCollisionPrimitives;
	//flag that controls the drawing of the ground
	static int drawGroundPlane;
	//flag that controls the drawing of the contact forces
	static int drawContactForces;
	//flag that controls the drawing of the desired pose that is being tracked
	static int drawDesiredPose;
	//controls the phase at which the target pose is drawn
	static double targetPosePhase;
	//flag that controls the drawing of the push interface
	static int drawPushInterface;
	//flag that controls the drawing of the curve editor
	static int drawCurveEditor;
	//flag that controls the drawing of the canvas
	static int drawCanvas;
	//flag that controls the camera tracking
	static int followCharacter;
	//flag that controls the joint position display
	static int drawJoints;
	//flag that controls the drawing of screenshots
	static int drawScreenShots;
	//flag that controls the capturing of the 3D world in OBJ files
	static int drawWorldShots;
	//flag that controls the capturing of the controller
	static int drawControlShots;
	//a text variable to display the current control shot displayed
	static char* currControlShotStr;
	//flat that indicates if the controller D and V trajectories should be updated on next step
	static int updateDVTraj;
	//indicates wether or not to use shaders
	static bool useShader;
	//indicates wether or not to use the console
	static bool useConsole;

	//these params define the ground plane - for drawing purposes only
	static double a, b, c, d;

	Globals(void);
	~Globals(void);

	static void changeCurrControlShotStr( int currControlShot );
};


//print in an openGL window. The raster position needs to be already defined.
void gprintf(const char *format, ...);

//print in an openGL window with a large font. The raster position needs to be already defined.
void glargeprintf(const char *format, ...);



//declare some useful constants and data types here as well
#define STRLEN 200
#define MAX_LINE 255
typedef char STR[STRLEN];


#define MOUSE_LBUTTON 1
#define MOUSE_RBUTTON 2
#define MOUSE_MBUTTON 3
#define MOUSE_WHEEL_DOWN 4
#define MOUSE_WHEEL_UP 5

#define MOUSE_DOWN 1
#define MOUSE_DRAG 2
#define MOUSE_UP 3
#define MOUSE_MOVE 4


