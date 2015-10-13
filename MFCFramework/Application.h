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

#include <Vector3d.h>
#include <SimBiConFramework.h>

#include <set>

using namespace std;

/**
 * This is the class that is responsible with implementing the details of what the program does.
 */
class Application{
private:
	//this is the texture used for the ground

public:
	/**
	 * Constructor.
	 */
	Application(void);
	/**
	 * Destructor.
	 */
	~Application(void);


	/**
	 * This method gets called when the application gets initialized. 
	 */
	virtual void init();


	/**
	 * This method is used to restart the application.
	 */
	virtual void restart();

	/**
	 * This method is used to reload the application.
	 */
	virtual void reload();
    virtual Vector3d TenStep(int& controShotToWrite);
    virtual void setParameter(vector<Vector3d>);
    virtual void reloadParameters(vector<double> OptimizedParameters);


	/**
	 * This method will get called on idle. This method should be responsible with doing the work that the application needs to do 
	 * (i.e. run simulations, and so on).
	 */
	virtual Vector3d processTask();

    //this is the physical world that contains all the objects inside
    SimBiConFramework* conF;

};

