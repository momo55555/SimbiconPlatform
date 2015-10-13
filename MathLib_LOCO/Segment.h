#pragma once

#include "Point3d.h"
#include "Vector3d.h"

/*==========================================================================================================================================*
 *	This class provides the implementation of a segment, and a few useful method associated with them.                                      *
 *==========================================================================================================================================*/

class Segment
{
public:
	//the end points of the segment
	Point3d a, b;

	Segment(Point3d& a_, Point3d& b_);
	
	Segment(){
		a = Point3d();
		b = Point3d();
	}
	
	/**
		Copy constructor
	*/
	Segment(const Segment& other);

	/**
		Copy operator
	*/
	Segment& operator = (const Segment& other);


	~Segment();

	/**
		This method returns the point on the current segment that is closest to the point c that is passed in as a paremter.
	*/
	void getClosestPointTo(const Point3d& c, Point3d* result) const;

	/**
		This method returns the segment that connects the closest pair of points - one on the current segment, and one on the segment that is passed in. The
		'a' point of the resulting segment will lie on the current segment, while the 'b' point lies on the segment that is passed in as a parameter.
	*/
	void getShortestSegmentTo(const Segment& other, Segment* result) const;


	/**
		Draws the segment - for visualization purposes.
	*/
	void draw();

};